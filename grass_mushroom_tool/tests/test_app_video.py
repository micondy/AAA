import importlib.util
import json
import threading
import urllib.error
import urllib.request
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('gm_app', Path(__file__).parents[1]/'app.py')
app_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app_module)


@pytest.fixture
def running_app(tmp_path):
    app = app_module.App(tmp_path/'data', tmp_path/'bags', '/test/camera', enable_ros=False)
    session = app.store.create(date='20261105')
    session.add_sample()
    server = app_module.Server(('127.0.0.1', 0), app_module.Handler, app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    def post(path, body):
        req = urllib.request.Request(f'http://127.0.0.1:{server.server_port}'+path,
            data=json.dumps(body).encode(), headers={'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req) as response: return response.status, json.load(response)
        except urllib.error.HTTPError as exc:
            return exc.code, json.load(exc)
    post.port = server.server_port
    yield app, post
    server.shutdown()
    server.server_close()
    thread.join()


def test_recording_rejects_disabled_camera(running_app):
    app, post = running_app
    status, data = post('/api/video/start', {'sample':'S001'})
    assert status == 400 and '未启用' in data['error']


def test_recording_uses_aligned_topics_and_sample_context(running_app, monkeypatch):
    app, post = running_app
    app.ros = SimpleNamespace(available=lambda:True, status=lambda:{'aligned':True,'age':{'aligned_depth':0}})
    captured = {}
    def start(name, **kwargs):
        captured.update(kwargs)
        return {'name':name,'running':True}
    monkeypatch.setattr(app.video, 'start', start)
    status, data = post('/api/video/start', {'sample':'S001'})
    assert status == 200 and data['video']['running']
    assert '/test/camera/aligned_depth_to_color/image_raw' in captured['topics']
    assert not any('/points' in topic for topic in captured['topics'])
    assert captured['context']['sample'] == 'S001'


def test_extract_rejects_active_or_invalid_recording(running_app):
    _, post = running_app
    for name in ('../bad', 'missing'):
        status, _ = post('/api/video/extract', {'name':name})
        assert status == 400


def test_snapshot_sequence_after_deleted_group(running_app):
    app, post = running_app
    folder = app.store.require().root/'samples/S001'
    (folder/'s001-d435i-depth-30.png').touch()
    captured = {}
    def snapshot(root, sid, **kwargs):
        captured.update(kwargs)
        return []
    app.ros = SimpleNamespace(snapshot=snapshot)
    status, data = post('/api/snapshot', {'sample':'S001'})
    assert status == 200 and data['group'] == captured['seq'] == 31


@pytest.mark.parametrize('path', ['/api/video/stop', '/api/bag/stop', '/api/not-found'])
def test_post_body_does_not_corrupt_next_request_on_same_connection(running_app, path):
    import http.client
    _, post = running_app
    connection = http.client.HTTPConnection('127.0.0.1', post.port, timeout=5)
    try:
        connection.request('POST', path, '{}', {'Content-Type': 'application/json'})
        response = connection.getresponse()
        assert response.status in (400, 404)
        response.read()
        connection.request('GET', '/api/ping')
        response = connection.getresponse()
        body = response.read()
        assert response.status == 200, body.decode()
        assert json.loads(body)['ok'] is True
    finally:
        connection.close()


@pytest.mark.parametrize('header,expected,status', [
    (None,b'0123456789',200),('bytes=2-4',b'234',206),
    ('bytes=-3',b'789',206),('bytes=7-',b'789',206),('bytes=99-',b'',416),
])
def test_video_preview_supports_playback_and_seeking(running_app, header, expected, status):
    import http.client
    app, post = running_app
    folder = app.video.root/'recording'
    folder.mkdir()
    (folder/'metadata.yaml').touch()
    (app.video.root/'recording.recording.json').write_text(json.dumps({'name':'recording'}))
    preview = app.video.root/'previews/recording-rgbd.mp4'
    preview.parent.mkdir()
    preview.write_bytes(b'0123456789')
    c = http.client.HTTPConnection('127.0.0.1',post.port,timeout=5)
    try:
        c.request('GET','/video-preview?name=recording',headers={'Range':header} if header else {})
        r=c.getresponse()
        assert r.status == status
        assert r.read() == expected
        if status != 416:
            assert r.getheader('Accept-Ranges') == 'bytes'
    finally:
        c.close()
