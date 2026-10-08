import importlib.util
from pathlib import Path

import yaml


SCRIPT = Path(__file__).parents[1] / "scripts" / "calibrate_damiao_steering.py"
SPEC = importlib.util.spec_from_file_location("damiao_calibration", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeReader:
    def read_float(self, motor_id, master_id, register):
        assert (motor_id, master_id, register) == (1, 0x11, 0x51)
        return 1.25


def sample_config(direction=-1):
    wheels = {}
    for index, name in enumerate(MODULE.WHEEL_NAMES, 1):
        wheels[name] = {
            "steering": {
                "motor_id": index,
                "master_id": 0x10 + index,
                "direction": direction if name == "front_left" else 1,
                "offset_rad": 0.2,
            }
        }
    return {"wheels": wheels}


def test_read_wheel_and_offset_formula():
    result = MODULE.read_wheel(FakeReader(), sample_config(), "front_left")
    assert result["raw_position"] == 1.25
    assert result["joint_position"] == -1.05
    assert -result["raw_position"] * result["direction"] == 1.25


def test_save_offset_creates_backup(tmp_path):
    path = tmp_path / "chassis.yaml"
    config = sample_config()
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    backup = MODULE.save_offset(path, config, "front_left", 1.25)
    assert backup.is_file()
    saved = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert saved["wheels"]["front_left"]["steering"]["offset_rad"] == 1.25


def test_reader_request_contains_no_motion_command(monkeypatch):
    sent = []

    class FakeSocket:
        def bind(self, address):
            assert address == ("can0",)

        def send(self, frame):
            sent.append(MODULE.CAN_FRAME.unpack(frame))

        def recv(self, size):
            payload = bytes((1, 0, 0x33, 0x51)) + __import__("struct").pack("<f", 0.75)
            return MODULE.CAN_FRAME.pack(0x11, 8, payload)

        def close(self):
            pass

    monkeypatch.setattr(MODULE.socket, "socket", lambda *args: FakeSocket())
    monkeypatch.setattr(MODULE.select, "select", lambda *args: ([args[0][0]], [], []))
    reader = MODULE.DamiaoParameterReader("can0", 0.5)
    assert reader.read_float(1, 0x11, 0x51) == 0.75
    can_id, length, payload = sent[0]
    assert can_id == 0x7FF and length == 8
    assert payload[:4] == bytes((1, 0, 0x33, 0x51))
    assert 0xFC not in payload and 0xFE not in payload
