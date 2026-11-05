"use strict";

/* ============================ 小工具 ============================ */
const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));
const esc = (v) =>
  String(v ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

const pad2 = (n) => String(n).padStart(2, "0");
const range = (a, b) => {
  const out = [];
  for (let i = a; i <= b; i += 1) out.push(i);
  return out;
};

function fmtBytes(n) {
  n = Number(n) || 0;
  const units = ["B", "KB", "MB", "GB", "TB"];
  let i = 0;
  while (n >= 1024 && i < units.length - 1) {
    n /= 1024;
    i += 1;
  }
  return `${n >= 100 || i === 0 ? Math.round(n) : n.toFixed(1)} ${units[i]}`;
}

function fmtDuration(sec) {
  sec = Math.max(0, Math.round(Number(sec) || 0));
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  const s = sec % 60;
  return h ? `${h}:${pad2(m)}:${pad2(s)}` : `${pad2(m)}:${pad2(s)}`;
}

function slug(v) {
  return (
    String(v ?? "")
      .normalize("NFKC")
      .replace(/[^0-9A-Za-z\u4e00-\u9fff_-]+/g, "-")
      .replace(/-{2,}/g, "-")
      .replace(/^[-_]+|[-_]+$/g, "")
      .slice(0, 32) || "x"
  );
}

function defaultDate() {
  const d = new Date();
  return `${d.getFullYear()}-${pad2(d.getMonth() + 1)}-${pad2(d.getDate())}`;
}
const todayStamp = () => defaultDate().replace(/-/g, "");

/* ============================ 状态 ============================ */
const state = {
  data: null,
  sampleId: null,
  env: { room: null, row: 1, layer: 1, view: "bed" },
  view: "samples",
  pending: null,
  signature: "",
  drafts: {},
  streamMode: "rgbd",
};

let uploading = 0;

/* 未保存的输入：轮询重绘时不丢失正在输入的内容 */
const SAMPLE_FIELDS = new Set([
  "m-room", "m-row", "m-layer", "m-cluster",
  "m-height", "m-width", "m-weight", "m-notes",
]);
const DRAFT_IDS = new Set([...SAMPLE_FIELDS, "ns-date", "ns-operator", "ns-rooms", "ns-notes", "room-config", "bag-name", "video-recording"]);

const draftKey = (id) => (SAMPLE_FIELDS.has(id) ? `${id}@${state.sampleId || ""}` : id);
function draft(id, fallback) {
  const key = draftKey(id);
  return Object.prototype.hasOwnProperty.call(state.drafts, key) ? state.drafts[key] : fallback;
}
function dropDraft(id) {
  delete state.drafts[draftKey(id)];
}

/* ============================ 请求 ============================ */
async function api(path, options = {}) {
  const opts = { method: options.method || "GET", headers: {} };
  if (options.json !== undefined) {
    opts.body = JSON.stringify(options.json);
    opts.headers["Content-Type"] = "application/json";
  }
  const res = await fetch(path, opts);
  let data = {};
  try {
    data = await res.json();
  } catch (_) {
    /* 忽略空响应 */
  }
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

function toast(message, kind = "info") {
  const el = $("#toast");
  el.textContent = message;
  el.className = `toast show ${kind}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => {
    el.className = "toast";
  }, 2800);
}

/* ============================ 数据刷新 ============================ */
function signatureOf(data) {
  const s = data.session;
  return JSON.stringify([
    s ? s.id : null,
    s ? [s.sample_count, s.photo_count, s.env_count, JSON.stringify(s.rooms)] : null,
    (data.samples || []).map(
      (x) =>
        `${x.id}|${x.updated_at}|${x.photos.length}|${x.room}|${x.row}|${x.layer}|${x.mode}|${x.cluster_count}|${x.height_mm}|${x.width_mm}|${x.weight_g}|${x.notes}`
    ),
    (data.environment || []).map((x) => x.rel),
    data.coverage ? JSON.stringify(data.coverage) : null,
    (data.sessions || []).map((x) => x.id),
    [data.video?.running, data.video?.name, Math.floor(data.video?.elapsed || 0), data.video?.last],
    (data.video_recordings || []).map((x) => [x.name, x.ready, x.preview_ready]),
    [data.video_preview?.running, data.video_preview?.frames, data.video_preview?.output, data.video_preview?.error],
    [data.camera?.available, data.camera?.aligned, (data.camera?.age?.aligned_depth ?? 999) < 5],
    [data.bag?.running, Math.floor(data.bag?.elapsed || 0), data.bag?.last],
  ]);
}

async function refresh() {
  let data;
  try {
    data = await api("/api/state");
  } catch (err) {
    toast(`连接服务失败：${err.message}`, "error");
    return;
  }
  state.data = data;

  const samples = data.samples || [];
  if (!samples.some((s) => s.id === state.sampleId)) {
    state.sampleId = samples.length ? samples[samples.length - 1].id : null;
  }
  const rooms = (data.coverage && data.coverage.rooms) || [];
  if (!rooms.some((r) => r.name === state.env.room)) {
    state.env.room = rooms.length ? rooms[0].name : null;
  }
  const rows = rooms.find((r) => r.name === state.env.room)?.rows || 0;
  if (rows && state.env.row > rows) state.env.row = 1;
  const layers = rooms.find((r) => r.name === state.env.room)?.layers || 0;
  if (layers && state.env.layer > layers) state.env.layer = 1;

  const sig = signatureOf(data);
  const changed = sig !== state.signature;
  state.signature = sig;

  if (changed) renderAll();
  else renderChips();
}

/* ============================ 取数据小助手 ============================ */
const currentSample = () => (state.data?.samples || []).find((s) => s.id === state.sampleId) || null;
const slotDef = (key) => (state.data?.slots || []).find((s) => s.key === key);
const envViewDef = (key) => (state.data?.env_views || []).find((v) => v.key === key);
const currentRoom = () => ((state.data?.coverage?.rooms) || []).find((r) => r.name === state.env.room) || null;

function expectedSampleName(sample, slot) {
  const seq = 1 + (sample.photos || []).filter((p) => p.slot === slot).length;
  return `${sample.id.toLowerCase()}-${slot}${seq > 1 ? "-" + seq : ""}.jpg`;
}

function expectedEnvName() {
  const { room, row, layer, view } = state.env;
  const scope = envViewDef(view)?.scope || "room";
  const parts = ["env", slug(room)];
  if (scope !== "room") parts.push(`r${pad2(row)}`);
  if (scope === "bed") parts.push(`l${pad2(layer)}`);
  parts.push(view);
  const same = (state.data?.environment || []).filter(
    (s) =>
      s.room === slug(room) &&
      s.view === view &&
      (scope === "room" ? true : Number(s.row) === Number(row)) &&
      (scope === "bed" ? Number(s.layer) === Number(layer) : true)
  ).length;
  return `${parts.join("-")}${same ? "-" + (same + 1) : ""}.jpg`;
}

/* ============================ 顶栏 ============================ */
function setChip(el, text, cls) {
  el.textContent = text;
  el.className = `chip ${cls || ""}`;
}

function renderChips() {
  const d = state.data;
  if (!d) return;

  const cam = d.camera || {};
  const chipCam = $("#chip-camera");
  const colorAge = cam.age ? cam.age.color : null;
  if (!cam.available) setChip(chipCam, "相机 未启用", "warn");
  else if (colorAge != null && colorAge < 5) {
    const size = cam.size && cam.size.color ? ` ${cam.size.color[0]}×${cam.size.color[1]}` : "";
    setChip(chipCam, `相机 在线${size}`, "ok");
  } else if (cam.error) setChip(chipCam, "相机 出错", "warn");
  else setChip(chipCam, "相机 无数据", "warn");

  const bag = d.bag || {};
  setChip(
    $("#chip-bag"),
    bag.running ? `● 录制 ${fmtDuration(bag.elapsed)} · ${fmtBytes(bag.bytes)}` : "录包 待机",
    bag.running ? "live" : ""
  );

  const disk = d.disk || {};
  const freeGb = disk.free ? disk.free / 1024 ** 3 : null;
  setChip(
    $("#chip-disk"),
    freeGb == null ? "磁盘 —" : `磁盘余 ${freeGb.toFixed(0)} GB`,
    freeGb != null && freeGb < 5 ? "warn" : ""
  );

  const line = $("#session-line");
  const s = d.session;
  line.textContent = s
    ? `${s.id} · ${s.sample_count} 个样本 · ${s.photo_count} 张样本照片 · ${s.env_count} 张环境照片`
    : "未建立会话";
}

/* ============================ 总渲染 ============================ */
function renderAll() {
  const active = document.activeElement;
  const focusId = active && active.id && active.tagName !== "BODY" ? active.id : null;
  const caret = focusId && typeof active.selectionStart === "number" ? active.selectionStart : null;

  renderChips();
  renderSamples();
  renderEnv();
  renderProgress();
  renderSettings();

  if (focusId) {
    const el = document.getElementById(focusId);
    if (el) {
      el.focus();
      if (caret != null) {
        try {
          el.setSelectionRange(caret, caret);
        } catch (_) {
          /* number 输入框不支持 */
        }
      }
    }
  }
}

function noSessionHtml() {
  return `<div class="card center">
    <h2>还没有采集会话</h2>
    <p class="muted">先建一个会话，之后拍的照片会自动命名并保存到 <code>data/grass_mushroom_日期/</code>。</p>
    <button class="primary" data-tabjump="settings">去新建会话</button>
  </div>`;
}

/* ============================ 样本视图 ============================ */
function renderSamples() {
  const el = $("#view-samples");
  const d = state.data;
  if (!d) {
    el.innerHTML = `<div class="card center"><p class="muted">正在连接采集服务…</p></div>`;
    return;
  }
  if (!d.session) {
    el.innerHTML = noSessionHtml();
    return;
  }
  const samples = d.samples || [];
  const sample = currentSample();
  el.innerHTML = `
    ${sampleNavHtml(sample, samples)}
    ${sample ? sampleBodyHtml(sample) : `<div class="card center"><p class="muted">还没有样本，点“＋ 新建样本”开始。</p></div>`}
    ${stripHtml(samples)}
  `;
}

function sampleNavHtml(sample, samples) {
  const index = samples.findIndex((s) => s.id === sample?.id);
  return `
  <div class="sample-nav">
    <button data-nav="-1" ${index <= 0 ? "disabled" : ""}>‹</button>
    <div class="sidbox">
      <span class="sid-label">编号</span>
      <span class="sid-value">${sample ? esc(sample.id.replace(/^S/, "")) : "—"}</span>
      <span class="sid-sub">${sample ? `${esc(sample.id)} · 第 ${index + 1}/${samples.length} 个` : "未选择"}</span>
    </div>
    <button data-nav="1" ${index < 0 || index >= samples.length - 1 ? "disabled" : ""}>›</button>
  </div>
  <div class="quick-row">
    <button class="primary" id="new-sample">＋ 新建样本</button>
    ${sample ? `<button class="danger" id="delete-sample">删除样本</button>` : ""}
  </div>`;
}

function roomOptions(current) {
  const rooms = ((state.data?.coverage?.rooms) || []).map((r) => r.name);
  const all = Array.from(new Set(current ? [current, ...rooms] : rooms));
  return (
    `<option value="">（未指定）</option>` +
    all
      .map((name) => `<option value="${esc(name)}" ${name === current ? "selected" : ""}>${esc(name)}</option>`)
      .join("")
  );
}

function sampleBodyHtml(sample) {
  const photos = sample.photos || [];
  const taken = new Set(photos.map((p) => p.slot));
  const required = state.data.required_slots || [];
  const done = required.filter((k) => taken.has(k)).length;

  const groups = [];
  for (const slot of state.data.slots || []) {
    let group = groups.find((g) => g.name === slot.group);
    if (!group) {
      group = { name: slot.group, slots: [] };
      groups.push(group);
    }
    group.slots.push(slot);
  }

  return `
  ${sensorCardHtml(sample)}

  <div class="card">
    <div class="card-head">
      <h2>样本 ${esc(sample.id)}</h2>
      <span class="tag ${done === required.length ? "ok" : "req"}">必需 ${done}/${required.length}</span>
    </div>
    <div class="grid-2">
      <div>
        <label class="field-label">菇房</label>
        <select id="m-room">${roomOptions(draft("m-room", sample.room) || sample.room)}</select>
      </div>
      <div>
        <label class="field-label">生长形态</label>
        <div class="seg">
          <button data-mode="individual" class="${sample.mode === "individual" ? "on" : ""}">单个</button>
          <button data-mode="cluster" class="${sample.mode === "cluster" ? "on" : ""}">成簇</button>
        </div>
      </div>
      <div>
        <label class="field-label">排</label>
        <input id="m-row" type="number" inputmode="numeric" min="0" value="${esc(draft("m-row", sample.row ?? ""))}">
      </div>
      <div>
        <label class="field-label">层</label>
        <input id="m-layer" type="number" inputmode="numeric" min="0" value="${esc(draft("m-layer", sample.layer ?? ""))}">
      </div>
      <div>
        <label class="field-label">簇内个数</label>
        <input id="m-cluster" type="number" inputmode="numeric" min="0" value="${esc(draft("m-cluster", sample.cluster_count ?? ""))}" placeholder="成簇时填">
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-head">
      <h2>测量记录</h2>
      <span class="muted">只填这三项，其余事后按照片分类</span>
    </div>
    <div class="grid-3">
      <div><label class="field-label">总高度 / mm</label><input id="m-height" type="number" inputmode="decimal" step="0.1" min="0" value="${esc(draft("m-height", sample.height_mm ?? ""))}"></div>
      <div><label class="field-label">最大宽度 / mm</label><input id="m-width" type="number" inputmode="decimal" step="0.1" min="0" value="${esc(draft("m-width", sample.width_mm ?? ""))}"></div>
      <div><label class="field-label">重量 / g</label><input id="m-weight" type="number" inputmode="decimal" step="0.1" min="0" value="${esc(draft("m-weight", sample.weight_g ?? ""))}"></div>
    </div>
    <div style="margin-top:10px">
      <label class="field-label">备注</label>
      <textarea id="m-notes" rows="2" placeholder="例如：破损、变色、被遮挡">${esc(draft("m-notes", sample.notes || ""))}</textarea>
    </div>
  </div>

  ${groups
    .map(
      (g) => `
    <div class="slot-group-title">${esc(g.name)}</div>
    <div class="slots">${g.slots.map((slot) => slotCardHtml(sample, slot)).join("")}</div>`
    )
    .join("")}`;
}

function slotCardHtml(sample, slot) {
  const photos = (sample.photos || []).filter((p) => p.slot === slot.key);
  return `
  <article class="slot ${photos.length ? "done" : ""}">
    <header>
      <span class="slot-title">${esc(slot.label)}</span>
      ${slot.required ? '<span class="tag req">必需</span>' : '<span class="tag">可选</span>'}
      ${photos.length ? `<span class="tag ok">${photos.length} 张</span>` : ""}
    </header>
    <p class="slot-hint">${esc(slot.hint)}</p>
    ${
      photos.length
        ? `<div class="thumbs">${photos.map(thumbHtml).join("")}</div>`
        : `<div class="expected">将保存为 <code>${esc(expectedSampleName(sample, slot.key))}</code></div>`
    }
    <div class="slot-actions">
      <button class="${photos.length ? "" : "blue"}" data-shoot="${slot.key}">📷 ${photos.length ? "重拍 / 补充" : "拍照"}</button>
    </div>
  </article>`;
}

function thumbHtml(photo) {
  return `
  <figure class="thumb">
    <img src="/thumb/${encodeURI(photo.rel)}?w=360" alt="${esc(photo.name)}" loading="lazy" data-preview="${esc(photo.rel)}">
    <figcaption title="${esc(photo.name)}">${esc(photo.name)}</figcaption>
    <button class="del" data-del="${esc(photo.rel)}" title="删除这张">✕</button>
  </figure>`;
}

function sensorCardHtml(sample) {
  const cam = state.data.camera || {};
  const files = (sample.photos || []).filter((p) => String(p.slot).startsWith("d435i-"));
  const colorAge = cam.age ? cam.age.color : null;
  const depthAge = cam.age ? (cam.aligned ? cam.age.aligned_depth : cam.age.raw_depth) : null;
  const online = cam.available && colorAge != null && colorAge < 5;
  const depthOk = online && depthAge != null && depthAge < 5;
  const sizeText = cam.size && cam.size.color ? `${cam.size.color[0]}×${cam.size.color[1]}` : "";
  const alignText = !cam.available
    ? "未启用 ROS"
    : depthOk
      ? cam.aligned
        ? `深度已对齐 ${sizeText}`
        : "原始深度仅供预览，快照需等待对齐"
      : "等待深度";
  const mode = state.streamMode || "rgbd";
  const modes = [
    ["rgbd", "彩色 + 深度"],
    ["color", "仅彩色"],
    ["depth", "仅深度"],
  ];
  // 只在本页可见时拉流：切标签/锁屏自动断开，省流量省电
  const streamOn = cam.available && state.view === "samples" && document.visibilityState === "visible";

  return `
  <div class="card">
    <div class="card-head">
      <h2>D435i 取景</h2>
      <span class="chip ${depthOk && cam.aligned ? "ok" : "warn"}">${alignText}</span>
    </div>
    ${
      streamOn
        ? `<div class="viewfinder">
             <img src="/stream/${mode}.mjpg" alt="D435i 实时画面" />
           </div>`
        : `<div class="viewfinder viewfinder-off">
             <span>${cam.available ? "实时画面已暂停（回到「样本」页继续）" : "未载入 ROS 环境，无法预览相机"}</span>
           </div>`
    }
    ${
      cam.available
        ? `<div class="stream-modes">
             ${modes
               .map(
                 ([key, label]) =>
                   `<button data-stream="${key}" class="${mode === key ? "on" : ""}">${label}</button>`
               )
               .join("")}
           </div>
           <p class="muted" style="margin:6px 0 10px">
             左：彩色 · 右：深度伪彩（红=近，蓝=远，黑=无效；范围按当前场景自动拉伸）。
             抓取只保存对齐深度；未收到对齐深度时会报错，不会保存原始深度。
             可以拍多组，第二组起文件名自动加 <code>-2</code>，不会覆盖。
           </p>`
        : `<p class="muted" style="margin-bottom:8px">拍照与测量仍可正常使用。</p>`
    }
    <button class="blue wide" id="snapshot" ${cam.available ? "" : "disabled"}>
      📸 抓取 ${esc(sample.id)} 的 RGB-D（彩色 + 对齐深度 + 点云）
    </button>
    ${videoCardHtml(sample, cam)}
    ${files.length ? `<div class="thumbs" style="margin-top:10px">${files.map(sensorFileHtml).join("")}</div>` : ""}
  </div>`;
}

function videoCardHtml(sample, cam) {
  const video = state.data.video || {};
  const job = state.data.video_preview || {};
  const recordings = (state.data.video_recordings || []).filter(r => r.ready);
  const selected = recordings.find(r => r.name === draft("video-recording", "")) || recordings[0];
  return `<div style="margin-top:20px">
    <h3>RGB-D 录制 / 回放</h3>
    <p class="muted">原始录制保留彩色、16 位深度及时间戳。生成播放预览后可直接观看，无需抽帧或保存照片。</p>
    ${video.running
      ? `<p>● 正在录制 ${esc(video.context?.sample || "")} · ${fmtDuration(video.elapsed)} · ${fmtBytes(video.bytes)}</p>
         <button class="danger wide" id="video-stop">■ 停止 RGB-D 录制并收尾</button>`
      : `<button class="primary wide" id="video-start" ${cam.available && video.available ? "" : "disabled"}>● 为 ${esc(sample.id)} 开始 RGB-D 连续录制</button>`}
    ${recordings.length ? `<label class="field" style="margin-top:12px">已完成录制<select id="video-recording">${recordings.map(r => `<option value="${esc(r.name)}" ${selected?.name === r.name ? "selected" : ""}>${esc(r.sample)} · ${esc(r.name)}</option>`).join("")}</select></label>
      ${selected?.preview_ready
        ? `<video controls playsinline preload="metadata" style="width:100%;margin-top:12px" src="/video-preview?name=${encodeURIComponent(selected.name)}"></video><p class="muted">左：彩色 · 右：深度伪彩。预览用于观看，原始毫米深度仍保存在 MCAP 中。</p>`
        : `<button class="blue wide" id="video-preview" ${job.running ? "disabled" : ""}>${job.running ? "正在生成播放预览…" : "生成可播放预览（彩色 + 深度）"}</button>`}` : ""}
    ${job.running ? `<p class="muted">正在处理 · 已编码 ${job.frames || 0} 组</p>` : job.error ? `<p class="muted">预览生成失败：${esc(job.error)}</p>` : ""}
  </div>`;
}

function sensorFileHtml(photo) {
  if (/\.(png|jpe?g|webp|gif)$/i.test(photo.name || "")) return thumbHtml(photo);
  const ext = ((photo.name || "").split(".").pop() || "").toUpperCase();
  return `
  <figure class="thumb filechip">
    <a class="chip" href="/media/${encodeURI(photo.rel)}" download="${esc(photo.name)}">下载 ${esc(ext)}</a>
    <figcaption title="${esc(photo.name)}">${esc(photo.name)}</figcaption>
    <button class="del" data-del="${esc(photo.rel)}" title="删除">✕</button>
  </figure>`;
}

function stopStream() {
  const img = document.querySelector(".viewfinder img");
  if (img) {
    img.removeAttribute("src");
    img.src = "";
  }
}

function stripHtml(samples) {
  if (!samples.length) return "";
  const required = state.data.required_slots || [];
  return `<div class="card">
    <div class="card-head"><h3>全部样本</h3><span class="muted">${samples.length} 个</span></div>
    <div class="sample-strip">
      ${samples
        .map((s) => {
          const taken = new Set((s.photos || []).map((p) => p.slot));
          const missing = required.some((k) => !taken.has(k));
          return `<button data-sample="${esc(s.id)}" class="${s.id === state.sampleId ? "on" : ""} ${missing ? "partial" : ""}">${esc(s.id)}${missing ? " ·" : " ✓"}</button>`;
        })
        .join("")}
    </div>
  </div>`;
}

/* ============================ 环境视图 ============================ */
function renderEnv() {
  const el = $("#view-env");
  const d = state.data;
  if (!d) return;
  if (!d.session) {
    el.innerHTML = noSessionHtml();
    return;
  }
  const rooms = (d.coverage && d.coverage.rooms) || [];
  if (!rooms.length) {
    el.innerHTML = `<div class="card center">
      <h2>还没有菇房</h2>
      <p class="muted">在“设置”里每行填一间：<code>A,12,3</code>（名称, 排数, 层数）。</p>
      <button data-tabjump="settings">去设置</button>
    </div>`;
    return;
  }

  const room = rooms.find((r) => r.name === state.env.room) || rooms[0];
  state.env.room = room.name;
  const scope = envViewDef(state.env.view)?.scope || "room";

  el.innerHTML = `
  <div class="card">
    <div class="card-head"><h2>环境扫描</h2><span class="muted">先全景，再逐排、逐层</span></div>
    <label class="field-label">菇房</label>
    <div class="chiprow">
      ${rooms.map((r) => `<button data-room="${esc(r.name)}" class="${r.name === room.name ? "on" : ""}">${esc(r.name)}</button>`).join("")}
    </div>
    ${
      scope !== "room"
        ? `<label class="field-label" style="margin-top:12px">棚架（排）</label>
           ${
             room.rows
               ? `<div class="chiprow">${range(1, room.rows)
                   .map((n) => `<button data-row="${n}" class="${n === state.env.row ? "on" : ""}">${n} 排</button>`)
                   .join("")}</div>`
               : `<input id="env-row-input" type="number" inputmode="numeric" min="1" value="${state.env.row}">`
           }`
        : ""
    }
    ${
      scope === "bed"
        ? `<label class="field-label" style="margin-top:12px">层</label>
           ${
             room.layers
               ? `<div class="chiprow">${range(1, room.layers)
                   .map((n) => `<button data-layer="${n}" class="${n === state.env.layer ? "on" : ""}">${n} 层</button>`)
                   .join("")}</div>`
               : `<input id="env-layer-input" type="number" inputmode="numeric" min="1" value="${state.env.layer}">`
           }`
        : ""
    }
    <label class="field-label" style="margin-top:12px">拍什么</label>
    <div class="chiprow">
      ${(d.env_views || [])
        .map(
          (v) =>
            `<button data-envview="${v.key}" class="${v.key === state.env.view ? "on" : ""}">${esc(v.label)}${v.required ? "" : " ·可选"}</button>`
        )
        .join("")}
    </div>
    <p class="muted" style="margin-top:8px">${esc(envViewDef(state.env.view)?.hint || "")}</p>
  </div>

  <button class="primary big-capture" id="env-shoot">
    📷 拍照保存
    <small>${esc(expectedEnvName())}</small>
  </button>

  ${matrixHtml(room)}
  ${envGalleryHtml(room)}`;
}

function matrixHtml(room) {
  if (!room.rows || !room.layers) return "";
  const done = new Set((room.bed_shots || []).map(([r, l]) => `${r}-${l}`));
  return `<div class="card">
    <div class="card-head"><h3>培养床覆盖</h3><span class="muted">点格子跳到该层拍</span></div>
    <div class="matrix"><table>
      <tr><th></th>${range(1, room.layers).map((l) => `<th>${l} 层</th>`).join("")}</tr>
      ${range(1, room.rows)
        .map(
          (r) =>
            `<tr><th>${r} 排</th>${range(1, room.layers)
              .map((l) => {
                const on = done.has(`${r}-${l}`);
                const sel = state.env.row === r && state.env.layer === l;
                return `<td><button data-cell="${r}:${l}" class="${on ? "done" : ""} ${sel ? "sel" : ""}">${on ? "✓" : "·"}</button></td>`;
              })
              .join("")}</tr>`
        )
        .join("")}
    </table></div>
  </div>`;
}

function envGalleryHtml(room) {
  const shots = (state.data.environment || []).filter((s) => s.room === room.name).slice().reverse();
  if (!shots.length) return `<div class="card"><p class="muted">这间菇房还没有环境照片。</p></div>`;
  return `<div class="card">
    <div class="card-head"><h3>${esc(room.name)} 环境照片</h3><span class="muted">${shots.length} 张</span></div>
    <div class="thumbs">${shots.map(thumbHtml).join("")}</div>
  </div>`;
}

/* ============================ 进度视图 ============================ */
function renderProgress() {
  const el = $("#view-progress");
  const d = state.data;
  if (!d) return;
  if (!d.session) {
    el.innerHTML = noSessionHtml();
    return;
  }
  const s = d.session;
  const cov = d.coverage || { rooms: [], samples: {} };
  const disk = d.disk || {};
  const bag = d.bag || {};
  const cam = d.camera || {};
  const required = d.required_slots || [];

  const missing = (d.samples || [])
    .map((sample) => {
      const taken = new Set((sample.photos || []).map((p) => p.slot));
      const miss = required.filter((k) => !taken.has(k)).map((k) => slotDef(k)?.label || k);
      return miss.length ? { id: sample.id, miss } : null;
    })
    .filter(Boolean);

  el.innerHTML = `
  <div class="stat-grid">
    <div class="stat"><div class="v">${s.sample_count}</div><div class="k">样本总数</div></div>
    <div class="stat"><div class="v">${cov.samples?.complete ?? 0}</div><div class="k">必需照片齐全</div></div>
    <div class="stat"><div class="v">${s.photo_count}</div><div class="k">样本照片</div></div>
    <div class="stat"><div class="v">${s.env_count}</div><div class="k">环境照片</div></div>
  </div>

  ${roomProgressHtml(cov)}

  <div class="card">
    <div class="card-head"><h3>待补拍</h3><span class="muted">${missing.length} 个样本</span></div>
    ${
      missing.length
        ? `<div class="rules">${missing
            .map((m) => `<div class="kv"><span>${esc(m.id)}</span><span>${esc(m.miss.join("、"))}</span></div>`)
            .join("")}</div>`
        : `<p class="muted">所有样本的必需照片都齐了 ✅</p>`
    }
  </div>

  ${bagCardHtml(bag, cam)}

  <div class="card">
    <div class="card-head"><h3>导出</h3><span class="muted">CSV 可直接用 Excel 打开</span></div>
    <div class="grid-3">
      <button data-export="measurements.csv">测量表</button>
      <button data-export="photos.csv">照片清单</button>
      <button data-export="environment.csv">环境清单</button>
    </div>
  </div>

  <div class="card">
    <div class="card-head"><h3>存储</h3></div>
    <div class="kv"><span>数据目录</span><span>${esc(s.dir)}</span></div>
    <div class="kv"><span>本会话占用</span><span>${fmtBytes(s.bytes)}</span></div>
    <div class="kv"><span>磁盘剩余</span><span>${fmtBytes(disk.free)}</span></div>
    <div class="bar ${disk.used_percent > 92 ? "warn" : ""}" style="margin-top:8px"><i style="width:${Math.min(100, disk.used_percent || 0)}%"></i></div>
    <button class="ghost wide" id="copy-path" style="margin-top:10px">复制数据目录路径</button>
  </div>`;
}

function roomProgressHtml(cov) {
  const rooms = cov.rooms || [];
  if (!rooms.length) return "";
  const pct = (a, b) => (b ? Math.round((100 * a) / b) : 0);
  return rooms
    .map((room) => {
      const rowTotal = room.rows || 0;
      const rowDone = (room.row_shots || []).length;
      const bedTotal = (room.rows || 0) * (room.layers || 0);
      const bedDone = (room.bed_shots || []).length;
      return `<div class="card">
      <div class="card-head"><h3>${esc(room.name)} 房</h3><span class="muted">${room.photos} 张</span></div>
      <div class="kv"><span>菇房全景</span><span>${room.overview ? "已拍" : "缺"}</span></div>
      <div class="kv"><span>棚架（排）</span><span>${rowDone}/${rowTotal}</span></div>
      <div class="bar" style="margin:4px 0 10px"><i style="width:${pct(rowDone, rowTotal)}%"></i></div>
      <div class="kv"><span>培养床（排 × 层）</span><span>${bedDone}/${bedTotal}</span></div>
      <div class="bar" style="margin-top:4px"><i style="width:${pct(bedDone, bedTotal)}%"></i></div>
    </div>`;
    })
    .join("");
}

function bagCardHtml(bag, cam) {
  const running = !!bag.running;
  return `<div class="card">
    <div class="card-head">
      <h3>rosbag 全量录制</h3>
      <span class="chip ${running ? "live" : ""}">${running ? "录制中 " + fmtDuration(bag.elapsed) : "待机"}</span>
    </div>
    <p class="muted" style="margin-bottom:8px">录制所有话题（含隐藏话题）：彩色、深度、点云、IMU 全部进包。</p>
    ${
      running
        ? `<div class="kv"><span>包名</span><span>${esc(bag.name)}</span></div>
           <div class="kv"><span>大小</span><span>${fmtBytes(bag.bytes)}</span></div>
           <button class="danger wide" id="bag-stop">■ 停止录制（正常收尾）</button>`
        : `<label class="field-label">包名（留空按日期生成）</label>
           <input id="bag-name" placeholder="grass_mushroom_${todayStamp()}" value="${esc(draft("bag-name", ""))}" ${bag.available ? "" : "disabled"}>
           <button class="primary wide" id="bag-start" style="margin-top:10px" ${bag.available ? "" : "disabled"}>● 开始录制</button>
           ${bag.available ? "" : `<p class="muted" style="margin-top:8px">找不到 ros2 命令：请用 start.sh 启动，或先 source ROS 环境再运行 app.py。</p>`}`
    }
    <p class="muted" style="margin-top:8px">相机前缀 ${esc(cam.prefix || "")} · ${cam.available ? (cam.aligned ? "对齐深度就绪" : "等待对齐深度（原始深度仅供预览）") : "快照不可用"}</p>
    ${bag.last ? `<p class="muted">上次录制 ${esc(bag.last.name)}（返回码 ${bag.last.code}）</p>` : ""}
  </div>`;
}

/* ============================ 设置视图 ============================ */
function roomsText(rooms) {
  return (rooms || []).map((r) => [r.name, r.rows || 0, r.layers || 0].join(",")).join("\n");
}

function renderSettings() {
  const el = $("#view-settings");
  const d = state.data;
  if (!d) return;
  const s = d.session;
  const sessions = d.sessions || [];
  el.innerHTML = `
  <div class="card">
    <div class="card-head"><h2>采集会话</h2></div>
    ${
      s
        ? `<div class="kv"><span>当前会话</span><span>${esc(s.id)}</span></div>
           <div class="kv"><span>采集日期</span><span>${esc(s.date || "")}</span></div>
           <div class="kv"><span>目录</span><span>${esc(s.dir)}</span></div>`
        : `<p class="muted">还没有会话。</p>`
    }
    ${
      sessions.length > 1
        ? `<label class="field-label" style="margin-top:12px">切换到其它会话</label>
           <div class="chiprow">${sessions
             .map((x) => `<button data-open-session="${esc(x.id)}" class="${x.id === s?.id ? "on" : ""}">${esc(x.id)}</button>`)
             .join("")}</div>`
        : ""
    }
  </div>

  <div class="card">
    <div class="card-head"><h3>新建会话</h3><span class="muted">一般一天一个会话</span></div>
    <div class="grid-2">
      <div><label class="field-label">日期</label><input id="ns-date" type="date" value="${esc(draft("ns-date", defaultDate()))}"></div>
      <div><label class="field-label">采集员</label><input id="ns-operator" placeholder="姓名 / 工号" value="${esc(draft("ns-operator", ""))}"></div>
    </div>
    <label class="field-label" style="margin-top:10px">菇房（名称,排数,层数，每行一间）</label>
    <textarea id="ns-rooms" rows="3" placeholder="A,12,3&#10;B,10,3">${esc(draft("ns-rooms", ""))}</textarea>
    <label class="field-label" style="margin-top:10px">备注</label>
    <input id="ns-notes" placeholder="例如：东侧棚架补光" value="${esc(draft("ns-notes", ""))}">
    <button class="primary wide" id="create-session" style="margin-top:12px">创建并开始采集</button>
  </div>

  <div class="card">
    <div class="card-head"><h3>菇房与棚架</h3><span class="muted">用于环境覆盖统计</span></div>
    <textarea id="room-config" rows="4" placeholder="A,12,3">${esc(draft("room-config", roomsText(s?.rooms || [])))}</textarea>
    <button class="blue wide" id="save-rooms" style="margin-top:10px">保存菇房配置</button>
  </div>

  <div class="card">
    <div class="card-head"><h3>命名规则</h3></div>
    <div class="rules">
      <p><b>样本照片</b> <code>s007-inplace-far.jpg</code></p>
      <p>槽位顺序：原位远照 → 原位近照 →（左/右补充）→ 采摘后整簇 → 正面 → 反面 → 底部 →（簇内个体）→ 标尺特写 → 秤读数。</p>
      <p><b>环境照片</b> <code>env-A-overview.jpg</code>、<code>env-A-r02-row.jpg</code>、<code>env-A-r02-l01-bed.jpg</code></p>
      <p><b>D435i</b> <code>s007-d435i-color.png</code>、<code>s007-d435i-depth.png</code>、<code>s007-d435i-points.ply</code>、<code>s007-d435i-meta.json</code></p>
      <p>同一槽位重拍会自动加序号 <code>-2</code>、<code>-3</code>，不会覆盖旧照片。</p>
      <p>触摸屏键盘快捷键：<code>1</code>–<code>0</code> 依次拍第 1–10 个槽位，<code>N</code> 新建样本。</p>
    </div>
  </div>

  <div class="card">
    <div class="card-head"><h3>手机访问地址</h3><span class="muted">手机需与电脑在同一网络</span></div>
    ${
      (d.addresses || []).length
        ? (d.addresses || [])
            .map(
              (a) => `<div class="kv">
            <span>${esc(a.iface)} · ${esc(a.kind)}</span>
            <span><a class="link" href="${esc(a.url)}">${esc(a.url)}</a></span>
          </div>`
            )
            .join("")
        : `<p class="muted">未检测到局域网地址，请先连上 Wi-Fi 或手机热点。</p>`
    }
    <p class="muted" style="margin-top:8px">
      手机开热点时选无线网卡那一条（10.x / 192.168.x）；代理软件的 TUN 地址（198.18.x）
      手机连不上，已自动排除。打不开时先用手机浏览器打开 <code>/api/ping</code> 确认连通。
    </p>
    <button class="ghost wide" id="copy-url" style="margin-top:8px">复制第一个地址</button>
  </div>

  <div class="card">
    <div class="card-head"><h3>连接信息</h3></div>
    <div class="kv"><span>数据根目录</span><span>${esc(d.data_root || "")}</span></div>
    <div class="kv"><span>录包目录</span><span>${esc(d.bag_root || "")}</span></div>
    <div class="kv"><span>相机前缀</span><span>${esc(d.camera?.prefix || "")}</span></div>
    <div class="kv"><span>实时取景</span><span>${d.streams || 0} 路</span></div>
    <div class="kv"><span>缩略图</span><span>${d.pillow ? "Pillow 已启用" : "未装 Pillow（直接用原图）"}</span></div>
  </div>`;
}

/* ============================ 交互 ============================ */
function switchTab(name) {
  if (name !== "samples") stopStream();
  state.view = name;
  $$(".tabbar button").forEach((b) => b.classList.toggle("active", b.dataset.tab === name));
  ["samples", "env", "progress", "settings"].forEach((key) => {
    $(`#view-${key}`).classList.toggle("hidden", key !== name);
  });
  renderAll();
}

function openLightbox(rel) {
  const box = $("#lightbox");
  box.querySelector("img").src = `/media/${rel}`;
  box.classList.remove("hidden");
}

async function stepSample(delta) {
  const samples = state.data?.samples || [];
  const index = samples.findIndex((s) => s.id === state.sampleId);
  const next = samples[index + delta];
  if (next) {
    state.sampleId = next.id;
    renderAll();
  }
}

async function createSample() {
  if (!state.data?.session) return toast("请先新建会话", "error");
  const payload = { room: state.env.room, row: state.env.row, layer: state.env.layer };
  if ((envViewDef(state.env.view)?.scope || "room") === "room") {
    delete payload.row;
    delete payload.layer;
  }
  try {
    const res = await api("/api/sample", { method: "POST", json: payload });
    state.sampleId = res.id;
    await refresh();
    toast(`新建样本 ${res.id}`, "ok");
  } catch (err) {
    toast(err.message, "error");
  }
}

async function removeSample() {
  const sample = currentSample();
  if (!sample) return;
  if (!confirm(`从索引中删除样本 ${sample.id}？\n照片文件仍保留在磁盘上。`)) return;
  try {
    await api(`/api/sample/${encodeURIComponent(sample.id)}`, { method: "DELETE" });
    state.sampleId = null;
    await refresh();
    toast("已删除", "ok");
  } catch (err) {
    toast(err.message, "error");
  }
}

async function patchSample(patch) {
  if (!state.sampleId) return;
  const res = await api(`/api/sample/${encodeURIComponent(state.sampleId)}`, { method: "PATCH", json: patch });
  const list = state.data.samples || [];
  const index = list.findIndex((s) => s.id === res.sample.id);
  if (index >= 0) list[index] = res.sample;
}

function openPicker() {
  const input = $("#file-input");
  input.value = "";
  input.click();
}

function captureSlot(slot) {
  if (!state.sampleId) return toast("请先新建样本", "error");
  state.pending = { target: "sample", sample: state.sampleId, slot };
  openPicker();
}

function captureEnv() {
  const { room, row, layer, view } = state.env;
  if (!room) return toast("请先选择菇房", "error");
  const scope = envViewDef(view)?.scope || "room";
  state.pending = { target: "env", room, view };
  if (scope !== "room") state.pending.row = String(row);
  if (scope === "bed") state.pending.layer = String(layer);
  openPicker();
}

function uploadStart() {
  uploading += 1;
  const chip = $("#chip-upload");
  chip.classList.remove("hidden");
  chip.classList.add("live");
  chip.textContent = "上传 0%";
}

function uploadProgress(fraction) {
  const chip = $("#chip-upload");
  chip.textContent = `上传 ${Math.round(fraction * 100)}%`;
}

function uploadEnd() {
  uploading = Math.max(0, uploading - 1);
  if (!uploading) {
    const chip = $("#chip-upload");
    chip.classList.add("hidden");
    chip.classList.remove("live");
  }
}

function uploadFile(file, url) {
  return new Promise((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", url, true);
    xhr.setRequestHeader("Content-Type", file.type || "application/octet-stream");
    uploadStart();
    xhr.upload.onprogress = (ev) => {
      if (ev.lengthComputable) uploadProgress(ev.loaded / ev.total);
    };
    xhr.onload = () => {
      uploadEnd();
      let data = {};
      try {
        data = JSON.parse(xhr.responseText);
      } catch (_) {
        /* 忽略 */
      }
      if (xhr.status >= 200 && xhr.status < 300) resolve(data);
      else reject(new Error(data.error || `上传失败 HTTP ${xhr.status}`));
    };
    xhr.onerror = () => {
      uploadEnd();
      reject(new Error("上传失败：网络中断"));
    };
    xhr.send(file);
  });
}

async function handleFiles(input) {
  const files = Array.from(input.files || []);
  const pending = state.pending;
  state.pending = null;
  input.value = "";
  if (!files.length || !pending) return;

  for (const file of files) {
    const query = new URLSearchParams();
    Object.entries(pending).forEach(([key, value]) => query.set(key, value));
    query.set("source", "phone");
    try {
      const res = await uploadFile(file, `/api/photo?${query.toString()}`);
      toast(`已保存 ${res.name}`, "ok");
    } catch (err) {
      toast(err.message, "error");
    }
  }
  await refresh();
}

async function deletePhoto(rel) {
  if (!confirm(`删除这张照片？\n${rel}`)) return;
  try {
    await api("/api/photo/delete", { method: "POST", json: { rel } });
    await refresh();
    toast("已删除", "ok");
  } catch (err) {
    toast(err.message, "error");
  }
}

async function snapshot() {
  const sample = currentSample();
  if (!sample) return;
  const button = $("#snapshot");
  if (button) {
    button.disabled = true;
    button.textContent = "抓取中…";
  }
  try {
    const res = await api("/api/snapshot", { method: "POST", json: { sample: sample.id, stride: 2 } });
    toast(`已保存第 ${res.group} 组 RGB-D（${res.files.length} 个文件）`, "ok");
    await refresh();
  } catch (err) {
    toast(err.message, "error");
    renderAll();
  }
}

async function startVideo() {
  const sample = currentSample();
  if (!sample) return;
  const button = $("#video-start");
  if (button) button.disabled = true;
  try {
    await api("/api/video/start", { method: "POST", json: { sample: sample.id } });
    toast("已开始 RGB-D 连续录制", "ok");
  } catch (err) { toast(err.message, "error"); }
  await refresh();
}

async function stopVideo() {
  const button = $("#video-stop");
  if (button) { button.disabled = true; button.textContent = "正在停止并保存…"; }
  try {
    await api("/api/video/stop", { method: "POST", json: {} });
    toast("RGB-D 录制已保存，可选择录制生成播放预览", "ok");
  } catch (err) { toast(err.message, "error"); }
  await refresh();
}

async function previewVideo() {
  const name = $("#video-recording")?.value;
  if (!name) return;
  const button = $("#video-preview");
  if (button) button.disabled = true;
  try {
    const result = await api("/api/video/preview", { method: "POST", json: { name } });
    toast(result.ready ? "预览已就绪" : "正在生成播放预览，原始录制保持不变", "ok");
  } catch (err) { toast(err.message, "error"); }
  await refresh();
  renderAll();
}

async function startBag() {
  const name = ($("#bag-name")?.value || "").trim();
  try {
    await api("/api/bag/start", { method: "POST", json: { name } });
    toast("开始录制 rosbag", "ok");
    await refresh();
  } catch (err) {
    toast(err.message, "error");
  }
}

async function stopBag() {
  try {
    await api("/api/bag/stop", { method: "POST", json: {} });
    toast("已停止录制，正在收尾", "ok");
    await refresh();
  } catch (err) {
    toast(err.message, "error");
  }
}

async function createSession() {
  const payload = {
    date: ($("#ns-date")?.value || "").replace(/-/g, ""),
    operator: $("#ns-operator")?.value || "",
    rooms: $("#ns-rooms")?.value || "",
    notes: $("#ns-notes")?.value || "",
  };
  try {
    const res = await api("/api/session", { method: "POST", json: payload });
    ["ns-date", "ns-operator", "ns-rooms", "ns-notes"].forEach(dropDraft);
    state.sampleId = null;
    await refresh();
    toast(`会话 ${res.session.id} 已建立`, "ok");
    switchTab("samples");
  } catch (err) {
    toast(err.message, "error");
  }
}

async function openSession(id) {
  try {
    await api("/api/session/open", { method: "POST", json: { id } });
    state.sampleId = null;
    await refresh();
    toast(`已切换到 ${id}`, "ok");
  } catch (err) {
    toast(err.message, "error");
  }
}

async function saveRooms() {
  const rooms = $("#room-config")?.value || "";
  try {
    await api("/api/session/config", { method: "POST", json: { rooms } });
    dropDraft("room-config");
    await refresh();
    toast("菇房配置已保存", "ok");
  } catch (err) {
    toast(err.message, "error");
  }
}

async function copyText(text, label) {
  if (!text) return;
  try {
    await navigator.clipboard.writeText(text);
    toast(`${label || "内容"}已复制`, "ok");
  } catch (_) {
    toast(text);
  }
}

async function copyPath() {
  await copyText(state.data?.session?.dir || "", "路径");
}

async function copyFirstUrl() {
  const url = (state.data?.addresses || [])[0]?.url || state.data?.urls?.[0] || "";
  await copyText(url, "地址");
}

/* ============================ 事件绑定 ============================ */
document.addEventListener("click", async (ev) => {
  const target = ev.target.closest("[data-preview], button");
  if (!target) return;

  if (target.dataset.preview) {
    openLightbox(target.dataset.preview);
    return;
  }

  const data = target.dataset;
  if (data.tab) return switchTab(data.tab);
  if (data.tabjump) return switchTab(data.tabjump);
  if (data.stream) {
    state.streamMode = data.stream;
    return renderAll();
  }
  if (data.nav) return stepSample(Number(data.nav));
  if (target.id === "new-sample") return createSample();
  if (target.id === "delete-sample") return removeSample();
  if (data.sample) {
    state.sampleId = data.sample;
    return renderAll();
  }
  if (data.shoot) return captureSlot(data.shoot);
  if (data.del) return deletePhoto(data.del);
  if (data.mode) {
    try {
      await patchSample({ mode: data.mode });
      await refresh();
    } catch (err) {
      toast(err.message, "error");
    }
    return;
  }
  if (data.room) {
    state.env.room = data.room;
    state.env.row = 1;
    state.env.layer = 1;
    return renderAll();
  }
  if (data.row) {
    state.env.row = Number(data.row);
    return renderAll();
  }
  if (data.layer) {
    state.env.layer = Number(data.layer);
    return renderAll();
  }
  if (data.cell) {
    const [row, layer] = data.cell.split(":").map(Number);
    state.env.row = row;
    state.env.layer = layer;
    state.env.view = "bed";
    return renderAll();
  }
  if (data.envview) {
    state.env.view = data.envview;
    return renderAll();
  }
  if (target.id === "env-shoot") return captureEnv();
  if (target.id === "snapshot") return snapshot();
  if (target.id === "video-start") return startVideo();
  if (target.id === "video-stop") return stopVideo();
  if (target.id === "video-preview") return previewVideo();
  if (target.id === "bag-start") return startBag();
  if (target.id === "bag-stop") return stopBag();
  if (target.id === "create-session") return createSession();
  if (target.id === "save-rooms") return saveRooms();
  if (data.openSession) return openSession(data.openSession);
  if (data.export) {
    window.location.href = `/export/${data.export}`;
    return;
  }
  if (target.id === "copy-path") return copyPath();
  if (target.id === "copy-url") return copyFirstUrl();
});

document.addEventListener("input", (ev) => {
  const id = ev.target.id;
  if (DRAFT_IDS.has(id)) state.drafts[draftKey(id)] = ev.target.value;
});

document.addEventListener("change", async (ev) => {
  if (ev.target.id === "video-recording") {
    state.drafts["video-recording"] = ev.target.value;
    return renderAll();
  }
  const el = ev.target;
  if (!el.id) return;
  if (el.id === "file-input") return handleFiles(el);

  const patches = {
    "m-room": () => ({ room: el.value }),
    "m-row": () => ({ row: el.value }),
    "m-layer": () => ({ layer: el.value }),
    "m-cluster": () => ({ cluster_count: el.value }),
    "m-height": () => ({ height_mm: el.value }),
    "m-width": () => ({ width_mm: el.value }),
    "m-weight": () => ({ weight_g: el.value }),
    "m-notes": () => ({ notes: el.value }),
  };
  if (patches[el.id]) {
    try {
      await patchSample(patches[el.id]());
      dropDraft(el.id);
    } catch (err) {
      toast(err.message, "error");
    }
    return;
  }
  if (el.id === "env-row-input") {
    state.env.row = Math.max(1, Number(el.value) || 1);
    return renderAll();
  }
  if (el.id === "env-layer-input") {
    state.env.layer = Math.max(1, Number(el.value) || 1);
    return renderAll();
  }
});

$("#lightbox").addEventListener("click", () => {
  $("#lightbox").classList.add("hidden");
  $("#lightbox img").src = "";
});

document.addEventListener("keydown", (ev) => {
  if (ev.metaKey || ev.ctrlKey || ev.altKey) return;
  const el = ev.target;
  if (el && el.matches && el.matches("input, textarea, select")) return;
  if (state.view !== "samples") return;

  const digits = "1234567890";
  const index = digits.indexOf(ev.key);
  if (index >= 0) {
    const slot = (state.data?.slots || [])[index];
    if (slot) {
      ev.preventDefault();
      captureSlot(slot.key);
    }
    return;
  }
  if (ev.key.toLowerCase() === "n") {
    ev.preventDefault();
    createSample();
  }
});

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "hidden") stopStream();
  else renderAll();
});

/* ============================ 启动 ============================ */
if ("serviceWorker" in navigator) {
  navigator.serviceWorker.register("/sw.js").catch(() => {
    /* 局域网 HTTP 下部分浏览器不注册 SW，忽略即可 */
  });
}

refresh();
setInterval(refresh, 6000);
