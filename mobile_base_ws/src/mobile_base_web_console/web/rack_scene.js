"use strict";

// These shelves come from rack_map geometry, not from Isaac ground-truth IDs.
function drawSemanticRacks(ctx, project, racks, zoom, topDown = false) {
  const faces = [];
  const labels = [];
  const posts = [];
  let visibleLayers = 0;
  for (const rack of racks) {
    const layers = rack.layers || [];
    const valid = layers.filter(layer => Array.isArray(layer.boundary) &&
      layer.boundary.length === 4 && layer.boundary.every(point =>
        Array.isArray(point) && point.length === 3 && point.every(Number.isFinite)));
    if (!valid.length) continue;
    visibleLayers += valid.length;
    const base = valid[0].boundary;
    const top = Math.max(...valid.map(layer => layer.center[2])) + 0.12;
    // Posts are structural estimates: a bed polygon does not directly measure them.
    for (const corner of topDown ? [] : base) {
      posts.push({bottom: project(corner[0], corner[1], 0.05),
        top: project(corner[0], corner[1], top)});
    }
    for (const layer of topDown ? [valid.reduce((a, b) => a.center[2] > b.center[2] ? a : b)] : valid) {
      const boundary = layer.boundary;
      const topPoints = boundary.map(point => project(...point));
      const retained = !layer.observed;
      const hue = retained ? "#6f8f9b" : "#68d49a";
      const surface = retained ? "#43606ba8" : "#398f69d9";
      const front = layer.front_center || layer.center;
      let frontEdge = 0, best = Infinity;
      for (let i = 0; i < 4; i++) {
        const a = boundary[i], b = boundary[(i + 1) % 4];
        const distance = Math.hypot((a[0] + b[0]) / 2 - front[0],
          (a[1] + b[1]) / 2 - front[1]);
        if (distance < best) { best = distance; frontEdge = i; }
      }
      const a = boundary[frontEdge], b = boundary[(frontEdge + 1) % 4];
      const frontPoints = [project(...a), project(...b),
        project(b[0], b[1], b[2] - 0.07), project(a[0], a[1], a[2] - 0.07)];
      if (!topDown) faces.push({points: frontPoints, depth: frontPoints.reduce((s, p) => s + p.d, 0) / 4,
        fill: retained ? "#344f58bf" : "#28724fe6", stroke: hue, retained});
      faces.push({points: topPoints, depth: topPoints.reduce((s, p) => s + p.d, 0) / 4,
        fill: surface, stroke: hue, retained});
      const labelPosition = project(...layer.center);
      if (!topDown && zoom > 28) labels.push({position: labelPosition, text: zoom > 47 ?
        `${layer.level_number}层 · ${layer.stable_id}` : `${layer.level_number}层`, retained});
    }
    const labelPoint = project(rack.position[0], rack.position[1], top + 0.24);
    labels.push({position: labelPoint, text: rack.stable_id, rack: true});
  }
  ctx.save();
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 3;
  ctx.strokeStyle = "#b8cac5bb";
  for (const post of posts) {
    ctx.beginPath(); ctx.moveTo(post.bottom.x, post.bottom.y);
    ctx.lineTo(post.top.x, post.top.y); ctx.stroke();
  }
  ctx.restore();
  faces.sort((a, b) => b.depth - a.depth);
  for (const face of faces) {
    ctx.save(); ctx.beginPath();
    face.points.forEach((point, index) => index ? ctx.lineTo(point.x, point.y) :
      ctx.moveTo(point.x, point.y));
    ctx.closePath(); ctx.fillStyle = face.fill; ctx.fill();
    ctx.strokeStyle = face.stroke; ctx.lineWidth = face.retained ? 1.4 : 2;
    if (face.retained) ctx.setLineDash([5, 4]);
    ctx.stroke(); ctx.restore();
  }
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (const label of labels) {
    const {x, y} = label.position;
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    ctx.font = label.rack ? "bold 11px system-ui" : "10px system-ui";
    const width = ctx.measureText(label.text).width + 10;
    ctx.fillStyle = label.retained ? "#23343ad9" : "#0b2119e8";
    ctx.fillRect(x - width / 2, y - 10, width, 19);
    ctx.strokeStyle = label.retained ? "#96b4bd" : "#8ff0b7";
    ctx.strokeRect(x - width / 2, y - 10, width, 19);
    ctx.fillStyle = "#e7fff1"; ctx.fillText(label.text, x, y);
  }
  return visibleLayers;
}
