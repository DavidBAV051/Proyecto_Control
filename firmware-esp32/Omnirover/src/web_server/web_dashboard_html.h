#pragma once
#include <Arduino.h>

// Single-page real-time dashboard. Pure HTML/CSS/JS, no external CDN
// dependencies (the rover's WiFi AP has no internet access).
static const char DASHBOARD_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Omnirover Dashboard</title>
<style>
  :root {
    color-scheme: light;
    --bg: #f5f5f7;
    --card-bg: #ffffff;
    --card-border: rgba(0, 0, 0, .06);
    --text: #1d1d1f;
    --text-secondary: #6e6e73;
    --accent: #0071e3;
    --accent-soft: rgba(0, 113, 227, .1);
    --green: #34c759;
    --green-soft: rgba(52, 199, 89, .12);
    --red: #ff3b30;
    --radius: 18px;
    --shadow: 0 1px 2px rgba(0, 0, 0, .04), 0 12px 28px rgba(0, 0, 0, .06);
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "SF Pro Text", "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
  }
  header {
    position: sticky; top: 0; z-index: 10;
    display: flex; align-items: center; gap: 12px;
    padding: 16px 24px;
    background: rgba(245, 245, 247, .78);
    backdrop-filter: saturate(180%) blur(20px);
    -webkit-backdrop-filter: saturate(180%) blur(20px);
    border-bottom: 1px solid var(--card-border);
  }
  .dot {
    width: 10px; height: 10px; border-radius: 50%;
    background: var(--red); box-shadow: 0 0 0 4px rgba(255, 59, 48, .15);
    transition: background .2s, box-shadow .2s;
    flex-shrink: 0;
  }
  .dot.online { background: var(--green); box-shadow: 0 0 0 4px var(--green-soft); }
  header h1 { font-size: 1.15rem; font-weight: 600; margin: 0; letter-spacing: -.01em; }
  header .subtitle { font-size: .8rem; color: var(--text-secondary); }
  header .titles { display: flex; flex-direction: column; flex: 1; }
  #uptime {
    color: var(--text-secondary); font-size: .8rem;
    font-variant-numeric: tabular-nums;
  }
  main { max-width: 1080px; margin: 0 auto; padding: 24px; }
  .grid {
    display: grid; gap: 20px;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  }
  .card {
    background: var(--card-bg); border: 1px solid var(--card-border);
    border-radius: var(--radius); padding: 18px 20px;
    box-shadow: var(--shadow);
  }
  .card h2 {
    font-size: .78rem; margin: 0 0 14px; color: var(--text-secondary);
    text-transform: uppercase; letter-spacing: .06em; font-weight: 600;
  }
  .val { font-variant-numeric: tabular-nums; font-weight: 500; }

  /* Person tracking card */
  .status-pill {
    display: inline-flex; align-items: center; gap: 6px;
    padding: 5px 12px; border-radius: 999px;
    font-size: .85rem; font-weight: 600;
    background: #f2f2f4; color: var(--text-secondary);
    transition: background .2s, color .2s;
  }
  .status-pill.tracking { background: var(--green-soft); color: var(--green); }
  .status-pill .status-dot { width: 7px; height: 7px; border-radius: 50%; background: currentColor; }
  .coords { display: flex; gap: 24px; margin-top: 14px; }
  .coord { flex: 1; text-align: center; }
  .coord .label { font-size: .75rem; color: var(--text-secondary); text-transform: uppercase; letter-spacing: .05em; }
  .coord .value { font-size: 1.6rem; font-weight: 600; font-variant-numeric: tabular-nums; }

  /* Pose / tracking canvases */
  canvas#poseCanvas, canvas#camCanvas {
    width: 100%; height: auto; display: block;
    border-radius: 12px; background: #f2f2f4; margin-top: 10px;
  }
  .pose-readout {
    display: flex; justify-content: space-between; align-items: baseline;
    margin-top: 12px; padding-top: 10px; border-top: 1px solid var(--card-border);
  }
  .pose-readout span:first-child { color: var(--text-secondary); font-size: .85rem; }

  /* Encoders card */
  .enc-row {
    display: grid; grid-template-columns: 44px 1fr 70px;
    align-items: center; gap: 12px; margin-bottom: 10px;
  }
  .enc-row:last-child { margin-bottom: 0; }
  .enc-row span:first-child { font-weight: 600; font-size: .85rem; }
  .bar { height: 8px; background: #eef0f2; border-radius: 6px; overflow: hidden; }
  .bar > i { display: block; height: 100%; background: var(--accent); width: 50%; transition: width .2s; }

  /* Rover photo card */
  .robot-card { display: flex; flex-direction: column; }
  .robot-photo {
    position: relative; width: 100%; aspect-ratio: 4 / 3;
    background: #f2f2f4; border-radius: 12px; overflow: hidden;
    display: flex; align-items: center; justify-content: center;
    margin-bottom: 14px;
  }
  .robot-photo img { width: 100%; height: 100%; object-fit: cover; display: none; }
  .robot-placeholder { color: var(--text-secondary); font-size: .85rem; }
  .btn {
    display: inline-block; text-align: center; cursor: pointer;
    background: var(--accent-soft); color: var(--accent);
    font-weight: 600; font-size: .85rem;
    padding: 9px 14px; border-radius: 10px;
    transition: background .15s;
  }
  .btn:hover { background: rgba(0, 113, 227, .18); }
</style>
</head>
<body>
<header>
  <div class="dot" id="statusDot"></div>
  <div class="titles">
    <h1>Omnirover</h1>
    <span class="subtitle">Live Dashboard</span>
  </div>
  <div id="uptime">uptime --</div>
</header>

<main>
<div class="grid">
  <div class="card robot-card">
    <h2>Rover</h2>
    <div class="robot-photo">
      <img id="robotImg" alt="Rover photo">
      <span class="robot-placeholder" id="robotPlaceholder">No photo yet</span>
    </div>
    <label class="btn" for="robotFile" id="robotUploadLabel">Change Photo</label>
    <input type="file" id="robotFile" accept="image/*" hidden>
  </div>

  <div class="card">
    <h2>Rover Orientation &amp; Path</h2>
    <canvas id="poseCanvas" width="260" height="220"></canvas>
    <div class="pose-readout"><span>Yaw</span><span class="val" id="yawVal">--</span></div>
  </div>

  <div class="card">
    <h2>Person Tracking</h2>
    <span class="status-pill" id="camStatus"><span class="status-dot"></span><span id="camStatusText">Not Tracking</span></span>
    <canvas id="camCanvas" width="260" height="195"></canvas>
    <div class="coords">
      <div class="coord"><div class="label">X</div><div class="value" id="camX">--</div></div>
      <div class="coord"><div class="label">Y</div><div class="value" id="camY">--</div></div>
    </div>
  </div>

  <div class="card" style="grid-column: 1 / -1;">
    <h2>Encoders</h2>
    <div id="encoders"></div>
  </div>
</div>
</main>

<script>
const $ = (id) => document.getElementById(id);

// ---- Encoders module ----
for (let i = 0; i < 4; i++) {
  const row = document.createElement('div');
  row.className = 'enc-row';
  row.innerHTML = `<span>M${i}</span><div class="bar"><i id="encBar${i}"></i></div><span class="val" id="encVal${i}">--</span>`;
  $('encoders').appendChild(row);
}
function renderEncoders(enc) {
  enc.forEach((e, i) => {
    $('encVal' + i).textContent = e.vel.toFixed(0) + ' rpm';
    const pct = Math.max(0, Math.min(100, 50 + e.vel / 2));
    $('encBar' + i).style.width = pct + '%';
  });
}

// ---- Rover pose module (orientation + dead-reckoned path) ----
const poseCanvas = $('poseCanvas');
const poseCtx = poseCanvas.getContext('2d');
let poseTrail = [];
let poseWorldX = 0, poseWorldY = 0;
let poseLastT = null;

// Dead-reckons an approximate path from mean wheel speed (enc[].vel, rpm) and
// IMU yaw. Units are arbitrary (not real-world distance) since wheel radius
// isn't modeled here; good enough to visualize direction/shape of movement.
function stepPoseTrail(imu, enc) {
  const now = performance.now();
  if (poseLastT === null) { poseLastT = now; return; }
  const dt = Math.min(0.5, (now - poseLastT) / 1000);
  poseLastT = now;
  const avgVel = enc.reduce((sum, e) => sum + e.vel, 0) / (enc.length || 1);
  const speed = avgVel * 0.02;
  const yawRad = (imu.yaw || 0) * Math.PI / 180;
  poseWorldX += speed * Math.cos(yawRad) * dt;
  poseWorldY += speed * Math.sin(yawRad) * dt;
  poseTrail.push({ x: poseWorldX, y: poseWorldY });
  if (poseTrail.length > 400) poseTrail.shift();
}

function drawPose(imu) {
  const w = poseCanvas.width, h = poseCanvas.height;
  const cx = w / 2, cy = h / 2;
  poseCtx.clearRect(0, 0, w, h);
  poseCtx.fillStyle = '#f2f2f4';
  poseCtx.fillRect(0, 0, w, h);

  const pts = poseTrail.length ? poseTrail : [{ x: poseWorldX, y: poseWorldY }];
  const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const span = Math.max(maxX - minX, maxY - minY, 2);
  const scale = (Math.min(w, h) - 50) / span;
  const midX = (minX + maxX) / 2, midY = (minY + maxY) / 2;
  const toCanvas = (x, y) => [cx + (x - midX) * scale, cy - (y - midY) * scale];

  if (pts.length > 1) {
    poseCtx.lineWidth = 2;
    for (let i = 1; i < pts.length; i++) {
      const [x0, y0] = toCanvas(pts[i - 1].x, pts[i - 1].y);
      const [x1, y1] = toCanvas(pts[i].x, pts[i].y);
      poseCtx.strokeStyle = `rgba(0, 113, 227, ${0.15 + 0.6 * (i / pts.length)})`;
      poseCtx.beginPath();
      poseCtx.moveTo(x0, y0);
      poseCtx.lineTo(x1, y1);
      poseCtx.stroke();
    }
  }

  const [rx, ry] = toCanvas(poseWorldX, poseWorldY);
  const yawRad = (imu.yaw || 0) * Math.PI / 180;
  poseCtx.save();
  poseCtx.translate(rx, ry);
  poseCtx.rotate(-yawRad);
  const s = 16;
  poseCtx.fillStyle = '#0071e3';
  poseCtx.fillRect(-s / 2, -s / 2, s, s);
  poseCtx.fillStyle = '#ff3b30';
  poseCtx.beginPath();
  poseCtx.moveTo(s / 2 + 6, 0);
  poseCtx.lineTo(s / 2 - 4, -6);
  poseCtx.lineTo(s / 2 - 4, 6);
  poseCtx.closePath();
  poseCtx.fill();
  poseCtx.restore();
}

function renderPose(imu, enc) {
  $('yawVal').textContent = fmt(imu.yaw) + ' deg';
  stepPoseTrail(imu, enc);
  drawPose(imu);
}
function fmt(n) { return (typeof n === 'number') ? n.toFixed(2) : '--'; }

// ---- Person tracking module ----
const camCanvas = $('camCanvas');
const camCtx = camCanvas.getContext('2d');
const CAM_FRAME_W = 320, CAM_FRAME_H = 240; // default HuskyLens frame resolution

function drawCamBox(cam) {
  const w = camCanvas.width, h = camCanvas.height;
  camCtx.clearRect(0, 0, w, h);
  camCtx.fillStyle = '#f2f2f4';
  camCtx.fillRect(0, 0, w, h);
  camCtx.strokeStyle = 'rgba(0, 0, 0, .08)';
  camCtx.strokeRect(0.5, 0.5, w - 1, h - 1);
  if (!cam.valid) return;
  const sx = w / CAM_FRAME_W, sy = h / CAM_FRAME_H;
  const bw = Math.max(cam.w, 10) * sx, bh = Math.max(cam.h, 10) * sy;
  const bx = cam.x * sx - bw / 2, by = cam.y * sy - bh / 2;
  camCtx.fillStyle = 'rgba(52, 199, 89, .15)';
  camCtx.fillRect(bx, by, bw, bh);
  camCtx.strokeStyle = '#34c759';
  camCtx.lineWidth = 2;
  camCtx.strokeRect(bx, by, bw, bh);
}

function renderCam(cam) {
  $('camStatus').classList.toggle('tracking', !!cam.valid);
  $('camStatusText').textContent = cam.valid ? 'Tracking' : 'Not Tracking';
  $('camX').textContent = cam.valid ? cam.x : '--';
  $('camY').textContent = cam.valid ? cam.y : '--';
  drawCamBox(cam);
}

// ---- Rover photo module ----
const robotImg = $('robotImg');
const robotPlaceholder = $('robotPlaceholder');
function refreshRobotPhoto() { robotImg.src = '/robot.jpg?t=' + Date.now(); }
robotImg.onload = () => { robotImg.style.display = 'block'; robotPlaceholder.style.display = 'none'; };
robotImg.onerror = () => { robotImg.style.display = 'none'; robotPlaceholder.style.display = 'block'; };
refreshRobotPhoto();

$('robotFile').addEventListener('change', async (evt) => {
  const file = evt.target.files[0];
  evt.target.value = '';
  if (!file) return;
  const label = $('robotUploadLabel');
  const original = label.textContent;
  label.textContent = 'Uploading…';
  try {
    const form = new FormData();
    form.append('file', file, file.name);
    await fetch('/upload', { method: 'POST', body: form });
    refreshRobotPhoto();
  } catch (e) { /* upload failed, keep previous photo */ }
  label.textContent = original;
});

// ---- Telemetry dispatch ----
function applyTelemetry(d) {
  $('uptime').textContent = 'uptime ' + (d.t / 1000).toFixed(1) + ' s';
  renderPose(d.imu, d.enc);
  renderCam(d.cam);
  renderEncoders(d.enc);
}

let ws;
function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => $('statusDot').classList.add('online');
  ws.onclose = () => { $('statusDot').classList.remove('online'); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => {
    try { applyTelemetry(JSON.parse(evt.data)); } catch (e) { /* ignore malformed frame */ }
  };
}
connect();
</script>
</body>
</html>
)HTMLPAGE";
