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
  .rows { display: grid; grid-template-columns: 1fr 1fr; gap: 12px 20px; }
  .rows div {
    display: flex; justify-content: space-between; align-items: baseline;
    border-bottom: 1px solid var(--card-border); padding-bottom: 6px;
  }
  .rows div span:first-child { color: var(--text-secondary); font-size: .85rem; }
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
  .coords { display: flex; gap: 24px; margin-top: 18px; }
  .coord { flex: 1; text-align: center; }
  .coord .label { font-size: .75rem; color: var(--text-secondary); text-transform: uppercase; letter-spacing: .05em; }
  .coord .value { font-size: 1.6rem; font-weight: 600; font-variant-numeric: tabular-nums; }

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
    <h2>IMU</h2>
    <div class="rows">
      <div><span>Accel X</span><span class="val" id="ax">--</span></div>
      <div><span>Accel Y</span><span class="val" id="ay">--</span></div>
      <div><span>Accel Z</span><span class="val" id="az">--</span></div>
      <div><span>Gyro X</span><span class="val" id="gx">--</span></div>
      <div><span>Gyro Y</span><span class="val" id="gy">--</span></div>
      <div><span>Gyro Z</span><span class="val" id="gz">--</span></div>
      <div><span>Temp</span><span class="val" id="temp">--</span></div>
    </div>
  </div>

  <div class="card">
    <h2>Person Tracking</h2>
    <span class="status-pill" id="camStatus"><span class="status-dot"></span><span id="camStatusText">Not Tracking</span></span>
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

// ---- IMU module ----
function fmt(n) { return (typeof n === 'number') ? n.toFixed(2) : '--'; }
function renderIMU(imu) {
  $('ax').textContent = fmt(imu.ax); $('ay').textContent = fmt(imu.ay); $('az').textContent = fmt(imu.az);
  $('gx').textContent = fmt(imu.gx); $('gy').textContent = fmt(imu.gy); $('gz').textContent = fmt(imu.gz);
  $('temp').textContent = fmt(imu.temp) + ' C';
}

// ---- Person tracking module ----
function renderCam(cam) {
  $('camStatus').classList.toggle('tracking', !!cam.valid);
  $('camStatusText').textContent = cam.valid ? 'Tracking' : 'Not Tracking';
  $('camX').textContent = cam.valid ? cam.x : '--';
  $('camY').textContent = cam.valid ? cam.y : '--';
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
  renderIMU(d.imu);
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
