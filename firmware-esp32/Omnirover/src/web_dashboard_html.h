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
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 16px;
    background: #12161c; color: #e6e9ef;
    font-family: -apple-system, Segoe UI, Roboto, Arial, sans-serif;
  }
  header {
    display: flex; align-items: center; gap: 10px;
    margin-bottom: 16px; flex-wrap: wrap;
  }
  h1 { font-size: 1.3rem; margin: 0; flex: 1; }
  .dot {
    width: 12px; height: 12px; border-radius: 50%;
    background: #e5484d; box-shadow: 0 0 6px #e5484d;
    transition: background .2s, box-shadow .2s;
  }
  .dot.online { background: #2fbf71; box-shadow: 0 0 6px #2fbf71; }
  #uptime { color: #9aa4b2; font-size: .85rem; }
  .grid {
    display: grid; gap: 14px;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  }
  .card {
    background: #1b212b; border: 1px solid #2a3240;
    border-radius: 10px; padding: 14px 16px;
  }
  .card h2 {
    font-size: .95rem; margin: 0 0 10px; color: #9aa4b2;
    text-transform: uppercase; letter-spacing: .04em;
  }
  .rows { display: grid; grid-template-columns: 1fr 1fr; gap: 8px 16px; }
  .rows div span { color: #9aa4b2; margin-right: 6px; }
  .val { font-variant-numeric: tabular-nums; }
  canvas { background: #0d1117; border-radius: 8px; display: block; }
  .enc-row {
    display: grid; grid-template-columns: 70px 1fr 70px;
    align-items: center; gap: 10px; margin-bottom: 8px;
  }
  .bar { height: 10px; background: #0d1117; border-radius: 6px; overflow: hidden; }
  .bar > i { display: block; height: 100%; background: #4f8cff; width: 50%; }
</style>
</head>
<body>
<header>
  <div class="dot" id="statusDot"></div>
  <h1>Omnirover Dashboard</h1>
  <div id="uptime">uptime: --</div>
</header>

<div class="grid">
  <div class="card">
    <h2>IMU (BNO / MPU)</h2>
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
    <h2>Camera (HuskyLens)</h2>
    <canvas id="camCanvas" width="280" height="210"></canvas>
    <div class="rows" style="margin-top:10px">
      <div><span>ID</span><span class="val" id="camId">--</span></div>
      <div><span>Detected</span><span class="val" id="camValid">--</span></div>
      <div><span>X</span><span class="val" id="camX">--</span></div>
      <div><span>Y</span><span class="val" id="camY">--</span></div>
    </div>
  </div>

  <div class="card" style="grid-column: 1 / -1;">
    <h2>Encoders</h2>
    <div id="encoders"></div>
  </div>
</div>

<script>
const $ = (id) => document.getElementById(id);
const encRows = [];
for (let i = 0; i < 4; i++) {
  const row = document.createElement('div');
  row.className = 'enc-row';
  row.innerHTML = `<span>M${i}</span><div class="bar"><i id="encBar${i}"></i></div><span class="val" id="encVal${i}">--</span>`;
  $('encoders').appendChild(row);
}

const ctx = $('camCanvas').getContext('2d');
function drawCam(cam) {
  ctx.clearRect(0, 0, 280, 210);
  ctx.strokeStyle = '#2a3240';
  ctx.strokeRect(0, 0, 280, 210);
  if (!cam || !cam.valid) return;
  const sx = 280 / 320, sy = 210 / 240; // HuskyLens frame is 320x240
  const x = cam.x * sx, y = cam.y * sy;
  const w = cam.w * sx, h = cam.h * sy;
  ctx.strokeStyle = '#4f8cff';
  ctx.lineWidth = 2;
  ctx.strokeRect(x - w / 2, y - h / 2, w, h);
  ctx.fillStyle = '#4f8cff';
  ctx.beginPath();
  ctx.arc(x, y, 3, 0, Math.PI * 2);
  ctx.fill();
}

function fmt(n) { return (typeof n === 'number') ? n.toFixed(2) : '--'; }

function applyTelemetry(d) {
  $('uptime').textContent = 'uptime: ' + (d.t / 1000).toFixed(1) + ' s';

  $('ax').textContent = fmt(d.imu.ax); $('ay').textContent = fmt(d.imu.ay); $('az').textContent = fmt(d.imu.az);
  $('gx').textContent = fmt(d.imu.gx); $('gy').textContent = fmt(d.imu.gy); $('gz').textContent = fmt(d.imu.gz);
  $('temp').textContent = fmt(d.imu.temp) + ' C';

  $('camId').textContent = d.cam.id;
  $('camValid').textContent = d.cam.valid ? 'yes' : 'no';
  $('camX').textContent = d.cam.x;
  $('camY').textContent = d.cam.y;
  drawCam(d.cam);

  d.enc.forEach((e, i) => {
    $('encVal' + i).textContent = e.vel.toFixed(0) + ' rpm';
    const pct = Math.max(0, Math.min(100, 50 + e.vel / 2));
    $('encBar' + i).style.width = pct + '%';
  });
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
