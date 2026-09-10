"use strict";

/* ============================================================================
 *  MicroMouse Simulator – UI / rendering / main-worker bridge
 *  All physics & maze logic live in worker.js (off main thread).
 * ========================================================================== */

/* ---------- worker ---------- */
let worker = null;
let workerReady = false;
let lastSnapTime = 0;

function startWorker() {
  try {
    // document.baseURI includes ?vscode-livepreview=true → strip it for worker URL
    const base = document.baseURI.split('?')[0];          // "…/index.html"
    const workerUrl = new URL('./worker.js', base).href;  // "…/worker.js" (no query)
    console.log("[app] creating worker:", workerUrl);
    worker = new Worker(workerUrl);
    worker.onmessage = onWorkerMessage;
    worker.onerror = (e) => {
      console.error("Worker error:", e.message, e.filename, e.lineno);
      showFatal("Simulation worker failed: " + e.message);
    };
  } catch (err) {
    console.error("[app] Worker creation failed:", err);
    showFatal("Web Workers unavailable. Serve this folder via Live Server or any static server (double-click file:// may block workers).");
  }
}

/* ---------- local render state (mirrors worker) ---------- */
const RS = {
  mazeWalls: null,       // Uint8Array
  knownWall: null,       // Uint8Array
  knownBits: null,       // Uint8Array
  stepsArr: null,        // Uint16Array
  goals: [[7,7],[7,8],[8,7],[8,8]],
  mazeW: 16, mazeH: 16,
  trail: [],             // [x,y] pairs
  logs: [],
  path: null,
  segIdx: 0,
  x: 90, y: 90, th: Math.PI/2,
  v: 0, w: 0, curV: 0,
  tSim: 0, dist: 0, batV: 11.1,
  state: "IDLE", goalReached: false, atStart: false,
  tSearch: 0, tFast: 0,
  explored: 0,
  rawF: 0, rawL: 0, rawR: 0,
  crashFlash: 0,
};

/* ---------- DOM refs ---------- */
const mazeCanvas = document.getElementById("mazeCanvas");
const ctx = mazeCanvas.getContext("2d");
const stateBadge = document.getElementById("stateBadge");
const tglKnown = document.getElementById("tglKnown");
const tglActual = document.getElementById("tglActual");
const tglSteps = document.getElementById("tglSteps");
const tglPath = document.getElementById("tglPath");
const editHint = document.getElementById("editHint");
const consoleLog = document.getElementById("consoleLog");
const chartV = document.getElementById("chartV");
const chartW = document.getElementById("chartW");
const cV = chartV.getContext("2d");
const cW = chartW.getContext("2d");

let showKnown = true, showActual = false, showSteps = true, showPath = true;
let editMode = false;

const MAX_HIST = 300;
const vHist = [], wHist = [];

/* ---------- params (local mirror; worker owns truth) ---------- */
let params = null;

function collectParams() {
  return {
    cell: +el("p_cell").value, wallT: +el("p_wallT").value,
    rlen: +el("p_rlen").value, rwid: +el("p_rwid").value,
    track: +el("p_track").value, turnR: +el("p_turnR").value,
    s_vstr: +el("p_s_vstr").value, s_vcur: +el("p_s_vcur").value,
    s_vmax: +el("p_s_vmax").value, s_acc: +el("p_s_acc").value,
    s_dec: +el("p_s_dec").value, s_fbg: +el("p_s_fbg").value,
    s_preferUnknown: +el("p_s_prefUnk").value, s_revisit: +el("p_s_revisit").value,
    f_vmax: +el("p_f_vmax").value, f_vcur: +el("p_f_vcur").value,
    f_acc: +el("p_f_acc").value, f_dec: +el("p_f_dec").value,
    f_turnR: +el("p_f_turnR").value, f_v90: el("p_f_v90").checked,
    c_kp: +el("p_c_kp").value, c_ki: +el("p_c_ki").value, c_kd: +el("p_c_kd").value,
    c_bat: +el("p_c_bat").value, c_batsag: +el("p_c_batsag").value,
    u_sThr: +el("p_u_sThr").value, u_fThr: +el("p_u_fThr").value,
    u_hyst: +el("p_u_hyst").value, u_max: +el("p_u_max").value,
    u_beam: +el("p_u_beam").value, u_noise: +el("p_u_noise").value,
    u_miss: +el("p_u_miss").value, u_glitch: +el("p_u_glitch").value,
    o_drift: +el("p_o_drift").value, o_slip: +el("p_o_slip").value, o_dead: +el("p_o_dead").value,
  };
}

function el(id) { return document.getElementById(id); }

function pushParams() {
  if (worker) worker.postMessage({ t: "params", params: collectParams() });
}

function fillParamInputs() {
  params = collectParams();
}

/* ---------- beep ---------- */
let audioCtx = null;
function playBeep(freq, dur) {
  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    const o = audioCtx.createOscillator(), g = audioCtx.createGain();
    o.type = "sine"; o.frequency.value = freq;
    g.gain.value = 0.08; g.connect(audioCtx.destination); o.connect(g);
    const now = audioCtx.currentTime; o.start(now); o.stop(now + dur);
  } catch (_) {}
}
window.playBeep = playBeep;

/* ---------- worker message handler ---------- */
function onWorkerMessage(e) {
  const m = e.data;
  console.log("[app] worker msg:", m.t, m);
  if (m.t === "error") {
    consoleErr("Worker error: " + m.message);
    console.error(m.stack);
    showFatal("Worker error: " + m.message);
    return;
  }
  if (m.t === "ready") {
    workerReady = true;
    console.log("[app] worker ready, sending initial params");
    pushParams();
    return;
  }
  if (m.t === "backup") { downloadText("maze_backup.json", m.json); return; }
  if (m.t === "beep") { playBeep(m.f, m.d); return; }
  if (m.t !== "snap") return;

  const s = m.s;
  if (m.mazeDirty && m.mazeWalls) {
    RS.mazeWalls = m.mazeWalls;
    RS.mazeW = s.mazeW; RS.mazeH = s.mazeH;
    console.log("[app] received maze walls:", RS.mazeWalls.length, "cells");
  }
  RS.knownWall = m.knownWall;
  RS.knownBits = m.knownBits;
  RS.stepsArr = m.stepsArr;
  if (m.fullReset) { RS.trail = []; RS.logs = []; }
  if (m.newTrail && m.newTrail.length) for (const p of m.newTrail) RS.trail.push(p);
  if (m.newLogs && m.newLogs.length) for (const l of m.newLogs) RS.logs.push(l);

  RS.x = s.x; RS.y = s.y; RS.th = s.th;
  RS.v = s.v; RS.w = s.w; RS.curV = s.curV;
  RS.tSim = s.tSim; RS.dist = s.dist; RS.batV = s.batV;
  RS.state = s.state; RS.goalReached = s.goalReached; RS.atStart = s.atStart;
  RS.tSearch = s.tSearch; RS.tFast = s.tFast;
  RS.explored = s.explored;
  RS.rawF = s.rawF; RS.rawL = s.rawL; RS.rawR = s.rawR;
  RS.crashFlash = s.crashFlash;
  RS.path = s.path; RS.segIdx = s.segIdx;
  RS.goals = s.goals;
  console.log("[app] snapshot:", RS.state, RS.x.toFixed(1), RS.y.toFixed(1));
}

/* ---------- helpers ---------- */
function downloadText(name, text) {
  const blob = new Blob([text], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url; a.download = name; a.click();
  URL.revokeObjectURL(url);
}

window.__fatalErr = "";

window.addEventListener("error", (ev) => {
  const msg = ev.message || "Script error";
  window.__fatalErr += (window.__fatalErr ? "\n" : "") + msg;
  console.error("[global error]", msg);
  if (ev.filename) console.error("  at", ev.filename, ev.lineno + ":" + ev.colno);
});

function showFatal(msg) {
  consoleErr(msg);
  alert(msg);
}

function consoleErr(msg) {
  const d = document.createElement("div");
  d.className = "err";
  d.textContent = "[main] " + msg;
  consoleLog.appendChild(d);
}

/* ---------- maze helpers (from engine.js, loaded via script tag) ---------- */
const DIR_ENUM = window.DIR;
const WBIT_ENUM = window.WBIT;
const MAZE_PRESETS_LOCAL = window.MAZE_PRESETS;
function mazeHasWall(x, y, d) {
  if (!RS.mazeWalls) return true;
  const w = RS.mazeW, h = RS.mazeH;
  if (d === DIR_ENUM.N && y === h-1) return true;
  if (d === DIR_ENUM.S && y === 0) return true;
  if (d === DIR_ENUM.E && x === w-1) return true;
  if (d === DIR_ENUM.W && x === 0) return true;
  if (x < 0 || y < 0 || x >= w || y >= h) return true;
  return (RS.mazeWalls[y*w + x] & WBIT_ENUM[d]) !== 0;
}

/* ---------- render ---------- */
function drawSeg(x1, y1, x2, y2) {
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
}

function render() {
  if (!RS.mazeWalls) return;
  const p = params || collectParams();
  const w = RS.mazeW, h = RS.mazeH;
  const cellPx = 620 / (w + 1.2);
  const ox = (mazeCanvas.width - cellPx * w) / 2;
  const oy = (mazeCanvas.height - cellPx * h) / 2;
  const cell = p.cell;

  ctx.clearRect(0, 0, mazeCanvas.width, mazeCanvas.height);

  // grid
  ctx.strokeStyle = "#1a2333"; ctx.lineWidth = 0.5;
  for (let i = 0; i <= w; i++) { ctx.beginPath(); ctx.moveTo(ox + i*cellPx, oy); ctx.lineTo(ox + i*cellPx, oy + h*cellPx); ctx.stroke(); }
  for (let j = 0; j <= h; j++) { ctx.beginPath(); ctx.moveTo(ox, oy + j*cellPx); ctx.lineTo(ox + w*cellPx, oy + j*cellPx); ctx.stroke(); }

  // actual walls
  if (showActual) {
    ctx.strokeStyle = "rgba(228,85,109,0.25)"; ctx.lineWidth = 3;
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      if (mazeHasWall(x, y, DIR_ENUM.N)) drawSeg(ox + x*cellPx, oy + (y+1)*cellPx, ox + (x+1)*cellPx, oy + (y+1)*cellPx);
      if (mazeHasWall(x, y, DIR_ENUM.S)) drawSeg(ox + x*cellPx, oy + y*cellPx, ox + (x+1)*cellPx, oy + y*cellPx);
      if (mazeHasWall(x, y, DIR_ENUM.E)) drawSeg(ox + (x+1)*cellPx, oy + y*cellPx, ox + (x+1)*cellPx, oy + (y+1)*cellPx);
      if (mazeHasWall(x, y, DIR_ENUM.W)) drawSeg(ox + x*cellPx, oy + y*cellPx, ox + x*cellPx, oy + (y+1)*cellPx);
    }
  }

  // known walls
  if (showKnown && RS.knownWall && RS.knownBits) {
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      const i = y*w + x;
      for (const d of [DIR_ENUM.N, DIR_ENUM.E, DIR_ENUM.S, DIR_ENUM.W]) {
        if (!(RS.knownBits[i] & WBIT_ENUM[d])) continue;
        const has = (RS.knownWall[i] & WBIT_ENUM[d]) !== 0;
        ctx.strokeStyle = has ? "#7fb8f2" : "rgba(127,184,242,0.3)";
        ctx.setLineDash(has ? [] : [4,4]);
        if (d === DIR_ENUM.N) drawSeg(ox + x*cellPx, oy + (y+1)*cellPx, ox + (x+1)*cellPx, oy + (y+1)*cellPx);
        else if (d === DIR_ENUM.S) drawSeg(ox + x*cellPx, oy + y*cellPx, ox + (x+1)*cellPx, oy + y*cellPx);
        else if (d === DIR_ENUM.E) drawSeg(ox + (x+1)*cellPx, oy + y*cellPx, ox + (x+1)*cellPx, oy + (y+1)*cellPx);
        else if (d === DIR_ENUM.W) drawSeg(ox + x*cellPx, oy + y*cellPx, ox + x*cellPx, oy + (y+1)*cellPx);
      }
    }
    ctx.setLineDash([]);
  }

  // flood steps
  if (showSteps && RS.stepsArr && RS.state !== "IDLE") {
    ctx.font = "10px Consolas,monospace"; ctx.fillStyle = "rgba(111,227,161,0.8)";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      const s = RS.stepsArr[y*w + x];
      if (s !== 0xFFFF && s > 0) ctx.fillText(String(s), ox + (x+0.5)*cellPx, oy + (h-y-0.5)*cellPx);
    }
  }

  // path preview
  if (showPath && RS.path && RS.path.length && RS.state !== "IDLE") {
    ctx.strokeStyle = "#6fe3a1"; ctx.lineWidth = 2; ctx.setLineDash([6,4]);
    ctx.beginPath();
    let started = false;
    for (let s = RS.segIdx; s < RS.path.length; s++) {
      const pr = RS.path[s];
      if (pr.type === "line") {
        if (!started) { const fx = pr.from[0]/cell*cellPx + ox, fy = oy + (h - pr.from[1]/cell)*cellPx; ctx.moveTo(fx, fy); started = true; }
        const px = pr.to[0]/cell*cellPx + ox, py = oy + (h - pr.to[1]/cell)*cellPx;
        ctx.lineTo(px, py);
      } else if (pr.type === "arc" && pr.center) {
        const n = 8;
        for (let k = 1; k <= n; k++) {
          const a = pr.a0 + (pr.a1 - pr.a0) * k / n;
          const xx = pr.center[0] + pr.r * Math.cos(a);
          const yy = pr.center[1] + pr.r * Math.sin(a);
          const px = xx/cell*cellPx + ox, py = oy + (h - yy/cell)*cellPx;
          if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
        }
      }
    }
    ctx.stroke(); ctx.setLineDash([]);
  }

  // trail
  if (RS.trail.length > 1) {
    ctx.strokeStyle = "rgba(255,215,0,0.4)"; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i < RS.trail.length; i++) {
      const px = RS.trail[i][0]/cell*cellPx + ox;
      const py = oy + (h - RS.trail[i][1]/cell)*cellPx;
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }

  // goal + start highlights
  for (const g of RS.goals) {
    ctx.fillStyle = "rgba(111,227,161,0.15)";
    ctx.beginPath(); ctx.arc(ox + (g[0]+0.5)*cellPx, oy + (h-g[1]-0.5)*cellPx, cellPx*0.45, 0, Math.PI*2); ctx.fill();
  }
  ctx.fillStyle = "rgba(127,184,242,0.15)";
  ctx.beginPath(); ctx.arc(ox + 0.5*cellPx, oy + (h-0.5)*cellPx, cellPx*0.45, 0, Math.PI*2); ctx.fill();

  // sensor beams
  if (RS.state !== "IDLE") {
    const sensors = [
      { name: "F", offX: 0, offY: (p.rlen/2)*0.55, a: RS.th, val: RS.rawF, thr: p.u_fThr, col: "#ff7f7f" },
      { name: "L", offX: -(p.rwid/2)*0.55, offY: 0, a: RS.th - Math.PI/2, val: RS.rawL, thr: p.u_sThr, col: "#7fff7f" },
      { name: "R", offX: (p.rwid/2)*0.55, offY: 0, a: RS.th + Math.PI/2, val: RS.rawR, thr: p.u_sThr, col: "#7fafff" },
    ];
    for (const s of sensors) {
      const cs = Math.cos(s.a), sn = Math.sin(s.a);
      const ox0 = (RS.x + s.offX*cs - s.offY*sn)/cell*cellPx + ox;
      const oy0 = oy + (h - (RS.y + s.offX*sn + s.offY*cs)/cell)*cellPx;
      const endX = ox0 + cs * (s.val/cell)*cellPx;
      const endY = oy0 - sn * (s.val/cell)*cellPx;
      ctx.strokeStyle = s.col; ctx.lineWidth = 1; ctx.setLineDash([2,2]);
      ctx.beginPath(); ctx.moveTo(ox0, oy0); ctx.lineTo(endX, endY); ctx.stroke(); ctx.setLineDash([]);
      const thX = ox0 + cs * (s.thr/cell)*cellPx, thY = oy0 - sn * (s.thr/cell)*cellPx;
      ctx.beginPath(); ctx.arc(thX, thY, 3, 0, Math.PI*2); ctx.fillStyle = s.col; ctx.fill();
    }
  }

  // robot
  const cosH = Math.cos(RS.th), sinH = Math.sin(RS.th);
  const hw = p.rwid/2 * (cellPx/cell), hl = p.rlen/2 * (cellPx/cell);
  const rx = RS.x/cell*cellPx + ox, ry = oy + (h - RS.y/cell)*cellPx;
  ctx.save(); ctx.translate(rx, ry); ctx.rotate(RS.th);
  ctx.fillStyle = "#ffd700";
  ctx.beginPath(); ctx.moveTo(-hw, -hl); ctx.lineTo(hw, -hl); ctx.lineTo(hw, hl); ctx.lineTo(-hw, hl); ctx.closePath(); ctx.fill();
  ctx.strokeStyle = "#cfa800"; ctx.lineWidth = 2; ctx.stroke();
  ctx.fillStyle = "#e8c400";
  ctx.beginPath(); ctx.moveTo(0, -hl-4); ctx.lineTo(-4, -hl+4); ctx.lineTo(4, -hl+4); ctx.closePath(); ctx.fill();
  ctx.fillStyle = "#fff";
  ctx.beginPath(); ctx.arc(0, -hl*0.55, 2, 0, Math.PI*2); ctx.fill();
  ctx.beginPath(); ctx.arc(-hw*0.55, 0, 2, 0, Math.PI*2); ctx.fill();
  ctx.beginPath(); ctx.arc(hw*0.55, 0, 2, 0, Math.PI*2); ctx.fill();
  ctx.restore();

  // crash flash
  if (RS.crashFlash > 0) {
    ctx.fillStyle = `rgba(255,0,0,${0.5 * RS.crashFlash})`;
    ctx.fillRect(0, 0, mazeCanvas.width, mazeCanvas.height);
  }
}

/* ---------- charts ---------- */
function renderCharts() {
  cV.clearRect(0, 0, chartV.width, chartV.height);
  cV.fillStyle = "#0c1017"; cV.fillRect(0, 0, chartV.width, chartV.height);
  if (vHist.length > 1) {
    cV.strokeStyle = "#6fe3a1"; cV.lineWidth = 1.5; cV.beginPath();
    for (let i = 0; i < vHist.length; i++) {
      const x = (i / (MAX_HIST-1)) * chartV.width;
      const y = chartV.height - (vHist[i] / 1200) * chartV.height;
      if (i === 0) cV.moveTo(x, y); else cV.lineTo(x, y);
    }
    cV.stroke();
  }
  cV.fillStyle = "#6fe3a1"; cV.font = "10px monospace"; cV.fillText("v mm/s", 4, 10);

  cW.clearRect(0, 0, chartW.width, chartW.height);
  cW.fillStyle = "#0c1017"; cW.fillRect(0, 0, chartW.width, chartW.height);
  if (wHist.length > 1) {
    cW.strokeStyle = "#ffce7a"; cW.lineWidth = 1.5; cW.beginPath();
    for (let i = 0; i < wHist.length; i++) {
      const x = (i / (MAX_HIST-1)) * chartW.width;
      const y = chartW.height/2 - (wHist[i] / 10) * chartW.height/2;
      if (i === 0) cW.moveTo(x, y); else cW.lineTo(x, y);
    }
    cW.stroke();
  }
  cW.fillStyle = "#ffce7a"; cW.font = "10px monospace"; cW.fillText("ω rad/s", 4, 10);
}

/* ---------- telemetry ---------- */
function updateTelemetry() {
  el("stX").textContent = RS.x.toFixed(1);
  el("stY").textContent = RS.y.toFixed(1);
  el("stTh").textContent = (RS.th * 180 / Math.PI).toFixed(1);
  el("stV").textContent = RS.v.toFixed(1);
  el("stW").textContent = RS.w.toFixed(3);
  el("stT").textContent = RS.tSim.toFixed(2);
  el("stD").textContent = (RS.dist / 1000).toFixed(3);
  el("stE").textContent = RS.explored;
  el("sL").textContent = RS.rawL.toFixed(1);
  el("sF").textContent = RS.rawF.toFixed(1);
  el("sR").textContent = RS.rawR.toFixed(1);
  el("stB").textContent = RS.batV.toFixed(2);

  stateBadge.textContent = RS.state + (RS.goalReached && RS.atStart ? " (done)" : "");
  stateBadge.className = "badge " + RS.state.toLowerCase();

  if (RS.state !== "IDLE") {
    vHist.push(RS.v); if (vHist.length > MAX_HIST) vHist.shift();
    wHist.push(RS.w); if (wHist.length > MAX_HIST) wHist.shift();
  }
  renderCharts();
}

/* ---------- console ---------- */
let lastRenderedLogs = 0;
function renderLogs() {
  if (RS.logs.length === lastRenderedLogs) return;
  lastRenderedLogs = RS.logs.length;
  const lines = RS.logs.slice(-200);
  consoleLog.innerHTML = lines.map(l => `<span class="${l.cls}">[${l.t.toFixed(2)}s]</span> ${l.msg}`).join("\n");
  consoleLog.scrollTop = consoleLog.scrollHeight;
}

/* ---------- rAF loop (only rendering + UI; sim runs in worker) ---------- */
function uiLoop() {
  if (RS.mazeWalls) {
    render();
    updateTelemetry();
    lastSnapTime = performance.now();
  } else {
    // show loading while waiting for first snapshot
    ctx.fillStyle = "#0c1017";
    ctx.fillRect(0, 0, mazeCanvas.width, mazeCanvas.height);
    ctx.fillStyle = "#6fe3a1";
    ctx.font = "18px monospace";
    ctx.textAlign = "center";
    if (window.__fatalErr) {
      ctx.fillStyle = "#ff6b6b";
      ctx.font = "13px monospace";
      const linesEr = window.__fatalErr.split("\n");
      for (let i = 0; i < linesEr.length; i++) {
        ctx.fillText(linesEr[i].slice(0, 60), mazeCanvas.width/2, 120 + i * 24);
      }
      ctx.fillStyle = "#ffb3b3";
      ctx.font = "15px monospace";
      ctx.fillText("Try: hard-refresh (Ctrl+Shift+R) or reopen Live Server", mazeCanvas.width/2, mazeCanvas.height/2);
      if (!window.__fatalNotified) {
        window.__fatalNotified = true;
        showFatal("Fatal script error: " + window.__fatalErr.split("\n")[0]);
      }
    } else {
      ctx.fillText("Connecting to simulation worker...", mazeCanvas.width/2, mazeCanvas.height/2);
    }
  }

  updateTelemetry();
  renderLogs();

  // buttons state
  el("btnFast").disabled = !RS.atStart;
  el("btnSearch").disabled = (RS.state === "EXPLORING" || RS.state === "FAST" || RS.state === "RETURN");

  // heartbeat check
  if (workerReady && performance.now() - lastSnapTime > 2000 && lastSnapTime > 0) {
    consoleErr("Worker heartbeat lost! No snapshot for 2s");
    showFatal("Simulation worker stopped responding. Check console (F12).");
  }

  requestAnimationFrame(uiLoop);
}

/* ---------- controls ---------- */
el("btnSearch").onclick = () => { pushParams(); worker.postMessage({ t: "cmd", cmd: "search" }); };
el("btnFast").onclick = () => { pushParams(); worker.postMessage({ t: "cmd", cmd: "fast" }); };
el("btnStop").onclick = () => { worker.postMessage({ t: "cmd", cmd: "stop" }); };
el("btnResetPose").onclick = () => { pushParams(); worker.postMessage({ t: "cmd", cmd: "resetPose" }); };
el("btnResetMap").onclick = () => { worker.postMessage({ t: "cmd", cmd: "resetMap" }); };
el("btnNewMaze").onclick = () => { worker.postMessage({ t: "cmd", cmd: "newMaze", seed: Math.floor(Math.random()*1e9) }); };
el("btnEditWalls").onclick = () => {
  editMode = !editMode;
  editHint.hidden = !editMode;
  mazeCanvas.style.cursor = editMode ? "crosshair" : "default";
};
el("btnBackup").onclick = () => { worker.postMessage({ t: "cmd", cmd: "backup" }); };
el("btnRestore").onclick = () => { el("fileRestore").click(); };
el("fileRestore").onchange = (e) => {
  const f = e.target.files[0]; if (!f) return;
  const reader = new FileReader();
  reader.onload = () => { worker.postMessage({ t: "cmd", cmd: "restore", json: reader.result }); };
  reader.readAsText(f);
  e.target.value = "";
};
el("btnCrash").onclick = () => { worker.postMessage({ t: "cmd", cmd: "stop" }); };
el("btnClearLog").onclick = () => { RS.logs = []; lastRenderedLogs = -1; };
el("btnApplyParams").onclick = () => { pushParams(); };
el("btnDefaults").onclick = () => {
  // restore defaults from engine defaults on next reload; simplest: reload page
  location.reload();
};

el("tglKnown").onchange = () => { showKnown = tglKnown.checked; };
el("tglActual").onchange = () => { showActual = tglActual.checked; };
el("tglSteps").onchange = () => { showSteps = tglSteps.checked; };
el("tglPath").onchange = () => { showPath = tglPath.checked; };

document.getElementById("speedRange").oninput = () => {
  const m = Math.pow(2, parseFloat(document.getElementById("speedRange").value) * 4);
  el("speedOut").textContent = m.toFixed(2) + "×";
  worker.postMessage({ t: "mult", value: m });
};
el("chkPause").onchange = () => { worker.postMessage({ t: "pause", value: el("chkPause").checked }); };

/* ---------- maze presets ---------- */
const presetSel = el("mazePreset");
if (typeof MAZE_PRESETS_LOCAL !== "undefined") {
  MAZE_PRESETS_LOCAL.forEach(p => { const o = document.createElement("option"); o.value = p.seed; o.textContent = p.name; presetSel.appendChild(o); });
}
el("btnLoadPreset").onclick = () => { worker.postMessage({ t: "cmd", cmd: "newMaze", seed: +presetSel.value }); };

/* ---------- wall edit ---------- */
mazeCanvas.addEventListener("click", (e) => {
  if (!editMode || !workerReady || !RS.mazeWalls) return;
  const rect = mazeCanvas.getBoundingClientRect();
  const p = params || collectParams();
  const w = RS.mazeW, h = RS.mazeH;
  const cellPx = 620 / (w + 1.2);
  const ox = (mazeCanvas.width - cellPx * w) / 2;
  const oy = (mazeCanvas.height - cellPx * h) / 2;
  const px = e.clientX - rect.left, py = e.clientY - rect.top;
  const gx = (px - ox) / cellPx;
  const gy = h - 1 - (py - oy) / cellPx;
  const cx = Math.floor(gx + 0.5), cy = Math.floor(gy + 0.5);
  if (cx < 0 || cx >= w || cy < 0 || cy >= h) return;
  const dx = gx - (cx + 0.5), dy = gy - (cy + 0.5);
  let d;
  if (Math.abs(dx) > Math.abs(dy)) d = dx > 0 ? DIR_ENUM.E : DIR_ENUM.W;
  else d = dy > 0 ? DIR_ENUM.N : DIR_ENUM.S;
  const cur = mazeHasWall(cx, cy, d);
  worker.postMessage({ t: "cmd", cmd: "editWall", x: cx, y: cy, d, has: !cur });
  // local mirror
  const i = cy*w + cx;
  if (!cur) RS.mazeWalls[i] |= WBIT_ENUM[d]; else RS.mazeWalls[i] &= ~WBIT_ENUM[d];
});

/* ---------- param input live-apply (throttled) ---------- */
let paramsTimer = null;
document.querySelectorAll(".params input").forEach(inp => {
  inp.addEventListener("input", () => {
    clearTimeout(paramsTimer);
    paramsTimer = setTimeout(pushParams, 250);
  });
});

/* ---------- boot ---------- */
fillParamInputs();
startWorker();
requestAnimationFrame(uiLoop);
