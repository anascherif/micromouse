"use strict";

/* ============================================================================
 *  MicroMouse simulator — Web Worker
 *  Runs the Sim engine OFF the main thread so the UI stays responsive.
 *  Communicates with app.js via postMessage.
 * ========================================================================== */

importScripts("engine.js");

let sim = new Sim(42);
sim._seedForReset = 42;
let paused = false;
let mult = 4;
let lastTs = performance.now();
let accum = 0;
let trailSent = 0;
let logsSent = 0;
let mazeDirty = true;

const DT = 0.001;
const MAX_STEPS_PER_TICK = 600;

function snapshot() {
  const s = {
    x: sim.x, y: sim.y, th: sim.th,
    v: sim.v, w: sim.w, curV: sim.curV,
    tSim: sim.tSim, dist: sim.dist, batV: sim.batV,
    state: sim.state, goalReached: sim.goalReached, atStart: sim.atStart,
    tSearch: sim.tSearch, tFast: sim.tFast,
    explored: sim.known.exploredCellCount(),
    rawF: sim.rawDist.F, rawL: sim.rawDist.L, rawR: sim.rawDist.R,
    // shallow-copied data needed for rendering
    path: sim.path, segIdx: sim.segIdx,
    crashFlash: sim.crashFlash,
    goals: sim.goals,
    mazeW: sim.maze.w, mazeH: sim.maze.h,
  };
  return s;
}

function tick() {
  try {
    const now = performance.now();
    let dtReal = (now - lastTs) / 1000;
    if (dtReal > 0.1) dtReal = 0.1;
    lastTs = now;

    if (!paused) {
      accum += dtReal * mult;
      let steps = 0;
      while (accum >= DT && steps < MAX_STEPS_PER_TICK) {
        sim.step(DT);
        accum -= DT;
        steps++;
      }
      if (steps >= MAX_STEPS_PER_TICK) accum = 0;
    }

    // detect resets → arrays shrink
    let fullReset = false;
    if (sim.trail.length < trailSent || sim.logs.length < logsSent) {
      fullReset = true;
      trailSent = 0;
      logsSent = 0;
    }

    const msg = {
      t: "snap",
      s: snapshot(),
      mazeDirty,
      fullReset,
      mazeWalls: mazeDirty ? sim.maze.walls.slice() : null,
      knownWall: sim.known.wall.slice(),
      knownBits: sim.known.known.slice(),
      stepsArr: sim.known.steps.slice(),
      newTrail: sim.trail.slice(trailSent),
      newLogs: sim.logs.slice(logsSent),
    };
    mazeDirty = false;
    trailSent = sim.trail.length;
    logsSent = sim.logs.length;
    postMessage(msg);
  } catch (err) {
    postMessage({ t: "error", message: err.message, stack: err.stack });
  }
}

onmessage = (e) => {
  const m = e.data;
  switch (m.t) {
    case "mult": mult = m.value; break;
    case "pause": paused = m.value; break;
    case "params":
      sim.params = Object.assign({}, sim.params, m.params);
      break;
    case "cmd": {
      switch (m.cmd) {
        case "search": sim.startSearch(); break;
        case "fast": sim._planFastRun(); break;
        case "stop": sim._emergency("user"); break;
        case "resetPose": sim.reset(sim._seedForReset || 42); mazeDirty = true; trailSent = 0; logsSent = 0; break;
        case "resetMap":
          sim.known = new KnownMap(sim.maze);
          sim.known.markVisited(0, 0);
          sim._log(sim.tSim, "Map knowledge cleared", "warn");
          break;
        case "newMaze":
          sim.reset(m.seed);
          sim._seedForReset = m.seed;
          mazeDirty = true; trailSent = 0; logsSent = 0;
          break;
        case "editWall":
          // mirror app logic: flip true wall in real maze, mark as unknown in known map
          sim.maze.setWall(m.x, m.y, m.d, m.has);
          sim.known.setWall(m.x, m.y, m.d, false);
          mazeDirty = true;
          sim._log(sim.tSim, "Maze wall edited at " + m.x + "," + m.y, "warn");
          break;
        case "restore":
          try { sim.importBackup(m.json); } catch (err) { sim._log(sim.tSim, "Restore failed: " + err, "err"); }
          break;
        case "backup":
          postMessage({ t: "backup", json: sim.exportBackup() });
          break;
      }
      break;
    }
  }
};

globalThis.playBeep = (f, d) => postMessage({ t: "beep", f, d });

// Send initial snapshot immediately so UI gets maze data instantly
postMessage({
  t: "snap",
  s: snapshot(),
  mazeDirty: true,
  fullReset: false,
  mazeWalls: sim.maze.walls.slice(),
  knownWall: sim.known.wall.slice(),
  knownBits: sim.known.known.slice(),
  stepsArr: sim.known.steps.slice(),
  newTrail: [],
  newLogs: [],
});

setInterval(tick, 16); // ~60 Hz
postMessage({ t: "ready" });
