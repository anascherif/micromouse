"use strict";

/* ============================================================================
 *  MicroMouse simulator engine
 *  Faithful port of the firmware's algorithms: flood-fill / StepMap style,
 *  MazeSolver state machine (search -> return -> fast), HC-SR04 sensor model,
 *  and slalom path-following motion using the SAME numbers as config.h.
 * ========================================================================== */

/* --------- deterministic RNG --------- */
function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/* --------- directions (matches MazeLib) ---------
 * 0=N (+y), 1=E (+x), 2=S (-y), 3=W (-x) ---------- */
const DIR = { N: 0, E: 1, S: 2, W: 3 };
const WBIT = { N: 1, E: 2, S: 4, W: 8 };

function dirVec(d) { return [Math.cos(d * Math.PI / 2), Math.sin(d * Math.PI / 2)]; }
function leftDir(d)  { return (d + 3) % 4; }
function rightDir(d) { return (d + 1) % 4; }
function oppDir(d)   { return (d + 2) % 4; }

/* ============================================================================
 *  Maze (ground truth)
 * ========================================================================== */
class Maze {
  constructor(w, h, seed) {
    this.w = w; this.h = h;
    this.walls = new Uint8Array(w * h);
    this._seg = null;
    this._seed = seed >>> 0;
    this._generate();
  }
  idx(x, y) { return y * this.w + x; }
  hasWall(x, y, d) {
    if (d === DIR.N && y === this.h - 1) return true;
    if (d === DIR.S && y === 0) return true;
    if (d === DIR.E && x === this.w - 1) return true;
    if (d === DIR.W && x === 0) return true;
    if (x < 0 || y < 0 || x >= this.w || y >= this.h) return true;
    return (this.walls[this.idx(x, y)] & WBIT[d]) !== 0;
  }
  setWall(x, y, d, b) {
    if (x < 0 || y < 0 || x >= this.w || y >= this.h) return;
    const i = this.idx(x, y);
    if (b) this.walls[i] |= WBIT[d]; else this.walls[i] &= ~WBIT[d];
    const od = oppDir(d);
    const nx = x + Math.cos(od * Math.PI / 2);
    const ny = y + Math.sin(od * Math.PI / 2);
    if (nx >= 0 && ny >= 0 && nx < this.w && ny < this.h) {
      const j = this.idx(nx, ny);
      if (b) this.walls[j] |= WBIT[od]; else this.walls[j] &= ~WBIT[od];
    }
    this._seg = null;
  }

  _generate() {
    const w = this.w, h = this.h;
    for (let x = 0; x < w; x++) {
      this.walls[this.idx(x, 0)] |= WBIT.S;
      this.walls[this.idx(x, h - 1)] |= WBIT.N;
    }
    for (let y = 0; y < h; y++) {
      this.walls[this.idx(0, y)] |= WBIT.W;
      this.walls[this.idx(w - 1, y)] |= WBIT.E;
    }
    const visited = new Uint8Array(w * h);
    const rng = mulberry32(this._seed);
    const stack = [];
    stack.push([0, 0]); visited[this.idx(0, 0)] = 1;
    while (stack.length) {
      const [cx, cy] = stack[stack.length - 1];
      const nbrs = [];
      for (const d of [DIR.N, DIR.E, DIR.S, DIR.W]) {
        const nx = cx + Math.cos(d * Math.PI / 2);
        const ny = cy + Math.sin(d * Math.PI / 2);
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (visited[this.idx(nx, ny)]) continue;
        nbrs.push(d);
      }
      if (nbrs.length === 0) { stack.pop(); continue; }
      const d = nbrs[(rng() * nbrs.length) | 0];
      const nx = cx + Math.cos(d * Math.PI / 2);
      const ny = cy + Math.sin(d * Math.PI / 2);
      this.setWall(cx, cy, d, false);
      visited[this.idx(nx, ny)] = 1;
      stack.push([nx, ny]);
    }
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      for (const d of [DIR.N, DIR.E]) {
        if (rng() < 0.12) {
          const nx = x + Math.cos(d * Math.PI / 2);
          const ny = y + Math.sin(d * Math.PI / 2);
          if (nx >= 0 && ny >= 0 && nx < w && ny < h && this.hasWall(x, y, d)) {
            this.setWall(x, y, d, false);
          }
        }
      }
    }
    this._seg = null;
  }

  _segments() {
    if (this._seg) return this._seg;
    const seg = [];
    for (let y = 0; y < this.h; y++) for (let x = 0; x < this.w; x++) {
      const i = this.idx(x, y);
      const w = this.walls[i];
      if (w & WBIT.N) seg.push([x, y + 1, x + 1, y + 1]);
      if (w & WBIT.S) seg.push([x, y, x + 1, y]);
      if (w & WBIT.E) seg.push([x + 1, y, x + 1, y + 1]);
      if (w & WBIT.W) seg.push([x, y, x, y + 1]);
    }
    this._seg = seg;
    return seg;
  }

  raycast(ox, oy, dx, dy, maxC) {
    const seg = this._segments();
    let best = maxC;
    for (let k = 0; k < seg.length; k++) {
      const [x1, y1, x2, y2] = seg[k];
      const sx = x2 - x1, sy = y2 - y1;
      const denom = dx * sy - dy * sx;
      if (Math.abs(denom) < 1e-9) continue;
      const t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom;
      const u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;
      if (t < 0) continue;
      if (u < -1e-6 || u > 1 + 1e-6) continue;
      if (t < best) best = t;
    }
    return best;
  }
}

/* ============================================================================
 *  Known map (robot's discovered walls) + flood fill
 * ========================================================================== */
class KnownMap {
  constructor(maze) {
    this.w = maze.w; this.h = maze.h;
    this.wall = new Uint8Array(this.w * this.h);
    this.known = new Uint8Array(this.w * this.h);
    this.visited = new Uint8Array(this.w * this.h);
    this.steps = new Uint16Array(this.w * this.h);
  }
  idx(x, y) { return y * this.w + x; }
  isKnown(x, y, d) {
    if (x < 0 || y < 0 || x >= this.w || y >= this.h) return 1;
    return (this.known[this.idx(x, y)] & WBIT[d]) !== 0;
  }
  getWall(x, y, d) {
    if (!this.isKnown(x, y, d)) return false;
    return (this.wall[this.idx(x, y)] & WBIT[d]) !== 0;
  }
  setWall(x, y, d, has) {
    if (x < 0 || y < 0 || x >= this.w || y >= this.h) return;
    const i = this.idx(x, y);
    this.wall[i] = (this.wall[i] & ~WBIT[d]) | (has ? WBIT[d] : 0);
    this.known[i] |= WBIT[d];
    const od = oppDir(d);
    const nx = x + Math.cos(od * Math.PI / 2);
    const ny = y + Math.sin(od * Math.PI / 2);
    if (nx >= 0 && ny >= 0 && nx < this.w && ny < this.h) {
      const j = this.idx(nx, ny);
      this.wall[j] = (this.wall[j] & ~WBIT[od]) | (has ? WBIT[od] : 0);
      this.known[j] |= WBIT[od];
    }
  }
  markVisited(x, y) {
    if (x < 0 || y < 0 || x >= this.w || y >= this.h) return;
    this.visited[this.idx(x, y)] = 1;
  }
  visitedCount(x, y) { return this.visited[this.idx(x, y)] || 0; }
  exploredCellCount() {
    let n = 0;
    for (let i = 0; i < this.visited.length; i++) if (this.visited[i]) n++;
    return n;
  }

  flood(goals) {
    const w = this.w, h = this.h, N = w * h;
    for (let i = 0; i < N; i++) this.steps[i] = 0xFFFF;
    const q = [];
    for (const [gx, gy] of goals) {
      this.steps[gy * w + gx] = 0;
      q.push(gx, gy);
    }
    let head = 0;
    while (head < q.length) {
      const cx = q[head++], cy = q[head++];
      const cur = this.steps[cy * w + cx];
      for (const d of [DIR.N, DIR.E, DIR.S, DIR.W]) {
        if (this.getWall(cx, cy, d)) continue;
        const nx = cx + Math.cos(d * Math.PI / 2);
        const ny = cy + Math.sin(d * Math.PI / 2);
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (this.steps[ny * w + nx] > cur + 1) {
          this.steps[ny * w + nx] = cur + 1;
          q.push(nx, ny);
        }
      }
    }
    return this.steps;
  }
}

/* Choose next directions toward goal, like MazeSolver+StepMap.calcNextDirections */
function nextDirections(map, x, y, headDir, goals, opts) {
  map.flood(goals);
  const dirs = [];
  let cx = x, cy = y, ch = headDir;
  const w = map.w, h = map.h;
  for (let guard = 0; guard < w * h + 5; guard++) {
    const stepHere = map.steps[cy * w + cx];
    if (stepHere === 0) break;
    let best = null, bestScore = Infinity;
    for (const d of [DIR.N, DIR.E, DIR.S, DIR.W]) {
      const nx = cx + Math.cos(d * Math.PI / 2);
      const ny = cy + Math.sin(d * Math.PI / 2);
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (map.getWall(cx, cy, d)) continue;
      const ns = map.steps[ny * w + nx];
      if (ns === 0xFFFF) continue;
      let turn = 0;
      const diff = ((d - ch) + 4) % 4;
      if (diff === 1 || diff === 3) turn = 1;
      else if (diff === 2) turn = 2;
      let unk = 0;
      if (map.known[ny * w + nx] !== 0xF) unk = -opts.preferUnknown;
      const vis = map.visited[ny * w + nx] ? opts.revisitPenalty : 0;
      const score = ns * 100 + turn * 50 + vis - unk * 10;
      if (score < bestScore) { bestScore = score; best = d; }
    }
    if (best === null) break;
    dirs.push(best);
    cx += Math.cos(best * Math.PI / 2);
    cy += Math.sin(best * Math.PI / 2);
    ch = best;
  }
  return dirs;
}

/* ============================================================================
 *  Simulator: motion + executor (MazeSolver state machine)
 * ========================================================================== */
class Sim {
  constructor(seed = 42) {
    this.params = this._defaults();
    this.reset(seed);
  }

  _defaults() {
    return {
      cell: 180, wallT: 12, rlen: 100, rwid: 100, track: 125, turnR: 60,
      s_vstr: 300, s_vcur: 240, s_vmax: 600, s_acc: 3000, s_dec: 2000, s_fbg: 20,
      s_preferUnknown: 0.9, s_revisit: 4,
      f_vmax: 800, f_vcur: 450, f_acc: 4000, f_dec: 4000, f_turnR: 50, f_v90: false,
      c_kp: 1.0, c_ki: 0.1, c_kd: 0.01, c_bat: 11.1, c_batsag: 0.15,
      u_sThr: 70, u_fThr: 120, u_hyst: 1.15, u_max: 4000, u_beam: 8,
      u_noise: 3, u_miss: 0.005, u_glitch: 0.0,
      o_drift: 0.02, o_slip: 0.02, o_dead: 8,
    };
  }

  applyDefaults() { this.params = this._defaults(); }

  reset(seed = 42) {
    this.maze = new Maze(16, 16, seed >>> 0);
    this.known = new KnownMap(this.maze);
    for (let x = 0; x < this.maze.w; x++) {
      this.known.setWall(x, 0, DIR.S, true);
      this.known.setWall(x, this.maze.h - 1, DIR.N, true);
    }
    for (let y = 0; y < this.maze.h; y++) {
      this.known.setWall(0, y, DIR.W, true);
      this.known.setWall(this.maze.w - 1, y, DIR.E, true);
    }
    this.known.setWall(0, 0, DIR.E, this.maze.hasWall(0, 0, DIR.E));
    this.known.setWall(0, 0, DIR.N, this.maze.hasWall(0, 0, DIR.N));

    this.goals = [[7, 7], [7, 8], [8, 7], [8, 8]];
    this.startCell = [0, 0];
    this.startDir = DIR.N;

    this.x = (this.startCell[0] + 0.5) * this.params.cell;
    this.y = (this.startCell[1] + 0.5) * this.params.cell;
    this.th = this.startDir * Math.PI / 2;
    this.v = 0; this.w = 0;
    this.tSim = 0;
    this.dist = 0;
    this.crashFlash = 0;
    this.batV = this.params.c_bat;

    this.state = "IDLE";
    this.goalReached = false;
    this.atStart = false;
    this.tSearch = 0; this.tFast = 0;

    this.path = null;
    this.segIdx = 0;
    this.sInSeg = 0;
    this.curV = 0;
    this.segV = null;
    this.primActions = [];

    this.sensors = { L: 0, F: 0, R: 0 };
    this.rawDist = { L: 0, F: 0, R: 0 };
    this.trail = [[this.x, this.y]];
    this.known.markVisited(0, 0);

    this._rng = mulberry32((seed + 7) >>> 0);
    this.logs = [];
    this._log(0, "Boot. Maze " + this.maze.w + "x" + this.maze.h + " (seed " + seed + ")");
  }

  _log(t, msg, cls = "") { this.logs.push({ t, msg, cls }); if (this.logs.length > 500) this.logs.shift(); }

  _sonarRaw() {
    const p = this.params;
    const maxC = (p.u_max + p.cell) / p.cell;
    const th = this.th;
    const sensors = [
      { name: "F", offX: 0, offY: (p.rlen / 2) * 0.55, a: th },
      { name: "L", offX: -(p.rwid / 2) * 0.55, offY: 0, a: th - Math.PI / 2 },
      { name: "R", offX:  (p.rwid / 2) * 0.55, offY: 0, a: th + Math.PI / 2 },
    ];
    const out = {};
    for (const s of sensors) {
      const cs = Math.cos(s.a), sn = Math.sin(s.a);
      const ox = (this.x + s.offX) / p.cell;
      const oy = (this.y + s.offY) / p.cell;
      const d = this.maze.raycast(ox, oy, cs, sn, maxC);
      let mm = d * p.cell - p.wallT / 2;
      mm = Math.max(20, Math.min(p.u_max, mm));
      if (this._rng() < p.u_miss) mm = 0;
      if (this._rng() < p.u_glitch) mm = 20 + (this._rng() * 40);
      const noise = (this._rng() - 0.5) * 2 * p.u_noise;
      mm = Math.max(0, mm + noise);
      out[s.name] = mm;
    }
    this.rawDist = out;
    return out;
  }

  currentCell() {
    const c = this.params.cell;
    return [
      Math.max(0, Math.min(this.maze.w - 1, Math.floor(this.x / c))),
      Math.max(0, Math.min(this.maze.h - 1, Math.floor(this.y / c)))
    ];
  }

  senseAndUpdate() {
    const p = this.params;
    const raw = this._sonarRaw();
    const sens = {};
    sens.F = raw.F < p.u_fThr;
    sens.L = raw.L < p.u_sThr;
    sens.R = raw.R < p.u_sThr;
    this.sensors = raw;
    const [cx, cy] = this.currentCell();
    const headDir = ((Math.round(this.th / (Math.PI / 2)) % 4) + 4) % 4;
    if (sens.L) this.known.setWall(cx, cy, leftDir(headDir), true);
    if (sens.F) this.known.setWall(cx, cy, headDir, true);
    if (sens.R) this.known.setWall(cx, cy, rightDir(headDir), true);
    this.known.markVisited(cx, cy);
  }

  _buildPathFromCellActions(actions, turnR) {
    const c = this.params.cell;
    let pos = [this.x, this.y];
    let head = ((Math.round(this.th / (Math.PI / 2)) % 4) + 4) % 4;
    const prims = [];
    let pendingStraightLen = 0;

    for (const a of actions) {
      if (a.k === "straight") {
        pendingStraightLen += a.n * c;
      } else if (a.k === "turn") {
        const newHead = (head + a.dir + 4) % 4;
        const r = Math.max(8, Math.min(turnR, pendingStraightLen / 2 + 1e-3, c / 2 + 1e-3));
        const entry = Math.min(pendingStraightLen, r);
        const beforeEntry = Math.max(0, pendingStraightLen - entry);
        if (beforeEntry > 1e-3) {
          prims.push({ type: "line", head, len: beforeEntry, vmax: a.vmax, vend: a.vcur });
          const v = dirVec(head);
          pos = [pos[0] + v[0] * beforeEntry, pos[1] + v[1] * beforeEntry];
        }
        const arcLen = r * (Math.PI / 2);
        prims.push({ type: "arc", r, head, dir: a.dir, arcLen, vmax: a.vcur, vend: a.vcur });
        pos = [pos[0] + dirVec(newHead)[0] * r, pos[1] + dirVec(newHead)[1] * r];
        pendingStraightLen = r;
        head = newHead;
      }
    }
    if (pendingStraightLen > 1e-3) {
      prims.push({ type: "line", head, len: pendingStraightLen, vmax: this.params.s_vstr, vend: 0 });
    }

    let cur = [this.x, this.y];
    for (const p of prims) {
      p.from = cur.slice();
      if (p.type === "line") {
        const v = dirVec(p.head);
        cur = [cur[0] + v[0] * p.len, cur[1] + v[1] * p.len];
        p.to = cur.slice();
      } else {
        const v = dirVec(p.head);
        const sign = (p.dir > 0) ? 1 : -1;
        const norm = [-v[1], v[0]];
        const cx = cur[0] + sign * p.r * norm[0];
        const cy = cur[1] + sign * p.r * norm[1];
        p.center = [cx, cy];
        const a0 = Math.atan2(cur[1] - cy, cur[0] - cx);
        const a1 = a0 + sign * (Math.PI / 2);
        p.a0 = a0; p.a1 = a1;
        cur = [cx + p.r * Math.cos(a1), cy + p.r * Math.sin(a1)];
        p.to = cur.slice();
      }
    }
    return prims;
  }

  _planSpeeds(prims, opts) {
    const n = prims.length;
    const vEnd = new Array(n + 1).fill(0);
    for (let i = 0; i < n; i++) {
      if (prims[i].type === "line") vEnd[i + 1] = prims[i].vend ?? 0;
      else vEnd[i + 1] = opts.cornerSpeed;
    }
    vEnd[n] = 0;
    for (let i = n - 1; i >= 0; i--) {
      const p = prims[i];
      const L = (p.type === "line") ? p.len : p.arcLen;
      const d = opts.dec;
      const vAllow = Math.sqrt(vEnd[i + 1] * vEnd[i + 1] + 2 * d * L);
      vEnd[i] = Math.min(vAllow, vEnd[i]);
    }
    let vPrev = 0;
    for (let i = 0; i < n; i++) {
      const p = prims[i];
      const L = (p.type === "line") ? p.len : p.arcLen;
      const a = opts.acc;
      const vCap = (p.type === "line") ? p.vmax : opts.cornerSpeed;
      const vAccel = Math.sqrt(vPrev * vPrev + 2 * a * L);
      vPrev = Math.min(vCap, vAccel, vEnd[i + 1]);
      vEnd[i] = Math.min(vEnd[i], vPrev);
      vPrev = vEnd[i];
    }
    return vEnd;
  }

  startSearch() {
    if (this.state === "EXPLORING" || this.state === "FAST" || this.state === "RETURN") return;
    this.reset(42);
    this.state = "EXPLORING";
    this.tSearch = 0;
    this._planExplorationStep();
    this._log(this.tSim, "MazeSolver: search run START", "ok");
    this._beep("START_INIT");
  }

  _planExplorationStep() {
    const p = this.params;
    const [cx, cy] = this.currentCell();
    const headDir = ((Math.round(this.th / (Math.PI / 2)) % 4) + 4) % 4;
    if (this.goalReached) { this._planReturn(); return; }
    const opts = { preferUnknown: p.s_preferUnknown, revisitPenalty: p.s_revisit };
    const dirs = nextDirections(this.known, cx, cy, headDir, this.goals, opts);
    if (dirs.length === 0) {
      if (this._isGoal(cx, cy)) {
        this.goalReached = true;
        this._log(this.tSim, "MazeSolver: GOAL reached", "ok");
        this._beep("CONFIRM");
        this.state = "GOAL";
        this._planReturn();
        return;
      } else {
        this._log(this.tSim, "MazeSolver: stuck? emergency", "err");
        this._emergency("no-path");
        return;
      }
    }
    const first = dirs[0];
    const actions = [];
    if (first !== headDir) {
      const diff = (((first - headDir) % 4) + 4) % 4;
      if (diff === 1) actions.push({ k: "turn", dir: +1, vmax: p.s_vstr, vcur: p.s_vcur });
      else if (diff === 3) actions.push({ k: "turn", dir: -1, vmax: p.s_vstr, vcur: p.s_vcur });
      else if (diff === 2) {
        actions.push({ k: "turn", dir: +1, vmax: p.s_vstr, vcur: p.s_vcur });
        actions.push({ k: "turn", dir: +1, vmax: p.s_vstr, vcur: p.s_vcur });
      }
    }
    const nCells = Math.min(dirs.length, 3);
    const vmax = (nCells >= 2) ? p.s_vmax : p.s_vstr;
    actions.push({ k: "straight", n: nCells, vmax, vend: p.s_vcur });
    this._buildAndStart(actions, p.turnR, { acc: p.s_acc, dec: p.s_dec, cornerSpeed: p.s_vcur });
  }

  _planReturn() {
    const p = this.params;
    const [cx, cy] = this.currentCell();
    const headDir = ((Math.round(this.th / (Math.PI / 2)) % 4) + 4) % 4;
    const opts = { preferUnknown: 0, revisitPenalty: 0 };
    const dirs = nextDirections(this.known, cx, cy, headDir, [this.startCell], opts);
    if (dirs.length === 0) {
      this.atStart = true;
      this.state = "AT_START";
      this._log(this.tSim, "MazeSolver: returned to start (t=" + this.tSearch.toFixed(2) + "s)", "ok");
      this._beep("COMPLETE");
      return;
    }
    this.state = "RETURN";
    const actions = [];
    let curHead = headDir;
    let i = 0;
    while (i < dirs.length) {
      const d = dirs[i];
      if (d !== curHead) {
        const diff = (((d - curHead) % 4) + 4) % 4;
        if (diff === 1) actions.push({ k: "turn", dir: +1, vmax: p.s_vstr, vcur: p.s_vcur });
        else if (diff === 3) actions.push({ k: "turn", dir: -1, vmax: p.s_vstr, vcur: p.s_vcur });
        else { actions.push({ k: "turn", dir: +1, vmax: p.s_vstr, vcur: p.s_vcur }); actions.push({ k: "turn", dir: +1, vmax: p.s_vstr, vcur: p.s_vcur }); }
        curHead = d;
      }
      let j = i;
      while (j < dirs.length && dirs[j] === d) j++;
      const n = j - i;
      const vmax = (n >= 2) ? p.s_vmax : p.s_vstr;
      actions.push({ k: "straight", n, vmax, vend: p.s_vcur });
      i = j;
    }
    this._buildAndStart(actions, p.turnR, { acc: p.s_acc, dec: p.s_dec, cornerSpeed: p.s_vcur });
  }

  _planFastRun() {
    const p = this.params;
    if (!this.atStart) { this._log(this.tSim, "MazeSolver: fast run aborted — not at start", "warn"); return; }
    const [cx, cy] = this.currentCell();
    const headDir = ((Math.round(this.th / (Math.PI / 2)) % 4) + 4) % 4;
    const dirs = nextDirections(this.known, cx, cy, headDir, this.goals, { preferUnknown: 0, revisitPenalty: 0 });
    if (dirs.length === 0) { this._log(this.tSim, "MazeSolver: fast run no path", "err"); return; }
    const actions = [];
    let curHead = headDir;
    for (let i = 0; i < dirs.length; i++) {
      const d = dirs[i];
      if (d !== curHead) {
        const diff = (((d - curHead) % 4) + 4) % 4;
        if (diff === 1) actions.push({ k: "turn", dir: +1, vmax: p.f_vmax, vcur: p.f_vcur });
        else if (diff === 3) actions.push({ k: "turn", dir: -1, vmax: p.f_vmax, vcur: p.f_vcur });
        else { actions.push({ k: "turn", dir: +1, vmax: p.f_vmax, vcur: p.f_vcur }); actions.push({ k: "turn", dir: +1, vmax: p.f_vmax, vcur: p.f_vcur }); }
        curHead = d;
      }
      let j = i;
      while (j < dirs.length && dirs[j] === d) j++;
      const n = j - i;
      const vmax = (n >= 2) ? p.f_vmax : p.f_vcur;
      actions.push({ k: "straight", n, vmax, vend: p.f_vcur });
      i = j - 1;
    }
    this.state = "FAST";
    this.tFast = 0;
    this._buildAndStart(actions, p.f_turnR, { acc: p.f_acc, dec: p.f_dec, cornerSpeed: p.f_vcur });
    this._log(this.tSim, "MazeSolver: fast run START", "ok");
    this._beep("CONFIRM");
  }

  _buildAndStart(actions, turnR, opts) {
    const prims = this._buildPathFromCellActions(actions, turnR);
    const speeds = this._planSpeeds(prims, opts);
    this.path = prims;
    this.segIdx = 0;
    this.sInSeg = 0;
    this.curV = 0;
    this.segV = speeds;
    this.primActions = actions;
  }

  _isGoal(x, y) {
    for (const g of this.goals) if (g[0] === x && g[1] === y) return true;
    return false;
  }

  _beep(kind) {
    const notes = { BOOT: 880, START_INIT: 660, CONFIRM: 880, COMPLETE: 1320, SHORT: 660, ERROR: 200, EMERGENCY: 140, CANCEL: 300 };
    const f = notes[kind] || 660;
    this._log(this.tSim, "bz: " + kind + " (" + f + " Hz)", "bz");
    if (typeof globalThis !== "undefined" && globalThis.playBeep) globalThis.playBeep(f, 0.06);
  }

  _emergency(reason) {
    this.state = "EMERGENCY";
    this.path = null;
    this.v = 0; this.w = 0;
    this.curV = 0;
    this._log(this.tSim, "EMERGENCY: " + reason + " (imu.accel.z > 2*g)", "err");
    this._beep("EMERGENCY");
    this.crashFlash = 0.6;
  }

  step(dt) {
    if (this.state === "EMERGENCY" || this.state === "IDLE" || this.state === "AT_START" || this.state === "DONE") {
      this.v *= Math.exp(-dt * 6);
      this.w *= Math.exp(-dt * 8);
      this._drainBattery(dt, false);
      this.tSim += dt;
      this.crashFlash = Math.max(0, this.crashFlash - dt);
      return;
    }
    if (!this.path || this.segIdx >= this.path.length) {
      if (this.state === "EXPLORING" || this.state === "GOAL") {
        this.senseAndUpdate();
        this._drainBattery(dt, true);
        this._planExplorationStep();
      } else if (this.state === "RETURN") {
        this.senseAndUpdate();
        this._planReturn();
      }
      if (!this.path || this.segIdx >= this.path.length) return;
    }
    const p = this.path[this.segIdx];
    const targetV = this.segV[this.segIdx + 1];
    const L = (p.type === "line") ? p.len : p.arcLen;
    if (L <= 0) { this.segIdx++; this.sInSeg = 0; return; }
    const a = this.params.s_acc;
    const d = this.params.s_dec;
    const slope = (this.curV < targetV) ? a : d;
    let dv = slope * dt;
    if (this.curV < targetV) this.curV = Math.min(targetV, this.curV + dv);
    else if (this.curV > targetV) this.curV = Math.max(targetV, this.curV - dv);
    const ds = Math.min(L - this.sInSeg, this.curV * dt);
    this.sInSeg += ds;
    this.dist += ds;
    const slipF = this.params.o_slip;
    const deadT = this.params.o_dead / 1000;
    const drift = this.params.o_drift * Math.PI / 180;
    let effV = this.curV * (1 - slipF);
    if (this.tSim < deadT) effV *= 0;
    if (p.type === "line") {
      const v = dirVec(p.head);
      this.x += v[0] * ds * (effV / Math.max(this.curV, 1e-6));
      this.y += v[1] * ds * (effV / Math.max(this.curV, 1e-6));
      this.v = effV; this.w = 0;
    } else {
      const w = (p.dir > 0 ? +1 : -1) * this.curV / p.r;
      this.w = w;
      this.th += w * dt;
      const hd = p.head;
      this.x += ds * Math.cos(hd);
      this.y += ds * Math.sin(hd);
      this.v = this.curV;
    }
    this.th += drift * dt;
    this._drainBattery(dt, this.curV > 50);
    this.tSim += dt;
    if (this.state === "FAST") this.tFast += dt;
    else if (this.state === "EXPLORING" || this.state === "GOAL" || this.state === "RETURN") this.tSearch += dt;

    const raw = this._sonarRaw();
    if (raw.F < (this.params.rlen / 2 + this.params.wallT / 2) * 0.9) {
      if (Math.abs(this.curV) > 50) this._emergency("crash front wall");
    }

    if (this.sInSeg >= L - 1e-3) {
      this.segIdx++;
      this.sInSeg = 0;
      this.trail.push([this.x, this.y]);
      if (this.state === "EXPLORING" || this.state === "GOAL") {
        this.senseAndUpdate();
        if (!this.goalReached) this._planExplorationStep();
        else this._planReturn();
      } else if (this.state === "RETURN") {
        this.senseAndUpdate();
        this._planReturn();
      } else if (this.state === "FAST") {
        this.senseAndUpdate();
        const [cx, cy] = this.currentCell();
        if (this._isGoal(cx, cy)) {
          this.state = "DONE";
          this._log(this.tSim, "FAST DONE in " + this.tFast.toFixed(2) + "s", "ok");
          this._beep("COMPLETE");
        }
      }
    }
    this.crashFlash = Math.max(0, this.crashFlash - dt);
  }

  _drainBattery(dt, moving) {
    const drop = moving ? this.params.c_batsag * 0.01 : 0.0008;
    this.batV = Math.max(6.0, this.batV - drop * dt);
  }

  exportBackup() {
    const records = [];
    for (let y = 0; y < this.known.h; y++) for (let x = 0; x < this.known.w; x++) {
      for (const d of [DIR.N, DIR.E, DIR.S, DIR.W]) {
        if (this.known.isKnown(x, y, d)) {
          records.push({ x, y, d, b: this.known.getWall(x, y, d) });
        }
      }
    }
    return JSON.stringify({
      format: "mm-sim/v1",
      w: this.known.w, h: this.known.h,
      records,
      tSearch: this.tSearch, tFast: this.tFast,
    }, null, 2);
  }

  importBackup(json) {
    const obj = JSON.parse(json);
    if (obj.format !== "mm-sim/v1") throw new Error("Unknown format");
    const k = new KnownMap(this.maze);
    for (const r of obj.records) {
      k.setWall(r.x, r.y, r.d, r.b);
    }
    this.known = k;
    this._log(this.tSim, "Restored " + obj.records.length + " wall records", "ok");
    this._beep("SHORT");
  }
}

const MAZE_PRESETS = [
  { name: "Classic (seed 42)", seed: 42 },
  { name: "Tight turns (seed 7)", seed: 7 },
  { name: "Open loops (seed 13)", seed: 13 },
  { name: "Spiral (seed 99)", seed: 99 },
  { name: "Diagonals (seed 2024)", seed: 2024 },
];

const _g = (typeof globalThis !== "undefined") ? globalThis : window;
_g.Sim = Sim;
_g.Maze = Maze;
_g.KnownMap = KnownMap;
_g.MAZE_PRESETS = MAZE_PRESETS;
_g.nextDirections = nextDirections;
_g.DIR = DIR;
_g.WBIT = WBIT;