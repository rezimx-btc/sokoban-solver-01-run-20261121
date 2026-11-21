// sokoban-solver — push-optimal Sokoban solver for make.fun.
//
// Strategy: the generator builds each level as one hallway with 1-4
// small rooms off it. A box pushed out of its room can never come back,
// so every box stays in its room and the push-optimal total P* is the
// sum of the rooms' optima. Each room is therefore solved on its own
// with push-optimal A* (cost = pushes, player reachability by BFS
// between pushes, dead-square + 2x2-block pruning, min-assignment
// heuristic over precomputed BFS distances), and the per-room move
// strings are stitched together with hallway walks between rooms.
//
// Only node built-ins are used. The file is erasable TypeScript (type
// annotations, interfaces and `as` casts only) so node runs it directly.

import * as fs from "node:fs";
import * as path from "node:path";

const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
] as const;

interface Level {
  w: number;
  h: number;
  wall: boolean[];
  goals: number[];
  goalSet: Set<number>;
  boxes: number[];
  player: number;
}

interface Room {
  cells: number[];
  cellSet: Set<number>;
  boxes: number[];
  goals: number[];
  doors: number[];
}

function parseLevel(text: string): Level {
  const lines = text.split(/\r?\n/);
  while (lines.length > 0 && lines[lines.length - 1].length === 0) lines.pop();
  const h = lines.length;
  let w = 0;
  for (const ln of lines) w = Math.max(w, ln.length);
  const wall = new Array<boolean>(w * h).fill(true);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    const ln = lines[y];
    for (let x = 0; x < w; x++) {
      const ch = x < ln.length ? ln[x] : " ";
      const c = y * w + x;
      if (ch === "#") continue;
      wall[c] = false;
      if (ch === "." || ch === "*" || ch === "+") goals.push(c);
      if (ch === "$" || ch === "*") boxes.push(c);
      if (ch === "@" || ch === "+") player = c;
    }
  }
  return { w, h, wall, goals, goalSet: new Set(goals), boxes, player };
}

// Split the level into the hallway row (the player's row) and the rooms:
// the connected components of open floor once the hallway row is removed.
// A room's door cells are the room cells adjacent to a hallway cell.
function detectRooms(level: Level): { rooms: Room[]; hallway: Set<number> } {
  const { w, h, wall, player } = level;
  const hallRow = Math.floor(player / w);
  const hallway = new Set<number>();
  for (let x = 0; x < w; x++) {
    const c = hallRow * w + x;
    if (!wall[c]) hallway.add(c);
  }
  const seen = new Set<number>();
  const rooms: Room[] = [];
  const boxSet = new Set(level.boxes);
  const goalSet = level.goalSet;
  for (let c = 0; c < w * h; c++) {
    if (wall[c] || hallway.has(c) || seen.has(c)) continue;
    const comp: number[] = [];
    const stack = [c];
    seen.add(c);
    while (stack.length > 0) {
      const cur = stack.pop() as number;
      comp.push(cur);
      const cx = cur % w;
      const cy = Math.floor(cur / w);
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || hallway.has(n) || seen.has(n)) continue;
        seen.add(n);
        stack.push(n);
      }
    }
    const boxes = comp.filter((cc) => boxSet.has(cc));
    if (boxes.length === 0) continue;
    const goals = comp.filter((cc) => goalSet.has(cc));
    const doors = comp.filter((cc) => {
      const cx = cc % w;
      const cy = Math.floor(cc / w);
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (hallway.has(ny * w + nx)) return true;
      }
      return false;
    });
    rooms.push({ cells: comp, cellSet: new Set(comp), boxes, goals, doors });
  }
  return { rooms, hallway };
}

// All-pairs shortest walking distances between open cells, ignoring
// boxes. dist[a * n + b] is the step distance, or -1 when unreachable.
function computeDistances(level: Level): Int32Array {
  const n = level.w * level.h;
  const dist = new Int32Array(n * n).fill(-1);
  for (let s = 0; s < n; s++) {
    if (level.wall[s]) continue;
    const q: number[] = [s];
    dist[s * n + s] = 0;
    let qi = 0;
    while (qi < q.length) {
      const cur = q[qi++];
      const cd = dist[s * n + cur];
      const cx = cur % level.w;
      const cy = Math.floor(cur / level.w);
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= level.w || ny >= level.h) continue;
        const nb = ny * level.w + nx;
        if (level.wall[nb] || dist[s * n + nb] !== -1) continue;
        dist[s * n + nb] = cd + 1;
        q.push(nb);
      }
    }
  }
  return dist;
}

// Static dead squares of a room: non-goal cells from which a box can
// never reach a goal. Computed by reverse reachability from the goals:
// a box at p could previously have been at p - d when the player could
// stand at p - 2d (the push that brought it to p). A room cell not
// reached this way is truly dead even with a freely teleporting
// player, so pruning it is sound. (A wall-run rule is not enough: a
// box can slide past the end of a goal-less wall run and escape.)
function computeDeadSquares(level: Level, room: Room): Set<number> {
  const live = new Set<number>(room.goals);
  const q: number[] = room.goals.slice();
  const { w, h, wall } = level;
  const open = (x: number, y: number): boolean =>
    x >= 0 && y >= 0 && x < w && y < h && !wall[y * w + x];
  let qi = 0;
  while (qi < q.length) {
    const p = q[qi++];
    const px = p % w;
    const py = Math.floor(p / w);
    for (const d of DIRS) {
      const prevX = px - d.dx;
      const prevY = py - d.dy;
      if (!open(prevX, prevY)) continue;
      const prev = prevY * w + prevX;
      if (!room.cellSet.has(prev) || live.has(prev)) continue;
      const plX = px - 2 * d.dx;
      const plY = py - 2 * d.dy;
      if (!open(plX, plY)) continue;
      live.add(prev);
      q.push(prev);
    }
  }
  const dead = new Set<number>();
  for (const c of room.cells) if (!live.has(c)) dead.add(c);
  return dead;
}

interface Reach {
  prev: Int32Array;
  minCell: number;
}

// Player-reachability BFS from `from` over open cells, treating every
// cell in `blocked` (boxes) as an obstacle. prev[start] = start.
function bfsReach(level: Level, from: number, blocked: Set<number>): Reach {
  const n = level.w * level.h;
  const prev = new Int32Array(n).fill(-1);
  prev[from] = from;
  const q: number[] = [from];
  let minCell = from;
  let qi = 0;
  while (qi < q.length) {
    const cur = q[qi++];
    if (cur < minCell) minCell = cur;
    const cx = cur % level.w;
    const cy = Math.floor(cur / level.w);
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= level.w || ny >= level.h) continue;
      const nb = ny * level.w + nx;
      if (level.wall[nb] || blocked.has(nb) || prev[nb] !== -1) continue;
      prev[nb] = cur;
      q.push(nb);
    }
  }
  return { prev, minCell };
}

// Reconstruct the walk (LURD letters) from `from` to `to` via reach.prev.
function walkPath(level: Level, reach: Reach, from: number, to: number): string {
  const cells: number[] = [];
  let cur = to;
  while (cur !== from) {
    cells.push(cur);
    cur = reach.prev[cur];
  }
  cells.reverse();
  let out = "";
  let px = from % level.w;
  let py = Math.floor(from / level.w);
  for (const c of cells) {
    const x = c % level.w;
    const y = Math.floor(c / level.w);
    if (x === px + 1) out += "r";
    else if (x === px - 1) out += "l";
    else if (y === py + 1) out += "d";
    else out += "u";
    px = x;
    py = y;
  }
  return out;
}

// Admissible heuristic: the cheapest assignment of boxes to goals by
// precomputed walking distance. Each push moves one box one cell, so
// the sum of distances is a lower bound on the pushes still needed.
function assignmentHeuristic(
  boxes: number[],
  goals: number[],
  dist: Int32Array,
  n: number,
  cache: Map<string, number>
): number {
  const key = boxes.join(",");
  const hit = cache.get(key);
  if (hit !== undefined) return hit;
  const k = boxes.length;
  let best = Infinity;
  const used = new Array<boolean>(k).fill(false);
  const rec = (bi: number, acc: number): void => {
    if (acc >= best) return;
    if (bi === k) {
      best = acc;
      return;
    }
    for (let gi = 0; gi < k; gi++) {
      if (used[gi]) continue;
      const d = dist[boxes[bi] * n + goals[gi]];
      if (d < 0) continue;
      used[gi] = true;
      rec(bi + 1, acc + d);
      used[gi] = false;
    }
  };
  rec(0, 0);
  cache.set(key, best);
  return best;
}

// True when `dest` (just occupied by a pushed box) completes a 2x2 block
// of walls/boxes that traps a box not on a goal: no box inside such a
// block can ever move, because two of its four neighbours are inside
// the block (blocking two push directions) and the other two push
// directions need the player to stand on a block cell (occupied).
function formsDeadBlock(level: Level, boxes: Set<number>, dest: number): boolean {
  const w = level.w;
  const h = level.h;
  const dx = dest % w;
  const dy = Math.floor(dest / w);
  const filled = (x: number, y: number): boolean => {
    if (x < 0 || y < 0 || x >= w || y >= h) return true;
    const c = y * w + x;
    return level.wall[c] || boxes.has(c);
  };
  const corners: Array<[number, number]> = [
    [0, 0],
    [-1, 0],
    [0, -1],
    [-1, -1],
  ];
  for (const [ox, oy] of corners) {
    const x0 = dx + ox;
    const y0 = dy + oy;
    const cells: Array<[number, number]> = [
      [x0, y0],
      [x0 + 1, y0],
      [x0, y0 + 1],
      [x0 + 1, y0 + 1],
    ];
    let all = true;
    let badBox = false;
    for (const [cx, cy] of cells) {
      if (!filled(cx, cy)) {
        all = false;
        break;
      }
      if (cx >= 0 && cy >= 0 && cx < w && cy < h) {
        const c = cy * w + cx;
        if (boxes.has(c) && !level.goalSet.has(c)) badBox = true;
      }
    }
    if (all && badBox) return true;
  }
  return false;
}

interface SolverNode {
  boxes: number[];
  player: number;
  g: number;
  h: number;
  parent: number;
  seg: string;
  seq: number;
  regionMin: number;
}

// Binary min-heap of node indices, ordered by f = g + h, then h, then
// insertion order, so the search is fully deterministic.
class NodeHeap {
  private a: number[] = [];
  private nodes: SolverNode[];
  constructor(nodes: SolverNode[]) {
    this.nodes = nodes;
  }
  size(): number {
    return this.a.length;
  }
  private less(i: number, j: number): boolean {
    const x = this.nodes[this.a[i]];
    const y = this.nodes[this.a[j]];
    const fx = x.g + x.h;
    const fy = y.g + y.h;
    if (fx !== fy) return fx < fy;
    if (x.h !== y.h) return x.h < y.h;
    return x.seq < y.seq;
  }
  push(v: number): void {
    this.a.push(v);
    let i = this.a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(i, p)) break;
      const tmp = this.a[i];
      this.a[i] = this.a[p];
      this.a[p] = tmp;
      i = p;
    }
  }
  pop(): number {
    const top = this.a[0];
    const last = this.a.pop() as number;
    if (this.a.length > 0) {
      this.a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < this.a.length && this.less(l, m)) m = l;
        if (r < this.a.length && this.less(r, m)) m = r;
        if (m === i) break;
        const tmp = this.a[i];
        this.a[i] = this.a[m];
        this.a[m] = tmp;
        i = m;
      }
    }
    return top;
  }
}

interface RoomSolution {
  moves: string;
  finalBoxes: number[];
  endPlayer: number;
}

// Push-optimal A* for one room. The state is (sorted box positions,
// player reachability region); the region is canonicalised by its
// smallest cell, because the player's exact cell inside the region
// does not change which pushes are possible. Boxes of other rooms are
// fixed obstacles. needExit requires the final state to let the player
// reach the hallway, so the next room can be entered.
function solveRoom(
  level: Level,
  room: Room,
  startPlayer: number,
  otherBoxes: number[],
  dist: Int32Array,
  hallway: Set<number>,
  needExit: boolean,
  deadline: number
): RoomSolution | null {
  const n = level.w * level.h;
  const goalsSorted = room.goals.slice().sort((a, b) => a - b);
  const startBoxes = room.boxes.slice().sort((a, b) => a - b);
  const dead = computeDeadSquares(level, room);
  const hCache = new Map<string, number>();
  const isGoalBoxes = (b: number[]): boolean => {
    if (b.length !== goalsSorted.length) return false;
    for (let i = 0; i < b.length; i++) if (b[i] !== goalsSorted[i]) return false;
    return true;
  };
  const canExit = (playerPos: number, boxes: number[]): boolean => {
    const blocked = new Set<number>(otherBoxes);
    for (const b of boxes) blocked.add(b);
    const reach = bfsReach(level, playerPos, blocked);
    for (const hc of hallway) if (reach.prev[hc] !== -1) return true;
    return false;
  };
  if (isGoalBoxes(startBoxes)) {
    if (needExit && !canExit(startPlayer, startBoxes)) return null;
    return { moves: "", finalBoxes: startBoxes, endPlayer: startPlayer };
  }
  const h0 = assignmentHeuristic(startBoxes, goalsSorted, dist, n, hCache);
  if (!Number.isFinite(h0)) return null;
  const nodes: SolverNode[] = [];
  let seq = 0;
  const heap = new NodeHeap(nodes);
  const closed = new Map<string, number>();
  const startBlocked = new Set<number>(otherBoxes);
  for (const b of startBoxes) startBlocked.add(b);
  const startReach = bfsReach(level, startPlayer, startBlocked);
  const root: SolverNode = {
    boxes: startBoxes,
    player: startPlayer,
    g: 0,
    h: h0,
    parent: -1,
    seg: "",
    seq: seq++,
    regionMin: startReach.minCell,
  };
  nodes.push(root);
  heap.push(0);
  closed.set(startBoxes.join(",") + "|" + startReach.minCell, 0);
  const MAX_EXPANSIONS = 600000;
  let expansions = 0;
  while (heap.size() > 0) {
    const ni = heap.pop();
    const node = nodes[ni];
    const nodeKey = node.boxes.join(",") + "|" + node.regionMin;
    if ((closed.get(nodeKey) as number) < node.g) continue;
    if (isGoalBoxes(node.boxes)) {
      if (!needExit || canExit(node.player, node.boxes)) {
        const parts: string[] = [];
        let cur: SolverNode = node;
        while (cur.parent !== -1) {
          parts.push(cur.seg);
          cur = nodes[cur.parent];
        }
        parts.reverse();
        return { moves: parts.join(""), finalBoxes: node.boxes, endPlayer: node.player };
      }
      // Solved but the player is trapped inside: keep searching for a
      // solution that leaves an exit.
    }
    expansions++;
    if (expansions > MAX_EXPANSIONS) return null;
    if ((expansions & 1023) === 0 && Date.now() > deadline) return null;
    const blocked = new Set<number>(otherBoxes);
    for (const b of node.boxes) blocked.add(b);
    const reach = bfsReach(level, node.player, blocked);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const bx = b % level.w;
      const by = Math.floor(b / level.w);
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di];
        const fx = bx - d.dx;
        const fy = by - d.dy;
        const tx = bx + d.dx;
        const ty = by + d.dy;
        if (fx < 0 || fy < 0 || fx >= level.w || fy >= level.h) continue;
        if (tx < 0 || ty < 0 || tx >= level.w || ty >= level.h) continue;
        const from = fy * level.w + fx;
        const dest = ty * level.w + tx;
        if (reach.prev[from] === -1) continue;
        if (level.wall[dest] || blocked.has(dest)) continue;
        if (!room.cellSet.has(dest)) continue;
        if (dead.has(dest)) continue;
        const nb = node.boxes.slice();
        nb[bi] = dest;
        nb.sort((a, c) => a - c);
        const nbSet = new Set<number>(nb);
        if (formsDeadBlock(level, nbSet, dest)) continue;
        const h = assignmentHeuristic(nb, goalsSorted, dist, n, hCache);
        if (!Number.isFinite(h)) continue;
        const nblocked = new Set<number>(otherBoxes);
        for (const x of nb) nblocked.add(x);
        const nreach = bfsReach(level, b, nblocked);
        const nkey = nb.join(",") + "|" + nreach.minCell;
        const ng = node.g + 1;
        const prevG = closed.get(nkey);
        if (prevG !== undefined && prevG <= ng) continue;
        closed.set(nkey, ng);
        const seg = walkPath(level, reach, node.player, from) + d.ch;
        const nn: SolverNode = {
          boxes: nb,
          player: b,
          g: ng,
          h,
          parent: ni,
          seg,
          seq: seq++,
          regionMin: nreach.minCell,
        };
        nodes.push(nn);
        heap.push(nodes.length - 1);
      }
    }
  }
  return null;
}

// Solve a whole level room by room and stitch the move strings. The
// player carries over between rooms: each room's search starts where
// the previous one left the player, and its first walk already covers
// the hallway transit. Returns "" when any room cannot be solved.
function solveLevel(level: Level, deadline: number): string {
  const dist = computeDistances(level);
  const detected = detectRooms(level);
  let rooms = detected.rooms;
  let hallway = detected.hallway;
  const covered = new Set<number>();
  let valid = rooms.length > 0;
  for (const r of rooms) {
    if (r.boxes.length === 0 || r.boxes.length !== r.goals.length) valid = false;
    for (const b of r.boxes) covered.add(b);
  }
  if (valid && covered.size !== level.boxes.length) valid = false;
  if (!valid) {
    // Fallback: treat the whole level as one room (no decomposition).
    const cells: number[] = [];
    for (let c = 0; c < level.w * level.h; c++) if (!level.wall[c]) cells.push(c);
    rooms = [
      { cells, cellSet: new Set(cells), boxes: level.boxes.slice(), goals: level.goals.slice(), doors: [] },
    ];
    hallway = new Set<number>();
  }
  const globalBoxes = level.boxes.slice();
  let player = level.player;
  let moves = "";
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri];
    const needExit = ri < rooms.length - 1;
    const roomBoxesNow = globalBoxes.filter((b) => room.cellSet.has(b));
    const otherBoxes = globalBoxes.filter((b) => !room.cellSet.has(b));
    const roomNow: Room = {
      cells: room.cells,
      cellSet: room.cellSet,
      boxes: roomBoxesNow,
      goals: room.goals,
      doors: room.doors,
    };
    const sol = solveRoom(level, roomNow, player, otherBoxes, dist, hallway, needExit, deadline);
    if (sol === null) return "";
    moves += sol.moves;
    const nextGlobal: number[] = [];
    for (const b of globalBoxes) if (!room.cellSet.has(b)) nextGlobal.push(b);
    for (const b of sol.finalBoxes) nextGlobal.push(b);
    globalBoxes.length = 0;
    for (const b of nextGlobal) globalBoxes.push(b);
    player = sol.endPlayer;
  }
  return moves;
}

function main(): void {
  const inputDir = process.env.MAKEFUN_INPUT_DIR || "/input";
  const outputDir = process.env.MAKEFUN_OUTPUT_DIR || "/output";
  const started = Date.now();
  const overallDeadline = started + 50000;
  let ids: string[] = [];
  try {
    ids = fs
      .readdirSync(inputDir, { withFileTypes: true })
      .filter((d) => d.isDirectory())
      .map((d) => d.name)
      .sort();
  } catch {
    ids = [];
  }
  for (const id of ids) {
    const levelPath = path.join(inputDir, id, "level.txt");
    const instDeadline = Math.min(overallDeadline, Date.now() + 15000);
    let solution = "";
    try {
      const text = fs.readFileSync(levelPath, "utf8");
      const level = parseLevel(text);
      solution = solveLevel(level, instDeadline);
    } catch {
      solution = "";
    }
    const outDir = path.join(outputDir, id);
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, "solution.txt"), solution);
  }
}

main();
