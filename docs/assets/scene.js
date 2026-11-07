// The hero stage: three machine cells, each working its own planning grid.
//
// The grid, the route, the velocity profile and every tool position come from
// the compiled library, so what moves on screen is what the library computes.
// The cells rotate on a timer and each visit lays out a fresh grid, so the
// stage keeps showing new work without any of it being posed by hand.

import { palette } from "./palette.js";

const PITCH = 1;
const V_MAX = 3.6;
const A_MAX = 3;
const J_MAX = 11;
const DWELL = 1.3;
const MIN_HOLD = 7;

const CELLS = [
  {
    id: "portal",
    label: "portal",
    cols: 12,
    rows: 8,
    seed: 0x51ed,
    camera: { pos: [13.2, 11.6, 14.9], look: [0, 1.15, 0] },
  },
  {
    id: "scara",
    label: "scara",
    cols: 10,
    rows: 8,
    seed: 0x2b9c,
    camera: { pos: [9.4, 10.2, 15.2], look: [0, 1.5, -1.6] },
  },
  {
    id: "arm",
    label: "articulated",
    cols: 9,
    rows: 7,
    seed: 0x7a13,
    camera: { pos: [8.8, 9.8, 14.4], look: [0, 1.9, -1.5] },
  },
  {
    id: "rover",
    label: "rover",
    cols: 7,
    rows: 5,
    seed: 0x1d3f,
    camera: { pos: [5.4, 5.4, 8.0], look: [0, 0.6, -0.4], follow: 0.72 },
  },
];

const canvas = document.getElementById("cell-canvas");
const stage = canvas?.closest(".stage");
const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)");

start();

function start() {
  if (!canvas || !stage) return;

  const note = stage.querySelector("[data-stage-note]");
  const env = stage.querySelector("[data-stage-env]");
  const meter = stage.querySelector("[data-cell-meter]");
  const reticle = stage.querySelector("[data-reticle]");
  const retX = stage.querySelector("[data-ret-x]");
  const retZ = stage.querySelector("[data-ret-z]");
  const chips = [...stage.querySelectorAll("[data-cell]")];

  // --- the library ------------------------------------------------------

  function ask(request) {
    const fn = globalThis.MoonMCL;
    if (typeof fn !== "function") return null;
    const out = fn(request);
    if (!out || out.startsWith("ERR")) return null;
    return out;
  }

  // --- layouts ----------------------------------------------------------

  function rng(seed) {
    let s = seed >>> 0;
    return () => {
      s = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(s ^ (s >>> 15), 1 | s);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  // A start on the left edge, a goal on the right edge, and a handful of
  // clusters in between. Clusters never touch the two end columns, so the
  // ends stay open while the middle fills with castings.
  function layoutOf(cfg, seed) {
    const r = rng(seed);
    const blocked = new Set();
    const clusters = Math.max(3, Math.round((cfg.cols * cfg.rows) / 15));

    for (let k = 0; k < clusters; k++) {
      let c = 2 + Math.floor(r() * (cfg.cols - 4));
      let row = Math.floor(r() * cfg.rows);
      const len = 2 + Math.floor(r() * 5);
      for (let s = 0; s < len; s++) {
        blocked.add(row * cfg.cols + c);
        const turn = r();
        if (turn < 0.34) c = Math.min(cfg.cols - 3, c + 1);
        else if (turn < 0.52) c = Math.max(2, c - 1);
        else row = Math.min(cfg.rows - 1, Math.max(0, row + (r() < 0.5 ? 1 : -1)));
      }
    }

    const start = Math.floor(r() * cfg.rows) * cfg.cols;
    const goal = Math.floor(r() * cfg.rows) * cfg.cols + (cfg.cols - 1);
    blocked.delete(start);
    blocked.delete(goal);
    return { start, goal, blocked: [...blocked].sort((a, b) => a - b) };
  }

  function route(cfg, layout) {
    const raw = ask(
      `plan;${cfg.rows};${cfg.cols};${layout.blocked.join("_")};${layout.start};${layout.goal};0`,
    );
    if (!raw || raw === "-") return null;
    const path = raw.split("\n").filter(Boolean).map(Number);
    if (path.length < 2 || path.some((n) => !Number.isFinite(n))) return null;
    return path;
  }

  // Keep re-seeding until the goal is reachable; a walled-off goal is a fine
  // result for the library but a dull three seconds for the stage.
  function workable(cfg, seed) {
    for (let k = 0; k < 28; k++) {
      const layout = layoutOf(cfg, (seed + k * 7919) >>> 0);
      const path = route(cfg, layout);
      if (path) return { ...layout, path };
    }
    const layout = { start: 0, goal: cfg.cols * cfg.rows - 1, blocked: [] };
    const path = route(cfg, layout);
    return path ? { ...layout, path } : null;
  }

  function profile(distance) {
    const raw = ask(`scurve;${distance};${V_MAX};${A_MAX};${J_MAX};480`);
    if (!raw) return null;
    return raw
      .split("\n")
      .filter(Boolean)
      .map((row) => row.split(",").map(Number));
  }

  // The rover is driven, not posed: the planner picks the route, the pursuit
  // law steers and the drive integrates its own pose. One row per tick.
  function roverTrack(cfg, layout) {
    const raw = ask(
      `rover;${cfg.rows};${cfg.cols};${layout.blocked.join("_")};${layout.start};${layout.goal};0.9;1.5;2.2;0.05;420`,
    );
    if (!raw || raw === "-") return null;
    const rows = raw.split("\n").filter(Boolean).map((r) => r.split(",").map(Number));
    if (rows.length < 4 || rows.some((r) => r.some((v) => !Number.isFinite(v)))) return null;
    return rows;
  }

  // --- small geometry helpers ------------------------------------------

  const box = (THREE, w, h, d, m) => new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m);
  const cyl = (THREE, r1, r2, h, m, seg = 20) =>
    new THREE.Mesh(new THREE.CylinderGeometry(r1, r2, h, seg), m);

  function shadowTexture(THREE) {
    const size = 128;
    const c = document.createElement("canvas");
    c.width = c.height = size;
    const g = c.getContext("2d");
    const grad = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    grad.addColorStop(0, "rgba(0,0,0,0.58)");
    grad.addColorStop(0.5, "rgba(0,0,0,0.26)");
    grad.addColorStop(1, "rgba(0,0,0,0)");
    g.fillStyle = grad;
    g.fillRect(0, 0, size, size);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }

  // A vertical gradient, used as the scene environment: metals need something
  // to reflect, and a two-stop sky keeps the machined surfaces legible without
  // a full studio rig.
  function paintEnv(canvas, colors) {
    const g = canvas.getContext("2d");
    const grad = g.createLinearGradient(0, 0, 0, canvas.height);
    grad.addColorStop(0, colors.paper);
    grad.addColorStop(0.42, colors.panel2);
    grad.addColorStop(0.72, colors.ink3);
    grad.addColorStop(1, colors.ink);
    g.fillStyle = grad;
    g.fillRect(0, 0, canvas.width, canvas.height);
  }

  function environment(THREE, colors) {
    const c = document.createElement("canvas");
    c.width = 8;
    c.height = 128;
    paintEnv(c, colors);
    const tex = new THREE.CanvasTexture(c);
    tex.mapping = THREE.EquirectangularReflectionMapping;
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }

  // --- one cell ---------------------------------------------------------

  function buildCell(THREE, cfg, seed, colors, shadowTex) {
    const layout = workable(cfg, seed);
    if (!layout) return null;

    const poses = cfg.id === "rover" ? roverTrack(cfg, layout) : null;
    if (cfg.id === "rover" && !poses) return null;

    const painted = [];
    const root = new THREE.Group();

    const std = (key, metalness, roughness) => {
      const m = new THREE.MeshStandardMaterial({
        color: new THREE.Color(colors[key]),
        metalness,
        roughness,
      });
      painted.push([m, key, "color"]);
      return m;
    };

    const glow = (key, intensity, opacity = 1) => {
      const m = new THREE.MeshStandardMaterial({
        color: new THREE.Color(colors[key]),
        emissive: new THREE.Color(colors[key]),
        emissiveIntensity: intensity,
        metalness: 0.15,
        roughness: 0.4,
        transparent: opacity < 1,
        opacity,
      });
      painted.push([m, key, "color"], [m, key, "emissive"]);
      return m;
    };

    const line = (key, opacity) => {
      const m = new THREE.LineBasicMaterial({
        color: new THREE.Color(colors[key]),
        transparent: opacity < 1,
        opacity,
      });
      painted.push([m, key, "color"]);
      return m;
    };

    const M = {
      plate: std("line2", 0.5, 0.42),
      rim: std("ink3", 0.5, 0.55),
      steel: std("ink2", 0.85, 0.28),
      frame: std("ink2", 0.35, 0.55),
      block: std("ink3", 0.2, 0.84),
      cap: glow("amber", 0.3, 0.34),
      accent: glow("amber", 1.4),
      beacon: glow("amber", 1.2),
      hair: line("line2", 0.5),
      faint: line("line2", 0.22),
      tape: line("amber", 0.25),
      cable: line("line2", 0.45),
    };

    const shadowMat = new THREE.MeshBasicMaterial({
      map: shadowTex,
      transparent: true,
      opacity: 0.5,
      depthWrite: false,
    });
    shadowTex.userData.shared = true;

    function shade(x, z, w, d, opacity = 0.5, parent = root) {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, d), shadowMat.clone());
      m.material.opacity = opacity;
      m.rotation.x = -Math.PI / 2;
      m.position.set(x, 0.012, z);
      parent.add(m);
    }

    const bedW = cfg.cols * PITCH;
    const bedD = cfg.rows * PITCH;
    const halfW = bedW / 2;
    const halfD = bedD / 2;

    // --- the bed --------------------------------------------------------

    const plate = box(THREE, bedW + 1, 0.24, bedD + 1, M.plate);
    plate.position.y = -0.13;
    root.add(plate);

    const rim = box(THREE, bedW + 1.55, 0.1, bedD + 1.55, M.rim);
    rim.position.y = -0.24;
    root.add(rim);

    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        const foot = cyl(THREE, 0.11, 0.11, 0.22, M.steel, 12);
        foot.position.set(sx * (halfW + 0.55), -0.36, sz * (halfD + 0.55));
        root.add(foot);
        const pad = box(THREE, 0.46, 0.05, 0.46, M.frame);
        pad.position.set(sx * (halfW + 0.42), 0, sz * (halfD + 0.42));
        root.add(pad);
        const bolt = cyl(THREE, 0.05, 0.05, 0.06, M.steel, 6);
        bolt.position.set(sx * (halfW + 0.42), 0.04, sz * (halfD + 0.42));
        root.add(bolt);
      }
    }

    const pts = [];
    for (let c = 0; c <= cfg.cols; c++) {
      const x = -halfW + c * PITCH;
      pts.push(x, 0.003, -halfD, x, 0.003, halfD);
    }
    for (let r = 0; r <= cfg.rows; r++) {
      const z = -halfD + r * PITCH;
      pts.push(-halfW, 0.003, z, halfW, 0.003, z);
    }
    const gridGeo = new THREE.BufferGeometry();
    gridGeo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    root.add(new THREE.LineSegments(gridGeo, M.hair));

    // Survey grid on the floor, and floor tape around the cell.
    const far = [];
    for (let k = -10; k <= 10; k++) {
      const x = -halfW + k * PITCH * 1.6;
      far.push(x, -0.31, -15, x, -0.31, 15);
      const z = -halfD + k * PITCH * 1.6;
      far.push(-17, -0.31, z, 17, -0.31, z);
    }
    const farGeo = new THREE.BufferGeometry();
    farGeo.setAttribute("position", new THREE.Float32BufferAttribute(far, 3));
    root.add(new THREE.LineSegments(farGeo, M.faint));

    const taped = (x0, z0, x1, z1) => {
      const g = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(x0, -0.3, z0),
        new THREE.Vector3(x1, -0.3, z0),
        new THREE.Vector3(x1, -0.3, z1),
        new THREE.Vector3(x0, -0.3, z1),
        new THREE.Vector3(x0, -0.3, z0),
      ]);
      root.add(new THREE.Line(g, M.tape));
    };
    taped(-halfW - 1.05, -halfD - 1.05, halfW + 1.05, halfD + 1.05);

    // Edge scale: a tick per column along the front, per row along the left.
    const ticks = [];
    for (let c = 1; c < cfg.cols; c++) {
      const x = -halfW + c * PITCH;
      ticks.push(x, 0.004, halfD + 0.62, x, 0.004, halfD + 0.78);
    }
    for (let r = 1; r < cfg.rows; r++) {
      const z = -halfD + r * PITCH;
      ticks.push(-halfW - 0.78, 0.004, z, -halfW - 0.62, 0.004, z);
    }
    const tickGeo = new THREE.BufferGeometry();
    tickGeo.setAttribute("position", new THREE.Float32BufferAttribute(ticks, 3));
    root.add(new THREE.LineSegments(tickGeo, M.tape));

    // --- castings -------------------------------------------------------

    const cellOf = (idx) => [idx % cfg.cols, Math.floor(idx / cfg.cols)];
    const worldOf = (col, row) => [
      -halfW + (col + 0.5) * PITCH,
      -halfD + (row + 0.5) * PITCH,
    ];

    const blockGeo = new THREE.BoxGeometry(PITCH * 0.84, 1, PITCH * 0.84);
    for (const idx of layout.blocked) {
      const [c, r] = cellOf(idx);
      const [x, z] = worldOf(c, r);
      const h = 0.95 + ((idx * 37) % 5) * 0.13;
      const m = new THREE.Mesh(blockGeo, M.block);
      m.scale.y = h;
      m.position.set(x, h / 2, z);
      root.add(m);

      const cap = box(THREE, PITCH * 0.84, 0.012, PITCH * 0.84, M.cap);
      cap.position.set(x, h + 0.007, z);
      root.add(cap);

      shade(x, z, PITCH * 1.9, PITCH * 1.9, 0.42);
    }

    // Blanks on cells the route keeps clear, so the tool never cuts through one.
    const routeSet = new Set(layout.path);
    const near = (idx) => {
      const [c, r] = cellOf(idx);
      for (let dc = -1; dc <= 1; dc++) {
        for (let dr = -1; dr <= 1; dr++) {
          const cc = c + dc;
          const rr = r + dr;
          if (cc < 0 || rr < 0 || cc >= cfg.cols || rr >= cfg.rows) continue;
          if (routeSet.has(rr * cfg.cols + cc)) return true;
        }
      }
      return false;
    };
    const blockedSet = new Set(layout.blocked);
    let blanks = 0;
    for (let idx = 0; idx < cfg.cols * cfg.rows && blanks < 3; idx += 3) {
      if (routeSet.has(idx) || blockedSet.has(idx) || near(idx)) continue;
      const [c, r] = cellOf(idx);
      const [x, z] = worldOf(c, r);
      const blank = box(THREE, 0.5, 0.18, 0.5, M.steel);
      blank.position.set(x, 0.09, z);
      root.add(blank);
      shade(x, z, 1.1, 1.1, 0.34);
      blanks++;
    }

    // --- route ----------------------------------------------------------

    const nodes = layout.path.map((idx) => {
      const [c, r] = cellOf(idx);
      const [x, z] = worldOf(c, r);
      return new THREE.Vector3(x, 0.04, z);
    });

    // A path drawn as a ribbon, not a hairline: a one-pixel line disappears on
    // a bed this size, and the cut has to read as material removed.
    function ribbon(path, width, opacity) {
      const n = path.length;
      const pos = new Float32Array(n * 6);
      const idx = [];
      for (let i = 0; i < n; i++) {
        const a = path[Math.max(0, i - 1)];
        const b = path[Math.min(n - 1, i + 1)];
        let dx = b.x - a.x;
        let dz = b.z - a.z;
        const len = Math.hypot(dx, dz) || 1;
        dx /= len;
        dz /= len;
        const nx = -dz * width;
        const nz = dx * width;
        pos[i * 6] = path[i].x + nx;
        pos[i * 6 + 1] = 0.045;
        pos[i * 6 + 2] = path[i].z + nz;
        pos[i * 6 + 3] = path[i].x - nx;
        pos[i * 6 + 4] = 0.045;
        pos[i * 6 + 5] = path[i].z - nz;
      }
      for (let i = 0; i < n - 1; i++) {
        const a = i * 2;
        idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
      geo.setIndex(idx);
      const m = new THREE.MeshBasicMaterial({
        color: new THREE.Color(colors.amber),
        transparent: true,
        opacity,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      painted.push([m, "amber", "color"]);
      root.add(new THREE.Mesh(geo, m));
      return { geo, count: idx.length };
    }

    const planned = ribbon(nodes, 0.03, 0.12);
    const halo = ribbon(nodes, 0.14, 0.2);
    const cut = ribbon(nodes, 0.065, 1);
    planned.geo.setDrawRange(0, planned.count);
    halo.geo.setDrawRange(0, 0);
    cut.geo.setDrawRange(0, 0);

    const goalRing = new THREE.Mesh(new THREE.TorusGeometry(0.3, 0.014, 8, 40), M.accent);
    goalRing.rotation.x = -Math.PI / 2;
    goalRing.position.copy(nodes[nodes.length - 1]).setY(0.05);
    root.add(goalRing);

    const startMark = box(THREE, 0.34, 0.01, 0.07, M.accent);
    startMark.position.copy(nodes[0]).setY(0.05);
    root.add(startMark);
    const startMark2 = startMark.clone();
    startMark2.rotation.y = Math.PI / 2;
    root.add(startMark2);

    // --- the machine ----------------------------------------------------

    const dims = { halfW, halfD, bedW, bedD };
    const machine =
      cfg.id === "portal"
        ? portal(THREE, M, dims, root)
        : cfg.id === "scara"
          ? scara(THREE, M, dims, root, shade)
          : cfg.id === "rover"
            ? rover(THREE, M, dims, root, shade)
            : arm(THREE, M, dims, root, shade);

    // --- motion ---------------------------------------------------------

    const distance = (nodes.length - 1) * PITCH;
    const track = poses
      ? null
      : (profile(distance) ?? [
          [0, 0, 0],
          [1.5, distance, 0],
        ]);
    const total = poses ? poses[poses.length - 1][0] + 0.5 : track[track.length - 1][0];

    function sample(t) {
      const last = track.length - 1;
      let lo = 0;
      let hi = last;
      while (hi - lo > 1) {
        const mid = (lo + hi) >> 1;
        if (track[mid][0] <= t) lo = mid;
        else hi = mid;
      }
      const a = track[lo];
      const b = track[hi];
      const span = b[0] - a[0] || 1;
      const f = Math.min(1, Math.max(0, (t - a[0]) / span));
      return { s: a[1] + (b[1] - a[1]) * f, v: a[2] + (b[2] - a[2]) * f };
    }

    function update(t) {
      let px;
      let pz;
      let speed;

      if (poses) {
        const i = Math.min(poses.length - 1, Math.max(0, Math.floor(t / 0.05)));
        const row = poses[i];
        px = row[1];
        pz = row[2];
        speed = row[4];
        machine.update(px, pz, row[3], row[4], row[5], 0.05);
      } else {
        const { s, v } = sample(t);
        const at = Math.min(distance, Math.max(0, s));
        const k = Math.min(nodes.length - 2, Math.floor(at / PITCH));
        const f = Math.min(1, Math.max(0, at / PITCH - k));
        px = nodes[k].x + (nodes[k + 1].x - nodes[k].x) * f;
        pz = nodes[k].z + (nodes[k + 1].z - nodes[k].z) * f;
        speed = v;
        machine.update(px, pz, v);
      }

      // The cut ends where the tool is: the nearest route node is the one just
      // left behind.
      let near = 0;
      let best = Infinity;
      for (let i = 0; i < nodes.length; i++) {
        const dx = nodes[i].x - px;
        const dz = nodes[i].z - pz;
        const d2 = dx * dx + dz * dz;
        if (d2 < best) {
          best = d2;
          near = i;
        }
      }
      cut.geo.setDrawRange(0, near * 6);
      halo.geo.setDrawRange(0, near * 6);

      goalRing.scale.setScalar(1 + Math.sin(t * 2.4) * 0.06);

      return { s: near * PITCH, v: speed, k: near };
    }

    function repaint(next) {
      for (const [m, key, prop] of painted) m[prop].set(next[key]);
      renderer.render(scene, camera);
    }

    function dispose() {
      const geos = new Set();
      const mats = new Set();
      root.traverse((o) => {
        if (o.geometry) geos.add(o.geometry);
        if (o.material) {
          for (const m of [].concat(o.material)) mats.add(m);
        }
      });
      for (const g of geos) g.dispose();
      for (const m of mats) {
        if (m.map && !m.map.userData.shared) m.map.dispose();
        m.dispose();
      }
    }

    return {
      cfg,
      root,
      nodes,
      total,
      distance,
      update,
      repaint,
      dispose,
      tip: machine.tip,
      hold: Math.min(32, Math.max(MIN_HOLD, total + (poses ? 2.6 : DWELL + 2.8))),
    };
  }

  // --- machines ---------------------------------------------------------

  function portal(THREE, M, d, root) {
    const postX = d.halfW + 0.85;
    const beamY = 3.5;
    const span = d.bedD + 2.4;

    for (const sx of [-1, 1]) {
      const rail = box(THREE, 0.22, 0.15, span, M.steel);
      rail.position.set(sx * postX, 0.075, 0);
      root.add(rail);
      for (const sz of [-1, 1]) {
        const stop = box(THREE, 0.34, 0.26, 0.16, M.frame);
        stop.position.set(sx * postX, 0.12, (sz * span) / 2);
        root.add(stop);
      }
    }

    const group = new THREE.Group();
    root.add(group);

    for (const sx of [-1, 1]) {
      const foot = box(THREE, 0.52, 0.16, 0.52, M.steel);
      foot.position.set(sx * postX, 0.1, 0);
      group.add(foot);
      const post = box(THREE, 0.32, beamY, 0.4, M.frame);
      post.position.set(sx * postX, beamY / 2, 0);
      group.add(post);
      // A dark web down each post, for depth.
      const web = box(THREE, 0.1, beamY - 0.5, 0.09, M.steel);
      web.position.set(sx * (postX - 0.19), beamY / 2, 0.16);
      group.add(web);
    }

    const beam = box(THREE, postX * 2 + 0.4, 0.36, 0.44, M.frame);
    beam.position.set(0, beamY, 0);
    group.add(beam);

    const strip = box(THREE, postX * 2 - 0.2, 0.02, 0.02, M.accent);
    strip.position.set(0, beamY + 0.19, 0.23);
    group.add(strip);

    const carriage = box(THREE, 0.7, 0.42, 0.6, M.steel);
    carriage.position.set(0, beamY - 0.21, 0);
    group.add(carriage);

    const status = box(THREE, 0.16, 0.06, 0.02, M.beacon);
    status.position.set(0.2, beamY - 0.15, 0.31);
    carriage.add(status);

    const ram = box(THREE, 0.13, 1, 0.13, M.steel);
    group.add(ram);

    const spindle = cyl(THREE, 0.11, 0.055, 0.32, M.steel, 20);
    group.add(spindle);

    const tip = new THREE.Object3D();
    group.add(tip);

    // Beacon on the far post.
    const lamp = cyl(THREE, 0.085, 0.085, 0.2, M.frame, 12);
    lamp.position.set(-postX, beamY + 0.28, 0);
    group.add(lamp);
    const lens = new THREE.Mesh(new THREE.SphereGeometry(0.085, 12, 10), M.beacon);
    lens.position.set(-postX, beamY + 0.42, 0);
    group.add(lens);

    // The energy chain, a line from the beam's fixed anchor to the carriage.
    const CABLE = 12;
    const cablePos = new Float32Array(CABLE * 3);
    const cableGeo = new THREE.BufferGeometry();
    cableGeo.setAttribute("position", new THREE.BufferAttribute(cablePos, 3));
    const cable = new THREE.Line(cableGeo, M.cable);
    group.add(cable);
    const anchor = new THREE.Vector3(-postX + 0.3, beamY + 0.22, 0);

    function update(x, z, v) {
      group.position.z = z;
      carriage.position.x = x;

      const toolY = 0.36 - Math.min(0.1, v * 0.016);
      const top = beamY - 0.42;
      const len = top - toolY - 0.26;
      ram.scale.y = len;
      ram.position.set(x, top - len / 2, 0);
      spindle.position.set(x, toolY + 0.2, 0);
      tip.position.set(x, toolY, 0);

      for (let i = 0; i < CABLE; i++) {
        const f = i / (CABLE - 1);
        cablePos[i * 3] = anchor.x + (x - anchor.x) * f;
        cablePos[i * 3 + 1] = anchor.y - Math.sin(Math.PI * f) * 0.5;
        cablePos[i * 3 + 2] = 0;
      }
      cableGeo.attributes.position.needsUpdate = true;
    }

    return { update, tip };
  }

  function scara(THREE, M, d, root, shade) {
    const baseZ = -d.halfD - 0.62;
    const planeY = 2.35;
    const need = Math.hypot(d.halfW, d.halfD - baseZ) + 0.3;
    const L1 = need * 0.56;
    const L2 = need * 0.5;

    const plate = box(THREE, 1.9, 0.18, 1.9, M.rim);
    plate.position.set(0, 0.09, baseZ);
    root.add(plate);
    const column = cyl(THREE, 0.54, 0.66, 2.3, M.frame, 24);
    column.position.set(0, 1.15, baseZ);
    root.add(column);
    const collar = cyl(THREE, 0.72, 0.72, 0.16, M.steel, 24);
    collar.position.set(0, 2.28, baseZ);
    root.add(collar);
    const lamp = new THREE.Mesh(new THREE.SphereGeometry(0.06, 10, 8), M.beacon);
    lamp.position.set(0, 1.9, baseZ + 0.56);
    root.add(lamp);
    shade(0, baseZ, 3, 3, 0.5);

    const group = new THREE.Group();
    group.position.set(0, planeY, baseZ);
    root.add(group);

    const shoulder = cyl(THREE, 0.4, 0.4, 0.52, M.steel, 20);
    group.add(shoulder);
    const shoulderHouse = box(THREE, 0.68, 0.46, 0.68, M.frame);
    group.add(shoulderHouse);

    const link1 = box(THREE, L1 - 0.5, 0.38, 0.54, M.frame);
    link1.position.set((L1 - 0.5) / 2 + 0.25, 0, 0);
    group.add(link1);
    const rib1 = box(THREE, L1 - 1, 0.08, 0.34, M.steel);
    rib1.position.set((L1 - 0.5) / 2 + 0.25, 0.22, 0);
    group.add(rib1);
    const duct = box(THREE, L1 - 1.1, 0.07, 0.14, M.steel);
    duct.position.set((L1 - 0.5) / 2 + 0.25, -0.23, 0.18);
    group.add(duct);

    const stripe1 = box(THREE, L1 - 1.9, 0.05, 0.02, M.accent);
    stripe1.position.set((L1 - 0.5) / 2 + 0.25, 0.09, 0.28);
    group.add(stripe1);

    // Counterweight behind the shoulder, the way a SCARA carries its motor.
    const weight = box(THREE, 0.82, 0.4, 0.52, M.steel);
    weight.position.set(-0.72, 0, 0);
    group.add(weight);

    const elbow = new THREE.Group();
    elbow.position.set(L1, 0, 0);
    group.add(elbow);
    const elbowHouse = cyl(THREE, 0.32, 0.32, 0.5, M.steel, 18);
    elbow.add(elbowHouse);

    const link2 = box(THREE, L2 - 0.4, 0.32, 0.44, M.frame);
    link2.position.set((L2 - 0.4) / 2 + 0.2, 0, 0);
    elbow.add(link2);
    const rib2 = box(THREE, L2 - 1, 0.07, 0.28, M.steel);
    rib2.position.set((L2 - 0.4) / 2 + 0.2, 0.19, 0);
    elbow.add(rib2);

    const wrist = cyl(THREE, 0.24, 0.24, 0.4, M.steel, 18);
    wrist.position.set(L2, 0, 0);
    elbow.add(wrist);

    const quillLen = planeY - 0.32;
    const quill = cyl(THREE, 0.1, 0.1, quillLen, M.steel, 14);
    quill.position.set(L2, -quillLen / 2, 0);
    elbow.add(quill);

    const head = box(THREE, 0.34, 0.16, 0.34, M.steel);
    head.position.set(L2, -quillLen + 0.04, 0);
    elbow.add(head);
    for (const sx of [-1, 1]) {
      const finger = box(THREE, 0.05, 0.18, 0.13, M.steel);
      finger.position.set(L2 + sx * 0.11, -quillLen - 0.06, 0);
      elbow.add(finger);
    }

    const tip = new THREE.Object3D();
    tip.position.set(L2, -quillLen - 0.02, 0);
    elbow.add(tip);

    function update(x, z) {
      const dx = x;
      const dz = z - baseZ;
      const raw = Math.hypot(dx, dz);
      const reach = L1 + L2 - 0.05;
      const floor = Math.abs(L1 - L2) + 0.05;
      const dist = Math.min(reach, Math.max(floor, raw));
      const cosE = (dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
      const e = Math.acos(Math.min(1, Math.max(-1, cosE)));

      // Two solutions mirror about the base-to-target line; take the one whose
      // elbow sits further behind the bed, which is how a SCARA is built.
      const pick = (s) => {
        const t1 =
          Math.atan2(dz, dx) - Math.atan2(L2 * Math.sin(s * e), L1 + L2 * Math.cos(s * e));
        const ez = baseZ + Math.sin(t1) * L1;
        return { t1, s, ez };
      };
      const a = pick(1);
      const b = pick(-1);
      const chosen = a.ez < b.ez ? a : b;

      group.rotation.y = -chosen.t1;
      elbow.rotation.y = -chosen.s * e;
    }

    return { update, tip };
  }

  function arm(THREE, M, d, root, shade) {
    const baseZ = -d.halfD - 0.72;
    const shoulderY = 2.05;
    const toolY = 0.34;
    const wristDown = 0.62;
    const need = Math.hypot(d.halfW, d.halfD - baseZ) + 0.25;
    const L1 = need * 0.54;
    const L2 = need * 0.47;

    const plate = box(THREE, 1.9, 0.18, 1.9, M.rim);
    plate.position.set(0, 0.09, baseZ);
    root.add(plate);
    const plinth = cyl(THREE, 0.64, 0.84, 1.25, M.frame, 24);
    plinth.position.set(0, 0.7, baseZ);
    root.add(plinth);
    const waist = cyl(THREE, 0.6, 0.6, 0.52, M.steel, 24);
    waist.position.set(0, 1.5, baseZ);
    root.add(waist);
    const lamp = new THREE.Mesh(new THREE.SphereGeometry(0.06, 10, 8), M.beacon);
    lamp.position.set(0, 1.1, baseZ + 0.64);
    root.add(lamp);
    shade(0, baseZ, 3.2, 3.2, 0.5);

    const yaw = new THREE.Group();
    yaw.position.set(0, shoulderY, baseZ);
    root.add(yaw);

    const shoulder = cyl(THREE, 0.4, 0.4, 0.7, M.steel, 20);
    shoulder.rotation.x = Math.PI / 2;
    yaw.add(shoulder);

    const upper = new THREE.Group();
    yaw.add(upper);

    const link1 = box(THREE, L1 - 0.3, 0.42, 0.5, M.frame);
    link1.position.set((L1 - 0.3) / 2 + 0.15, 0, 0);
    upper.add(link1);
    const rib1 = box(THREE, L1 - 0.9, 0.08, 0.34, M.steel);
    rib1.position.set((L1 - 0.3) / 2 + 0.15, 0.25, 0);
    upper.add(rib1);
    const duct1 = box(THREE, L1 - 1, 0.08, 0.16, M.steel);
    duct1.position.set((L1 - 0.3) / 2 + 0.15, -0.26, 0.16);
    upper.add(duct1);
    const motor = box(THREE, 0.5, 0.34, 0.42, M.steel);
    motor.position.set(-0.5, 0, 0);
    upper.add(motor);

    const stripe1 = box(THREE, L1 - 1.7, 0.05, 0.02, M.accent);
    stripe1.position.set((L1 - 0.3) / 2 + 0.15, 0.09, 0.26);
    upper.add(stripe1);

    const forearm = new THREE.Group();
    forearm.position.set(L1, 0, 0);
    upper.add(forearm);
    const elbowHouse = cyl(THREE, 0.29, 0.29, 0.52, M.steel, 18);
    elbowHouse.rotation.x = Math.PI / 2;
    forearm.add(elbowHouse);

    const link2 = box(THREE, L2 - 0.3, 0.34, 0.42, M.frame);
    link2.position.set((L2 - 0.3) / 2 + 0.15, 0, 0);
    forearm.add(link2);
    const rib2 = box(THREE, L2 - 0.9, 0.07, 0.26, M.steel);
    rib2.position.set((L2 - 0.3) / 2 + 0.15, 0.21, 0);
    forearm.add(rib2);
    const stripe2 = box(THREE, L2 - 1.5, 0.05, 0.02, M.accent);
    stripe2.position.set((L2 - 0.3) / 2 + 0.15, 0.07, 0.22);
    forearm.add(stripe2);

    const wrist = new THREE.Group();
    wrist.position.set(L2, 0, 0);
    forearm.add(wrist);
    const wristHouse = cyl(THREE, 0.21, 0.21, 0.36, M.steel, 16);
    wristHouse.rotation.x = Math.PI / 2;
    wrist.add(wristHouse);

    const tool = box(THREE, 0.17, 0.44, 0.17, M.steel);
    tool.position.set(0, -0.24, 0);
    wrist.add(tool);
    const head = box(THREE, 0.3, 0.12, 0.3, M.steel);
    head.position.set(0, -0.49, 0);
    wrist.add(head);
    for (const sx of [-1, 1]) {
      const finger = box(THREE, 0.045, 0.16, 0.11, M.steel);
      finger.position.set(sx * 0.1, -0.61, 0);
      wrist.add(finger);
    }

    const tip = new THREE.Object3D();
    tip.position.set(0, -0.62, 0);
    wrist.add(tip);

    function update(x, z) {
      const dx = x;
      const dz = z - baseZ;
      const r0 = Math.hypot(dx, dz);
      const dy = toolY + wristDown - shoulderY;
      const limit = L1 + L2 - 0.05;
      const r = Math.min(r0, Math.sqrt(Math.max(1e-4, limit * limit - dy * dy)));
      const dist = Math.hypot(r, dy);
      const cosE = (dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
      const e = Math.acos(Math.min(1, Math.max(-1, cosE)));

      // Elbow up: of the two solutions, take the one whose elbow rides higher.
      const pick = (s) => {
        const a2 = s * e;
        const a1 = Math.atan2(dy, r) - Math.atan2(L2 * Math.sin(a2), L1 + L2 * Math.cos(a2));
        return { a1, a2, y: L1 * Math.sin(a1) };
      };
      const a = pick(1);
      const b = pick(-1);
      const chosen = a.y > b.y ? a : b;

      yaw.rotation.y = -Math.atan2(dz, dx);
      upper.rotation.z = chosen.a1;
      forearm.rotation.z = chosen.a2;
      wrist.rotation.z = -(chosen.a1 + chosen.a2);
    }

    return { update, tip };
  }

  function rover(THREE, M, d, root, shade) {
    const group = new THREE.Group();
    group.scale.setScalar(1.25);
    root.add(group);

    const deck = box(THREE, 0.92, 0.15, 0.72, M.frame);
    deck.position.y = 0.4;
    group.add(deck);

    const top = box(THREE, 0.74, 0.05, 0.56, M.frame);
    top.position.y = 0.5;
    group.add(top);

    const load = box(THREE, 0.42, 0.2, 0.42, M.steel);
    load.position.set(-0.1, 0.62, 0);
    group.add(load);

    const skirt = box(THREE, 0.78, 0.16, 0.58, M.steel);
    skirt.position.y = 0.26;
    group.add(skirt);

    for (const sz of [-1, 1]) {
      const stripe = box(THREE, 0.66, 0.035, 0.015, M.accent);
      stripe.position.set(0, 0.44, sz * 0.365);
      group.add(stripe);
    }

    // Wheels: the cylinder is baked onto the lateral axis so it can simply
    // roll about z.
    const tyre = new THREE.CylinderGeometry(0.2, 0.2, 0.13, 20);
    tyre.rotateX(Math.PI / 2);
    const hub = new THREE.CylinderGeometry(0.075, 0.075, 0.15, 12);
    hub.rotateX(Math.PI / 2);
    const wheels = [];
    for (const sz of [-1, 1]) {
      const wheel = new THREE.Mesh(tyre, M.block);
      wheel.position.set(0, 0.2, sz * 0.37);
      group.add(wheel);
      wheels.push(wheel);
      const cap = new THREE.Mesh(hub, M.steel);
      cap.position.copy(wheel.position);
      group.add(cap);
    }

    const caster = new THREE.Mesh(new THREE.SphereGeometry(0.07, 12, 10), M.steel);
    caster.position.set(0.36, 0.09, 0);
    group.add(caster);

    const sensor = box(THREE, 0.16, 0.13, 0.22, M.steel);
    sensor.position.set(0.34, 0.52, 0);
    group.add(sensor);

    const mast = cyl(THREE, 0.028, 0.028, 0.56, M.steel, 10);
    mast.position.set(-0.3, 0.74, 0);
    group.add(mast);

    const lamp = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 10), M.beacon);
    lamp.position.set(-0.3, 1.04, 0);
    group.add(lamp);

    for (const sx of [-1, 1]) {
      const lamp2 = box(THREE, 0.05, 0.05, 0.16, M.beacon);
      lamp2.position.set(0.45, 0.36, sx * 0.2);
      group.add(lamp2);
    }

    const tip = new THREE.Object3D();
    tip.position.set(-0.3, 1.04, 0);
    group.add(tip);

    shade(0, 0, 1.9, 1.5, 0.45, group);

    function update(x, z, theta, v, w, dt) {
      group.position.set(x, 0, z);
      group.rotation.y = -theta;
      const roll = (v * dt) / 0.2;
      for (const wheel of wheels) wheel.rotation.z -= roll;
    }

    return { update, tip };
  }

  // --- the stage --------------------------------------------------------
  let THREE;
  let renderer;
  let scene;
  let camera;
  const look = { x: 0, y: 1, z: 0 };
  const cam = { x: 0, y: 0, z: 0 };
  let camTarget = { pos: [11.8, 10.2, 13.2], look: [0, 1, 0] };

  let cell = null;
  let cellIndex = -1;
  let rev = 0;
  let elapsed = 0;
  let hold = 20;
  let running = true;
  let first = true;

  const sparks = { count: 0, pos: null, vel: null, life: null, geo: null, points: null };
  const tipWorld = { position: null };

  (async () => {
    try {
      THREE = await import("three");
    } catch {
      return; // the static drawing stays
    }
    tipWorld.position = new THREE.Vector3();
    const colors = palette();
    const shadowTex = shadowTexture(THREE);

    scene = new THREE.Scene();
    scene.environment = environment(THREE, colors);
    scene.environmentIntensity = 0.6;
    renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: true,
      powerPreference: "high-performance",
    });
    renderer.setClearColor(0x000000, 0);

    camera = new THREE.PerspectiveCamera(37, 1, 0.1, 140);
    camera.position.set(...camTarget.pos);

    const hemi = new THREE.HemisphereLight(
      new THREE.Color(colors.paper),
      new THREE.Color(colors.ink),
      0.55,
    );
    scene.add(hemi);
    const key = new THREE.DirectionalLight(new THREE.Color(colors.paper), 2.4);
    key.position.set(6, 14, 8);
    scene.add(key);
    const fill = new THREE.DirectionalLight(new THREE.Color(colors.paper), 0.45);
    fill.position.set(-9, 6, 11);
    scene.add(fill);
    const rim = new THREE.DirectionalLight(new THREE.Color(colors.amber2), 0.95);
    rim.position.set(-10, 5, -9);
    scene.add(rim);
    const work = new THREE.PointLight(new THREE.Color(colors.amber), 1.7, 5.4, 2);
    scene.add(work);

    // --- sparks ---------------------------------------------------------

    const SPARKS = innerWidth < 640 ? 56 : 120;
    sparks.count = SPARKS;
    sparks.pos = new Float32Array(SPARKS * 3);
    sparks.vel = new Float32Array(SPARKS * 3);
    sparks.life = new Float32Array(SPARKS);
    sparks.geo = new THREE.BufferGeometry();
    sparks.geo.setAttribute("position", new THREE.BufferAttribute(sparks.pos, 3));
    sparks.points = new THREE.Points(
      sparks.geo,
      new THREE.PointsMaterial({
        color: new THREE.Color(colors.amber2),
        size: 0.055,
        transparent: true,
        opacity: 0.9,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
        sizeAttenuation: true,
      }),
    );
    sparks.points.frustumCulled = false;
    scene.add(sparks.points);

    // --- mounting -------------------------------------------------------

    const params = new URLSearchParams(location.search);
    const query = params.get("cell");
    let startIndex = CELLS.findIndex((c) => c.id === query);
    if (startIndex < 0) startIndex = 0;
    // `?t=0.4` freezes the cell at that fraction of its move, which is how the
    // page is photographed; without it the cell starts at rest.
    const frozen = Number(params.get("t"));
    const frozenPose = Number.isFinite(frozen) && frozen >= 0 && frozen <= 1;

    function chipState() {
      for (const btn of chips) {
        btn.setAttribute(
          "aria-pressed",
          btn.dataset.cell === CELLS[cellIndex].id ? "true" : "false",
        );
      }
    }

    function mount(index, snap = false) {
      const cfg = CELLS[index];
      const built = buildCell(THREE, cfg, (cfg.seed + rev * 7919) >>> 0, palette(), shadowTex);
      if (!built) return;

      if (cell) {
        scene.remove(cell.root);
        cell.dispose();
      }

      cell = built;
      cellIndex = index;
      rev += 1;
      scene.add(cell.root);

      elapsed = 0;
      hold = cell.hold;
      camTarget = cfg.camera;
      if (frozenPose) elapsed = Math.min(1, frozen) * cell.total;

      if (snap) {
        cam.x = cfg.camera.pos[0];
        cam.y = cfg.camera.pos[1];
        cam.z = cfg.camera.pos[2];
        look.x = cfg.camera.look[0];
        look.y = cfg.camera.look[1];
        look.z = cfg.camera.look[2];
      }

      if (note) {
        note.textContent = `cell 0${index + 1} · ${cfg.label} · ${cfg.cols} × ${cfg.rows} · rev ${String(rev).padStart(2, "0")}`;
      }
      if (env) {
        env.textContent = `x ±${(cell.cfg.cols * PITCH / 2).toFixed(2)} · z ±${(cell.cfg.rows * PITCH / 2).toFixed(2)}`;
      }
      chipState();
      placeStatic();
    }

    function placeStatic() {
      if (!cell) return;
      cell.update(cell.total * 0.58);
      cell.tip.getWorldPosition(tipWorld.position);
      renderer.render(scene, camera);
    }

    function advance(dir) {
      mount((cellIndex + dir + CELLS.length) % CELLS.length);
    }

    for (const btn of chips) {
      btn.addEventListener("click", () => {
        const index = CELLS.findIndex((c) => c.id === btn.dataset.cell);
        if (index >= 0 && index !== cellIndex) mount(index);
        else if (index === cellIndex) elapsed = 0;
      });
    }

    // --- frame ----------------------------------------------------------

    function resize() {
      const rect = stage.getBoundingClientRect();
      const w = Math.max(240, rect.width);
      const h = Math.max(240, rect.height);
      const dpr = Math.min(2, devicePixelRatio || 1);
      renderer.setPixelRatio(dpr);
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    }

    const pointer = { x: 0, y: 0, tx: 0, ty: 0 };

    if (!reduceMotion.matches) {
      stage.addEventListener("pointermove", (e) => {
        const rect = stage.getBoundingClientRect();
        pointer.tx = ((e.clientX - rect.left) / rect.width - 0.5) * 2;
        pointer.ty = ((e.clientY - rect.top) / rect.height - 0.5) * 2;
      });
      stage.addEventListener("pointerleave", () => {
        pointer.tx = 0;
        pointer.ty = 0;
      });
    }

    function reticleAt() {
      if (!reticle || !cell) return;
      cell.tip.getWorldPosition(tipWorld.position);
      const v = tipWorld.position.clone().project(camera);
      const w = stage.clientWidth;
      const h = stage.clientHeight;
      const x = Math.min(w - 78, Math.max(34, (v.x * 0.5 + 0.5) * w));
      const y = Math.min(h - 62, Math.max(30, (-v.y * 0.5 + 0.5) * h));
      reticle.style.transform = `translate(${x.toFixed(1)}px, ${y.toFixed(1)}px)`;
      if (retX) retX.textContent = `x ${signed(tipWorld.position.x)}`;
      if (retZ) retZ.textContent = `z ${signed(tipWorld.position.z)}`;
    }

    function signed(v) {
      const s = v.toFixed(2);
      return v >= 0 ? `+${s}` : s.replace("-", "\u2212");
    }

    let prev = performance.now();
    let lastHud = 0;
    let raf = 0;

    const hud = {
      progress: document.querySelector("[data-hud='progress']"),
      feed: document.querySelector("[data-hud='feed']"),
      segment: document.querySelector("[data-hud='segment']"),
      route: document.querySelector("[data-hud='length']"),
    };

    function frame(now) {
      raf = requestAnimationFrame(frame);
      const dt = Math.min(0.05, (now - prev) / 1000);
      prev = now;
      if (!cell) return;

      if (running && !reduceMotion.matches && !frozenPose) {
        elapsed += dt;
        if (elapsed > hold) {
          advance(1);
          return;
        }
      }

      const t = Math.min(cell.total, elapsed);
      const state = cell.update(t);

      // HUD, at about twelve updates a second.
      if (now - lastHud > 80) {
        lastHud = now;
        const pct = Math.round((state.s / cell.distance) * 100);
        if (hud.progress) hud.progress.textContent = `${pct}%`;
        if (hud.feed) hud.feed.textContent = `${state.v.toFixed(2)} m/s`;
        if (hud.segment) hud.segment.textContent = `${state.k + 1}/${cell.nodes.length - 1}`;
        if (hud.route) hud.route.textContent = `${cell.nodes.length} cells`;
        if (meter) meter.style.transform = `scaleX(${(elapsed / hold).toFixed(4)})`;
      }

      // Sparks off the tool, while it is cutting and low enough to strike.
      cell.tip.getWorldPosition(tipWorld.position);
      work.position.copy(tipWorld.position).y += cell.cfg.id === "rover" ? 1.5 : 0.32;

      for (let i = 0; i < sparks.count; i++) {
        if (sparks.life[i] > 0) {
          sparks.life[i] -= dt;
          sparks.vel[i * 3 + 1] -= 5.2 * dt;
          sparks.pos[i * 3] += sparks.vel[i * 3] * dt;
          sparks.pos[i * 3 + 1] += sparks.vel[i * 3 + 1] * dt;
          sparks.pos[i * 3 + 2] += sparks.vel[i * 3 + 2] * dt;
          if (sparks.pos[i * 3 + 1] < 0.01) sparks.life[i] = 0;
        } else {
          sparks.pos[i * 3 + 1] = -99;
        }
      }
      if (state.v > 0.25 && elapsed < cell.total && tipWorld.position.y < 0.6) {
        for (let n = 0; n < 3; n++) {
          const i = Math.floor(Math.random() * sparks.count);
          sparks.life[i] = 0.4 + Math.random() * 0.5;
          sparks.pos[i * 3] = tipWorld.position.x + (Math.random() - 0.5) * 0.07;
          sparks.pos[i * 3 + 1] = tipWorld.position.y;
          sparks.pos[i * 3 + 2] = tipWorld.position.z + (Math.random() - 0.5) * 0.07;
          sparks.vel[i * 3] = (Math.random() - 0.5) * 1.6;
          sparks.vel[i * 3 + 1] = 0.6 + Math.random() * 1.2;
          sparks.vel[i * 3 + 2] = (Math.random() - 0.5) * 1.6;
        }
      }
      sparks.geo.attributes.position.needsUpdate = true;

      // Camera: glide to the cell's angle, then add the pointer's offset. A
      // cell that names `follow` keeps part of its aim on the tool, which is
      // what holds a rover of this size in frame as it crosses the bed.
      const narrow = stage.clientWidth < 620;
      const k = first ? 1 : 0.055;
      const scale = narrow ? 1.2 : 1.06;
      cam.x += (camTarget.pos[0] * scale - cam.x) * k;
      cam.y += (camTarget.pos[1] * scale - cam.y) * k;
      cam.z += (camTarget.pos[2] * scale - cam.z) * k;

      let lx = camTarget.look[0];
      const ly = camTarget.look[1];
      let lz = camTarget.look[2];
      if (camTarget.follow) {
        cell.tip.getWorldPosition(tipWorld.position);
        lx += (tipWorld.position.x - lx) * camTarget.follow;
        lz += (tipWorld.position.z - lz) * camTarget.follow;
      }
      look.x += (lx - look.x) * (first ? 1 : 0.045);
      look.y += (ly - look.y) * (first ? 1 : 0.045);
      look.z += (lz - look.z) * (first ? 1 : 0.045);
      first = false;

      pointer.x += (pointer.tx - pointer.x) * 0.05;
      pointer.y += (pointer.ty - pointer.y) * 0.05;
      const drift = reduceMotion.matches ? 0 : Math.sin(elapsed * 0.17) * 0.05;

      camera.position.set(
        cam.x + pointer.x * 0.7 + drift * 6,
        cam.y - pointer.y * 0.45,
        cam.z - drift * 2.2,
      );
      camera.lookAt(look.x, look.y, look.z);

      renderer.render(scene, camera);
      reticleAt();
    }

    function updateTheme() {
      const next = palette();
      sparkColor(next);
      if (scene.environment) {
        paintEnv(scene.environment.image, next);
        scene.environment.needsUpdate = true;
      }
      hemi.color.set(next.paper);
      hemi.groundColor.set(next.ink);
      key.color.set(next.paper);
      fill.color.set(next.paper);
      rim.color.set(next.amber2);
      work.color.set(next.amber);
      if (cell) cell.repaint(next);
      renderer.render(scene, camera);
    }

    function sparkColor(next) {
      sparks.points.material.color.set(next.amber2);
    }

    resize();
    mount(startIndex, true);
    stage.dataset.live = cell ? "1" : "0";
    renderer.render(scene, camera);

    new ResizeObserver(() => {
      resize();
      if (!running) renderer.render(scene, camera);
    }).observe(stage);
    document.addEventListener("themechange", updateTheme);

    const io = new IntersectionObserver((entries) => {
      running = entries[0].isIntersecting;
      prev = performance.now();
    });
    io.observe(stage);
    document.addEventListener("visibilitychange", () => {
      running = !document.hidden;
      prev = performance.now();
    });

    if (reduceMotion.matches) {
      renderer.render(scene, camera);
    } else {
      raf = requestAnimationFrame(frame);
    }

    reduceMotion.addEventListener("change", () => {
      cancelAnimationFrame(raf);
      prev = performance.now();
      if (reduceMotion.matches) {
        placeStatic();
        if (meter) meter.style.transform = "scaleX(0.58)";
      } else {
        raf = requestAnimationFrame(frame);
      }
    });
  })();
}
