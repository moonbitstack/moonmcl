// The hero stage: a portal machine on a planning grid. The grid, the obstacles,
// the tool path and the velocity profile all come from the compiled library, so
// the motion on screen is the motion the library computes.

import { palette, alpha } from "./palette.js";

const canvas = document.getElementById("cell-canvas");
const stage = canvas?.closest(".stage");

const GRID = { cols: 12, rows: 8, start: 0, goal: 95 };
const BLOCKED = [27, 28, 41, 42, 53, 54, 62, 63, 64];
const PITCH = 1.06;
const V_MAX = 3.4;
const A_MAX = 2.8;
const J_MAX = 10.0;
const DWELL = 1.4;

const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)");

function ask(request) {
  const fn = globalThis.MoonMCL;
  if (typeof fn !== "function") return null;
  const out = fn(request);
  if (!out || out.startsWith("ERR")) return null;
  return out;
}

function plan() {
  const raw = ask(
    `plan;${GRID.rows};${GRID.cols};${BLOCKED.join("_")};${GRID.start};${GRID.goal};0`,
  );
  if (!raw) return null;
  const path = raw.split("\n").filter(Boolean).map(Number);
  if (path.length < 2 || Number.isNaN(path[0])) return null;
  return path;
}

function profile(distance) {
  const raw = ask(`scurve;${distance};${V_MAX};${A_MAX};${J_MAX};480`);
  if (!raw) return null;
  return raw
    .split("\n")
    .filter(Boolean)
    .map((row) => row.split(",").map(Number));
}

async function main() {
  if (!canvas || !stage) return;

  const path = plan();
  if (!path) return; // the static drawing stays

  const distance = (path.length - 1) * PITCH;
  const track = profile(distance);
  if (!track) return;

  let THREE;
  try {
    THREE = await import("three");
  } catch {
    return; // no module, no scene; the fallback drawing stays
  }

  const colors = palette();
  const scene = new THREE.Scene();

  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: true,
    powerPreference: "high-performance",
  });
  renderer.setClearColor(0x000000, 0);

  const camera = new THREE.PerspectiveCamera(37, 1, 0.1, 120);
  const look = new THREE.Vector3(0, 1.05, 0);

  // --- materials ---------------------------------------------------------

  const mat = {
    plate: new THREE.MeshStandardMaterial({
      color: new THREE.Color(colors.panel2),
      metalness: 0.35,
      roughness: 0.78,
    }),
    steel: new THREE.MeshStandardMaterial({
      color: new THREE.Color(colors.ink2),
      metalness: 0.82,
      roughness: 0.42,
    }),
    frame: new THREE.MeshStandardMaterial({
      color: new THREE.Color(colors.panel2),
      metalness: 0.6,
      roughness: 0.55,
    }),
    block: new THREE.MeshStandardMaterial({
      color: new THREE.Color(colors.ink3),
      metalness: 0.25,
      roughness: 0.85,
    }),
    accent: new THREE.MeshStandardMaterial({
      color: new THREE.Color(colors.amber),
      emissive: new THREE.Color(colors.amber),
      emissiveIntensity: 1.6,
      metalness: 0.2,
      roughness: 0.5,
    }),
    hair: new THREE.LineBasicMaterial({
      color: new THREE.Color(colors.line2),
      transparent: true,
      opacity: 0.55,
    }),
    faint: new THREE.LineBasicMaterial({
      color: new THREE.Color(colors.line2),
      transparent: true,
      opacity: 0.22,
    }),
    route: new THREE.LineBasicMaterial({
      color: new THREE.Color(colors.amber),
      transparent: true,
      opacity: 0.28,
    }),
    trace: new THREE.LineBasicMaterial({
      color: new THREE.Color(colors.amber),
      transparent: true,
      opacity: 0.95,
    }),
  };

  // --- the bed and its grid ---------------------------------------------

  const bedW = GRID.cols * PITCH;
  const bedD = GRID.rows * PITCH;

  const plate = new THREE.Mesh(
    new THREE.BoxGeometry(bedW + 1.1, 0.26, bedD + 1.1),
    mat.plate,
  );
  plate.position.y = -0.14;
  scene.add(plate);

  const pts = [];
  const halfW = bedW / 2;
  const halfD = bedD / 2;
  for (let c = 0; c <= GRID.cols; c++) {
    const x = -halfW + c * PITCH;
    pts.push(x, 0.002, -halfD, x, 0.002, halfD);
  }
  for (let r = 0; r <= GRID.rows; r++) {
    const z = -halfD + r * PITCH;
    pts.push(-halfW, 0.002, z, halfW, 0.002, z);
  }
  const gridGeo = new THREE.BufferGeometry();
  gridGeo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  scene.add(new THREE.LineSegments(gridGeo, mat.hair));

  // A wider survey grid under the whole cell, to give the frame some ground.
  const far = [];
  for (let k = -9; k <= 9; k++) {
    far.push(-halfW + k * PITCH * 1.5, -0.02, -14, -halfW + k * PITCH * 1.5, -0.02, 14);
    far.push(-16, -0.02, -halfD + k * PITCH * 1.5, 16, -0.02, -halfD + k * PITCH * 1.5);
  }
  const farGeo = new THREE.BufferGeometry();
  farGeo.setAttribute("position", new THREE.Float32BufferAttribute(far, 3));
  scene.add(new THREE.LineSegments(farGeo, mat.faint));

  // Obstacles: the blocked cells stand up as castings.
  const cellAt = (idx) => [idx % GRID.cols, Math.floor(idx / GRID.cols)];
  const world = (col, row) => [
    -halfW + (col + 0.5) * PITCH,
    -halfD + (row + 0.5) * PITCH,
  ];
  const blockGeo = new THREE.BoxGeometry(PITCH * 0.84, 1, PITCH * 0.84);
  for (const idx of BLOCKED) {
    const [c, r] = cellAt(idx);
    const [x, z] = world(c, r);
    const h = 1.05 + ((idx * 37) % 5) * 0.12;
    const m = new THREE.Mesh(blockGeo, mat.block);
    m.scale.y = h;
    m.position.set(x, h / 2, z);
    scene.add(m);

    const cap = new THREE.Mesh(
      new THREE.BoxGeometry(PITCH * 0.84, 0.012, PITCH * 0.84),
      mat.accent,
    );
    cap.material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(colors.amber),
      emissive: new THREE.Color(colors.amber),
      emissiveIntensity: 0.25,
      transparent: true,
      opacity: 0.35,
      roughness: 0.6,
    });
    cap.position.set(x, h + 0.006, z);
    scene.add(cap);
  }

  // --- the route --------------------------------------------------------

  const nodes = path.map((idx) => {
    const [c, r] = cellAt(idx);
    const [x, z] = world(c, r);
    return new THREE.Vector3(x, 0.035, z);
  });

  const routeGeo = new THREE.BufferGeometry().setFromPoints(nodes);
  scene.add(new THREE.Line(routeGeo, mat.route));

  const traceGeo = new THREE.BufferGeometry();
  const tracePos = new Float32Array((nodes.length + 2) * 3);
  traceGeo.setAttribute("position", new THREE.BufferAttribute(tracePos, 3));
  traceGeo.setDrawRange(0, 0);
  const trace = new THREE.Line(traceGeo, mat.trace);
  scene.add(trace);

  // Goal marker: a ring on the last cell.
  const goalRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.3, 0.012, 8, 40),
    mat.accent,
  );
  goalRing.rotation.x = -Math.PI / 2;
  goalRing.position.copy(nodes[nodes.length - 1]).setY(0.05);
  scene.add(goalRing);

  // Start marker: a small cross.
  const startMark = new THREE.Mesh(
    new THREE.BoxGeometry(0.34, 0.008, 0.06),
    mat.accent,
  );
  startMark.position.copy(nodes[0]).setY(0.05);
  scene.add(startMark);

  // --- the portal -------------------------------------------------------

  const postX = halfW + 0.75;
  const beamY = 3.35;
  const gantry = new THREE.Group();
  scene.add(gantry);

  const postGeo = new THREE.BoxGeometry(0.3, beamY, 0.34);
  const railGeo = new THREE.BoxGeometry(0.16, 0.1, bedD + 2.4);

  for (const sx of [-1, 1]) {
    const rail = new THREE.Mesh(railGeo, mat.steel);
    rail.position.set(sx * postX, 0.05, 0);
    scene.add(rail);
  }

  const posts = [];
  for (const sx of [-1, 1]) {
    const post = new THREE.Mesh(postGeo, mat.frame);
    post.position.set(sx * postX, beamY / 2, 0);
    gantry.add(post);
    posts.push(post);
  }

  const beam = new THREE.Mesh(
    new THREE.BoxGeometry(postX * 2 + 0.3, 0.3, 0.36),
    mat.frame,
  );
  beam.position.set(0, beamY, 0);
  gantry.add(beam);

  const carriage = new THREE.Mesh(
    new THREE.BoxGeometry(0.62, 0.34, 0.52),
    mat.steel,
  );
  carriage.position.set(0, beamY - 0.16, 0);
  gantry.add(carriage);

  const ram = new THREE.Mesh(new THREE.BoxGeometry(0.13, 1, 0.13), mat.steel);
  gantry.add(ram);

  const spindle = new THREE.Mesh(
    new THREE.CylinderGeometry(0.11, 0.055, 0.3, 20),
    mat.steel,
  );
  gantry.add(spindle);

  const tip = new THREE.Mesh(
    new THREE.SphereGeometry(0.045, 12, 10),
    mat.accent,
  );
  scene.add(tip);

  // --- sparks -----------------------------------------------------------

  const SPARKS = 110;
  const sparkPos = new Float32Array(SPARKS * 3);
  const sparkVel = new Float32Array(SPARKS * 3);
  const sparkLife = new Float32Array(SPARKS);
  const sparkGeo = new THREE.BufferGeometry();
  sparkGeo.setAttribute("position", new THREE.BufferAttribute(sparkPos, 3));
  const sparks = new THREE.Points(
    sparkGeo,
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
  sparks.frustumCulled = false;
  scene.add(sparks);

  // --- light ------------------------------------------------------------

  const hemi = new THREE.HemisphereLight(
    new THREE.Color(colors.paper),
    new THREE.Color(colors.ink),
    1.15,
  );
  scene.add(hemi);

  const key = new THREE.DirectionalLight(new THREE.Color(colors.paper), 2.1);
  key.position.set(7, 13, 9);
  scene.add(key);

  const rim = new THREE.DirectionalLight(new THREE.Color(colors.amber2), 0.85);
  rim.position.set(-9, 5, -8);
  scene.add(rim);

  const work = new THREE.PointLight(new THREE.Color(colors.amber), 2.6, 6.5, 2);
  scene.add(work);

  // --- placement --------------------------------------------------------

  function sample(t) {
    // The profile is sampled evenly; find the bracket and interpolate.
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
    return {
      s: a[1] + (b[1] - a[1]) * f,
      v: a[2] + (b[2] - a[2]) * f,
    };
  }

  const total = track[track.length - 1][0];

  function place(t) {
    const { s, v } = sample(t);
    const clamped = Math.min(distance, Math.max(0, s));
    const k = Math.min(nodes.length - 2, Math.floor(clamped / PITCH));
    const f = Math.min(1, Math.max(0, clamped / PITCH - k));
    const from = nodes[k];
    const to = nodes[k + 1];
    const px = from.x + (to.x - from.x) * f;
    const pz = from.z + (to.z - from.z) * f;

    // The portal: the beam rides in z, the carriage in x, the ram reaches down.
    gantry.position.z = pz;
    carriage.position.x = px;
    const toolY = 0.42 - Math.min(0.12, v * 0.02);
    const ramLen = beamY - 0.32 - toolY;
    ram.scale.y = ramLen;
    ram.position.set(px, beamY - 0.32 - ramLen / 2, pz);
    spindle.position.set(px, toolY + 0.2, pz);
    tip.position.set(px, toolY + 0.02, pz);
    work.position.set(px, toolY + 0.28, pz);

    // Advance the trace.
    for (let i = 0; i <= k; i++) {
      tracePos[i * 3] = nodes[i].x;
      tracePos[i * 3 + 1] = nodes[i].y + 0.02;
      tracePos[i * 3 + 2] = nodes[i].z;
    }
    tracePos[(k + 1) * 3] = px;
    tracePos[(k + 1) * 3 + 1] = 0.055;
    tracePos[(k + 1) * 3 + 2] = pz;
    traceGeo.attributes.position.needsUpdate = true;
    traceGeo.setDrawRange(0, k + 2);

    return { s: clamped, v, k };
  }

  // --- readouts ---------------------------------------------------------

  const hud = {
    progress: document.querySelector("[data-hud='progress']"),
    feed: document.querySelector("[data-hud='feed']"),
    segment: document.querySelector("[data-hud='segment']"),
    length: document.querySelector("[data-hud='length']"),
  };

  if (hud.length) hud.length.textContent = `${path.length} cells`;

  let lastHud = 0;
  function readout(t, state) {
    if (t - lastHud < 0.08) return;
    lastHud = t;
    const pct = Math.round((state.s / distance) * 100);
    if (hud.progress) hud.progress.textContent = `${pct}%`;
    if (hud.feed) hud.feed.textContent = `${state.v.toFixed(2)} m/s`;
    if (hud.segment) hud.segment.textContent = `${state.k + 1}/${nodes.length - 1}`;
  }

  // --- frame ------------------------------------------------------------

  function resize() {
    const box = stage.getBoundingClientRect();
    const w = Math.max(240, box.width);
    const h = Math.max(240, box.height);
    const dpr = Math.min(2, devicePixelRatio || 1);
    renderer.setPixelRatio(dpr);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    const narrow = w < 620;
    camera.position.set(
      narrow ? 15 : 11.6,
      narrow ? 12.4 : 10.4,
      narrow ? 17 : 13.4,
    );
    camera.lookAt(look);
    camera.updateProjectionMatrix();
  }

  const pointer = { x: 0, y: 0, tx: 0, ty: 0 };

  if (!reduceMotion.matches) {
    stage.addEventListener("pointermove", (e) => {
      const box = stage.getBoundingClientRect();
      pointer.tx = ((e.clientX - box.left) / box.width - 0.5) * 2;
      pointer.ty = ((e.clientY - box.top) / box.height - 0.5) * 2;
    });
    stage.addEventListener("pointerleave", () => {
      pointer.tx = 0;
      pointer.ty = 0;
    });
  }

  let clock = 0;
  let running = true;
  let raf = 0;
  let prev = performance.now();

  function step(now) {
    const dt = Math.min(0.05, (now - prev) / 1000);
    prev = now;

    if (running && !reduceMotion.matches) clock += dt;
    const cycle = total + DWELL;
    const local = clock % cycle;
    const state = place(Math.min(total, local));
    readout(local, state);

    // sparks
    for (let i = 0; i < SPARKS; i++) {
      if (sparkLife[i] > 0) {
        sparkLife[i] -= dt;
        sparkVel[i * 3 + 1] -= 5.2 * dt;
        sparkPos[i * 3] += sparkVel[i * 3] * dt;
        sparkPos[i * 3 + 1] += sparkVel[i * 3 + 1] * dt;
        sparkPos[i * 3 + 2] += sparkVel[i * 3 + 2] * dt;
        if (sparkPos[i * 3 + 1] < 0.01) sparkLife[i] = 0;
      } else {
        sparkPos[i * 3 + 1] = -99;
      }
    }
    if (state.v > 0.25 && local < total) {
      for (let n = 0; n < 3; n++) {
        const i = Math.floor(Math.random() * SPARKS);
        sparkLife[i] = 0.4 + Math.random() * 0.5;
        sparkPos[i * 3] = tip.position.x + (Math.random() - 0.5) * 0.06;
        sparkPos[i * 3 + 1] = tip.position.y;
        sparkPos[i * 3 + 2] = tip.position.z + (Math.random() - 0.5) * 0.06;
        sparkVel[i * 3] = (Math.random() - 0.5) * 1.5;
        sparkVel[i * 3 + 1] = 0.6 + Math.random() * 1.1;
        sparkVel[i * 3 + 2] = (Math.random() - 0.5) * 1.5;
      }
    }
    sparkGeo.attributes.position.needsUpdate = true;

    // camera drift and pointer parallax
    pointer.x += (pointer.tx - pointer.x) * 0.06;
    pointer.y += (pointer.ty - pointer.y) * 0.06;
    const drift = reduceMotion.matches ? 0 : Math.sin(clock * 0.16) * 0.055;
    const radius = Math.hypot(camera.position.x, camera.position.z);
    const base = Math.atan2(camera.position.x, camera.position.z) + drift * 0.4;
    camera.position.x = Math.sin(base) * radius + pointer.x * 0.55;
    camera.position.z = Math.cos(base) * radius;
    camera.position.y = (stage.clientWidth < 620 ? 12.4 : 10.4) + pointer.y * 0.3;
    camera.lookAt(look);

    renderer.render(scene, camera);
    raf = requestAnimationFrame(step);
  }

  function updateTheme() {
    const next = palette();
    mat.plate.color.set(next.panel2);
    mat.steel.color.set(next.ink2);
    mat.frame.color.set(next.panel2);
    mat.block.color.set(next.ink3);
    mat.accent.color.set(next.amber);
    mat.accent.emissive.set(next.amber);
    mat.hair.color.set(next.line2);
    mat.faint.color.set(next.line2);
    mat.route.color.set(next.amber);
    mat.trace.color.set(next.amber);
    sparks.material.color.set(next.amber2);
    hemi.color.set(next.paper);
    hemi.groundColor.set(next.ink);
    key.color.set(next.paper);
    rim.color.set(next.amber2);
    work.color.set(next.amber);
    renderer.render(scene, camera);
  }

  resize();
  stage.dataset.live = "1";
  place(reduceMotion.matches ? total * 0.58 : 0);
  renderer.render(scene, camera);

  new ResizeObserver(resize).observe(stage);
  document.addEventListener("themechange", updateTheme);

  // Stop the loop while the stage is off screen or the tab is hidden.
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
    raf = requestAnimationFrame(step);
  }

  reduceMotion.addEventListener("change", () => {
    cancelAnimationFrame(raf);
    prev = performance.now();
    if (reduceMotion.matches) {
      place(total * 0.58);
      renderer.render(scene, camera);
    } else {
      raf = requestAnimationFrame(step);
    }
  });
}

main();
