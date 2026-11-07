// The showcase: every figure is computed by the library compiled to JS.
//
// A demo is set up once — its surface, its chart, its own state — and then hands
// back an update function that runs on every control change. Nothing in the
// update path allocates a renderer, so the controls stay smooth.

import { lineChart, surface, legend } from "./chart.js";
import { palette, alpha } from "./palette.js";

function ask(request) {
  const fn = globalThis.MoonMCL;
  if (typeof fn !== "function") return null;
  const out = fn(request);
  if (!out || out.startsWith("ERR")) return null;
  return out;
}

function rows(request) {
  const out = ask(request);
  if (!out || out === "-") return [];
  return out
    .split("\n")
    .filter(Boolean)
    .map((line) => line.split(",").map(Number));
}

const f2 = (v) => (Number.isFinite(v) ? v.toFixed(2) : "—");
const f3 = (v) => (Number.isFinite(v) ? v.toFixed(3) : "—");

function bind(name, init) {
  const root = document.querySelector(`[data-demo="${name}"]`);
  if (!root) return;

  const put = (key, text) => {
    const el = root.querySelector(`[data-r="${key}"]`);
    if (el) el.textContent = text;
  };
  const read = () => {
    const p = {};
    for (const el of root.querySelectorAll("input[data-k]")) {
      p[el.dataset.k] = Number(el.value);
      const out = root.querySelector(`[data-out="${el.dataset.k}"]`);
      if (out && out.textContent !== el.value) out.textContent = el.value;
    }
    return p;
  };

  const canvas = root.querySelector("canvas[data-chart]");
  let chart = null;
  if (canvas) {
    try {
      const opts = JSON.parse(canvas.dataset.opts || "{}");
      // A caption under the plot needs a band of its own below the axis.
      if (root.querySelector(".hint")) opts.hintSpace = true;
      chart = lineChart(canvas, opts);
    } catch {
      chart = null;
    }
  }

  let update = null;
  try {
    update = init({ root, put, chart, canvas, legendBox: root.querySelector("[data-legend]") });
  } catch (err) {
    put("status", String(err && err.message ? err.message : err));
    return;
  }

  let queued = false;
  const run = () => {
    if (queued) return;
    queued = true;
    requestAnimationFrame(() => {
      queued = false;
      const p = read();
      if (!globalThis.MoonMCL) {
        put("status", "engine not loaded");
        return;
      }
      try {
        update?.(p);
      } catch (err) {
        put("status", String(err && err.message ? err.message : err));
      }
    });
  };

  for (const el of root.querySelectorAll("input[data-k]")) {
    el.addEventListener("input", run);
  }
  for (const btn of root.querySelectorAll("[data-seg] button")) {
    btn.addEventListener("click", () => {
      for (const other of root.querySelectorAll("[data-seg] button")) {
        other.setAttribute("aria-pressed", String(other === btn));
      }
      run();
    });
  }
  document.addEventListener("themechange", run);

  // First paint with the values the markup ships with.
  const p = read();
  if (!globalThis.MoonMCL) {
    put("status", "engine not loaded");
  } else {
    try {
      update?.(p);
    } catch (err) {
      put("status", String(err && err.message ? err.message : err));
    }
  }
}

// --- 01 trajectory --------------------------------------------------------

bind("scurve", ({ put, chart }) => {
  return (p) => {
    const d = rows(`scurve;${p.distance};${p.vmax};${p.amax};${p.jmax};280`);
    if (!d.length) return;
    chart.update({
      series: [
        { name: "v", color: "amber", points: d.map((r) => [r[0], r[2]]), width: 2 },
        { name: "a", color: "ink2", points: d.map((r) => [r[0], r[3]]), width: 1.2, dash: [4, 3] },
        { name: "s", color: "signal", points: d.map((r) => [r[0], r[1]]), axis: "r", width: 1.4, fill: true },
      ],
      rules: [{ y: p.vmax }],
    });
    put("duration", `${f2(d[d.length - 1][0])} s`);
    put("peakv", `${f2(Math.max(...d.map((r) => r[2])))} m/s`);
    put("peaka", `${f2(Math.max(...d.map((r) => Math.abs(r[3]))))} m/s²`);
    put("reach", `${f2(d[d.length - 1][1])} m`);
  };
});

// --- 02 planning ----------------------------------------------------------

bind("planning", ({ root, put, canvas }) => {
  if (!canvas) return null;

  const COLS = 14;
  const LINES = 9;
  const start = 0;
  const goal = LINES * COLS - 1;
  const walls = new Set([
    3 * COLS + 4, 4 * COLS + 4, 5 * COLS + 4, 5 * COLS + 5, 5 * COLS + 6,
    2 * COLS + 9, 3 * COLS + 9, 4 * COLS + 9,
  ]);
  let hover = -1;
  let path = [];

  const geometry = (w, h) => {
    const size = Math.min((w - 28) / COLS, (h - 28) / LINES);
    return { size, ox: (w - size * COLS) / 2, oy: (h - size * LINES) / 2 };
  };

  const compute = () => {
    const algo = root.querySelector("[data-seg] button[aria-pressed='true']")?.dataset.algo === "dijkstra" ? 1 : 0;
    const raw = ask(`plan;${LINES};${COLS};${[...walls].join("_")};${start};${goal};${algo}`);
    path = raw && raw !== "-" ? raw.split("\n").map(Number) : [];
    put("cells", path.length ? String(path.length) : "none");
    put("steps", path.length ? String(path.length - 1) : "—");
    put("walls", String(walls.size));
    put("algo", algo ? "Dijkstra" : "A*");
  };

  const draw = (ctx, w, h) => {
    const c = palette();
    const { size, ox, oy } = geometry(w, h);
    ctx.save();

    for (let r = 0; r < LINES; r++) {
      for (let col = 0; col < COLS; col++) {
        const idx = r * COLS + col;
        const x = ox + col * size;
        const y = oy + r * size;
        const blocked = walls.has(idx);
        ctx.fillStyle = blocked ? alpha(c.ink, 0.1) : alpha(c.panel2, 0.45);
        ctx.fillRect(x + 1, y + 1, size - 2, size - 2);
        ctx.strokeStyle = alpha(c.line, 0.85);
        ctx.lineWidth = 1;
        ctx.strokeRect(x + 0.5, y + 0.5, size - 1, size - 1);
        if (blocked) {
          ctx.strokeStyle = alpha(c.ink, 0.3);
          ctx.beginPath();
          ctx.moveTo(x + 3, y + 3);
          ctx.lineTo(x + size - 3, y + size - 3);
          ctx.moveTo(x + size - 3, y + 3);
          ctx.lineTo(x + 3, y + size - 3);
          ctx.stroke();
        } else if (idx === hover) {
          ctx.fillStyle = alpha(c.amber, 0.15);
          ctx.fillRect(x + 1, y + 1, size - 2, size - 2);
        }
      }
    }

    if (path.length > 1) {
      ctx.beginPath();
      path.forEach((idx, i) => {
        const x = ox + ((idx % COLS) + 0.5) * size;
        const y = oy + (Math.floor(idx / COLS) + 0.5) * size;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.strokeStyle = alpha(c.amber, 0.22);
      ctx.lineWidth = Math.max(5, size * 0.36);
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.stroke();
      ctx.strokeStyle = c.amber;
      ctx.lineWidth = 1.8;
      ctx.stroke();
    }

    const mark = (idx, colour, label, above) => {
      const x = ox + ((idx % COLS) + 0.5) * size;
      const y = oy + (Math.floor(idx / COLS) + 0.5) * size;
      ctx.beginPath();
      ctx.arc(x, y, Math.max(3, size * 0.17), 0, Math.PI * 2);
      ctx.fillStyle = colour;
      ctx.fill();
      ctx.fillStyle = c.ink3;
      ctx.font = '500 9px "IBM Plex Mono", monospace';
      ctx.textAlign = "center";
      ctx.fillText(label, x, above ? y - size * 0.4 : y + size * 0.55);
    };
    mark(start, c.signal, "start", false);
    mark(goal, c.alarm, "goal", true);
    ctx.restore();
  };

  const surf = surface(canvas, draw);

  const hit = (event) => {
    const rect = canvas.getBoundingClientRect();
    const { size, ox, oy } = geometry(rect.width, rect.height);
    const col = Math.floor((event.clientX - rect.left - ox) / size);
    const row = Math.floor((event.clientY - rect.top - oy) / size);
    if (col < 0 || row < 0 || col >= COLS || row >= LINES) return -1;
    return row * COLS + col;
  };

  canvas.addEventListener("pointermove", (e) => {
    const idx = hit(e);
    if (idx === hover) return;
    hover = idx;
    surf.redraw();
  });
  canvas.addEventListener("pointerleave", () => {
    hover = -1;
    surf.redraw();
  });
  canvas.addEventListener("click", (e) => {
    const idx = hit(e);
    if (idx < 0 || idx === start || idx === goal) return;
    if (walls.has(idx)) walls.delete(idx);
    else walls.add(idx);
    compute();
    surf.redraw();
  });

  root.querySelector("[data-reset]")?.addEventListener("click", () => {
    walls.clear();
    compute();
    surf.redraw();
  });

  return () => {
    compute();
    surf.redraw();
  };
});

// --- 03 PID ---------------------------------------------------------------

bind("pid", ({ put, chart, legendBox }) => {
  return (p) => {
    const d = rows(`pid;${p.kp};${p.ki};${p.kd};6;0.02;400;${p.setpoint};${p.gain};${p.tau}`);
    if (!d.length) return;
    chart.update({
      series: [
        { name: "y", color: "amber", points: d.map((r) => [r[0], r[1]]), width: 2 },
        { name: "u", color: "ink2", points: d.map((r) => [r[0], r[2]]), axis: "r", width: 1.2 },
      ],
      rules: [{ y: p.setpoint }],
    });
    if (legendBox) {
      legend(legendBox, [
        { name: "measurement", color: "amber" },
        { name: "output", color: "ink2" },
        { name: "setpoint", color: "ink3", dash: true },
      ]);
    }

    const target = p.setpoint;
    const band = Math.max(0.02 * Math.abs(target), 1e-3);
    let t10 = NaN;
    let t90 = NaN;
    let peak = 0;
    for (const [t, y] of d) {
      if (Number.isNaN(t10) && y >= 0.1 * target) t10 = t;
      if (Number.isNaN(t90) && y >= 0.9 * target) t90 = t;
      if (y > peak) peak = y;
    }
    const rise = Number.isNaN(t10) || Number.isNaN(t90) ? NaN : t90 - t10;

    let settle = 0;
    for (let i = d.length - 1; i >= 0; i--) {
      if (Math.abs(d[i][1] - target) > band) {
        settle = d[Math.min(d.length - 1, i + 1)][0];
        break;
      }
    }
    const finalErr = Math.abs(d[d.length - 1][1] - target);
    put("rise", Number.isNaN(rise) ? "—" : `${f2(rise)} s`);
    put("overshoot", target > 0 ? `${f2(Math.max(0, (peak - target) / target) * 100)} %` : "—");
    put("settle", `${f2(settle)} s`);
    put("error", f3(finalErr));
    put("peak", f2(Math.max(...d.map((r) => Math.abs(r[2])))));
  };
});

// --- 04 estimation --------------------------------------------------------

bind("kf", ({ put, chart }) => {
  return (p) => {
    const d = rows(`kf;${p.q};${p.r};0.1;160`);
    if (!d.length) return;
    chart.update({
      band: { points: d.map((r) => [r[0], r[3] - 2 * r[4], r[3] + 2 * r[4]]) },
      series: [
        { name: "estimate", color: "amber", points: d.map((r) => [r[0], r[3]]), width: 2 },
        { name: "measurement", color: "ink3", points: d.map((r) => [r[0], r[2]]), width: 1 },
        { name: "truth", color: "signal", points: d.map((r) => [r[0], r[1]]), width: 1.2, dash: [5, 4] },
      ],
    });
    const rms = (key) =>
      Math.sqrt(d.reduce((s, r) => s + (r[key] - r[1]) ** 2, 0) / d.length);
    const eEst = rms(3);
    const eMeas = rms(2);
    put("sd", f3(d[d.length - 1][4]));
    put("rms", f3(eEst));
    put("rmsm", f3(eMeas));
    put("ratio", `${(eMeas / Math.max(eEst, 1e-9)).toFixed(1)}×`);
  };
});

// --- 05 modern control ----------------------------------------------------

bind("modern", ({ put, chart }) => {
  return (p) => {
    const steps = 40;
    const a = rows(`lqr;${p.x0};${p.v0};${steps};${p.q};${p.r}`);
    const m = rows(`mpc;${p.x0};${p.v0};${p.horizon};${steps};${p.q};${p.r}`);
    if (!a.length || !m.length) return;
    chart.update({
      series: [
        { name: "x lqr", color: "signal", points: a.map((r) => [r[0], r[1]]), width: 1.8 },
        { name: "x mpc", color: "amber", points: m.map((r) => [r[0], r[1]]), width: 1.8 },
        { name: "u lqr", color: "signal", points: a.map((r) => [r[0], r[3]]), axis: "r", width: 1, dash: [3, 3] },
        { name: "u mpc", color: "amber", points: m.map((r) => [r[0], r[3]]), axis: "r", width: 1, dash: [3, 3] },
      ],
    });
    const settle = (list) => {
      const band = 0.02 * Math.abs(list[0][1]) || 0.02;
      for (let i = list.length - 1; i >= 0; i--) {
        if (Math.abs(list[i][1]) > band) return list[Math.min(list.length - 1, i + 1)][0];
      }
      return 0;
    };
    put("settleL", `${f2(settle(a))} s`);
    put("settleM", `${f2(settle(m))} s`);
    put("peakL", f2(Math.max(...a.map((r) => Math.abs(r[3])))));
    put("peakM", f2(Math.max(...m.map((r) => Math.abs(r[3])))));
    put("residual", f3(Math.abs(m[m.length - 1][1])));
  };
});

// --- 06 path following ----------------------------------------------------

bind("follow", ({ put, canvas }) => {
  if (!canvas) return null;

  let data = [];

  const draw = (ctx, w, h) => {
    const c = palette();
    if (!data.length) return;
    const radius = 8;
    const pad = 26;
    const size = Math.min(w - pad * 2, h - pad * 2);
    const ox = (w - size) / 2;
    const oy = (h - size) / 2;
    const span = radius * 2.7;
    const s = size / span;
    const X = (x) => ox + size / 2 + x * s;
    const Y = (y) => oy + size / 2 + (y - radius) * s;

    ctx.save();
    ctx.strokeStyle = alpha(c.line, 0.7);
    ctx.lineWidth = 1;
    for (let k = -7; k <= 7; k++) {
      ctx.beginPath();
      ctx.moveTo(X(k * 2), oy);
      ctx.lineTo(X(k * 2), oy + size);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(ox, Y(radius + k * 2));
      ctx.lineTo(ox + size, Y(radius + k * 2));
      ctx.stroke();
    }

    ctx.beginPath();
    ctx.arc(X(0), Y(radius), radius * s, 0, Math.PI * 2);
    ctx.strokeStyle = alpha(c.ink, 0.45);
    ctx.lineWidth = 1.4;
    ctx.setLineDash([5, 5]);
    ctx.stroke();
    ctx.setLineDash([]);

    const trace = (key, colour, width) => {
      ctx.beginPath();
      data.forEach((r, i) => {
        const x = X(r[key]);
        const y = Y(r[key + 1]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.strokeStyle = colour;
      ctx.lineWidth = width;
      ctx.lineJoin = "round";
      ctx.stroke();
    };
    trace(1, c.signal, 1.8);
    trace(4, c.amber, 1.8);

    for (const [key, colour] of [[1, c.signal], [4, c.amber]]) {
      ctx.beginPath();
      ctx.arc(X(data[0][key]), Y(data[0][key + 1]), 3.2, 0, Math.PI * 2);
      ctx.fillStyle = colour;
      ctx.fill();
    }

    ctx.fillStyle = c.ink3;
    ctx.font = '500 9.5px "IBM Plex Mono", monospace';
    ctx.textAlign = "left";
    ctx.fillText("reference path: circle, r = 8 m", ox + 2, oy + 12);
    ctx.restore();
  };

  const surf = surface(canvas, draw);

  return (p) => {
    data = rows(`follow;${p.wheelbase};${p.speed};0.02;700;${p.lookahead}`);
    if (!data.length) return;
    surf.redraw();
    const last = data[data.length - 1];
    put("pp", f3(last[3]));
    put("st", f3(last[6]));
    put("peakpp", f3(Math.max(...data.map((r) => r[3]))));
    put("peakst", f3(Math.max(...data.map((r) => r[6]))));
  };
});

// --- 07 drive -------------------------------------------------------------

bind("drive", ({ put, canvas }) => {
  if (!canvas) return null;

  let data = [];

  const draw = (ctx, w, h) => {
    const c = palette();
    if (!data.length) return;
    const split = h * 0.68;
    const pad = 20;

    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (const r of data) {
      minX = Math.min(minX, r[1]);
      maxX = Math.max(maxX, r[1]);
      minY = Math.min(minY, r[2]);
      maxY = Math.max(maxY, r[2]);
    }
    const spanX = Math.max(maxX - minX, 0.4);
    const spanY = Math.max(maxY - minY, 0.4);
    const scale = Math.min((w - pad * 2) / spanX, (split - pad * 2) / spanY);
    const ox = (w - spanX * scale) / 2 - minX * scale;
    const oy = (split - pad - spanY * scale) / 2 - minY * scale;
    const X = (x) => ox + x * scale;
    const Y = (y) => split - pad - (oy + y * scale) + pad;

    ctx.save();
    ctx.beginPath();
    data.forEach((r, i) => {
      const x = X(r[1]);
      const y = Y(r[2]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = alpha(c.amber, 0.2);
    ctx.lineWidth = 9;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.stroke();
    ctx.strokeStyle = c.amber;
    ctx.lineWidth = 1.6;
    ctx.stroke();

    ctx.strokeStyle = alpha(c.ink, 0.5);
    ctx.lineWidth = 1;
    for (let i = 0; i < data.length; i += 20) {
      const r = data[i];
      const x = X(r[1]);
      const y = Y(r[2]);
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(r[3]) * 8, y - Math.sin(r[3]) * 8);
      ctx.stroke();
    }

    const top = split + 26;
    const bot = h - 18;
    const maxW = Math.max(0.1, ...data.map((r) => Math.max(Math.abs(r[4]), Math.abs(r[5]))));
    const wx = (t) => pad + (t / data[data.length - 1][0]) * (w - pad * 2);
    const wy = (v) => bot - (v / maxW) * (bot - top);
    ctx.strokeStyle = alpha(c.line, 0.9);
    ctx.beginPath();
    ctx.moveTo(pad, wy(0));
    ctx.lineTo(w - pad, wy(0));
    ctx.stroke();
    for (const [key, colour] of [[4, c.signal], [5, c.amber]]) {
      ctx.beginPath();
      data.forEach((r, i) => {
        const x = wx(r[0]);
        const y = wy(r[key]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.strokeStyle = colour;
      ctx.lineWidth = 1.4;
      ctx.stroke();
    }
    ctx.fillStyle = c.ink3;
    ctx.font = '500 9.5px "IBM Plex Mono", monospace';
    ctx.textAlign = "left";
    ctx.fillText("wheel velocities  vl / vr", pad, top - 9);
    ctx.restore();
  };

  const surf = surface(canvas, draw);

  return (p) => {
    data = rows(`drive;${p.track};${p.speed};${p.turn};0.05;220`);
    if (!data.length) return;
    surf.redraw();
    const last = data[data.length - 1];
    put("x", f2(last[1]));
    put("y", f2(last[2]));
    put("theta", `${f2((last[3] * 180) / Math.PI)}°`);
    put("vl", f2(last[4]));
    put("vr", f2(last[5]));
  };
});
