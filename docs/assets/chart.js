// A small plotting surface: device-pixel-correct sizing, theme-aware colours,
// a primary and a secondary y axis, a shaded band, and a hover crosshair that
// reports every series at the pointed x.

import { palette, alpha } from "./palette.js";

export function surface(canvas, render) {
  const ctx = canvas.getContext("2d");
  const api = {
    ctx,
    width: 0,
    height: 0,
    pointer: { x: 0, y: 0, inside: false },
  };

  function paint() {
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(80, Math.round(rect.width));
    const h = Math.max(80, Math.round(rect.height));
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    api.width = w;
    api.height = h;
    ctx.clearRect(0, 0, w, h);
    render(ctx, w, h, api);
  }

  api.redraw = paint;

  new ResizeObserver(paint).observe(canvas);
  document.addEventListener("themechange", paint);

  canvas.addEventListener("pointermove", (e) => {
    const rect = canvas.getBoundingClientRect();
    api.pointer.x = e.clientX - rect.left;
    api.pointer.y = e.clientY - rect.top;
    api.pointer.inside = true;
    paint();
  });
  canvas.addEventListener("pointerleave", () => {
    api.pointer.inside = false;
    paint();
  });

  paint();
  return api;
}

function niceStep(span, target) {
  const raw = span / target;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  const step = norm > 5 ? 10 : norm > 2 ? 5 : norm > 1 ? 2 : 1;
  return step * mag;
}

function ticks(min, max, count) {
  if (!(max > min)) return [min];
  const step = niceStep(max - min, count);
  const out = [];
  const first = Math.ceil(min / step) * step;
  for (let v = first; v <= max + step * 1e-6; v += step) out.push(v);
  return out;
}

function decimals(step) {
  if (step >= 1) return 0;
  return Math.min(4, Math.max(1, Math.ceil(-Math.log10(step)) + 1));
}

export function lineChart(canvas, opts = {}) {
  // The bottom band holds the tick labels and the axis unit; a figure with a
  // caption under the plot keeps a second band below them for it.
  const pad = { l: 58, r: opts.dual ? 58 : 20, t: 16, b: opts.hintSpace ? 62 : 36 };
  let data = { series: [], band: null, rules: [] };
  let api;

  function bounds(list) {
    let lo = Infinity;
    let hi = -Infinity;
    for (const s of list) {
      for (const [, y] of s.points) {
        if (!Number.isFinite(y)) continue;
        if (y < lo) lo = y;
        if (y > hi) hi = y;
      }
    }
    if (!Number.isFinite(lo)) return [0, 1];
    if (hi === lo) {
      const m = Math.abs(hi) || 1;
      return [lo - m * 0.1, hi + m * 0.1];
    }
    const padY = (hi - lo) * 0.08;
    return [lo - padY, hi + padY];
  }

  function render(ctx, w, h) {
    const c = palette();
    const plot = { x: pad.l, y: pad.t, w: w - pad.l - pad.r, h: h - pad.t - pad.b };
    if (plot.w < 20 || plot.h < 20) return;

    const all = data.series;
    if (!all.length) return;
    const left = all.filter((s) => s.axis !== "r");
    const right = all.filter((s) => s.axis === "r");
    const xs = all[0].points.map((p) => p[0]);
    const xMin = Math.min(...xs);
    const xMax = Math.max(...xs);
    const leftList = left.length ? left.slice() : all.slice();
    if (data.band) {
      leftList.push({ points: data.band.points.map(([x, lo]) => [x, lo]) });
      leftList.push({ points: data.band.points.map(([x, , hi]) => [x, hi]) });
    }
    const [yMin, yMax] = bounds(leftList);
    const [rMin, rMax] = right.length ? bounds(right) : [0, 1];

    const px = (x) => plot.x + ((x - xMin) / (xMax - xMin || 1)) * plot.w;
    const py = (y) => plot.y + plot.h - ((y - yMin) / (yMax - yMin || 1)) * plot.h;
    const ry = (y) => plot.y + plot.h - ((y - rMin) / (rMax - rMin || 1)) * plot.h;

    ctx.font = '500 10px "IBM Plex Mono", ui-monospace, monospace';
    ctx.textBaseline = "middle";

    // horizontal grid
    const yt = ticks(yMin, yMax, 4);
    const dy = decimals(niceStep(yMax - yMin, 4));
    ctx.save();
    for (const v of yt) {
      const y = py(v);
      ctx.strokeStyle = alpha(c.line, 0.85);
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(plot.x, Math.round(y) + 0.5);
      ctx.lineTo(plot.x + plot.w, Math.round(y) + 0.5);
      ctx.stroke();
      ctx.fillStyle = c.ink3;
      ctx.textAlign = "right";
      ctx.fillText(v.toFixed(dy), plot.x - 8, y);
    }
    ctx.restore();

    // secondary axis
    if (right.length) {
      const rt = ticks(rMin, rMax, 3);
      const dr = decimals(niceStep(rMax - rMin, 3));
      ctx.save();
      ctx.fillStyle = alpha(c.ink3, 0.9);
      ctx.textAlign = "left";
      for (const v of rt) ctx.fillText(v.toFixed(dr), plot.x + plot.w + 8, ry(v));
      ctx.restore();
    }

    // vertical grid
    const xt = ticks(xMin, xMax, 6);
    const dx = decimals(niceStep(xMax - xMin, 6));
    ctx.save();
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.fillStyle = c.ink3;
    for (const v of xt) {
      const x = px(v);
      if (x < plot.x - 1 || x > plot.x + plot.w + 1) continue;
      ctx.strokeStyle = alpha(c.line, 0.5);
      ctx.beginPath();
      ctx.moveTo(Math.round(x) + 0.5, plot.y);
      ctx.lineTo(Math.round(x) + 0.5, plot.y + plot.h);
      ctx.stroke();
      ctx.fillText(v.toFixed(dx), x, plot.y + plot.h + 9);
    }
    ctx.restore();

    // axis labels
    ctx.save();
    ctx.fillStyle = c.ink3;
    ctx.font = '500 10px "IBM Plex Mono", ui-monospace, monospace';
    if (opts.xLabel) {
      ctx.textAlign = "right";
      ctx.textBaseline = "top";
      ctx.fillText(opts.xLabel, plot.x + plot.w, plot.y + plot.h + 21);
    }
    // Axis units run along their axis, centred on it, so a long label cannot
    // be clipped by the top or the foot of the canvas.
    if (opts.yLabel) {
      ctx.save();
      ctx.translate(12, plot.y + plot.h / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(opts.yLabel, 0, 0);
      ctx.restore();
    }
    if (opts.y2Label && right.length) {
      ctx.save();
      ctx.translate(w - 8, plot.y + plot.h / 2);
      ctx.rotate(Math.PI / 2);
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(opts.y2Label, 0, 0);
      ctx.restore();
    }
    ctx.restore();

    // band
    if (data.band && data.band.points.length) {
      ctx.beginPath();
      const bp = data.band.points;
      ctx.moveTo(px(bp[0][0]), py(bp[0][1]));
      for (const [x, lo] of bp) ctx.lineTo(px(x), py(lo));
      for (let i = bp.length - 1; i >= 0; i--) {
        ctx.lineTo(px(bp[i][0]), py(bp[i][2]));
      }
      ctx.closePath();
      ctx.fillStyle = alpha(c.amber, 0.14);
      ctx.fill();
    }

    // rules (a reference level such as the setpoint)
    for (const rule of data.rules) {
      const y = py(rule.y);
      ctx.save();
      ctx.strokeStyle = alpha(c.ink3, 0.7);
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(plot.x, y);
      ctx.lineTo(plot.x + plot.w, y);
      ctx.stroke();
      ctx.restore();
    }

    // series
    for (const s of data.series) {
      const map = s.axis === "r" ? ry : py;
      const colour = s.color ? c[s.color] || s.color : c.ink;
      if (s.fill) {
        ctx.beginPath();
        const p0 = s.points[0];
        ctx.moveTo(px(p0[0]), plot.y + plot.h);
        for (const [x, y] of s.points) ctx.lineTo(px(x), map(y));
        ctx.lineTo(px(s.points[s.points.length - 1][0]), plot.y + plot.h);
        ctx.closePath();
        ctx.fillStyle = alpha(colour, 0.09);
        ctx.fill();
      }
      ctx.beginPath();
      ctx.strokeStyle = colour;
      ctx.lineWidth = s.width || 1.7;
      ctx.lineJoin = "round";
      if (s.dash) ctx.setLineDash(s.dash);
      let started = false;
      for (const [x, y] of s.points) {
        const X = px(x);
        const Y = map(y);
        if (!started) {
          ctx.moveTo(X, Y);
          started = true;
        } else ctx.lineTo(X, Y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // crosshair
    if (api.pointer.inside && !opts.static) {
      const x = xMin + ((api.pointer.x - plot.x) / plot.w) * (xMax - xMin);
      const ref = data.series[0];
      let best = 0;
      for (let i = 0; i < ref.points.length; i++) {
        if (Math.abs(ref.points[i][0] - x) < Math.abs(ref.points[best][0] - x)) best = i;
      }
      const at = ref.points[best][0];
      const X = px(at);
      ctx.save();
      ctx.strokeStyle = alpha(c.ink, 0.28);
      ctx.beginPath();
      ctx.moveTo(X, plot.y);
      ctx.lineTo(X, plot.y + plot.h);
      ctx.stroke();

      const rows = data.series.map((s) => ({
        colour: s.color ? c[s.color] || s.color : c.ink,
        name: s.name,
        value: s.points[best]?.[1],
      }));
      const bw = 108;
      const bh = rows.length * 15 + 10;
      let bx = X + 10;
      if (bx + bw > plot.x + plot.w) bx = X - bw - 10;
      const by = plot.y + 6;
      ctx.fillStyle = c.panel;
      ctx.strokeStyle = c.line2;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(bx, by, bw, bh, 2);
      ctx.fill();
      ctx.stroke();
      ctx.font = '500 10px "IBM Plex Mono", ui-monospace, monospace';
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      rows.forEach((row, i) => {
        const y = by + 12 + i * 15;
        ctx.fillStyle = row.colour;
        ctx.fillRect(bx + 8, y - 1, 10, 2);
        ctx.fillStyle = c.ink2;
        ctx.fillText(row.name, bx + 24, y);
        ctx.fillStyle = c.ink;
        ctx.textAlign = "right";
        ctx.fillText(
          Number.isFinite(row.value) ? row.value.toFixed(3) : "—",
          bx + bw - 8,
          y,
        );
        ctx.textAlign = "left";
      });
      ctx.restore();

      for (const s of data.series) {
        const map = s.axis === "r" ? ry : py;
        const v = s.points[best]?.[1];
        if (!Number.isFinite(v)) continue;
        ctx.beginPath();
        ctx.arc(X, map(v), 2.6, 0, Math.PI * 2);
        ctx.fillStyle = s.color ? c[s.color] || s.color : c.ink;
        ctx.fill();
      }
    }
  }

  api = surface(canvas, render);
  return {
    update(next) {
      data = { rules: [], band: null, ...next };
      api.redraw();
    },
    redraw: api.redraw,
  };
}

export function legend(container, items) {
  const c = palette();
  container.innerHTML = "";
  for (const item of items) {
    const span = document.createElement("span");
    const swatch = document.createElement("i");
    swatch.style.background = c[item.color] || item.color;
    if (item.dash) {
      swatch.style.background = "none";
      swatch.style.backgroundImage = `repeating-linear-gradient(90deg, ${c[item.color] || item.color} 0 3px, transparent 3px 6px)`;
    }
    span.append(swatch, item.name);
    container.append(span);
  }
}
