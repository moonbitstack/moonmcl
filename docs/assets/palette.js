// Colour tokens as the browser resolved them. A custom property can hold
// light-dark(), so reading it back gives the authored text, not a colour; a
// probe element set to `color: var(--token)` resolves it to rgb().

const KEYS = {
  paper: "--paper",
  paper2: "--paper-2",
  panel: "--panel",
  panel2: "--panel-2",
  ink: "--ink",
  ink2: "--ink-2",
  ink3: "--ink-3",
  line: "--line",
  line2: "--line-2",
  amber: "--amber",
  amber2: "--amber-2",
  signal: "--signal",
  alarm: "--alarm",
};

let probe = null;

function element() {
  if (!probe) {
    probe = document.createElement("div");
    probe.setAttribute("aria-hidden", "true");
    probe.style.cssText =
      "position:absolute;width:0;height:0;overflow:hidden;visibility:hidden;pointer-events:none";
    document.body.append(probe);
  }
  return probe;
}

export function token(name) {
  const el = element();
  el.style.color = `var(${name})`;
  return getComputedStyle(el).color;
}

export function palette() {
  const out = {};
  for (const [key, name] of Object.entries(KEYS)) out[key] = token(name);
  return out;
}

export function alpha(color, a) {
  const m = color.match(/rgba?\(([^)]+)\)/);
  if (!m) return color;
  const [r, g, b] = m[1].split(/[,\s/]+/).map(Number);
  return `rgba(${r}, ${g}, ${b}, ${a})`;
}
