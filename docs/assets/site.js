// Shared behaviour: the theme switch, the page scroll marker, the guide's
// table of contents, and the syntax colouring and copy button on code blocks.

const root = document.documentElement;
const STORE = "moonmcl:theme";

// A reload forgets the choice and returns to the system setting; a link from
// another page keeps it, so the theme follows the reader around the site.
const nav = performance.getEntriesByType?.("navigation")?.[0];
if (nav && nav.type === "reload") {
  try {
    sessionStorage.removeItem(STORE);
  } catch {
    /* storage blocked: the system setting stands */
  }
}

function stored() {
  try {
    return sessionStorage.getItem(STORE);
  } catch {
    return null;
  }
}

function apply(theme) {
  if (theme === "light" || theme === "dark") {
    root.dataset.theme = theme;
  } else {
    delete root.dataset.theme;
  }
  const meta = document.querySelector('meta[name="theme-color"]');
  if (meta) {
    const paper = getComputedStyle(root).getPropertyValue("--paper").trim();
    if (paper) meta.setAttribute("content", paper);
  }
  document.dispatchEvent(new CustomEvent("themechange"));
}

apply(stored());

document.querySelector("[data-theme-toggle]")?.addEventListener("click", () => {
  const now =
    root.dataset.theme ||
    (matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
  const next = now === "dark" ? "light" : "dark";
  try {
    sessionStorage.setItem(STORE, next);
  } catch {
    /* storage blocked: the switch still applies to this page */
  }
  apply(next);
});

// --- page scroll marker ---------------------------------------------------

const markTop = () => root.classList.toggle("at-top", window.scrollY === 0);
addEventListener("scroll", markTop, { passive: true });
markTop();

// --- current page in the bar ---------------------------------------------

const here = location.pathname.split("/").pop() || "index.html";
for (const link of document.querySelectorAll(".nav a[href]")) {
  const target = link.getAttribute("href").split("/").pop();
  if (target === here) link.setAttribute("aria-current", "page");
}

// --- code colouring and copy ---------------------------------------------

const KEYWORDS = new Set([
  "pub", "fn", "let", "mut", "if", "else", "match", "for", "in", "while",
  "return", "struct", "enum", "trait", "impl", "typealias", "type", "const",
  "guard", "is", "true", "false", "using", "extend", "derive", "async", "test",
  "raise", "break", "continue", "self", "abort", "ignore", "extern", "and",
  "not", "try", "catch", "with", "priv", "suberror", "init", "main",
]);

const esc = (s) =>
  s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

function highlight(src) {
  let out = "";
  let i = 0;
  const n = src.length;
  const isWord = (c) => /[A-Za-z_0-9]/.test(c);

  while (i < n) {
    const c = src[i];

    if (c === "/" && src[i + 1] === "/") {
      let j = src.indexOf("\n", i);
      if (j < 0) j = n;
      out += `<span class="tk-c">${esc(src.slice(i, j))}</span>`;
      i = j;
      continue;
    }

    if (c === '"') {
      let j = i + 1;
      while (j < n && src[j] !== '"') {
        if (src[j] === "\\") j++;
        j++;
      }
      j = Math.min(j + 1, n);
      out += `<span class="tk-s">${esc(src.slice(i, j))}</span>`;
      i = j;
      continue;
    }

    if (/[0-9]/.test(c)) {
      let j = i;
      while (j < n && /[0-9._eE+\-]/.test(src[j]) && !(src[j] === "-" && !/[eE]/.test(src[j - 1]))) {
        j++;
      }
      out += `<span class="tk-n">${esc(src.slice(i, j))}</span>`;
      i = j;
      continue;
    }

    if (/[A-Za-z_]/.test(c)) {
      let j = i;
      while (j < n && isWord(src[j])) j++;
      const word = src.slice(i, j);
      if (KEYWORDS.has(word)) {
        out += `<span class="tk-k">${esc(word)}</span>`;
      } else if (/^[A-Z]/.test(word)) {
        out += `<span class="tk-t">${esc(word)}</span>`;
      } else {
        let k = j;
        while (k < n && src[k] === " ") k++;
        out += src[k] === "(" ? `<span class="tk-f">${esc(word)}</span>` : esc(word);
      }
      i = j;
      continue;
    }

    out += esc(c);
    i++;
  }
  return out;
}

for (const pre of document.querySelectorAll("pre[data-lang]")) {
  const code = pre.querySelector("code");
  if (!code) continue;
  const raw = code.textContent;
  code.innerHTML = highlight(raw);

  const box = document.createElement("div");
  box.className = "code-block";
  pre.replaceWith(box);
  box.append(pre);

  const btn = document.createElement("button");
  btn.type = "button";
  btn.className = "copy";
  btn.textContent = "copy";
  btn.setAttribute("aria-label", "Copy this block");
  btn.addEventListener("click", async () => {
    try {
      await navigator.clipboard.writeText(raw);
      btn.textContent = "copied";
    } catch {
      btn.textContent = "select";
    }
    setTimeout(() => (btn.textContent = "copy"), 1400);
  });
  box.append(btn);
}

// --- guide table of contents ---------------------------------------------

const toc = document.querySelector(".guide-nav, .api-nav");
if (toc) {
  const links = [...toc.querySelectorAll('a[href^="#"]')];
  const targets = links
    .map((a) => document.getElementById(a.getAttribute("href").slice(1)))
    .filter(Boolean);

  const setActive = (id) => {
    for (const a of links) {
      a.classList.toggle("active", a.getAttribute("href") === `#${id}`);
    }
  };

  if (targets.length && "IntersectionObserver" in window) {
    const seen = new Map();
    const io = new IntersectionObserver(
      (entries) => {
        for (const e of entries) seen.set(e.target.id, e.isIntersecting);
        const first = targets.find((t) => seen.get(t.id));
        if (first) setActive(first.id);
      },
      { rootMargin: "-20% 0px -70% 0px", threshold: 0 },
    );
    targets.forEach((t) => io.observe(t));
  }
}

// --- reveal on scroll -----------------------------------------------------

const reveals = document.querySelectorAll(".reveal");
if (reveals.length && "IntersectionObserver" in window) {
  const io = new IntersectionObserver(
    (entries) => {
      for (const e of entries) {
        if (e.isIntersecting) {
          e.target.classList.add("in");
          io.unobserve(e.target);
        }
      }
    },
    { rootMargin: "0px 0px -12% 0px" },
  );
  reveals.forEach((el) => io.observe(el));
} else {
  reveals.forEach((el) => el.classList.add("in"));
}
