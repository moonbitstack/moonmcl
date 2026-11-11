#!/usr/bin/env python3
"""Regenerate `docs/api.html` and the counters on the home page.

Two sources feed the page. The *surface* - which types, methods and values are
public, and their canonical signatures - comes from each package's
`pkg.generated.mbti`, the file `moon info` writes. The *prose* - the summary of
a type, the note on a field, the meaning of an argument - comes from the `///`
comments in the sources, which the interface file does not carry.

Nothing on the page is typed by hand, so a rerun against an unchanged library
produces an identical file and `git diff --exit-code docs/` stays empty.
"""

from __future__ import annotations

import html
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
SRC = ROOT / "src"
MODULE = "moonbitstack/moonmcl"

PACKAGES = [
    {
        "key": "linear",
        "title": "linear",
        "blurb": "Dense linear algebra on plain arrays: a row-major `Mat`, a flat "
        "`Vec`, and the factorisations the control solvers are built on.",
    },
    {
        "key": "contract",
        "title": "contract",
        "blurb": "The traits every algorithm is written against — `Model`, "
        "`Controller`, `Estimator`, `Trajectory`, `Kinematics`, `Space`, `Validity` "
        "and `Planner` — and the shared records that go with them.",
    },
    {
        "key": "control",
        "title": "control",
        "blurb": "Feedback: the PID family and its combinators, LQR and its integral "
        "form, the iterative LQR, a box-constrained MPC, the two geometric path "
        "followers, computed torque, impedance and a barrier filter.",
    },
    {
        "key": "estimation",
        "title": "estimation",
        "blurb": "State estimation: the linear Kalman filter, its extended and "
        "unscented forms over any plant, a particle filter, the one-dimensional "
        "trackers and the backward smoothing pass.",
    },
    {
        "key": "trajectory",
        "title": "trajectory",
        "blurb": "Single-axis motion: the trapezoid and the jerk-limited profile, "
        "polynomials, a natural cubic spline, a clamped B-spline, and the "
        "synchroniser that runs several axes on one clock.",
    },
    {
        "key": "planning",
        "title": "planning",
        "blurb": "Motion planning: the sampling planners over any state space, the "
        "Dubins and Reeds-Shepp car spaces, the two local planners, and the bridge "
        "to the graph package's grid searches.",
    },
    {
        "key": "kinematics",
        "title": "kinematics",
        "blurb": "Mechanisms: the wheeled bases and their steering, the n-link "
        "planar arm with a damped inverse, the delta cell, the SCARA and the gantry.",
    },
]


# --------------------------------------------------------------------------
# reading the sources
# --------------------------------------------------------------------------


def _strip_doc(line: str) -> str:
    body = line.strip()[3:]
    return body[1:] if body.startswith(" ") else body


def _split_params(text: str) -> list[str]:
    """Split a parameter list on commas that sit outside any bracket."""
    out, depth, cur = [], 0, ""
    for ch in text:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur)
    return [p.strip() for p in out if p.strip()]


PARAM_RE = re.compile(r"^(?:mut\s+)?(\w+)\s*(~)?\s*:\s*(.+?)(?:\s*=\s*(.+))?$", re.S)


def _params(text: str) -> list[dict]:
    params = []
    for raw in _split_params(text):
        m = PARAM_RE.match(raw.replace("\n", " "))
        if not m:
            params.append({"name": raw, "type": "", "label": raw})
            continue
        name, optional, ty, _default = m.group(1), m.group(2), m.group(3), m.group(4)
        if optional:
            params.append({"name": name, "type": ty.strip(), "label": f"{name}? : {ty.strip()}"})
        else:
            params.append({"name": name, "type": ty.strip(), "label": f"{name} : {ty.strip()}"})
    return params


class Surface:
    """Everything the sources say about the public surface of one package."""

    def __init__(self) -> None:
        self.fns: dict[str, dict] = {}
        self.types: dict[str, dict] = {}

    def scan(self, path: Path) -> None:
        lines = path.read_text(encoding="utf-8").splitlines()
        doc: list[str] = []
        i = 0
        while i < len(lines):
            line = lines[i]
            stripped = line.strip()

            if stripped == "///|":
                doc = []
                i += 1
                continue
            if stripped.startswith("///"):
                doc.append(_strip_doc(line))
                i += 1
                continue
            if not stripped:
                doc = []
                i += 1
                continue
            if stripped.startswith("#"):
                i += 1
                continue

            trait = re.match(r"pub(?:\(open\))?\s+trait\s+(\w+)", stripped)
            if trait:
                name = trait.group(1)
                body_lines, i = self._block(lines, i)
                members, member_doc = [], []
                for raw in body_lines:
                    text = raw.strip()
                    if text == "///|":
                        continue
                    if text.startswith("///"):
                        member_doc.append(_strip_doc(raw))
                        continue
                    if text.startswith("fn ") or text.startswith("pub fn "):
                        members.append({"sig": text, "doc": member_doc})
                        member_doc = []
                self.types[name] = {
                    "kind": "trait",
                    "doc": list(doc),
                    "fields": [],
                    "members": members,
                }
                doc = []
                continue

            kind = re.match(r"pub\s+(struct|enum|fn)\b", stripped)
            if kind and kind.group(1) == "struct":
                name = re.match(r"pub\s+struct\s+(\w+)", stripped).group(1)
                body, i = self._block(lines, i)
                self.types[name] = {
                    "kind": "struct",
                    "doc": list(doc),
                    "fields": _fields(body),
                }
                doc = []
                continue
            if kind and kind.group(1) == "fn":
                text, i = self._header(lines, i)
                self._fn(text, doc)
                doc = []
                continue
            if kind and kind.group(1) == "enum":
                name = re.match(r"pub\s+enum\s+(\w+)", stripped).group(1)
                self.types[name] = {"kind": "enum", "doc": list(doc), "fields": []}
                body, i = self._block(lines, i)
                doc = []
                continue
            doc = []
            i += 1

    @staticmethod
    def _block(lines: list[str], start: int) -> tuple[list[str], int]:
        """The lines of a brace block that opens on line `start`."""
        body, i = [], start + 1
        while i < len(lines):
            if lines[i].strip().startswith("}"):
                i += 1
                break
            body.append(lines[i])
            i += 1
        return body, i

    @staticmethod
    def _header(lines: list[str], start: int) -> tuple[str, int]:
        """The text of a `fn` header, up to the brace that opens the body."""
        buf, i = "", start
        while i < len(lines):
            buf += lines[i] + "\n"
            brace = buf.find("{")
            if brace >= 0:
                return buf[:brace], i + 1
            i += 1
        return buf, i

    def _fn(self, header: str, doc: list[str]) -> None:
        m = re.match(r"pub\s+fn\s+([\w:]+)\s*\((.*)\)\s*(?:->\s*(.+))?\s*$", header.strip(), re.S)
        if not m:
            return
        name, raw_params, ret = m.group(1), m.group(2), m.group(3)
        params = _params(raw_params)
        body_doc, param_doc = _doc_parts(doc)
        self.fns[name] = {
            "params": params,
            "ret": (ret or "Unit").strip(),
            "doc": body_doc,
            "param_doc": param_doc,
        }


def _fields(lines: list[str]) -> list[dict]:
    out, comment = [], []
    for line in lines:
        s = line.strip()
        if s.startswith("//"):
            comment.append(s[2:].strip())
            continue
        m = re.match(r"(\w+)\s*:\s*(.+?),?$", s)
        if m:
            out.append({"name": m.group(1), "type": m.group(2), "doc": " ".join(comment)})
        comment = []
    return out


def _doc_parts(doc: list[str]) -> tuple[list[str], dict[str, str]]:
    """Separate `@param` notes from the prose of a doc comment."""
    prose, params = [], {}
    for line in doc:
        m = re.match(r"@param\s+(\w+)\s+(.*)$", line.strip())
        if m:
            params[m.group(1)] = m.group(2).strip()
        else:
            prose.append(line)
    return prose, params


# --------------------------------------------------------------------------
# reading the interface files
# --------------------------------------------------------------------------


def parse_mbti(path: Path) -> dict:
    pkg = {"path": "", "values": [], "types": [], "methods": {}, "aliases": [], "traits": []}
    section = None
    lines = path.read_text(encoding="utf-8").splitlines()
    current = None
    i = 0
    while i < len(lines):
        raw = lines[i]
        line = raw.strip()

        if line.startswith("package "):
            pkg["path"] = line.split('"')[1]
            i += 1
            continue
        if line.startswith("// Values"):
            section = "values"
        elif line.startswith("// Errors"):
            section = "errors"
        elif line.startswith("// Types and methods"):
            section = "types"
        elif line.startswith("// Type aliases"):
            section = "aliases"
        elif line.startswith("// Traits"):
            section = "traits"
        elif re.match(r"pub(?:\(open\))?\s+trait\s+", line):
            name = re.match(r"pub(?:\(open\))?\s+trait\s+(\w+)", line).group(1)
            members, i = [], i + 1
            while i < len(lines) and not lines[i].startswith("}"):
                if lines[i].strip():
                    members.append(lines[i].strip())
                i += 1
            pkg["traits"].append({"kind": "trait", "name": name, "members": members})
            pkg["methods"][name] = []
            i += 1
            continue
        elif line.startswith("pub struct ") or line.startswith("pub enum "):
            kind, name = line.split()[1], line.split()[2].rstrip("{").strip()
            body, i = [], i + 1
            while i < len(lines) and not lines[i].startswith("}"):
                if lines[i].strip():
                    body.append(lines[i].strip())
                i += 1
            current = {
                "kind": kind,
                "name": name,
                "members": body,
                "attrs": pkg.pop("_attr", []),
            }
            pkg["types"].append(current)
            pkg["methods"][name] = []
            i += 1
            continue
        elif line.startswith("#"):
            if section == "types":
                pkg["_attr"] = [line]
            i += 1
            continue
        elif line.startswith("pub fn "):
            name = re.match(r"pub fn\s+([\w:]+)", line).group(1)
            entry = {"name": name, "sig": line}
            if "::" in name:
                if section == "types" and "::" in name:
                    owner = name.split("::")[0]
                    pkg["methods"].setdefault(owner, []).append(entry)
            elif section == "values":
                pkg["values"].append(entry)
            i += 1
            continue
        elif line.startswith("pub type ") or line.startswith("pub typealias "):
            pkg["aliases"].append(line)
        i += 1

    pkg.pop("_attr", None)
    return pkg


# --------------------------------------------------------------------------
# rendering
# --------------------------------------------------------------------------


def inline(text: str) -> str:
    """Escape a run of prose and turn the backtick spans into code."""
    out = html.escape(text, quote=False)
    out = re.sub(r"`([^`]+)`", r"<code>\1</code>", out)
    out = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", out)
    return out


def prose(lines: list[str]) -> str:
    """Render a doc comment body: paragraphs, bullet lists, fenced samples."""
    blocks, cur = [], []
    for line in lines:
        if line.strip():
            cur.append(line)
        elif cur:
            blocks.append(cur)
            cur = []
    if cur:
        blocks.append(cur)

    out = []
    for block in blocks:
        if all(l.strip().startswith("- ") for l in block):
            items = "".join(f"<li>{inline(l.strip()[2:])}</li>" for l in block)
            out.append(f"<ul>{items}</ul>")
        else:
            out.append(f"<p>{inline(' '.join(l.strip() for l in block))}</p>")
    return "\n".join(out)


def signature(name: str, info: dict) -> str:
    head = f"pub fn {name}("
    params = [p["label"] for p in info["params"]]
    tail = f") -> {info['ret']}"
    flat = head + ", ".join(params) + tail
    if len(flat) <= 78:
        return flat
    body = "".join(f"  {p},\n" for p in params)
    return f"{head}\n{body}{tail}"


def render_entry(anchor: str, kind: str, name: str, sig: str, doc: str, extra: str = "") -> str:
    parts = [
        f'<article class="api-entry" id="{anchor}">',
        "  <h3>"
        f'<span class="kind">{kind}</span>'
        f'<span>{html.escape(name)}</span>'
        f'<a href="#{anchor}" aria-label="Link to {html.escape(name)}">#</a>'
        "</h3>",
        f'  <div class="sig">{html.escape(sig)}</div>',
    ]
    if extra:
        parts.append(extra)
    if doc:
        parts.append(f'  <div class="doc">\n{doc}\n  </div>')
    parts.append("</article>")
    return "\n".join(parts)


def render_fields(fields: list[dict]) -> str:
    if not fields:
        return ""
    rows = []
    for f in fields:
        note = f'<span class="fd">{inline(f["doc"])}</span>' if f["doc"] else ""
        rows.append(
            f'    <div><dt>{html.escape(f["name"])}</dt>'
            f'<dd>{html.escape(f["type"])}{note}</dd></div>'
        )
    return '  <dl class="fields">\n' + "\n".join(rows) + "\n  </dl>"


def link_types(text: str, known: dict[str, str]) -> str:
    """Point `Type` spans at the entry that describes them."""

    def swap(m: re.Match) -> str:
        name = m.group(1)
        if name in known:
            return f'<a href="#{known[name]}"><code>{name}</code></a>'
        return m.group(0)

    return re.sub(r"<code>(\w+)</code>", swap, text)


def build(verbose: bool) -> tuple[str, dict[str, int], list[str]]:
    missing: list[str] = []
    packages = []
    known: dict[str, str] = {}

    for spec in PACKAGES:
        pkg_dir = SRC / spec["key"]
        surface = Surface()
        for path in sorted(pkg_dir.glob("*.mbt")):
            if path.name.endswith("_test.mbt"):
                continue
            surface.scan(path)
        mbti = parse_mbti(pkg_dir / "pkg.generated.mbti")
        for t in mbti["types"]:
            known.setdefault(t["name"], f'{spec["key"]}.{t["name"]}')
        for t in mbti["traits"]:
            known.setdefault(t["name"], f'{spec["key"]}.{t["name"]}')
        packages.append({"spec": spec, "surface": surface, "mbti": mbti})

    counts: dict[str, int] = {}
    nav: list[str] = []
    bodies: list[str] = []

    for item in packages:
        spec, surface, mbti = item["spec"], item["surface"], item["mbti"]
        key = spec["key"]
        total = len(mbti["types"]) + len(mbti["values"]) + len(mbti["traits"])
        total += sum(len(v) for v in mbti["methods"].values())
        counts[key] = total

        nav.append(f'<a class="top" href="#{key}">{key}</a>')
        body = [
            f'<section class="api-pkg" id="{key}">',
            "  <header>",
            f"    <h2>{spec['title']}</h2>",
            f'    <div class="path">{html.escape(mbti["path"] or MODULE + "/" + key)}</div>',
            "  </header>",
            f"  <p>{inline(spec['blurb'])}</p>",
        ]

        if mbti["types"]:
            body.append('  <p class="api-group">Types</p>')

        for t in mbti["types"]:
            name = t["name"]
            src = surface.types.get(name, {})
            anchor = f"{key}.{name}"
            doc = prose(src.get("doc", []))
            doc = link_types(doc, known)
            fields = render_fields(src.get("fields", []))

            decl_lines = [f"pub {t['kind']} {name} {{"] + [f"  {m}" for m in t["members"]] + ["}"]
            body.append(
                render_entry(anchor, t["kind"], name, "\n".join(decl_lines), doc, fields)
            )
            nav.append(f'<a href="#{anchor}">{name}</a>')

            for method in mbti["methods"].get(name, []):
                full = method["name"]
                info = surface.fns.get(full)
                if info is None:
                    if full.endswith("::to_repr"):
                        info = {
                            "params": [{"label": "self", "name": "self"}],
                            "ret": "@debug.Repr",
                            "doc": ["The `@debug.Debug` representation of this value."],
                            "param_doc": {},
                        }
                    else:
                        info = {"params": [], "ret": "", "doc": [], "param_doc": {}}
                if not info["doc"]:
                    missing.append(full)
                m_anchor = f"{key}.{full.replace('::', '.')}"
                body.append(
                    render_entry(
                        m_anchor,
                        "fn",
                        full,
                        signature(full, info),
                        link_types(prose(info["doc"]), known),
                    )
                )

        if mbti["traits"]:
            body.append('  <p class="api-group">Traits</p>')
        for t in mbti["traits"]:
            name = t["name"]
            src = surface.types.get(name, {})
            anchor = f"{key}.{name}"
            doc = link_types(prose(src.get("doc", [])), known)
            decl = (
                f"pub(open) trait {name} {{\n"
                + "\n".join(f"  {m}" for m in t["members"])
                + "\n}"
            )
            rows = []
            for member in src.get("members", []):
                note = ""
                if member["doc"]:
                    note = f'<span class="fd">{inline(" ".join(member["doc"]))}</span>'
                signature_line = member["sig"]
                method_name = re.match(r"(?:pub )?fn\s+(\w+)", signature_line)
                label = method_name.group(1) if method_name else signature_line
                rows.append(
                    f'    <div><dt>{html.escape(label)}</dt>'
                    f"<dd>{html.escape(signature_line)}{note}</dd></div>"
                )
            extra = ""
            if rows:
                extra = '  <dl class="fields">\n' + "\n".join(rows) + "\n  </dl>"
            body.append(render_entry(anchor, "trait", name, decl, doc, extra))
            nav.append(f'<a href="#{anchor}">{name}</a>')

        if mbti["values"]:
            body.append('  <p class="api-group">Values</p>')
        for value in mbti["values"]:
            name = value["name"]
            info = surface.fns.get(name) or {"params": [], "ret": "", "doc": [], "param_doc": {}}
            if not info["doc"]:
                missing.append(name)
            anchor = f"{key}.{name}"
            body.append(
                render_entry(
                    anchor,
                    "fn",
                    name,
                    signature(name, info),
                    link_types(prose(info["doc"]), known),
                )
            )
            nav.append(f'<a href="#{anchor}">{name}</a>')

        body.append("</section>")
        bodies.append("\n".join(body))
        if verbose:
            print(f"  {key}: {total} items")

    total = sum(counts.values())
    head = f"""<div class="page-head">
  <p class="tag">API reference</p>
  <h1>Every public item</h1>
  <p class="lede">
    {total} types, methods and values across {len(counts)} packages. The surface is
    read from <code>pkg.generated.mbti</code> and the notes from the source comments;
    the page is generated, so it cannot drift from the code.
  </p>
  <p class="meta">
    Import a package directly, or import <code>{html.escape(MODULE)}</code> for the
    whole surface under one name.
  </p>
</div>"""

    nav_html = (
        '<nav class="api-nav" aria-label="API contents">\n<p class="grp">Packages</p>\n'
        + "\n".join(nav)
        + "\n</nav>"
    )
    page = TEMPLATE.format(
        head=head,
        nav=nav_html,
        body="\n\n".join(bodies),
        total=total,
        packages=len(counts),
    )
    return page, counts, missing


TEMPLATE = """<!doctype html>
<html lang="en-GB">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>API reference — MoonMCL</title>
<meta name="description" content="Generated reference for every public type, trait, method and value in MoonMCL: the contracts, linear algebra, control, estimation, trajectory, planning and kinematics.">
<meta name="theme-color" content="#e8e7e2">
<link rel="icon" href="favicon.svg" type="image/svg+xml">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wght@400;500;600;700&family=IBM+Plex+Mono:wght@400;500&display=swap">
<link rel="stylesheet" href="assets/base.css">
<link rel="stylesheet" href="assets/pages.css">
<script>
  (() => {{
    try {{
      const nav = performance.getEntriesByType("navigation")[0];
      if (nav && nav.type === "reload") sessionStorage.removeItem("moonmcl:theme");
      const t = sessionStorage.getItem("moonmcl:theme");
      if (t === "light" || t === "dark") document.documentElement.dataset.theme = t;
    }} catch {{}}
  }})();
</script>
</head>
<body>

<header class="topbar">
  <a class="brand" href="./">
    <svg viewBox="0 0 32 32" aria-hidden="true"><rect x="1.5" y="1.5" width="29" height="29" rx="1.5" fill="none" stroke="currentColor" stroke-opacity="0.35" stroke-width="1.5"/><path d="M6 24 L6 13 L15 13 L15 7 L26 7" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="square"/><circle cx="26" cy="7" r="2.6" fill="currentColor"/></svg>
    <b>MOONMCL</b>
    <i>motion control</i>
  </a>
  <nav class="nav" aria-label="Primary">
    <a href="./">Home</a>
    <a href="guide.html">Guide</a>
    <a href="showcase.html">Showcase</a>
    <a href="api.html">API</a>
    <a class="repo" href="https://github.com/moonbitstack/moonmcl" aria-label="Repository on GitHub">
      <svg viewBox="0 0 16 16" aria-hidden="true" fill="currentColor"><path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82a7.4 7.4 0 0 1 2-.27c.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.01 8.01 0 0 0 16 8c0-4.42-3.58-8-8-8Z"/></svg>
    </a>
  </nav>
  <button class="theme-toggle" type="button" data-theme-toggle aria-label="Switch between light and dark">
    <svg class="sun" viewBox="0 0 16 16" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.3"><circle cx="8" cy="8" r="3.1"/><path d="M8 1v1.6M8 13.4V15M1 8h1.6M13.4 8H15M3.1 3.1l1.1 1.1M11.8 11.8l1.1 1.1M12.9 3.1l-1.1 1.1M4.2 11.8l-1.1 1.1"/></svg>
    <svg class="moon" viewBox="0 0 16 16" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.3"><path d="M13.2 10.1A5.6 5.6 0 0 1 6.4 2.9a5.9 5.9 0 1 0 6.8 7.2Z"/></svg>
  </button>
</header>

<main class="shell">

{head}

<div class="api">
{nav}
<div class="api-main">

{body}

</div>
</div>

</main>

<footer class="foot">
  <div class="foot-in">
    <div>
      <a class="brand" href="./">
        <svg viewBox="0 0 32 32" aria-hidden="true"><rect x="1.5" y="1.5" width="29" height="29" rx="1.5" fill="none" stroke="currentColor" stroke-opacity="0.35" stroke-width="1.5"/><path d="M6 24 L6 13 L15 13 L15 7 L26 7" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="square"/><circle cx="26" cy="7" r="2.6" fill="currentColor"/></svg>
        <b>MOONMCL</b>
      </a>
      <p>MoonBit Motion Control Library. Apache-2.0.</p>
    </div>
    <div>
      <h3>Documentation</h3>
      <ul>
        <li><a href="guide.html">Guide</a></li>
        <li><a href="showcase.html">Showcase</a></li>
        <li><a href="api.html">API reference</a></li>
      </ul>
    </div>
    <div>
      <h3>Elsewhere</h3>
      <ul>
        <li><a href="https://github.com/moonbitstack/moonmcl">Repository</a></li>
        <li><a href="https://mooncakes.io/docs/moonbitstack/moonmcl">mooncakes</a></li>
        <li><a href="https://www.moonbitlang.com">MoonBit</a></li>
      </ul>
    </div>
  </div>
</footer>

<script type="module" src="assets/site.js"></script>
</body>
</html>
"""


def fill_home(counts: dict[str, int]) -> tuple[bool, int]:
    """Write the generated counters into the home page."""
    path = DOCS / "index.html"
    text = path.read_text(encoding="utf-8")
    total = sum(counts.values())

    text = re.sub(r'(<dd data-stat="modules">)[^<]*(</dd>)', rf"\g<1>{len(counts)}\g<2>", text)
    text = re.sub(r'(<dd data-stat="items">)[^<]*(</dd>)', rf"\g<1>{total}\g<2>", text)
    for key, value in counts.items():
        text = re.sub(
            rf'(data-stat="items-{key}">)[^<]*(<)', rf"\g<1>{value}\g<2>", text
        )

    changed = text != path.read_text(encoding="utf-8")
    if changed:
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
    return changed, total


def main() -> int:
    verbose = "-q" not in sys.argv
    page, counts, missing = build(verbose)

    target = DOCS / "api.html"
    previous = target.read_text(encoding="utf-8") if target.exists() else ""
    if page != previous:
        with open(target, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(page)
    home_changed, total = fill_home(counts)

    print(f"api.html: {total} items in {len(counts)} packages"
          f"{'' if page != previous else ' (unchanged)'}")
    print(f"index.html counters: {'updated' if home_changed else 'unchanged'}")

    if missing:
        print("\nundocumented public items:", file=sys.stderr)
        for name in missing:
            print(f"  {name}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
