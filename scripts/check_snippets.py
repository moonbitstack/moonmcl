"""Compile every MoonBit block in README.md, docs/skills/*/SKILL.md and the site pages.

A reader pastes these blocks into a project, so each one has to compile as written. The blocks of
one Markdown file are read in order, the way the page reads: a block may use a name that an earlier
block of the same file defined. A page's blocks are independent instead, because each one is shown
with its own output and has to stand alone. `<!-- nocheck -->` on the line above a block skips it; a
block opened with ```` ```mbt native ```` is checked on the native target only.

Usage: python scripts/check_snippets.py [--moon moon] [file ...]
"""

import argparse
import html
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FENCE = re.compile(r"(<!--\s*nocheck\s*-->[ \t]*\n)?```(mbt|moonbit)([^\n]*)\n(.*?)^```", re.S | re.M)
PAGE = re.compile(r'<pre data-lang="(mbt|moonbit)"[^>]*><code>(.*?)</code></pre>', re.S)
CONFIG = re.compile(r"(?:\s*//[^\n]*\n)*\s*(import \{|options\(|name\s*=|pkgtype\()")
ALIASED = re.compile(r'"([\w/-]+)"\s*@([a-z_][a-z0-9_]*)')
IMPORT = re.compile(r"^\s*import\s*\{[^}]*\}[ \t]*\n?", re.S)
TOP = re.compile(r"^(pub(\([a-z]+\))? |priv |fn |async fn |test |async test |struct |enum |suberror |impl |trait |typealias |type |extern |#[a-z])", re.M)
NAMED = re.compile(r"^(?:pub(?:\([a-z]+\))? )?(?:async )?fn ([A-Za-z_][A-Za-z0-9_]*)", re.M)
CORE = {"array", "bench", "bigint", "buffer", "builtin", "bytes", "char", "cmp", "coverage", "deque", "double", "encoding", "env", "error", "float", "hashmap", "hashset", "immut", "int", "int16", "int64", "json", "list", "math", "option", "queue", "quickcheck", "random", "rational", "ref", "result", "set", "sorted_map", "sorted_set", "strconv", "string", "test", "tuple", "uint", "uint16", "uint64", "unit"}
NESTED = {"utf8": "encoding/utf8", "utf16": "encoding/utf16"}


def module_info():
    text = (ROOT / "moon.mod").read_text(encoding="utf-8")
    name = re.search(r'name\s*=\s*"([^"]+)"', text).group(1)
    source = re.search(r'source\s*=\s*"([^"]+)"', text)
    deps = re.findall(r'"([\w-]+/[\w-]+)@[\d.]+"', text)
    return name, ROOT / (source.group(1) if source else "."), deps


def packages(module, src, deps):
    """Alias to package path: this module's packages first, then what its dependencies publish."""
    found = {}
    for pkg in sorted(src.rglob("moon.pkg")):
        rel = pkg.parent.relative_to(src).as_posix()
        if rel.startswith("snippet_check") or "examples" in rel.split("/") or "internal" in rel.split("/"):
            continue
        path = module if rel == "." else f"{module}/{rel}"
        found.setdefault(path.rsplit("/", 1)[-1], path)
    for dep in deps:
        found.setdefault(dep.rsplit("/", 1)[-1], dep)
        root = ROOT / ".mooncakes" / dep
        if root.is_dir():
            mod = (root / "moon.mod").read_text(encoding="utf-8") if (root / "moon.mod").exists() else ""
            m = re.search(r'source\s*=\s*"([^"]+)"', mod)
            base = root / (m.group(1) if m else ".")
            for pkg in sorted(base.rglob("moon.pkg")):
                rel = pkg.parent.relative_to(base).as_posix()
                if rel != "." and "internal" not in rel.split("/"):
                    found.setdefault(rel.rsplit("/", 1)[-1], f"{dep}/{rel}")
    return found


def is_config(body):
    """True for a block that only shows `moon.pkg` or `moon.mod` contents.

    A block that opens with an import list and then goes on to use it is code, and is checked: the
    imports are derived from the aliases the code uses."""
    if not CONFIG.match(body):
        return False
    rest = re.sub(r"//[^\n]*", "", IMPORT.sub("", body, count=1))
    return not re.search(r"\b(let|fn|assert_\w+|ignore|raise|for|match|if)\b", rest)


def blocks(path):
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".html":
        for m in PAGE.finditer(text):
            body = html.unescape(m.group(2))
            if is_config(body):
                continue
            yield text[: m.start(2)].count("\n") + 1, "native" in m.group(1), IMPORT.sub("", body, count=1)
        return
    for m in FENCE.finditer(text):
        if m.group(1) or "nocheck" in m.group(3):
            continue
        # The contents of a `moon.pkg` or `moon.mod` shown in a block are configuration, not code.
        if is_config(m.group(4)):
            continue
        yield text[: m.start(4)].count("\n") + 1, "native" in m.group(3), IMPORT.sub("", m.group(4), count=1)


def program(items, independent=False):
    """One package for one file and target: top-level blocks as they are, statement blocks each in a
    function that also holds the statement blocks before it, so a name defined earlier is in scope.

    On a page (`independent`) each block stands alone instead: it is shown with its own output, so
    it may not lean on a name a previous block defined."""
    out, spans, earlier = [], [], []
    n = 0
    for line, body in items:
        start = sum(s.count("\n") + 1 for s in out) + 1
        if TOP.search(body):
            chunk = body
        else:
            n += 1
            inner = body if independent else "\n".join(earlier + [body])
            chunk = f"async fn snippet_{n}() -> Unit raise {{\n{inner}\n}}"
            if not independent:
                earlier.append(body)
        out.append(chunk.rstrip("\n") + "\n")
        spans.append((start, start + chunk.count("\n"), line))
    return "\n".join(out), spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--moon", default="moon")
    ap.add_argument("files", nargs="*")
    args = ap.parse_args()
    module, src, deps = module_info()
    if module.endswith("CHANGE-ME"):
        print("the template itself: nothing to check until CHANGE-ME is replaced")
        return 0
    subprocess.run([args.moon, "update"], cwd=ROOT, capture_output=True)
    aliases = packages(module, src, deps)
    files = [Path(f) for f in args.files] or [
        ROOT / "README.md",
        *sorted((ROOT / "docs" / "skills").glob("*/SKILL.md")),
        *(ROOT / "docs").glob("*.html"),
    ]
    failed = 0
    for file in files:
        file = file if file.is_absolute() else ROOT / file
        if not file.exists():
            continue
        found = list(blocks(file))
        # A page states its imports once and then uses the alias in every block, so the aliases
        # declared anywhere in the file count for all of its blocks.
        local = dict(aliases)
        for path_, alias in ALIASED.findall(file.read_text(encoding="utf-8")):
            local[alias] = path_
        for native in (False, True):
            items = [(line, body) for line, nat, body in found if nat == native]
            if not items:
                continue
            code, spans = program(items, independent=file.suffix == ".html")
            used = set(re.findall(r"@([a-z_][a-z0-9_]*)\.", code))
            # path -> alias the block writes it under; the default alias is the last segment.
            wanted = {}
            for name in sorted(used):
                if name in local:
                    path_ = local[name]
                    wanted.setdefault(path_, None if name == path_.rsplit("/", 1)[-1] else name)
                elif name in CORE:
                    wanted.setdefault(f"moonbitlang/core/{name}", None)
                elif name in NESTED:
                    wanted.setdefault(f"moonbitlang/core/{NESTED[name]}", None)
            if "async test" in code:
                wanted.setdefault("moonbitlang/async", None)
            pkg = src / "snippet_check"
            shutil.rmtree(pkg, ignore_errors=True)
            pkg.mkdir(parents=True)
            head = "import {\n" + "".join(
                f'  "{p}"{"" if a is None else " @" + a},\n' for p, a in wanted.items()
            ) + "}\n" if wanted else ""
            if re.search(r"^(async )?fn main", code, re.M):
                head += '\noptions("is-main": true)\n'
            (pkg / "moon.pkg").write_text(head, encoding="utf-8", newline="\n")
            (pkg / "snippets.mbt").write_text(code, encoding="utf-8", newline="\n")
            target = "native" if native else "all"
            try:
                r = subprocess.run([args.moon, "check", "--target", target, str(pkg.relative_to(ROOT))], cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
            finally:
                shutil.rmtree(pkg, ignore_errors=True)
            out = r.stdout + r.stderr
            bad = set()
            for m in re.finditer(r"Error: \[\d+\][^\n]*\n[^\n]*snippets\.mbt:(\d+):\d+[^\n]*\n(?:[^\n]*\n){0,6}?[^\n]*╰[─┬ ]*(?:─ )?([^\n]+)", out):
                at = int(m.group(1))
                block = next((b for s, e, b in spans if s <= at <= e), None)
                bad.add((block, m.group(2).strip()))
            if r.returncode and not bad:
                bad.add((None, out.strip().splitlines()[-1] if out.strip() else "moon check failed"))
            for block, msg in sorted(bad, key=lambda x: (x[0] or 0)):
                print(f"{file.relative_to(ROOT).as_posix()}:{block}: {msg}")
            failed += len(bad)
            if not bad:
                print(f"{file.relative_to(ROOT).as_posix()}: {len(items)} {target} blocks compile")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
