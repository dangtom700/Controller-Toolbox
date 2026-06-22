"""
tools/check_build_target_drift.py -- catch compile.bat / compile.sh target-list drift.

compile.bat and compile.sh each hand-maintain an explicit, ordered list of every C++
target to build sequentially (see CONTRIBUTING.md "Build and Test Workflow"). A target
present in one but missing from the other silently never builds on that platform --
this caught a real drift (compile.sh was missing stewart_sim, stewart_robustness,
test_stewart_regression, and every other case study's *_robustness target).

This script does not build anything; it just diffs the two hand-maintained lists.

Usage:
    python tools/check_build_target_drift.py

Exits 1 (and prints the missing target names) if the two lists differ, 0 otherwise.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent

_TARGET_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _extract_bat_targets(text: str) -> list[str]:
    m = re.search(r"for %%T in \((.*?)\)\s*do\s*\(", text, re.DOTALL)
    if not m:
        raise ValueError("compile.bat: could not find 'for %%T in ( ... ) do (' block")
    return [line.strip() for line in m.group(1).splitlines() if _TARGET_RE.match(line.strip())]


def _extract_sh_targets(text: str) -> list[str]:
    m = re.search(r"^TARGETS=\((.*?)^\)", text, re.DOTALL | re.MULTILINE)
    if not m:
        raise ValueError("compile.sh: could not find 'TARGETS=( ... )' block")
    return [line.strip() for line in m.group(1).splitlines() if _TARGET_RE.match(line.strip())]


def main() -> int:
    bat_targets = _extract_bat_targets((_ROOT / "compile.bat").read_text())
    sh_targets = _extract_sh_targets((_ROOT / "compile.sh").read_text())

    bat_set, sh_set = set(bat_targets), set(sh_targets)
    missing_from_sh = [t for t in bat_targets if t not in sh_set]
    missing_from_bat = [t for t in sh_targets if t not in bat_set]

    if not missing_from_sh and not missing_from_bat:
        print(f"OK: compile.bat and compile.sh agree on {len(bat_targets)} targets.")
        return 0

    if missing_from_sh:
        print(f"In compile.bat but missing from compile.sh ({len(missing_from_sh)}):")
        for t in missing_from_sh:
            print(f"  - {t}")
    if missing_from_bat:
        print(f"In compile.sh but missing from compile.bat ({len(missing_from_bat)}):")
        for t in missing_from_bat:
            print(f"  - {t}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
