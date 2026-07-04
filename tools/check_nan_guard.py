"""
tools/check_nan_guard.py -- check the NaN-guard rule from CONTRIBUTING.md's
"Numerical Safety Rules": every IController::compute() override's first real
statement must be a non-finite guard (std::isfinite(...) check or ctrl::sanitize(...)).

This is a heuristic source scan, not a C++ parser -- like tools/mu_analysis.py's
column-name heuristic, it trades perfect precision for being cheap and catching real
regressions. Three kinds of exception are built in:
  - A compute() body whose first statement delegates to another compute()-like call
    (directly returned, e.g. DiscreteADRC -> computeTracking(), or assigned to a local,
    e.g. AdaptiveSmithPredictor -> sp_->compute()) is exempt: the guard is expected to
    live in the function it delegates to.
  - _EXEMPT_CLASSES below: composition/dispatcher classes (e.g. ControllerStack) whose
    compute() fans out to multiple already-guarded inner IController::compute() calls
    in a switch/loop, so the safety guarantee is transitive rather than visible as a
    single first statement.
  - _EXEMPT_INTERFACE_GUARD_CLASSES below: classes whose compute(double) first
    statement is a *usage*-contract guard (wrong overload for the current
    configuration), not a numeric-safety check, e.g. NeuralNetworkController::compute
    rejecting calls when n_input_features != 1 (the real isfinite guard is the very
    next statement), or PassivityBasedController::compute unconditionally throwing
    because it is MIMO-only (no value is ever returned on that path, so there is
    nothing to guard).

Usage:
    python tools/check_nan_guard.py

Exits 1 (and lists offending Class::compute / Class::computeVec locations) if any
non-exempt compute() lacks the guard as its first statement, or any computeVec() lacks a
finite guard before it commits to an output (see the computeVec rule below), 0 otherwise.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
_LIB = _ROOT / "lib"

_SIG_RE = re.compile(r"\b(\w+)::compute\s*\(\s*double\b")
_GUARD_RE = re.compile(r"isfinite|allFinite|ctrl::sanitize|sanitize\(")
# return otherFunction(...);  OR  <decl> = obj->compute(...);  /  <decl> = obj.compute(...);
_DELEGATION_RE = re.compile(
    r"^\s*return\s+\w+\s*\(.*\)\s*;\s*$"
    r"|^\s*(?:const\s+)?\w+\s+\w+\s*=\s*\w+(?:->|\.)compute\w*\s*\(.*\)\s*;\s*$"
)

# computeVec(VectorXd) overrides carry the same hold-last NaN contract as compute(double),
# but (unlike the scalar path) may legitimately lead with a dimension/usage check that throws
# before the finite guard - e.g. NeuralNetworkController (size check, then allFinite) and
# PassivityBasedController (even-length + lazy-init, then allFinite). So the rule here is
# looser than "first statement must guard": a finite guard must be PRESENT and must appear
# before the controller commits to an output via .noalias() (the hot-loop write that would
# otherwise consume a NaN and advance internal state). A body that instead delegates to
# another compute-like call is exempt (the guard lives in the callee).
_COMPUTEVEC_SIG_RE = re.compile(r"\b(\w+)::computeVec\s*\(")
_COMMIT_RE = re.compile(r"\.noalias\s*\(")
_DELEGATION_ANY_RE = re.compile(r"(?:->|\.)compute\w*\s*\(")

# Composition/dispatcher classes: their compute() fans out to multiple already-guarded
# inner IController::compute() calls (switch/loop), so no single first-statement guard
# applies. See module docstring.
_EXEMPT_CLASSES = {"ControllerStack"}

# Classes whose compute(double) leads with a usage-contract guard (wrong overload for
# the current configuration) rather than a numeric-safety check. See module docstring.
_EXEMPT_INTERFACE_GUARD_CLASSES = {"NeuralNetworkController", "PassivityBasedController"}


def _find_body(text: str, brace_start: int) -> str:
    depth = 0
    for i in range(brace_start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace_start + 1 : i]
    raise ValueError("unbalanced braces")


def _first_statement(body: str) -> str | None:
    for line in body.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("//"):
            continue
        return stripped
    return None


def check_file(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8", errors="replace")
    violations = []
    for m in _SIG_RE.finditer(text):
        class_name = m.group(1)
        if class_name in _EXEMPT_CLASSES or class_name in _EXEMPT_INTERFACE_GUARD_CLASSES:
            continue
        brace_pos = text.find("{", m.end())
        if brace_pos == -1:
            continue  # declaration only, not a definition
        # Definitions only - skip declarations ending in ';' before the next '{'
        semicolon_pos = text.find(";", m.end())
        if semicolon_pos != -1 and semicolon_pos < brace_pos:
            continue
        body = _find_body(text, brace_pos)
        first = _first_statement(body)
        if first is None:
            continue
        if _GUARD_RE.search(first):
            continue
        if _DELEGATION_RE.match(first):
            continue  # exempt: delegates to another compute-like function
        line_no = text.count("\n", 0, m.start()) + 1
        violations.append(f"{path.relative_to(_ROOT)}:{line_no}: {class_name}::compute() -> {first!r}")

    # --- computeVec(VectorXd) overrides: a finite guard must exist and precede .noalias() ---
    for m in _COMPUTEVEC_SIG_RE.finditer(text):
        class_name = m.group(1)
        if class_name in _EXEMPT_CLASSES:
            continue
        brace_pos = text.find("{", m.end())
        if brace_pos == -1:
            continue  # declaration only, not a definition
        semicolon_pos = text.find(";", m.end())
        if semicolon_pos != -1 and semicolon_pos < brace_pos:
            continue  # declaration ending in ';' before any body
        body = _find_body(text, brace_pos)
        guard = _GUARD_RE.search(body)
        if guard is not None:
            commit = _COMMIT_RE.search(body)
            if commit is None or guard.start() < commit.start():
                continue  # guarded before committing to an output
        elif _DELEGATION_ANY_RE.search(body):
            continue  # delegates to an already-guarded compute()/computeVec()
        line_no = text.count("\n", 0, m.start()) + 1
        first = _first_statement(body) or ""
        violations.append(f"{path.relative_to(_ROOT)}:{line_no}: {class_name}::computeVec() -> {first!r}")

    return violations


def main() -> int:
    all_violations = []
    for path in sorted(_LIB.glob("*.cpp")) + sorted(_LIB.glob("*.h")):
        all_violations.extend(check_file(path))

    if not all_violations:
        print("OK: every IController::compute(double) leads with a non-finite guard, and "
              "every computeVec() guards before committing to an output.")
        return 0

    print(f"{len(all_violations)} compute() override(s) missing the NaN guard as their first statement:")
    for v in all_violations:
        print(f"  {v}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
