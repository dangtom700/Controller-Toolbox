"""
tools/check_test_names.py -- reject Catch2 TEST_CASE names that break ctest registration.

Catch2 v3.5.4's `CatchAddTests.cmake` parses the `--list-tests` output with:

    string(REPLACE "\\n" ";" output "${output}")
    foreach(line ${output})

`${output}` is expanded UNQUOTED, so CMake treats an unbalanced `[` or `]` inside a test
name as a grouping delimiter and stops splitting the list on `;`. The offending test and
EVERY test declared after it collapse into a single bogus `add_test` whose Catch2 filter is
all their names concatenated -- which matches nothing.

Why this is worth a dedicated gate rather than review vigilance:

  * The symptom is actively misleading. CI reports one FAILED test with a name hundreds of
    characters long and "No tests ran" in its body. It reads as an assertion failure and is
    not one; the code under test is fine.
  * It silently DELETES coverage. The bundled tests stop executing. Here the bundle happened
    to match nothing and so failed loudly, but a bundle that matched something would have
    dropped 24 tests with a green tick.
  * It is invisible locally unless you run `ctest`. Compiling and running the test binary
    directly passes, because the executable is correct -- only registration is broken.

Real instance: "DDMR plant: wrapAngle maps into (-pi, pi]" (a mathematically correct
half-open interval) swallowed the 24 TEST_CASEs after it in test_ddmr_regression.cpp,
turning 32 tests into 7 + 1 permanently-red entry.

BALANCED pairs are safe -- "nuGap is bounded in [0,1] ..." registers correctly, which is why
test_autoscheduling, test_catch2_advanced and test_humidification were never affected. Only
the balance matters, so write half-open intervals in words:
    BAD :  TEST_CASE("wrapAngle maps into (-pi, pi]")
    GOOD:  TEST_CASE("wrapAngle maps into -pi (exclusive) to pi (inclusive)")

Note this checks NAMES only, never the trailing tag argument -- tags are `[...]` by design
and Catch2 parses them separately.

Usage:
    python tools/check_test_names.py

Exits 1 (listing file:line and the offending name) if any TEST_CASE name has unbalanced
square brackets, 0 otherwise.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
_TESTS = _ROOT / "tests"

# First string literal after TEST_CASE( -- the name. The tag argument that usually follows
# is deliberately not captured. Handles escaped quotes inside the name.
_TEST_CASE = re.compile(r'TEST_CASE\s*\(\s*"((?:[^"\\]|\\.)*)"')


def check_file(path: Path) -> list[str]:
    """Return a list of 'file:line  name' strings for unbalanced-bracket test names."""
    try:
        src = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:  # unreadable file is a real problem, not something to skip
        return [f"{path.name}: could not read ({exc})"]

    violations = []
    for m in _TEST_CASE.finditer(src):
        name = m.group(1)
        if name.count("[") != name.count("]"):
            line = src.count("\n", 0, m.start()) + 1
            rel = path.relative_to(_ROOT).as_posix()
            violations.append(f"{rel}:{line}  {name}")
    return violations


def main() -> int:
    if not _TESTS.is_dir():
        print(f"OK: no tests/ directory at {_TESTS} - nothing to check.")
        return 0

    sources = sorted(_TESTS.glob("*.cpp"))
    violations = []
    total = 0
    for path in sources:
        try:
            total += len(_TEST_CASE.findall(path.read_text(encoding="utf-8", errors="replace")))
        except OSError:
            pass
        violations.extend(check_file(path))

    if not violations:
        print(f"OK: {total} TEST_CASE names across {len(sources)} files have balanced brackets.")
        return 0

    print(f"{len(violations)} TEST_CASE name(s) with unbalanced [ or ] - these break "
          f"catch_discover_tests and will silently bundle every test declared after them:")
    for v in violations:
        print(f"  {v}")
    print("\nFix: write the interval in words, or balance the brackets. "
          "See CONTRIBUTING.md 'Step 4 - Add Catch2 tests'.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
