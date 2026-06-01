"""
run.py - Controller Toolbox build & test runner.

Three automated phases (no user input required):
  1. Non-ASCII clean - auto-replace known non-standard characters.
  2. Compile         - run compile.bat once.
  3. Run             - execute each .exe one-by-one, stream output, print summary.
"""
import os
import re
import sys
import datetime
import subprocess
from collections import defaultdict


def _ascii(s):
    """Escape non-ASCII bytes so every line printed is plain ASCII (cp1252-safe)."""
    return s.encode('ascii', errors='backslashreplace').decode('ascii')


class _Tee:
    """Write to two streams simultaneously (terminal + log file)."""
    def __init__(self, a, b):
        self._a, self._b = a, b

    def write(self, data):
        self._a.write(data)
        self._b.write(data)

    def flush(self):
        self._a.flush()
        self._b.flush()

    def __getattr__(self, name):
        return getattr(self._a, name)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

EXTENSIONS = ('.cpp', '.h', '.py', '.c', '.hpp', '.txt', '.md', '.cmake')

# Characters allowed in typical source code — everything else is flagged.
PATTERN = re.compile(r'[^\w\s\(\)\{\}\[\]:;.,\'\"\-=<>\/\\|`~?!@#$%^&*+]')

REPLACEMENTS = {
    # Dashes / arrows
    '—': '-',       # em dash —
    '-': '-',       # en dash -
    '→': '->',      # →
    '←': '<-',      # ←
    '⇒': '=>',      # ⇒
    '⇐': '<=',      # ⇐
    '⇄': '<->',     # ⇄
    # Math
    '≈': 'approx =',  # ≈
    '≠': '!=',      # ≠
    '≤': '<=',      # ≤
    '≥': '>=',      # ≥
    '×': '*',       # × (multiplication)
    '÷': '/',       # ÷
    '±': '+/-',     # ±
    '∞': 'inf',     # ∞
    '−': '-',       # − (minus sign)
    '²': '^2',      # ²
    '³': '^3',      # ³
    'α': 'alpha',   # α
    'β': 'beta',    # β
    'ω': 'omega',   # ω
    'σ': 'sigma',   # σ
    'δ': 'delta',   # δ
    'ε': 'epsilon', # ε
    'λ': 'lambda',  # λ
    'μ': 'mu',      # μ
    'π': 'pi',      # π
    # Quotes / punctuation
    '‘': "'",       # ' left single quote
    '’': "'",       # ' right single quote
    '“': '"',       # " left double quote
    '”': '"',       # " right double quote
    '…': '...',     # … ellipsis
    ' ': ' ',       # non-breaking space
    '•': '-',       # • bullet
    '·': '.',       # · middle dot
    '™': '(TM)',    # ™
    '®': '(R)',     # ®
    '©': '(C)',     # ©
    '─': '-',       # ─ box-drawing light horizontal
    '│': '|',       # │ box-drawing light vertical
    '‑': '-',       # ‑ non-breaking hyphen
    '⁻': '^-',      # ⁻ superscript minus
    '∧': '&&',      # ∧ logical and
    '∨': '||',      # ∨ logical or
    '°': '^\\circ', # ° degree symbol
    '§': 'Section ',# § section symbol
    '̂': '^',       # ̂ combining circumflex accent
    '∈': '\\in',    # ∈ element of
    '↓': 'v',       # ↓ down arrow
    '∫': '\\int',   # ∫ integral symbol
    '√': '\\sqrt',  # √ square root
    '‖': '||',      # ‖ double vertical line
    '≪': '<<',      # ≪ much less than
    '≫': '>>',      # ≫ much greater than
    '₌': '=',       # ₌ subscript equals
    '‾': '-',       # ‾ overline (used as hat/bar notation)
    '★': '*',       # ★ black star
    '✓': '(check)', # ✓ check mark
    '↔': '\\leftrightarrow',   # ↔ left-right arrow
    '↺': '(anticlockwise)',    # ↺ anticlockwise open circle arrow
    '⁺': '^+',      # ⁺ superscript plus
    # Superscripts / subscripts
    '¹': '^1',      # ¹ superscript 1
    '⁴': '^4',      # ⁴ superscript 4
    '⁵': '^5',      # ⁵ superscript 5
    'ⁿ': '^n',      # ⁿ superscript n
    'ᵈ': '^d',      # ᵈ superscript d (discrete)
    'ᵐ': '^m',      # ᵐ superscript m
    '₀': '0',       # ₀ subscript 0
    '₁': '1',       # ₁ subscript 1
    '₂': '2',       # ₂ subscript 2
    '₃': '3',       # ₃ subscript 3
    'ₘ': '_m',      # ₘ subscript m
    'ₙ': '_n',      # ₙ subscript n
    'ᵢ': '_i',      # ᵢ subscript i
    'ⱼ': '_j',      # ⱼ subscript j
    # Capital Greek
    'Δ': 'Delta',   # Δ capital delta
    'Σ': 'Sigma',   # Σ capital sigma
    'Φ': 'Phi',     # Φ capital phi
    'Γ': 'Gamma',   # Γ capital gamma
    'Λ': 'Lambda',  # Λ capital lambda
    'Π': 'Pi',      # Π capital pi
    'Θ': 'Theta',   # Θ capital theta
    'Ω': 'Omega',   # Ω capital omega
    # Lowercase Greek (not already covered)
    'θ': 'theta',   # θ theta
    'φ': 'phi',     # φ phi
    'τ': 'tau',     # τ tau
    'ρ': 'rho',     # ρ rho
    'γ': 'gamma',   # γ gamma
    'η': 'eta',     # η eta
    'ζ': 'zeta',    # ζ zeta
    'ξ': 'xi',      # ξ xi
    # Dot-notation
    'ẋ': 'xdot',    # ẋ x with dot above
    'ẍ': 'xddot',   # ẍ x with diaeresis (double-dot)
    'ẏ': 'ydot',    # ẏ y with dot above
    '̇': '.',       # ̇  combining dot above (standalone)
    # Hat notation
    'ŷ': 'yhat',    # ŷ y with circumflex
    'ĝ': 'ghat',    # ĝ g with circumflex
    # Additional math
    '∂': 'd',       # ∂ partial derivative
    '†': '^T',      # † dagger (transpose / pseudo-inverse)
    '⊥': '\\perp',  # ⊥ perpendicular
    '≡': '\\equiv', # ≡ identical to / defined as
    '∅': '\\emptyset', # ∅ empty set
    '∠': '\\angle', # ∠ angle
    # Box-drawing
    '├': '|',       # ├ box-drawing light vertical and right
    '└': '|',       # └ box-drawing light up and right
    '┐': '|',       # ┐ box-drawing light down and left
    '┬': '|',       # ┬ box-drawing light vertical and down
    '┴': '|',       # ┴ box-drawing light vertical and up
    '┌': '|',       # ┌ box-drawing light down and right
    '┘': '|',       # ┘ box-drawing light up and left
    '┼': '',        # ┼ box-drawing light vertical and horizontal
    '┤': '|',       # ┤ box-drawing light vertical and left
    '═': '=',       # ═ box-drawing double horizontal
    # Latin letters with diacritics (author names in references)
    'ö': 'o',       # ö o with umlaut
    'ä': 'a',       # ä a with umlaut
    'é': 'e',       # é e with acute
    'ÿ': 'y',       # ÿ y with diaeresis
    'Å': 'A',       # Å A with ring (e.g. Angstrom / Astrom)
    # Soft hyphen (invisible, just remove)
    '­': '',        # soft hyphen
    # Micro sign (distinct from mu)
    'µ': 'mu',      # µ micro sign U+00B5
    # Misc symbols
    '∇': 'nabla',   # ∇ nabla
    '∏': '\\prod',  # ∏ product symbol
    '⊗': '\\otimes',# ⊗ tensor product
    '▼': 'v',       # ▼ black down-pointing triangle
    '▶': '>',       # ▶ black right-pointing triangle
    '｜': '|',       # ｜ fullwidth vertical line
    '\U0001f7e2': '',    # 🟢 green circle
    '✅': '',        # ✅ white heavy check mark
    '❌': '',        # ❌ cross mark
}

_self_basename = os.path.basename(__file__)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _open_text(path):
    """Open a file for text reading, falling back to latin-1 if UTF-8 fails."""
    try:
        return open(path, 'r', encoding='utf-8')
    except UnicodeDecodeError:
        return open(path, 'r', encoding='latin-1', errors='replace')


def _divider(char='=', width=72):
    print(char * width)


# ---------------------------------------------------------------------------
# Phase 1 helpers
# ---------------------------------------------------------------------------

def scan_files(directory, show_context=True):
    """Scan source files for non-standard characters. Returns total hit count."""
    total_hits = 0
    files_hit = 0
    char_freq = defaultdict(int)

    _SKIP_DIRS = {'build', '.git', '__pycache__', 'docs'}
    for root, dirs, files in os.walk(directory):
        dirs[:] = [d for d in dirs if d not in _SKIP_DIRS]
        for filename in sorted(files):
            if filename == _self_basename:
                continue
            if not filename.endswith(EXTENSIONS):
                continue

            path = os.path.join(root, filename)
            file_hits = 0

            try:
                with _open_text(path) as f:
                    lines = f.readlines()
            except OSError as e:
                print(f'[ERROR] Cannot read {path}: {e}', file=sys.stderr)
                continue

            for line_no, line in enumerate(lines, start=1):
                found = PATTERN.findall(line)
                if not found:
                    continue

                unique = sorted(set(found))
                for ch in found:
                    char_freq[ch] += 1
                total_hits += len(found)
                file_hits += 1

                context = line.rstrip('\n')
                if len(context) > 120:
                    context = context[:117] + '...'

                chars_display = ', '.join(
                    f'U+{ord(c):04X} {_ascii(repr(c))}' for c in unique
                )
                print(f'  {path}:{line_no}  [{chars_display}]')
                if show_context:
                    print(f'    {_ascii(context)}')

            if file_hits:
                files_hit += 1

    print(f'\n--- Scan summary ---')
    print(f'Files with hits : {files_hit}')
    print(f'Total characters: {total_hits}')
    if char_freq:
        print('All offenders:')
        for ch, count in sorted(char_freq.items(), key=lambda x: -x[1]):
            note = (' -> ' + _ascii(REPLACEMENTS[ch])) if ch in REPLACEMENTS else ' (no auto-replacement)'
            print(f'  U+{ord(ch):04X} {_ascii(repr(ch)):12s} x{count}{note}')

    return total_hits


def apply_replacements(directory, dry_run=False):
    """Replace known non-standard characters with ASCII equivalents."""
    total_replacements = 0
    files_changed = 0

    _SKIP_DIRS = {'build', '.git', '__pycache__', 'docs'}
    for root, dirs, files in os.walk(directory):
        dirs[:] = [d for d in dirs if d not in _SKIP_DIRS]
        for filename in sorted(files):
            if filename == _self_basename:
                continue
            if not filename.endswith(EXTENSIONS):
                continue

            path = os.path.join(root, filename)

            try:
                with _open_text(path) as f:
                    original = f.read()
            except OSError as e:
                print(f'[ERROR] Cannot read {path}: {e}', file=sys.stderr)
                continue

            modified = original
            file_count = 0
            applied = []

            for char, replacement in REPLACEMENTS.items():
                occurrences = modified.count(char)
                if occurrences:
                    modified = modified.replace(char, replacement)
                    file_count += occurrences
                    applied.append(
                        f'U+{ord(char):04X} {_ascii(repr(char))} -> {_ascii(repr(replacement))} (x{occurrences})'
                    )

            if not file_count:
                continue

            total_replacements += file_count
            files_changed += 1
            tag = '[DRY RUN] ' if dry_run else ''
            print(f'{tag}{path}: {file_count} replacement(s)')
            for detail in applied:
                print(f'    {detail}')

            if not dry_run:
                try:
                    with open(path, 'w', encoding='utf-8') as f:
                        f.write(modified)
                except OSError as e:
                    print(f'[ERROR] Cannot write {path}: {e}', file=sys.stderr)

    tag = ' (dry run — no files written)' if dry_run else ''
    print(f'\n--- Replace summary ---')
    print(f'Files modified  : {files_changed}{tag}')
    print(f'Total replacements: {total_replacements}')

    return total_replacements


# ---------------------------------------------------------------------------
# Phase 1 — Non-ASCII clean
# ---------------------------------------------------------------------------

def phase_clean():
    cwd = os.getcwd()
    _divider()
    print(f'  Phase 1 — Non-ASCII scan')
    print(f'  Directory: {os.path.abspath(cwd)}')
    _divider()
    print()

    total_hits = scan_files(cwd, show_context=True)

    if total_hits == 0:
        print('\nAll clean — no non-ASCII characters found.\n')
        return

    print()
    apply_replacements(cwd, dry_run=False)
    print('\nFiles updated.\n')


# ---------------------------------------------------------------------------
# Phase 2 — Compile
# ---------------------------------------------------------------------------

def phase_compile():
    _divider()
    print('  Phase 2 — Compile')
    _divider()
    print()

    script = 'compile.bat'
    if not os.path.isfile(script):
        print(f'Error: {script} not found in {os.getcwd()}')
        raise SystemExit(1)

    script_abs = os.path.abspath(script)
    cwd = os.getcwd()
    with subprocess.Popen(script_abs, shell=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True,
                          encoding='utf-8', errors='backslashreplace',
                          cwd=cwd) as proc:
        for line in proc.stdout:
            sys.stdout.write(line)
        proc.wait()
        rc = proc.returncode

    if rc != 0:
        print(f'\nCompile FAILED (exit {rc}). Aborting.\n')
        raise SystemExit(rc)

    print('\nCompile succeeded.\n')


# ---------------------------------------------------------------------------
# Phase 3 — Run one-by-one
# ---------------------------------------------------------------------------

def phase_run():
    _divider()
    print('  Phase 3 — Run executables')
    _divider()
    print()

    # Discover .exe files (skip CMakeFiles dirs)
    exe_files = []
    for root, dirs, files in os.walk('build'):
        dirs[:] = [d for d in dirs if d != 'CMakeFiles']
        for f in sorted(files):
            if f.endswith('.exe'):
                exe_files.append(os.path.join(root, f))
    exe_files.sort()

    if not exe_files:
        print('No .exe files found under build/\n')
        raise SystemExit(1)

    total = len(exe_files)
    print(f'Found {total} executable(s):')
    for i, exe in enumerate(exe_files, 1):
        print(f'  [{i:>2}] {exe}')
    print()

    passed, failed = [], []

    for i, exe in enumerate(exe_files, 1):
        _divider('-')
        print(f'  [{i}/{total}]  {exe}')
        _divider('-')
        print()

        try:
            with subprocess.Popen([exe], stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True,
                                  encoding='utf-8', errors='backslashreplace') as proc:
                for line in proc.stdout:
                    sys.stdout.write(line)
                proc.wait()
                rc = proc.returncode
        except Exception as exc:
            print(f'\n  ERROR launching: {exc}')
            failed.append(exe)
            rc = -1

        print()
        if rc == 0:
            passed.append(exe)
            print(f'  EXIT 0 — PASSED')
        else:
            failed.append(exe)
            print(f'  EXIT {rc} — FAILED')
        print()

    # --- Final summary ---
    _divider()
    print(f'  Summary: {len(passed)} passed  |  {len(failed)} failed')
    _divider()

    if passed:
        print('\n  Passed:')
        for exe in passed:
            print(f'    {exe}')

    if failed:
        print('\n  Failed:')
        for exe in failed:
            print(f'    {exe}')

    print()

    if failed:
        raise SystemExit(1)


# ---------------------------------------------------------------------------
# Phase 4 — Run Python binding examples
# ---------------------------------------------------------------------------

def phase_python():
    """Run Python examples that use the ctrl_toolbox C++ bindings.

    Discovers all exNN_*.py files in examples/python/ and runs them via
    `conda run -n soft_robotics -- python <script>`.  Each script must exit 0 to pass.
    Scripts that import _setup_bindings require the .pyd to be built first.
    """
    _divider()
    print('  Phase 4 — Python binding examples')
    _divider()
    print()

    py_dir = os.path.join('examples', 'python')
    if not os.path.isdir(py_dir):
        print(f'[SKIP] {py_dir} not found\n')
        return

    import re as _re
    scripts = sorted([
        os.path.join(py_dir, f)
        for f in os.listdir(py_dir)
        if _re.match(r'ex\d+_.*\.py$', f)
    ])

    if not scripts:
        print('No exNN_*.py scripts found.\n')
        return

    total = len(scripts)
    print(f'Found {total} Python example(s):')
    for i, s in enumerate(scripts, 1):
        print(f'  [{i:>2}] {s}')
    print()

    py_passed, py_failed = [], []

    for i, script in enumerate(scripts, 1):
        _divider('-')
        print(f'  [{i}/{total}]  {script}')
        _divider('-')
        print()

        # Run each script from its own directory so relative paths (data/, utils/) resolve.
        script_dir  = os.path.dirname(os.path.abspath(script))
        script_abs  = os.path.abspath(script)
        cmd = ['conda', 'run', '-n', 'soft_robotics', '--', 'python', script_abs]
        try:
            with subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding='utf-8',
                errors='backslashreplace',
                cwd=script_dir
            ) as proc:
                for line in proc.stdout:
                    sys.stdout.write(line)
                proc.wait()
                rc = proc.returncode
        except Exception as exc:
            print(f'\n  ERROR launching: {exc}')
            py_failed.append(script)
            continue

        print()
        if rc == 0:
            py_passed.append(script)
            print('  EXIT 0 - PASSED')
        else:
            py_failed.append(script)
            print(f'  EXIT {rc} - FAILED')
        print()

    _divider()
    print(f'  Python summary: {len(py_passed)} passed  |  {len(py_failed)} failed')
    _divider()

    if py_failed:
        print('\n  Failed Python scripts:')
        for s in py_failed:
            print(f'    {s}')
        print()
        # Don't abort: Python failures are separate from C++ test failures.
        # They require the binding .pyd to be built; raise a warning instead.
        print('  WARNING: some Python examples failed. '
              'Re-build with -DCTRL_BUILD_PYTHON_BINDINGS=ON and retry.\n')


# ---------------------------------------------------------------------------
# Phase 5 — Bug report (scan completed log for [FAIL] entries)
# ---------------------------------------------------------------------------

def phase_bug_report(log_path):
    """Scan the run log for failure indicators and write a concise bug report.

    The report includes context lines around each failure (or cluster of failures)
    and a summary.  The log file is read with error resilience.
    """
    keywords = ['fail', 'error', 'exception', 'fatal', 'abort', 'assert', 'nan', 'warn']
    context_lines = 10          # lines before and after each failure marker

    # Phrases that mark a line as "passed / no bug" even if a keyword appears in it.
    # Checked case-insensitively before the keyword scan; any match skips the line.
    safe_phrases = [
        'no exception',          # "(no exception)" = expected no exception and got none
        '0 failed',              # "90 passed | 0 failed" summary line
        '0 fail',                # catches both "0 failed" and "0 failures"
        'exit 0',                # "EXIT 0 — PASSED" line per executable
        '[pass]',                # "[PASS] assert_close ..." test output
        'all tests passed',      # Catch2 final banner
        'passed | 0',            # "N passed | 0 failed" summary variant
    ]

    # Try to read the log - fall back to latin‑1 if UTF‑8 fails
    try:
        with open(log_path, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
    except UnicodeDecodeError:
        with open(log_path, 'r', encoding='latin-1', errors='replace') as f:
            lines = f.readlines()
    except OSError as e:
        print(f'[bug_report] Cannot read {log_path}: {e}')
        return

    # Collect line indices that match any keyword (case-insensitive).
    # Lines whose lowercase content contains any safe_phrase are skipped first
    # to avoid false positives from passing-test output.
    matches = []
    for i, line in enumerate(lines):
        line_lower = line.lower()
        if any(phrase in line_lower for phrase in safe_phrases):
            continue                          # passing context — not a real indicator
        for kw in keywords:
            if kw in line_lower:
                matches.append((i, kw))      # store line number and matched keyword
                break                         # avoid duplicate entries for same line

    if not matches:
        print('  No failure indicators found - no bug report written.\n')
        return

    # Merge failure contexts where their windows overlap or touch
    merged_blocks = []
    i = 0
    total_failures = len(matches)

    while i < total_failures:
        start_idx = matches[i][0]
        block_start = max(0, start_idx - context_lines)
        block_end = min(len(lines), start_idx + context_lines + 1)
        keywords_in_block = [matches[i][1]]

        # Expand block to include any subsequent matches that fall inside it
        j = i + 1
        while j < total_failures:
            next_idx = matches[j][0]
            # If next failure line is within current block or just adjacent,
            # we extend the block to cover it as well.
            if next_idx <= block_end + context_lines:   # +context_lines to glue close blocks
                block_end = min(len(lines), next_idx + context_lines + 1)
                keywords_in_block.append(matches[j][1])
                j += 1
            else:
                break
        merged_blocks.append((block_start, block_end, keywords_in_block, start_idx))
        i = j

    # Build root-path prefixes to strip so the report shows paths relative to
    # the project root ("Controller Toolbox/") instead of the full user directory.
    # The log contains both forward-slash (compiler) and backslash (shell) forms.
    _root = os.path.abspath('.')
    _root_fwd = _root.replace('\\', '/') + '/'   # e.g. "C:/Users/.../Controller Toolbox/"
    _root_bwd = _root.replace('/', '\\') + '\\'  # e.g. "C:\Users\...\Controller Toolbox\"

    def _strip_root(text):
        text = text.replace(_root_bwd, '')
        text = text.replace(_root_fwd, '')
        return text

    # Write the report
    report_name = "bug_report.txt"
    with open(report_name, 'w', encoding='utf-8') as report:
        from datetime import datetime
        report.write(f"Bug Report - generated from {_strip_root(log_path)} on {datetime.now()}\n")
        report.write(f"Total failure indicators found: {total_failures}\n")
        report.write(f"Merged into {len(merged_blocks)} block(s)\n")
        report.write("=" * 72 + "\n\n")

        for block_id, (block_start, block_end, kw_list, first_match_line) in enumerate(merged_blocks, 1):
            report.write(f"--- FAILURE BLOCK #{block_id} (keywords: {', '.join(set(kw_list))}) ---\n")
            report.write(f"Lines {block_start+1} - {block_end}\n\n")
            for line_no in range(block_start, block_end):
                prefix = ">>> " if line_no == first_match_line else "    "
                report.write(f"{prefix}{line_no+1:4d}: {_strip_root(lines[line_no])}")
            report.write("\n" + "-" * 72 + "\n\n")

    print(f'  Bug report written: {report_name}  ({total_failures} indicator(s) in {len(merged_blocks)} block(s))\n')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = f'run_{ts}.log'
    _log_file = open(log_path, 'w', encoding='utf-8')
    sys.stdout = _Tee(sys.__stdout__, _log_file)
    print(f'  Log: {log_path}\n')

    try:
        phase_clean()
        phase_compile()
        phase_run()
        phase_python()
    finally:
        sys.stdout = sys.__stdout__
        _log_file.close()

    phase_bug_report(log_path)
