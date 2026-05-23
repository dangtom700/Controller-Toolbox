import re
import os
import argparse
import sys
from collections import defaultdict

EXTENSIONS = ('.cpp', '.h', '.py', '.c', '.hpp', '.txt', '.md', '.cmake')

# Characters allowed in typical source code / prose — everything else is flagged
PATTERN = re.compile(r'[^\w\s\(\)\{\}\[\]:;.,\'\"\-=<>\/\\|`~?!@#$%^&*+]')

REPLACEMENTS = {
    # Dashes / arrows
    '—': '-',       # em dash —
    '–': '-',       # en dash –
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
    '⁻': '^-',       # ⁻ superscript minus
    '∧': '&&',      # ∧ logical and
    '∨': '||',      # ∨ logical or
    '°': '^\\circ', # ° degree symbol
    '§': 'Section ', # § section symbol
    '\u0302': '^',   # ̂ combining circumflex accent
    '∈': '\\in',     # ∈ element of
    '↓': 'v',     # ↓ down arrow
    '∫': '\\int',   # ∫ integral symbol
    '√': '\\sqrt',   # √ square root
    '‖': '||',      # ‖ double vertical line
    '★': '*',       # ★ black star
    '✓': '(check)',   # ✓ check mark
    '↔': '\\leftrightarrow',     # ↔ left-right arrow
    '↺': '(anticlockwise)',   # ↺ anticlockwise open circle arrow
    '⁺': '^+',       # ⁺ superscript plus
}

_self_basename = os.path.basename(__file__)


def _open_text(path):
    """Open a file, falling back to latin-1 if UTF-8 fails."""
    try:
        return open(path, 'r', encoding='utf-8')
    except UnicodeDecodeError:
        return open(path, 'r', encoding='latin-1', errors='replace')


def scan_files(directory, show_context=True):
    """Scan source files for non-standard characters and report them."""
    total_hits = 0
    files_hit = 0
    char_freq = defaultdict(int)

    for root, _, files in os.walk(directory):
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
                    f'U+{ord(c):04X} {repr(c)}' for c in unique
                )
                print(f'  {path}:{line_no}  [{chars_display}]')
                if show_context:
                    print(f'    {context}')

            if file_hits:
                files_hit += 1

    print(f'\n--- Scan summary ---')
    print(f'Files with hits : {files_hit}')
    print(f'Total characters: {total_hits}')
    if char_freq:
        print('All offenders:')
        for ch, count in sorted(char_freq.items(), key=lambda x: -x[1]):
            replaceable = ' -> ' + REPLACEMENTS[ch] if ch in REPLACEMENTS else ' (no auto-replacement)'
            print(f'  U+{ord(ch):04X} {repr(ch):12s} x{count}{replaceable}')


def apply_replacements(directory, dry_run=False):
    """Replace known non-standard characters with ASCII equivalents."""
    total_replacements = 0
    files_changed = 0

    for root, _, files in os.walk(directory):
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
                    applied.append(f'U+{ord(char):04X} {repr(char)} -> {repr(replacement)} (x{occurrences})')

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

    print(f'\n--- Replace summary ---')
    print(f'Files modified  : {files_changed}' + (' (dry run — no files written)' if dry_run else ''))
    print(f'Total replacements: {total_replacements}')


def main():
    parser = argparse.ArgumentParser(
        description='Scan and replace non-standard characters in source files.'
    )
    parser.add_argument('directory', nargs='?', default='.',
                        help='Root directory to process (default: current dir)')
    parser.add_argument('--scan', action='store_true',
                        help='Scan files and report non-standard characters')
    parser.add_argument('--replace', action='store_true',
                        help='Replace known non-standard characters with ASCII equivalents')
    parser.add_argument('--dry-run', action='store_true',
                        help='With --replace: show what would change without writing files')
    parser.add_argument('--no-context', action='store_true',
                        help='With --scan: suppress line-content display')
    args = parser.parse_args()

    if not args.scan and not args.replace:
        args.scan = True
        args.replace = True

    if args.scan:
        print(f'=== Scanning {os.path.abspath(args.directory)} ===\n')
        scan_files(args.directory, show_context=not args.no_context)
        print()

    if args.replace:
        print(f'=== Replacing in {os.path.abspath(args.directory)} ===\n')
        apply_replacements(args.directory, dry_run=args.dry_run)


if __name__ == '__main__':
    main()
