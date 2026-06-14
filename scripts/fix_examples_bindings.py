"""Replace inline MinGW DLL import block with `import _setup_bindings`."""
import os, re, sys

BLOCK_RE = re.compile(
    r"(?:# -- ctrl_toolbox import guard -+\r?\n)?"
    r"sys\.path\.insert\(0, os\.path\.join\(os\.path\.dirname\(__file__\).*?'build'.*?\)\)\r?\n"
    r"if sys\.platform == 'win32' and hasattr\(os, 'add_dll_directory'\):\r?\n"
    r"    for _p in \[r'C:\\\\msys64\\\\mingw64\\\\bin'\]:\r?\n"
    r"        if os\.path\.isdir\(_p\):\r?\n"
    r"            os\.add_dll_directory\(_p\)\r?\n"
)

REPLACEMENT = "import _setup_bindings  # noqa: F401\n"

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
target_dir = os.path.join(root, "examples", "python")
changed = []
for fn in sorted(os.listdir(target_dir)):
    if not fn.endswith(".py") or fn == "_setup_bindings.py":
        continue
    path = os.path.join(target_dir, fn)
    text = open(path, encoding="utf-8").read()
    new_text, n = BLOCK_RE.subn(REPLACEMENT, text)
    if n > 0:
        open(path, "w", encoding="utf-8").write(new_text)
        changed.append(fn)

print(f"Modified {len(changed)} files:")
for f in changed:
    print(f"  {f}")
