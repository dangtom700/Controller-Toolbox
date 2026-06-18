"""
_setup_bindings.py - repo-root entry point for ctrl_toolbox binding setup.

Adds the MinGW DLL directory on Windows and inserts build/bindings into sys.path.
Import from any Python file in the repo after adding the repo root to sys.path:

    _ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # depth varies
    sys.path.insert(0, _ROOT)
    import _setup_bindings  # noqa: F401
    import ctrl_toolbox as ctrl

Files in examples/python/ can simply write `import _setup_bindings` because
the examples/python/_setup_bindings.py shim is in the same directory and is
resolved first; it delegates here for the DLL path.
"""
import sys
import os

if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_root, "build", "bindings"))
