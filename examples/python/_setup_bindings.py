"""
Shared setup for ctrl_toolbox C++ binding examples.
Adds the MinGW DLL directory and build output to sys.path.

Usage at the top of each bindings example:
    import _setup_bindings  # noqa: F401
    import ctrl_toolbox as ctrl
"""
import sys, os

if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    for _p in [r"C:\msys64\mingw64\bin"]:
        if os.path.isdir(_p):
            os.add_dll_directory(_p)

# examples/python/ -> examples/ -> root
_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_root, "build", "bindings"))
