#!/usr/bin/env bash
# setup.sh  -- Bootstrap Controller Toolbox on Linux or macOS.
#
# Usage:
#   ./setup.sh                    # bindings only (fast re-build path)
#   ./setup.sh --full-build       # bindings + all C++ targets (~5-15 min)
#   ./setup.sh --skip-conda-create  # skip env create/update; just rebuild binding
#
# One-time prerequisites (install manually once, then re-run this script):
#
#   Ubuntu / Debian:
#     sudo apt update
#     sudo apt install -y gcc g++ cmake ninja-build libeigen3-dev
#
#   Fedora / RHEL / CentOS:
#     sudo dnf install -y gcc gcc-c++ cmake ninja-build eigen3-devel
#
#   Arch Linux:
#     sudo pacman -S --noconfirm gcc cmake ninja eigen
#
#   macOS (Homebrew):
#     brew install cmake ninja eigen
#     # Compiler: Xcode Command Line Tools already provide clang
#     xcode-select --install
#
#   All platforms:
#     Install Miniconda: https://docs.conda.io/en/latest/miniconda.html
#     Ensure 'conda' is on PATH (run 'conda init bash' or 'conda init zsh').

set -euo pipefail

FULL_BUILD=0
SKIP_CONDA_CREATE=0

for arg in "$@"; do
    case "$arg" in
        --full-build)        FULL_BUILD=1 ;;
        --skip-conda-create) SKIP_CONDA_CREATE=1 ;;
        -h|--help)
            head -35 "$0" | grep '^#' | sed 's/^# \?//'
            exit 0
            ;;
        *)
            echo "Unknown flag: $arg  (use --full-build or --skip-conda-create)" >&2
            exit 1
            ;;
    esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colour helpers
step() { echo; printf '\033[0;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[0;32m[OK]\033[0m %s\n'   "$*"; }
fail() { printf '\n    \033[0;31m[FAIL]\033[0m %s\n' "$*" >&2; exit 1; }
warn() { printf '    \033[0;33m[WARN]\033[0m %s\n' "$*"; }

# ── 1. Build toolchain ────────────────────────────────────────────────────────
step "Checking build toolchain (compiler, cmake, ninja, eigen3)"

OS="$(uname -s)"

_need() {
    local tool="$1"; local hint="$2"
    if ! command -v "$tool" &>/dev/null; then
        echo
        echo "  '$tool' not found."
        echo "  $hint"
        echo
        fail "Missing dependency: $tool"
    fi
    ok "$tool → $(command -v "$tool")"
}

# Compiler — accept clang or gcc
if command -v gcc &>/dev/null; then
    ok "compiler → gcc ($(gcc --version | head -1))"
elif command -v clang &>/dev/null; then
    ok "compiler → clang ($(clang --version | head -1))"
else
    case "$OS" in
        Linux)
            fail "No C++ compiler found.  Install gcc:
    Ubuntu/Debian: sudo apt install -y gcc g++
    Fedora/RHEL:   sudo dnf install -y gcc gcc-c++
    Arch:          sudo pacman -S gcc"
            ;;
        Darwin)
            fail "No C++ compiler found.  Run: xcode-select --install"
            ;;
        *)
            fail "No C++ compiler found (gcc or clang required)"
            ;;
    esac
fi

case "$OS" in
    Linux)  CMAKE_HINT="Ubuntu/Debian: sudo apt install -y cmake  |  Arch: sudo pacman -S cmake" ;;
    Darwin) CMAKE_HINT="brew install cmake" ;;
    *)      CMAKE_HINT="https://cmake.org/download/" ;;
esac
_need cmake "$CMAKE_HINT"

case "$OS" in
    Linux)  NINJA_HINT="Ubuntu/Debian: sudo apt install -y ninja-build  |  Arch: sudo pacman -S ninja" ;;
    Darwin) NINJA_HINT="brew install ninja" ;;
    *)      NINJA_HINT="https://ninja-build.org" ;;
esac
_need ninja "$NINJA_HINT"

# Eigen3 — not a binary; check for the header
EIGEN_FOUND=0
for dir in \
        /usr/include/eigen3 \
        /usr/local/include/eigen3 \
        /opt/homebrew/include/eigen3 \
        /opt/homebrew/opt/eigen/include/eigen3; do
    if [[ -f "$dir/Eigen/Dense" ]]; then
        ok "eigen3 → $dir"
        EIGEN_FOUND=1
        break
    fi
done
if [[ $EIGEN_FOUND -eq 0 ]]; then
    case "$OS" in
        Linux)
            echo
            echo "  Eigen3 headers not found."
            echo "  Ubuntu/Debian: sudo apt install -y libeigen3-dev"
            echo "  Fedora/RHEL:   sudo dnf install -y eigen3-devel"
            echo "  Arch:          sudo pacman -S eigen"
            echo
            # Non-fatal if CTRL_FETCH_EIGEN_IF_MISSING will be used; warn only.
            warn "Eigen3 not found — CMake will attempt FetchContent fallback (may be slow)."
            ;;
        Darwin)
            echo
            echo "  Eigen3 headers not found.  brew install eigen"
            echo
            warn "Eigen3 not found — CMake will attempt FetchContent fallback."
            ;;
    esac
fi

# ── 2. Conda ──────────────────────────────────────────────────────────────────
step "Checking conda"

if ! command -v conda &>/dev/null; then
    echo
    echo "  conda not found in PATH."
    echo "  Install Miniconda: https://docs.conda.io/en/latest/miniconda.html"
    echo "  Then run:  conda init bash   (or zsh / fish)"
    echo
    fail "conda not found"
fi
ok "conda → $(command -v conda)  ($(conda --version))"

# ── 3. Conda environment ──────────────────────────────────────────────────────
if [[ $SKIP_CONDA_CREATE -eq 0 ]]; then
    if conda env list | grep -qE '^soft_robotics\s'; then
        step "Updating existing 'soft_robotics' conda environment"
        conda env update -f "$ROOT/environment.yml" --prune
    else
        step "Creating 'soft_robotics' conda environment (~5 min first time)"
        conda env create -f "$ROOT/environment.yml"
    fi
    ok "conda environment 'soft_robotics' ready"
else
    warn "Skipping conda env create/update (--skip-conda-create)"
fi

# ── 4. Build Python bindings ──────────────────────────────────────────────────
step "Configuring Python bindings (cmake)"

EIGEN_ARG=""
if [[ $EIGEN_FOUND -eq 0 ]]; then
    EIGEN_ARG="-DCTRL_FETCH_EIGEN_IF_MISSING=ON"
fi

conda run -n soft_robotics -- \
    cmake -S "$ROOT" -B "$ROOT/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCTRL_BUILD_PYTHON_BINDINGS=ON \
        -G Ninja \
        $EIGEN_ARG

step "Building ctrl_toolbox binding target"
conda run -n soft_robotics -- \
    cmake --build "$ROOT/build" --target ctrl_toolbox

# Locate the built binding (.so on Linux, .so/.dylib on macOS)
BINDING=$(find "$ROOT/build/bindings" -name 'ctrl_toolbox*.so' -o -name 'ctrl_toolbox*.dylib' 2>/dev/null | head -1)
if [[ -z "$BINDING" ]]; then
    fail "ctrl_toolbox binding (.so/.dylib) not found under build/bindings/ — check cmake output above"
fi
ok "Built: $(basename "$BINDING")"

# ── 5. Smoke test ─────────────────────────────────────────────────────────────
step "Running smoke test"
conda run -n soft_robotics -- python "$ROOT/bindings/smoke_test.py"
ok "Smoke test passed"

# ── 6. Optional full C++ build ────────────────────────────────────────────────
if [[ $FULL_BUILD -eq 1 ]]; then
    step "Running compile.sh (full C++ build — this takes 5-15 min)"
    if [[ ! -x "$ROOT/compile.sh" ]]; then
        fail "compile.sh not found or not executable.  Run: chmod +x compile.sh"
    fi
    conda run -n soft_robotics -- bash "$ROOT/compile.sh"
    ok "All C++ targets built"
fi

# ── Done ──────────────────────────────────────────────────────────────────────
echo
echo "============================================================"
echo "  Setup complete."
echo
echo "  Run the full test suite:"
echo "    conda run -n soft_robotics -- python run.py"
echo
echo "  Run a single Python example:"
echo "    conda run -n soft_robotics -- python examples/python/ex01_tf_pid.py"
echo
echo "  Build all C++ targets (examples, tests, case studies):"
echo "    ./setup.sh --full-build"
echo "    -- or --"
echo "    ./compile.sh"
echo "============================================================"
