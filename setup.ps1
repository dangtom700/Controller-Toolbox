#Requires -Version 7
<#
.SYNOPSIS
    Bootstrap Controller Toolbox on a new Windows machine.

.DESCRIPTION
    Sets up a complete development environment from scratch:
      1. Validates MSYS2 MinGW64 (build toolchain: cmake, ninja, g++, eigen3)
      2. Creates or updates the 'soft_robotics' conda env from environment.yml
      3. Builds ctrl_toolbox.pyd (Python bindings) against the conda Python
      4. Runs bindings/smoke_test.py to verify the binding loads correctly
      5. Optionally runs compile.bat to build all C++ targets (examples, tests, case studies)

.PARAMETER FullBuild
    Also run compile.bat after the binding is verified. Takes 5-15 min extra.
    Default: skip (bindings only).

.PARAMETER SkipCondaCreate
    Skip conda env creation/update (useful if the env is already up to date and
    you just want to rebuild the binding).

.NOTES
    One-time prerequisites (not automated — install manually if missing):

      MSYS2 (https://www.msys2.org):
        Install to C:\msys64 (default), then run in MSYS2 MinGW64 terminal:
          pacman -Syu
          pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake ^
                    mingw-w64-x86_64-ninja mingw-w64-x86_64-eigen3

      Miniconda / Anaconda (https://docs.conda.io/en/latest/miniconda.html):
        Install and ensure 'conda' is on PATH (tick "Add to PATH" during install,
        or use the Anaconda Prompt).

    Typical first-time run:
      git clone <repo-url>
      cd "Controller Toolbox"
      .\setup.ps1

    Re-run after pulling new code that changes bindings:
      .\setup.ps1 -SkipCondaCreate

    Full build (all C++ examples, tests, case studies):
      .\setup.ps1 -FullBuild
#>

param(
    [switch]$FullBuild,
    [switch]$SkipCondaCreate
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$Root = $PSScriptRoot

# ── Helpers ──────────────────────────────────────────────────────────────────
function Write-Step([string]$msg) {
    Write-Host "`n==> $msg" -ForegroundColor Cyan
}
function Write-OK([string]$msg) {
    Write-Host "    [OK] $msg" -ForegroundColor Green
}
function Write-Fail([string]$msg) {
    Write-Host "`n    [FAIL] $msg" -ForegroundColor Red
    exit 1
}

# ── 1. MSYS2 MinGW64 ─────────────────────────────────────────────────────────
Write-Step "Checking MSYS2 MinGW64 build toolchain"

$Msys2Bin = "C:\msys64\mingw64\bin"
if (-not (Test-Path $Msys2Bin)) {
    Write-Host @"

    MSYS2 not found at C:\msys64.
    Install it from https://www.msys2.org, then run in the MSYS2 MinGW64 terminal:

        pacman -Syu
        pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake `
                  mingw-w64-x86_64-ninja mingw-w64-x86_64-eigen3
"@ -ForegroundColor Red
    exit 1
}

# Prepend MSYS2 so cmake/ninja/gcc are found before any other installations
$env:PATH = "$Msys2Bin;$env:PATH"

foreach ($tool in @('cmake', 'ninja', 'gcc')) {
    $cmd = Get-Command $tool -ErrorAction SilentlyContinue
    if (-not $cmd) {
        Write-Fail "$tool not found. In MSYS2 MinGW64: pacman -S mingw-w64-x86_64-$tool"
    }
    Write-OK "$tool → $($cmd.Source)"
}

# ── 2. Conda ──────────────────────────────────────────────────────────────────
Write-Step "Checking conda"

$conda = Get-Command conda -ErrorAction SilentlyContinue
if (-not $conda) {
    Write-Host @"

    conda not found in PATH.
    Install Miniconda from https://docs.conda.io/en/latest/miniconda.html
    (tick "Add to PATH" during install, or use 'conda init powershell' afterwards).
"@ -ForegroundColor Red
    exit 1
}
Write-OK "conda → $($conda.Source)"

if (-not $SkipCondaCreate) {
    $envList = conda env list 2>$null
    $envExists = $envList -match '(?m)^soft_robotics\s'

    if ($envExists) {
        Write-Step "Updating existing 'soft_robotics' conda environment"
        conda env update -f "$Root\environment.yml" --prune
        if ($LASTEXITCODE -ne 0) { Write-Fail "conda env update failed" }
    } else {
        Write-Step "Creating 'soft_robotics' conda environment (~5 min)"
        conda env create -f "$Root\environment.yml"
        if ($LASTEXITCODE -ne 0) { Write-Fail "conda env create failed" }
    }
    Write-OK "conda environment 'soft_robotics' ready"
} else {
    Write-Host "    [SKIP] conda env create/update (-SkipCondaCreate)" -ForegroundColor Yellow
}

# ── 3. Build Python bindings ──────────────────────────────────────────────────
Write-Step "Configuring Python bindings"
# Run cmake through 'conda run' so pybind11 installed in the conda env is
# discovered by CMake's find_package(pybind11), while cmake/ninja/g++ come
# from MSYS2 (which is prepended to PATH above).
conda run -n soft_robotics -- `
    cmake -S "$Root" -B "$Root\build" `
        -DCMAKE_BUILD_TYPE=Release `
        -DCTRL_BUILD_PYTHON_BINDINGS=ON `
        -G Ninja
if ($LASTEXITCODE -ne 0) { Write-Fail "CMake configure failed" }

Write-Step "Building ctrl_toolbox binding target"
conda run -n soft_robotics -- `
    cmake --build "$Root\build" --target ctrl_toolbox
if ($LASTEXITCODE -ne 0) { Write-Fail "ctrl_toolbox build failed" }

$pyd = Get-ChildItem "$Root\build\bindings\ctrl_toolbox*.pyd" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $pyd) {
    Write-Fail "ctrl_toolbox*.pyd not found under build\bindings\ — check cmake output above"
}
Write-OK "Built: $($pyd.Name)"

# ── 4. Smoke test ─────────────────────────────────────────────────────────────
Write-Step "Running smoke test"
conda run -n soft_robotics -- python "$Root\bindings\smoke_test.py"
if ($LASTEXITCODE -ne 0) { Write-Fail "Smoke test failed — the binding built but cannot be imported" }
Write-OK "Smoke test passed"

# ── 5. Optional full C++ build ────────────────────────────────────────────────
if ($FullBuild) {
    Write-Step "Running compile.bat (full C++ build — this takes 5-15 min)"
    & "$Root\compile.bat"
    if ($LASTEXITCODE -ne 0) { Write-Fail "compile.bat failed" }
    Write-OK "All C++ targets built"
}

# ── Done ──────────────────────────────────────────────────────────────────────
Write-Host @"

============================================================
  Setup complete.

  Run the full test suite:
    conda run -n soft_robotics -- python run.py

  Run a single Python example:
    conda run -n soft_robotics -- python examples\python\ex01_tf_pid.py

  Build all C++ targets (examples, tests, case studies):
    .\setup.ps1 -FullBuild
    — or —
    .\compile.bat
============================================================
"@ -ForegroundColor Green
