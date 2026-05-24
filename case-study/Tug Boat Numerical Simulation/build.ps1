# build.ps1 — Configure and build the Tug Boat Numerical Simulation
# Run from the project root:  .\build.ps1  [Release|Debug]

param([string]$Config = "Release")

$ProjectDir = $PSScriptRoot
$BuildDir   = "$ProjectDir\build"

# ── 1. Locate nlohmann/json.hpp ───────────────────────────────────────────────
$JsonHeader = "$ProjectDir\sim\include\json.hpp"
if (-not (Test-Path $JsonHeader)) {
    Write-Host "Downloading nlohmann/json single-header..."
    $url = "https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp"
    Invoke-WebRequest -Uri $url -OutFile $JsonHeader
}

# ── 2. Configure ──────────────────────────────────────────────────────────────
Write-Host "Configuring ($Config)..."
cmake -S $ProjectDir -B $BuildDir `
      -DCMAKE_BUILD_TYPE=$Config `
      -G "Visual Studio 17 2022" -A x64

if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed"; exit 1 }

# ── 3. Build ──────────────────────────────────────────────────────────────────
Write-Host "Building..."
cmake --build $BuildDir --config $Config

if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; exit 1 }

Write-Host ""
Write-Host "Build succeeded.  Binary: $BuildDir\$Config\tug_sim.exe"
Write-Host "Run with:  & '$BuildDir\$Config\tug_sim.exe' '$ProjectDir'"
