# Gather all .exe files in the build folder and run them one by one
import os
import subprocess

# Clear screen
subprocess.run("cls", shell=True)

# Compile the project first using compile.bat
compile_script = "compile.bat"
if not os.path.isfile(compile_script):
    print(f"Error: {compile_script} not found.")
    raise SystemExit(1)
subprocess.run(compile_script, check=True)
print("Compiled successfully.\n")

exe_files = []
for root, dirs, files in os.walk("build"):
    if "CMakeFiles" in dirs:
        dirs.remove("CMakeFiles")
    for file in files:
        if file.endswith(".exe"):
            exe_files.append(os.path.join(root, file))

if not exe_files:
    print("No .exe files found in build/")
    raise SystemExit(1)

print(f"Found {len(exe_files)} executable(s):")
for exe in exe_files:
    print(f"  {exe}")
print()

passed, failed = [], []

for i, exe in enumerate(exe_files, 1):
    print(f"[{i}/{len(exe_files)}] Running: {exe}")
    try:
        result = subprocess.run(
            exe, check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        if result.stdout.strip():
            print(result.stdout.rstrip())
        if result.stderr.strip():
            print("[stderr]", result.stderr.rstrip())
        passed.append(exe)
        print(f"  -> OK\n")
    except subprocess.CalledProcessError as e:
        print(f"  -> FAILED (exit {e.returncode})")
        if e.stdout.strip():
            print("[stdout]", e.stdout.rstrip())
        if e.stderr.strip():
            print("[stderr]", e.stderr.rstrip())
        failed.append(exe)
        print()

print("=" * 60)
print(f"Results: {len(passed)} passed, {len(failed)} failed")
if failed:
    print("Failed executables:")
    for exe in failed:
        print(f"  {exe}")
    raise SystemExit(1)
