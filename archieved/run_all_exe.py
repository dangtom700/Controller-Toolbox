# Gather all the .exe files in the build folder any run one by one
import os
import subprocess

exe_files = []
for root, dirs, files in os.walk("build"):
    # Ignore CMakeFiles directory
    if "CMakeFiles" in dirs:
        dirs.remove("CMakeFiles")

    for file in files:
        if file.endswith(".exe"):
            exe_files.append(os.path.join(root, file))

print("Found the following .exe files:")
for i, exe in enumerate(exe_files):
    print(f"Processing {i+1}/{len(exe_files)}: {exe}")
    try:
        result = subprocess.run(exe, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(f"Output of {exe}:\n{result.stdout}")
    except subprocess.CalledProcessError as e:
        print(f"Error running {exe}:\n{e.stderr}")