import os
import subprocess

# ===> Set your folder path here
FOLDER_PATH = "trunk/MCEVS/"  # Change this to your folder

FLAKE8_PATH = "/opt/anaconda3/bin/flake8"  # Change this if your flake8 is elsewhere
FLAKE8_IGNORE = "E501"


def list_python_files(root_folder):
    python_files = []
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            if file.endswith('.py'):
                python_files.append(os.path.join(root, file))
    return python_files


if __name__ == "__main__":
    if not os.path.isdir(FOLDER_PATH):
        print(f"Error: '{FOLDER_PATH}' is not a valid directory.")
    elif not os.path.isfile(FLAKE8_PATH):
        print(f"Error: flake8 not found at '{FLAKE8_PATH}'.")
    else:
        python_files = list_python_files(FOLDER_PATH)
        if not python_files:
            print("No Python files found.")
        else:
            print("Found Python files:")
            for file_path in python_files:
                print(f"\nChecking: {file_path}")
                result = subprocess.run(
                    [FLAKE8_PATH, f"--ignore={FLAKE8_IGNORE}", file_path],
                    capture_output=True,
                    text=True
                )
                if result.stdout.strip():
                    print("Flake8 output:")
                    print(result.stdout)
                else:
                    print("No Flake8 issues found.")
                if result.stderr.strip():
                    print("Flake8 STDERR:")
                    print(result.stderr)
