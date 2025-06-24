import os
import subprocess

# === CONFIGURATION ===
ROOT_DIR = "trunk/MCEVS/"                       # <-- Set your root directory here
SELECT_CODES = ['E1', 'E2', 'E3', 'W1', 'W2']   # <-- Set your Flake8 warning codes here


def find_python_files(root_dir):
    """Recursively yield all .py files under root_dir."""
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith('.py'):
                yield os.path.join(dirpath, filename)


def autopep8_fix(file_path, select_codes):
    """
    Run autopep8 on file_path with specific warning codes.
    Modifies the file in-place.
    """
    select_arg = ",".join(select_codes)
    command = [
        "/opt/anaconda3/bin/autopep8",
        "--in-place",
        f"--select={select_arg}",
        file_path
    ]
    subprocess.run(command, check=True)


def main():
    py_files = list(find_python_files(ROOT_DIR))
    print(f"Found {len(py_files)} Python files in {ROOT_DIR}.")

    for file_path in py_files:
        print(f"Fixing {file_path} ...")
        autopep8_fix(file_path, SELECT_CODES)
    print("Done.")


if __name__ == "__main__":
    main()
