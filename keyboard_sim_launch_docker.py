import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


# ==========================================
# BGR Simulator Docker Startup Script
# Python Edition
# Automatically selects OS config + NVIDIA acceleration
# ==========================================


def run(command, check=False):
    """
    Runs a command in the terminal.
    check=False means the script will continue even if the command fails.
    """
    print(f"\n> {' '.join(command)}")
    return subprocess.run(command, check=check)


def command_exists(command):
    """
    Checks if a command exists on this computer.
    Example: docker, nvidia-smi
    """
    return shutil.which(command) is not None


def command_works(command):
    """
    Checks if a command runs successfully.
    """
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0
    except Exception:
        return False


def is_windows():
    """
    Returns True if the script is running on Windows.
    """
    return platform.system().lower() == "windows"


def is_linux():
    """
    Returns True if the script is running on Linux.
    """
    return platform.system().lower() == "linux"


def main():
    repo_root = Path(__file__).resolve().parent

    compose_base = repo_root / "docker" / "docker-compose-base.yml"
    compose_linux = repo_root / "docker" / "docker-compose-linux.yml"
    compose_nvidia = repo_root / "docker" / "docker-compose-nvidia.yml"

    print("==========================================")
    print("BGR Simulator Docker Launcher")
    print("==========================================")

    # --------------------------------------------------
    # 1. Check Docker
    # --------------------------------------------------
    print("\n[1/5] Checking Docker...")

    if not command_exists("docker"):
        print("Error: Docker was not found.")
        print("Please install Docker Desktop and make sure it is running.")
        sys.exit(1)

    if not command_works(["docker", "version"]):
        print("Error: Docker exists, but Docker is not running.")
        print("Open Docker Desktop and try again.")
        sys.exit(1)

    print("Docker is available.")

    # --------------------------------------------------
    # 2. Cleanup
    # --------------------------------------------------
    print("\n[2/5] Cleaning old simulator container...")

    run(["docker", "stop", "bgr_simulator"])
    run(["docker", "rm", "bgr_simulator"])

    # --------------------------------------------------
    # 3. Check Compose files
    # --------------------------------------------------
    print("\n[3/5] Checking Docker Compose files...")

    if not compose_base.exists():
        print(f"Error: Cannot find base compose file:")
        print(compose_base)
        sys.exit(1)

    print(f"Base compose file: {compose_base}")

    if is_linux():
        if compose_linux.exists():
            print(f"Linux compose file found: {compose_linux}")
        else:
            print("Warning: Linux detected, but docker-compose-linux.yml was not found.")
            print("Continuing with base compose only.")

    if not compose_nvidia.exists():
        print("Warning: docker-compose-nvidia.yml was not found.")
        print("NVIDIA acceleration will be disabled.")

    # --------------------------------------------------
    # 4. GPU detection
    # --------------------------------------------------
    print("\n[4/5] Checking NVIDIA GPU support...")

    use_nvidia = False

    if command_exists("nvidia-smi") and command_works(["nvidia-smi"]):
        print("NVIDIA GPU detected on host.")

        if compose_nvidia.exists():
            docker_gpu_works = command_works([
                "docker",
                "run",
                "--rm",
                "--gpus",
                "all",
                "nvidia/cuda:12.4.1-base-ubuntu22.04",
                "nvidia-smi",
            ])

            if docker_gpu_works:
                print("Docker NVIDIA GPU support is working.")
                use_nvidia = True
            else:
                print("NVIDIA GPU exists, but Docker GPU support is not working.")
                print("Starting without NVIDIA acceleration.")
        else:
            print("NVIDIA compose file missing. Starting without NVIDIA acceleration.")
    else:
        print("No NVIDIA GPU detected. Starting without NVIDIA acceleration.")

    # --------------------------------------------------
    # 5. Build Docker Compose command
    # --------------------------------------------------
    print("\n[5/5] Starting simulator...")

    compose_command = [
        "docker",
        "compose",
        "-f",
        str(compose_base),
    ]

    if is_linux() and compose_linux.exists():
        print("OS mode: Linux")
        print("Adding Linux compose override.")
        compose_command += [
            "-f",
            str(compose_linux),
        ]
    elif is_windows():
        print("OS mode: Windows")
        print("Using base compose only for Windows.")
    else:
        print(f"OS mode: {platform.system()}")
        print("Using base compose only.")

    if use_nvidia:
        print("GPU mode: NVIDIA acceleration")
        compose_command += [
            "-f",
            str(compose_nvidia),
        ]
    else:
        print("GPU mode: regular Docker")

    compose_command += [
        "up",
        "--build",
    ]

    os.chdir(repo_root)
    run(compose_command, check=True)


if __name__ == "__main__":
    main()