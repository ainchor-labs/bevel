import argparse
import os
import shutil
import subprocess

def init_project():
    directories = [
        "levels",
        "scripts",
        "assets",
        "exports/web",
        "exports/linux",
        "exports/mac",
        "exports/windows"
    ]

    for dir in directories:
        os.makedirs(dir, exist_ok=True)
        print(f"Created: {dir}")

    # Create a default level file
    default_level = """\
name: Test Level
objects:
  player:
    position: [100, 100]
    size: [50, 50]
    color: GREEN
    scripts: []
"""
    with open("levels/test_level.yaml", "w") as f:
        f.write(default_level)

    print("Project initialized with default level.")


def build_project():
    print("Building (running) project locally...")

    try:
        # Launch run_game.py as a subprocess
        subprocess.run(["python", ".bevel/run_game.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running game: {e}")
    except FileNotFoundError:
        print("run_game.py not found. Did you create it?")

def export_project(platform):
    supported = ["web", "windows", "linux", "mac"]
    if platform not in supported:
        print(f"Unsupported platform: {platform}")
        return

    print(f"Exporting project for {platform}...")
    # Placeholder export logic — copy files to export dir
    shutil.copytree("assets", f"exports/{platform}/assets", dirs_exist_ok=True)
    shutil.copytree("levels", f"exports/{platform}/levels", dirs_exist_ok=True)
    shutil.copytree("scripts", f"exports/{platform}/scripts", dirs_exist_ok=True)
    print(f"Exported to exports/{platform}/")

def main():
    parser = argparse.ArgumentParser(prog="bevel", description="Bevel CLI - Game Dev Helper")
    subparsers = parser.add_subparsers(dest="command")

    # init
    subparsers.add_parser("init", help="Initialize a new project")

    # build
    subparsers.add_parser("build", help="Build project for local testing")

    # export
    export_parser = subparsers.add_parser("export", help="Export project for a specific platform")
    export_parser.add_argument("--platform", required=True, help="Target platform: web, windows, linux, mac")

    args = parser.parse_args()

    if args.command == "init":
        init_project()
    elif args.command == "build":
        build_project()
    elif args.command == "export":
        export_project(args.platform)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
