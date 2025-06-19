import argparse
import os
import shutil
import subprocess
import yaml

def init_project(project_name):
    if not project_name:
        print("Error: You must specify a project name (e.g. bevel init MyGame)")
        return

    target_path = os.path.join(os.getcwd(), project_name)
    if os.path.exists(target_path):
        print(f"Error: Directory {project_name} already exists.")
        return

    template_path = os.path.join(os.path.dirname(__file__), "../project_template")
    shutil.copytree(template_path, target_path)
    
    # Create bevel_config.yaml
    config_data = {
        "title": "Bevel Game",
        "window_width": 800,
        "window_height": 600,
        "default_level": "levels/test_level.yaml"
    }
    config_path = os.path.join(target_path, "bevel_config.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config_data, f)

    print(f"Project '{project_name}' initialized at {target_path}")
    print(f"Created bevel_config.yaml with default settings.")


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
