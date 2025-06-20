import argparse
import os
import shutil
import subprocess
import yaml
import importlib.resources as pkg_resources

def copy_template(src, dst):
    os.makedirs(dst, exist_ok=True)
    for item in src.iterdir():
        target = os.path.join(dst, item.name)
        if item.is_dir():
            copy_template(item, target)
        else:
            with pkg_resources.as_file(item) as file_path:
                shutil.copy(file_path, target)

def init_project(project_name):
    target_path = os.path.join(os.getcwd(), project_name)

    if os.path.exists(target_path):
        return

    with pkg_resources.as_file(pkg_resources.files("bevel.project_template")) as template_path:
        copy_template(template_path, target_path)

    # Now create bevel_config.yaml
    config = {
        "title": "Bevel Game",
        "fps": 60,
        "default_level": "levels/test_level.yaml",
        "window_width": 800,
        "window_height": 600
        
    }

    config_path = os.path.join(target_path, "bevel_config.yaml")
    with open(config_path, "w") as f:
        yaml.dump(config, f)

    print(f"Project '{project_name}' initialized at {target_path}")
    print(f"Created {config_path}")


def build_project():
    print("Building (running) project locally...")

    try:
        from .runner import run_game_main
        run_game_main()
    except Exception as e:
        print(f"Error running game: {e}")

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
    init_parser = subparsers.add_parser("init", help="Initialize a new project")
    init_parser.add_argument("--name", required=True, help="Name of the new project")

    # build
    subparsers.add_parser("build", help="Build project for local testing")

    # export
    export_parser = subparsers.add_parser("export", help="Export project for a specific platform")
    export_parser.add_argument("--platform", required=True, help="Target platform: web, windows, linux, mac")

    args = parser.parse_args()

    if args.command == "init":
        init_project(args.name)
    elif args.command == "build":
        build_project()
    elif args.command == "export":
        export_project(args.platform)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
