import click
import os
import shutil
from cli_utils import what_you_can_do, resource_path

@click.group
def cli():
    pass

@cli.command("init")
@click.argument("name")
def init(name):
    project_path = os.path.join(os.getcwd(), name)
    print(f"Creating project at {project_path}")

    template_dir = resource_path("project_template")

    if not os.path.exists(template_dir):
        print("Template directory not found. Please ensure the template is bundled correctly.")
        return

    if os.path.exists(project_path):
        print(f"Directory with name {name} already exists.")
        return

    try:
        shutil.copytree(template_dir, project_path)
        print(f"Project {name} created at {project_path}")
        print(what_you_can_do)
    except PermissionError:
        print(f"Permission denied: cannot create project at {project_path}")
    except Exception as e:
        print(f"An error occurred: {e}")

@cli.command("run")
def run():
    from runner import run_game_main
    run_game_main()

if __name__ == "__main__":
    cli()