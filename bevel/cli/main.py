"""CLI entry point for Bevel engine."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import click

from bevel import __version__


# Project template files
PROJECT_YAML_TEMPLATE = '''project_name: "{name}"
version: "0.1.0"

window:
  width: 1280
  height: 720
  title: "{name}"
  resizable: true
  vsync: true
  fps: 60

entry_scene: "scenes/main.yaml"

physics:
  enabled: true
  gravity: [0, 314]
'''

RESOURCES_YAML_TEMPLATE = '''# Resource definitions
# Textures, sounds, and fonts are lazy-loaded when first accessed

textures:
  # player: "assets/sprites/player.png"
  # enemy: "assets/sprites/enemy.png"

sounds:
  # jump: "assets/sounds/jump.wav"

fonts:
  # main: "assets/fonts/main.ttf"
'''

INPUT_YAML_TEMPLATE = '''# Input action definitions
actions:
  move_left:
    keys: [KEY_A, KEY_LEFT]
  move_right:
    keys: [KEY_D, KEY_RIGHT]
  move_up:
    keys: [KEY_W, KEY_UP]
  move_down:
    keys: [KEY_S, KEY_DOWN]
  jump:
    keys: [KEY_SPACE]
  action:
    keys: [KEY_E]
    mouse_buttons: [MOUSE_LEFT]
'''

MAIN_SCENE_TEMPLATE = '''scene_config:
  name: "Main Scene"
  type: "2D"
  background_color: [30, 30, 40, 255]

root:
  type: Node2D
  name: "Root"
  position: [0, 0]
  children:
    - type: Node2D
      name: "Player"
      position: [640, 360]
      scripts:
        - "scripts/player.py"

    - type: Camera2D
      name: "MainCamera"
      position: [640, 360]
      current: true
      smoothing_enabled: true
      smoothing_speed: 0.1
'''

PLAYER_SCRIPT_TEMPLATE = '''"""Player controller script."""

from bevel.input import Input
from bevel.utils.math import Vector2


class PlayerController:
    """Controls the player movement."""

    def __init__(self):
        self.node = None  # Will be set by the engine
        self.speed = 200.0

    def awake(self):
        """Called when the node is created."""
        print(f"Player '{self.node.name}' awakened!")

    def start(self):
        """Called before the first update."""
        print(f"Player '{self.node.name}' starting at {self.node.position}")

    def update(self, delta: float):
        """Called every frame."""
        # Get movement input
        direction = Input.get_vector("move_left", "move_right", "move_up", "move_down")

        # Move the player
        velocity = direction * self.speed * delta
        self.node.position = self.node.position + velocity

        # Check for jump
        if Input.is_action_just_pressed("jump"):
            print("Jump!")
'''


@click.group()
@click.version_option(version=__version__)
def cli() -> None:
    """Bevel - A CLI-based 2D game engine."""
    pass


@cli.command()
@click.argument("name")
@click.option("--path", "-p", default=".", help="Directory to create project in")
def new(name: str, path: str) -> None:
    """Create a new Bevel project.

    NAME is the name of the project to create.
    """
    project_path = Path(path) / name

    if project_path.exists():
        click.echo(f"Error: Directory '{project_path}' already exists.", err=True)
        sys.exit(1)

    click.echo(f"Creating new Bevel project: {name}")

    # Create directory structure
    dirs = [
        project_path,
        project_path / "scenes",
        project_path / "scripts",
        project_path / "assets" / "sprites",
        project_path / "assets" / "sounds",
        project_path / "assets" / "fonts",
    ]

    for dir_path in dirs:
        dir_path.mkdir(parents=True, exist_ok=True)
        click.echo(f"  Created: {dir_path.relative_to(Path(path))}")

    # Create project files
    files = [
        (project_path / "project.yaml", PROJECT_YAML_TEMPLATE.format(name=name)),
        (project_path / "resources.yaml", RESOURCES_YAML_TEMPLATE),
        (project_path / "input.yaml", INPUT_YAML_TEMPLATE),
        (project_path / "scenes" / "main.yaml", MAIN_SCENE_TEMPLATE),
        (project_path / "scripts" / "player.py", PLAYER_SCRIPT_TEMPLATE),
    ]

    for file_path, content in files:
        file_path.write_text(content)
        click.echo(f"  Created: {file_path.relative_to(Path(path))}")

    click.echo()
    click.echo(f"Project '{name}' created successfully!")
    click.echo()
    click.echo("To run your game:")
    click.echo(f"  cd {name}")
    click.echo("  bevel run")


@cli.command()
@click.argument("scene", required=False)
@click.option("--project", "-p", default=".", help="Project directory")
@click.option("--debug/--no-debug", default=False, help="Enable debug mode")
@click.option("--watch/--no-watch", default=True, help="Enable hot reload (watch for file changes)")
def run(scene: str | None, project: str, debug: bool, watch: bool) -> None:
    """Run a Bevel game.

    SCENE is an optional path to a specific scene file to load.
    If not provided, the entry_scene from project.yaml is used.

    Hot reload is enabled by default (--watch). Use --no-watch to disable it.
    """
    project_path = Path(project).resolve()

    if not (project_path / "project.yaml").exists():
        click.echo(f"Error: No project.yaml found in '{project_path}'", err=True)
        click.echo("Make sure you're in a Bevel project directory or use --project", err=True)
        sys.exit(1)

    # Set up logging level
    import logging
    from bevel.utils.logger import setup_logging
    level = logging.DEBUG if debug else logging.INFO
    setup_logging(level)

    click.echo(f"Starting Bevel game from: {project_path}")

    if watch:
        # Check if watchdog is available
        from bevel.development.hot_reload import is_hot_reload_available
        if not is_hot_reload_available():
            click.echo("Warning: Hot reload requires watchdog. Install with: pip install bevel[hotreload]", err=True)
            watch = False
        else:
            click.echo("Hot reload enabled - watching for file changes (use --no-watch to disable)")

    # Import and run engine
    from bevel.engine import create_engine

    engine = create_engine(project_path, hot_reload=watch)
    if not engine.initialize():
        click.echo("Error: Failed to initialize engine", err=True)
        sys.exit(1)

    exit_code = engine.run(scene)
    sys.exit(exit_code)


@cli.command()
@click.argument("scene_path")
@click.option("--project", "-p", default=".", help="Project directory")
def validate(scene_path: str, project: str) -> None:
    """Validate a scene file.

    SCENE_PATH is the path to the scene file to validate.
    """
    project_path = Path(project).resolve()
    scene_file = project_path / scene_path

    if not scene_file.exists():
        click.echo(f"Error: Scene file not found: {scene_file}", err=True)
        sys.exit(1)

    from bevel.core.scene_loader import SceneLoader

    loader = SceneLoader(project_path)
    scene = loader.load(scene_path)

    if scene is None:
        click.echo(f"Error: Failed to load scene: {scene_path}", err=True)
        sys.exit(1)

    click.echo(f"Scene '{scene.config.name}' is valid!")
    click.echo(f"  Type: {scene.config.type}")
    click.echo(f"  Background: {scene.config.background_color}")

    if scene.root is not None:
        node_count = 1 + len(list(scene.root.get_descendants()))
        click.echo(f"  Nodes: {node_count}")
        click.echo()
        click.echo("Node tree:")
        click.echo(scene.root.print_tree())


@cli.command()
def info() -> None:
    """Show information about the Bevel installation."""
    click.echo(f"Bevel Engine v{__version__}")
    click.echo()

    # Check dependencies
    click.echo("Dependencies:")

    # Check raylib
    try:
        import raylib
        click.echo(f"  raylib: installed")
    except ImportError:
        click.echo("  raylib: NOT INSTALLED")
        click.echo("    Install with: pip install raylib")

    # Check Box2D
    try:
        import Box2D
        click.echo(f"  Box2D: installed")
    except ImportError:
        click.echo("  Box2D: NOT INSTALLED")
        click.echo("    Install with: pip install box2d-py")

    # Check PyYAML
    try:
        import yaml
        click.echo(f"  PyYAML: installed")
    except ImportError:
        click.echo("  PyYAML: NOT INSTALLED")
        click.echo("    Install with: pip install pyyaml")

    # Check Click
    try:
        from importlib.metadata import version
        click_version = version("click")
    except Exception:
        click_version = "installed"
    click.echo(f"  Click: {click_version}")

    click.echo()
    click.echo(f"Python: {sys.version}")


if __name__ == "__main__":
    cli()
