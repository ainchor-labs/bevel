# Bevel Engine

A CLI-based 2D game engine built with Python.

## Installation

```bash
pip install -e .
```

## Quick Start

Create a new project:

```bash
bevel new my_game
cd my_game
```

Run your game:

```bash
bevel run
```

## Features

- YAML-based scene configuration
- Hierarchical node system
- Python scripting with lifecycle methods
- Box2D physics integration (2D) and Jolt physics (3D)
- raylib rendering
- **Hot reload** - Edit code and see changes instantly without restarting
- Resource management
- Input system with action mapping

## Hot Reload

Bevel includes a powerful hot reload system that lets you edit your game while it's running:

```bash
bevel run  # Hot reload enabled by default
```

Edit any `.py` or `.yaml` file in your `scenes/`, `scripts/`, or `prefabs/` directories and see changes immediately!

For more details, see [Hot Reload Guide](docs/HOT_RELOAD.md).

## Documentation

See the `examples/` directory for sample projects.
