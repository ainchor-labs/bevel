# Bevel 3D Demo

This example demonstrates the Phase 2 features of the Bevel game engine:

## Features Showcased

### 3D Node System
- `Node3D` - Base 3D node with Transform3D
- `Camera3D` - Perspective camera with smooth following
- `Light3D` - Directional lighting
- `MeshInstance` - 3D mesh rendering (capsule, box, sphere)
- `CollisionShape3D` - Physics collision shapes

### Multi-Script Support
- Player has both `PlayerMovement` and `PlayerHealth` scripts
- Scripts can access each other via `node.get_script("ScriptName")`
- Priority ordering for execution (movement before health)

### Signal System
- `player_damaged` signal connects Player to UI
- `player_died` signal for game over handling
- Signal definitions in YAML scene file
- Method callbacks on target nodes

### Input Contexts
- `gameplay` context for movement and actions
- `menu` context that blocks gameplay input
- Context switching for pause menu

### Scene Layers
- `world` layer for 3D objects (affected by camera)
- `ui` layer for health bar (camera-independent)

### Hot Reload
- Run with `--watch` flag to enable
- Scripts reload without losing state
- `preserve_state()` and `restore_state()` methods

## Running the Demo

```bash
# Navigate to the demo directory
cd examples/3d_demo

# Run the demo
bevel run

# Run with hot reload enabled
bevel run --watch

# Run with debug logging
bevel run --debug
```

## Controls

- **WASD / Arrow Keys** - Move
- **Space** - Jump
- **Shift** - Sprint
- **E** - Interact
- **Escape** - Pause

## Project Structure

```
3d_demo/
  project.yaml      - Project configuration
  resources.yaml    - Resource definitions and groups
  input.yaml        - Input contexts and actions
  scenes/
    main.yaml       - Main scene with 3D nodes and signals
  scripts/
    player_movement.py  - Player movement controller
    player_health.py    - Health system with signals
    camera_follow.py    - Smooth camera following
    health_ui.py        - UI health bar with signal handling
```

## Scene Hierarchy

```
Root (Node3D)
  MainCamera (Camera3D)
    scripts: camera_follow.py
  Sun (Light3D)
  Player (Node3D)
    scripts: player_movement.py, player_health.py
    PlayerMesh (MeshInstance)
    PlayerCollider (CollisionShape3D)
  Ground (Node3D)
    GroundMesh (MeshInstance)
    GroundCollider (CollisionShape3D)
  Obstacles (Node3D)
    Box1, Box2 (dynamic physics)
    Sphere (dynamic physics)
  UI (Node2D, layer: "ui")
    HealthBar (Node2D)
      scripts: health_ui.py
```

## Signal Connections

- `Player:player_damaged` -> `UI/HealthBar:on_player_damaged`
- `Player:player_died` -> `Root:on_player_died`
