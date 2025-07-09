import os
import time
import yaml
import raylibpy as rl
from cli_utils import resource_path
from level_manager import LEVEL_MANAGER
from managers.scene_manager import global_scene_manager

def load_game_config(config_file='config.yaml'):
    """Load game configuration from a YAML file."""
    with open(config_file, "r") as f:
        return yaml.safe_load(f)

def setup_raylib(game_config):
    """Initialize Raylib window and set environment variables."""
    lib_path = resource_path("libraylib.so.5.5.0")
    lib_dir = os.path.dirname(lib_path)
    os.environ["LD_LIBRARY_PATH"] = lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    rl.init_window(
        game_config.get("window_width", 800),
        game_config.get("window_height", 600),
        game_config.get("name", "Bevel")
    )
    rl.set_target_fps(game_config.get("target_fps", 60))
    rl.set_window_title(game_config.get("name", "Bevel").encode())

def run_game_main():
    # Load configuration
    game_config = load_game_config()
    setup_raylib(game_config)

    # Load initial scene
    initial_scene = game_config.get("initial_scene", "scenes/test_level.yaml")
    global_scene_manager.load_scene(initial_scene)

    # Timing setup
    last_time = time.time()
    physics_accumulator = 0.0
    physics_dt = 1.0 / 50.0

    # Main game loop
    while not rl.window_should_close():
        now = time.time()
        dt = now - last_time
        last_time = now

        # Update all entities
        for obj in LEVEL_MANAGER.objects.values():
            obj.update(dt)

        # Fixed-step physics updates
        physics_accumulator += dt
        while physics_accumulator >= physics_dt:
            for obj in LEVEL_MANAGER.objects.values():
                obj.physics_update(physics_dt)
            physics_accumulator -= physics_dt

        # Render
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        LEVEL_MANAGER.render()
        rl.end_drawing()

    rl.close_window()