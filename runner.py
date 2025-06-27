import os
import yaml
from cli_utils import resource_path

import raylibpy as rl
import time
from level_manager import LEVEL_MANAGER
from managers.scene_manager import global_scene_manager

def load_game_config(file='config.yaml'):
    with open(file, "r") as config:
        return yaml.safe_load(config)

# Call this at the top of run_game_main before any Raylib calls
def run_game_main():
        
    game_config = load_game_config()
    lib_path = resource_path("libraylib.so.5.5.0")
    lib_dir = os.path.dirname(lib_path)
    os.environ["LD_LIBRARY_PATH"] = lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    rl.init_window(
        game_config.get("window_width", 800),
        game_config.get("window_height", 600),
        game_config.get("name", "Bevel")
    )
    rl.set_target_fps(game_config.get('target_fps', 60))

    initial_scene = game_config.get("initial_scene", "scenes/test_level.yaml")

    global_scene_manager.load_scene(initial_scene)
    rl.set_window_title(game_config.get("name", "Bevel").encode())

    # Your existing game loop logic...
    last_time = time.time()
    physics_accumulator = 0.0
    physics_dt = 1.0 / 50.0

    while not rl.window_should_close():
        now = time.time()
        dt = now - last_time
        last_time = now

        for obj in LEVEL_MANAGER.objects.values():
            obj.update(dt)

        physics_accumulator += dt
        while physics_accumulator >= physics_dt:
            for obj in LEVEL_MANAGER.objects.values():
                obj.physics_update(physics_dt)
            physics_accumulator -= physics_dt

        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        for obj in LEVEL_MANAGER.objects.values():
            rl.draw_rectangle(int(obj.position.x), int(obj.position.y),
                              int(obj.size.x), int(obj.size.y), obj.color)
            if obj.debug:
                rl.draw_text(obj.name.encode(), int(obj.position.x), int(obj.position.y) - 20, 10, rl.DARKGRAY)
        rl.end_drawing()

    rl.close_window()
