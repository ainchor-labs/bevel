import os
import ctypes
from cli_utils import resource_path

import raylibpy as rl
import time
from level_loader import load_level
from level_manager import LEVEL_MANAGER

# Call this at the top of run_game_main before any Raylib calls
def run_game_main():
    lib_path = resource_path("libraylib.so.5.5.0")
    lib_dir = os.path.dirname(lib_path)
    os.environ["LD_LIBRARY_PATH"] = lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    rl.init_window(800, 600, b"Bevel Game")
    rl.set_target_fps(60)

    level_name = load_level("levels/test_level.yaml")
    rl.set_window_title(level_name.encode())

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
