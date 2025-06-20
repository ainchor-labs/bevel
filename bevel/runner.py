import os
import ctypes

import raylibpy as rl
import time
from .level_loader import load_level
from .level_manager import LEVEL_MANAGER

def ensure_raylib_so():
    # Where raylibpy expects the .so
    raylibpy_dir = os.path.dirname(raylibpy.__file__)
    target_dir = os.path.join(raylibpy_dir, "bin", "64bit")
    target_file = os.path.join(target_dir, "libraylib.so.5.5.0")

    # Where your packaged .so lives (inside bevel package)
    source_file = os.path.join(os.path.dirname(__file__), "libraylib.so.5.5.0")

    if not os.path.exists(target_file):
        print(f"Copying Raylib .so to: {target_file}")
        os.makedirs(target_dir, exist_ok=True)
        shutil.copy(source_file, target_file)
    else:
        print(f"Raylib .so already present at: {target_file}")

# Call this at the top of run_game_main before any Raylib calls
def run_game_main():
    ensure_raylib_so()

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
