import yaml
import raylibpy as rl
import time
from level_loader import load_level
from level_manager import LEVEL_MANAGER

def get_default_level_path():
    config_file = "bevel_config.yaml"
    try:
        with open(config_file, "r") as f:
            config = yaml.safe_load(f)
            return config.get("default_level", "levels/test_level.yaml")
    except FileNotFoundError:
        print(f"WARNING: {config_file} not found. Using fallback default level.")
        return "levels/test_level.yaml"

def run_game_main():
    rl.init_window(800, 600, b"Bevel Game")
    rl.set_target_fps(60)

    default_level_path = get_default_level_path()
    try:
        level_name = load_level(default_level_path)
    except FileNotFoundError:
        level_name = "No Level Found"
        print(f"WARNING: Default level {default_level_path} not found.")

    rl.set_window_title(level_name.encode())

    physics_accumulator = 0.0
    physics_dt = 1.0 / 50.0
    last_time = time.time()

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
            rl.draw_rectangle(
                int(obj.position.x),
                int(obj.position.y),
                int(obj.size.x),
                int(obj.size.y),
                obj.color
            )
            if obj.debug:
                rl.draw_text(
                    obj.name.encode(),
                    int(obj.position.x),
                    int(obj.position.y) - 20,
                    10,
                    rl.DARKGRAY
                )
        rl.end_drawing()

    rl.close_window()
