import yaml
import raylibpy as rl
import time
from level_loader import load_level
from level_manager import LEVEL_MANAGER

def get_game_config():
    try:
        with open("bevel_config.yaml", "r") as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print("WARNING: bevel_config.yaml not found. Using defaults.")
        return {
            "default_level": "levels/test_level.yaml",
            "window_width": 800,
            "window_height": 600,
            "title": "Bevel Game"
        }


def run_game_main():
    config = get_game_config()

    rl.init_window(config["window_width"], config["window_height"], config["title"].encode())
    rl.set_target_fps(60)

    try:
        level_name = load_level(config["default_level"])
    except FileNotFoundError:
        level_name = "No Level Found"
        print(f"WARNING: Default level {config['default_level']} not found.")

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