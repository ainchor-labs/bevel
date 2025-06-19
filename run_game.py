import raylibpy as rl
import time
from level_manager import LEVEL_MANAGER
from level_loader import load_level

def main():
    rl.init_window(800, 600, b"Game")

    level_name = load_level("levels/test_level.yaml")
    rl.set_window_title(level_name.encode())
    rl.set_target_fps(60)

    physics_accumulator = 0.0
    physics_dt = 1.0 / 50.0

    last_time = time.time()

    while not rl.window_should_close():
        now = time.time()
        dt = now - last_time
        last_time = now

        # Update all objects
        for obj in LEVEL_MANAGER.objects.values():
            obj.update(dt)

        # Fixed timestep physics
        physics_accumulator += dt
        while physics_accumulator >= physics_dt:
            for obj in LEVEL_MANAGER.objects.values():
                obj.physics_update(physics_dt)
            physics_accumulator -= physics_dt

        # Draw
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

if __name__ == "__main__":
    main()
