import raylibpy as rl

def start(entity):
    print(f"Player movement script started for: {entity.name}")

def update(dt, entity):
    speed = 200  # pixels per second

    if rl.is_key_down(rl.KEY_RIGHT):
        entity.position.x += speed * dt
    if rl.is_key_down(rl.KEY_LEFT):
        entity.position.x -= speed * dt
    if rl.is_key_down(rl.KEY_UP):
        entity.position.y -= speed * dt
    if rl.is_key_down(rl.KEY_DOWN):
        entity.position.y += speed * dt

def physics_update(dt, entity):
    # Example placeholder: maybe handle collisions here in future
    pass
