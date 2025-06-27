import raylibpy as rl
from managers.scene_manager import global_scene_manager

def start(entity):
    pass

def update(dt, entity):
    speed = 400  # pixels per second

    if rl.is_key_down(rl.KEY_RIGHT):
        entity.position.x += speed * dt
    if rl.is_key_down(rl.KEY_LEFT):
        entity.position.x -= speed * dt
    if rl.is_key_down(rl.KEY_UP):
        entity.position.y -= speed * dt
    if rl.is_key_down(rl.KEY_DOWN):
        entity.position.y += speed * dt

    if rl.is_key_pressed(rl.KEY_S):
        global_scene_manager.load_scene("scenes/level_2.yaml")

def physics_update(dt, entity):
    pass