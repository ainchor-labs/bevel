from level_manager import LEVEL_MANAGER as lm
import raylibpy as rl

def start(entity):
    pass

def update(dt, entity):
    speed = 100
    player_position = lm.get_object_position("player")
    direction = player_position - entity.position
    normalized_direction = rl.vector2_normalize(direction)
    velocity = normalized_direction * speed
    entity.position += velocity * dt


def physics_update(dt, entity):
    pass
