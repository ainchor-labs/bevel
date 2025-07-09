import json
from util.ldtk_json import ldtk_json_from_dict
from entity import Entity
from level_manager import LEVEL_MANAGER
from util.consts import COLOR_MAP
import raylibpy as rl

def load_ldtk_level(ldtk_path, level_identifier=None):
    with open(ldtk_path, "r") as f:
        ldtk_data = json.load(f)
    project = ldtk_json_from_dict(ldtk_data)

    # Find the level by identifier or use the first
    level = None
    if level_identifier:
        for lvl in project.levels:
            if lvl.identifier == level_identifier:
                level = lvl
                break
    if not level:
        level = project.levels[0]

    LEVEL_MANAGER.clear()

    # Load entities from all "Entities" layers
    entity_count = 0
    if level.layer_instances:
        for layer in level.layer_instances:
            if layer.type == "Entities":
                for ent in layer.entity_instances:
                    pos = ent.px
                    size = [ent.width, ent.height]
                    # Try to use smart_color, fallback to BLACK
                    color_name = ent.smart_color.lstrip("#").upper()
                    color = COLOR_MAP.get(color_name, rl.BLACK)
                    entity = Entity(ent.identifier, pos, size, color, scripts=[])
                    LEVEL_MANAGER.add_object(entity)
                    entity_count += 1

    print(f"Loaded LDtk level '{level.identifier}' with {entity_count} entities.")