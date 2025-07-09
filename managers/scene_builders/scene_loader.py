from managers.ldtk_loader import load_ldtk_level
import raylibpy as rl
from util.consts import COLOR_MAP

# Factory mapping for scene loaders
class SceneLoader:
    def __init__(self, manager, config):
        self.manager = manager
        self.config = config

    def load(self):
        raise NotImplementedError

class CompositeSceneLoader(SceneLoader):
    def load(self):
        # Load the composite scene data from config
        scene_props = self.config.get('scene', {})
        objects = scene_props.get('objects')
        if not objects:
            raise ValueError("Composite scene type requires 'objects' property in scene config")
        print(f"Loading Composite scene with objects: {list(objects.keys())}")

        scene_objects = []
        # Load the composite scene (custom logic, e.g., parse and instantiate objects)
        for name, obj_cfg in objects.items():
            # Simulate object creation
            scene_objects.append({'name': name, 'config': obj_cfg})

        # Render the scene (custom logic, e.g., draw objects to screen)
        for obj in scene_objects:
            print(f" - {obj['name']}: {obj['config']}")
            obj_type = obj['config'].get('type')
            if obj_type == 'sprite':
                sprite_path = obj['config'].get('sprite')
                position = obj['config'].get('position', [0, 0])
                if 'sprite' in obj['config']:
                    # Load texture (in a real app, cache textures to avoid reloading)
                    texture = rl.load_texture(sprite_path)
                    rl.draw_texture(texture, int(position[0]), int(position[1]), rl.WHITE)
                    rl.unload_texture(texture)
            elif obj_type == 'shape':
                shape = obj['config'].get('shape')
                color = obj['config'].get('color', [255, 255, 255, 255])
                rl_color = COLOR_MAP.get(color.upper(), rl.BLACK)
                if shape == 'rectangle':
                    rect = obj['config'].get('rect', [0, 0, 10, 10])
                    rl.draw_rectangle(int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]), rl_color)
                elif shape == 'circle':
                    center = obj['config'].get('center', [0, 0])
                    radius = obj['config'].get('radius', 10)
                    rl.draw_circle(int(center[0]), int(center[1]), int(radius), rl_color)


class LDtkSceneLoader(SceneLoader):
    def load(self):
        scene_props = self.config.get('scene', {}).get('properties', {})
        ldtk_path = scene_props.get('ldtk_path')
        level_identifier = scene_props.get('level_identifier')
        if not ldtk_path:
            raise ValueError("LDtk scene type requires 'ldtk_path' property")
        print(f"Loading LDtk scene: {ldtk_path} (level: {level_identifier})")
        load_ldtk_level(ldtk_path, level_identifier)