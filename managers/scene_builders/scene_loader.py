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
            # Fallback to top-level objects for compatibility
            objects = self.config.get('objects', {})
        
        if not objects:
            raise ValueError("Composite scene type requires 'objects' property in scene config")
        
        print(f"Loading Composite scene with objects: {list(objects.keys())}")

        # Create entities using the scene manager's create_entity method
        for obj_name, obj_data in objects.items():
            print(f"Creating entity '{obj_name}' from composite scene")
            self.manager._create_entity(obj_name, obj_data)


class LDtkSceneLoader(SceneLoader):
    def load(self):
        scene_props = self.config.get('scene', {}).get('properties', {})
        ldtk_path = scene_props.get('ldtk_path')
        level_identifier = scene_props.get('level_identifier')
        if not ldtk_path:
            raise ValueError("LDtk scene type requires 'ldtk_path' property")
        print(f"Loading LDtk scene: {ldtk_path} (level: {level_identifier})")
        load_ldtk_level(ldtk_path, level_identifier)