from level_manager import LEVEL_MANAGER
from util.consts import COLOR_MAP
import raylibpy as rl
from entity import Entity
import yaml
# import os
# from managers.ldtk_loader import load_ldtk_level
from managers.scene_builders.scene_loader import CompositeSceneLoader, LDtkSceneLoader

class SceneManager:
    def __init__(self):
        self.curr_scene = None
        self.curr_scene_config = None

    def load_scene(self, scene_file, level_name=None):
        """Load a scene from YAML config file using a factory builder pattern."""
        LEVEL_MANAGER.clear()
        self.curr_scene = scene_file

        # Parse the YAML scene configuration
        with open(scene_file, 'r') as file:
            config = yaml.safe_load(file)

        self.curr_scene_config = config

        # Determine scene type
        scene_type = config.get('scene', {}).get('type', 'composite')

        loader_factory = {
            'composite': CompositeSceneLoader,
            'ldtk': LDtkSceneLoader,
        }

        loader_cls = loader_factory.get(scene_type)
        if not loader_cls:
            raise ValueError(f"Unknown scene type: {scene_type}")

        loader = loader_cls(self, config)
        loader.load()

    def _load_composite_scene(self, config):
        """Load scene with objects defined directly in YAML."""
        objects = config.get('objects', {})

        for obj_name, obj_data in objects.items():
            self._create_entity(obj_name, obj_data)

    def _create_entity(self, obj_name, obj_data):
        """Create an entity from YAML object data (dict)."""
        pos = [obj_data.get('x', 0), obj_data.get('y', 0)]
        size = [obj_data.get('width', 50), obj_data.get('height', 50)]

        print(f"Creating entity '{obj_name}' at pos={pos}, size={size}")

        color_name = obj_data.get('color', 'BLACK')
        scripts = obj_data.get('scripts', [])
        debug = obj_data.get('debug', False)

        if isinstance(color_name, str):
            color = COLOR_MAP.get(color_name.upper(), rl.BLACK)
        else:
            color = color_name

        print(f"Entity '{obj_name}' color: {color_name} -> {color}")

        entity = Entity(obj_name, pos, size, color, scripts)
        entity.debug = debug

        if 'properties' in obj_data:
            entity.tiled_properties = dict(obj_data['properties'])

        entity.start()
        LEVEL_MANAGER.add_object(entity)

        print(f"Entity '{obj_name}' added to LEVEL_MANAGER. Total objects: {len(LEVEL_MANAGER.objects)}")
        print(f"Entity position after creation: {entity.position}")
        print(f"Entity size after creation: {entity.size}")
        print(f"Entity color after creation: {entity.color}")

    def get_scene_name(self):
        if self.curr_scene_config:
            return self.curr_scene_config.get("name", "Unnamed Scene")
        return "No scene loaded"

    def get_scene_description(self):
        if self.curr_scene_config:
            return self.curr_scene_config.get("description", "No description")
        return "No scene loaded"

    def get_scene_type(self):
        if self.curr_scene_config:
            return self.curr_scene_config.get('scene', {}).get('type', 'composite')
        return None

global_scene_manager = SceneManager()