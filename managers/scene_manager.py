from level_manager import LEVEL_MANAGER
from factories.entity_factory import EntityFactory
import yaml
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
        """Create an entity from YAML object data using the EntityFactory."""
        print(f"Creating entity '{obj_name}' from YAML data")
        
        # Use the factory to create the entity
        entity = EntityFactory.create_entity(obj_name, obj_data)
        
        # Start the entity and add to level manager
        entity.start()
        LEVEL_MANAGER.add_object(entity)
        
        print(f"Entity '{obj_name}' added to LEVEL_MANAGER. Total objects: {len(LEVEL_MANAGER.objects)}")
        print(f"Entity details - pos: {entity.position}, size: {entity.size}, type: {entity.entity_type}")
        
        return entity

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