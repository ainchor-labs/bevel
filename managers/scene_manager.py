from level_loader import load_level
from level_manager import LEVEL_MANAGER
from util.funcs import load_game_config
from util.file_parser import FileParser
from util.consts import COLOR_MAP
import raylibpy as rl
from entity import Entity

class SceneManager():
    def __init__(self):
        self.curr_scene = None
        self.curr_scene_config = None

    def load_scene(self, scene_file):
        from level_loader import load_level
        from level_manager import LEVEL_MANAGER
        from util.funcs import load_game_config

        LEVEL_MANAGER.clear()
        self.curr_scene = scene_file
        self.curr_scene_config = load_game_config(scene_file)

        parser = FileParser(scene_file)
        parser.load()

        for obj_name, obj_data in parser.get_objects().items():
            pos = obj_data.get('position', [0, 0])
            size = obj_data.get('size', [50, 50])
            color_name = obj_data.get('color', 'BLACK')
            scripts = obj_data.get('scripts', [])
            color = COLOR_MAP.get(color_name.upper(), rl.BLACK)

            entity = Entity(obj_name, pos, size, color, scripts)
            entity.debug = obj_data.get('debug', False)
            entity.start()

            LEVEL_MANAGER.add_object(entity)

    def get_scene_name(self):
        return self.curr_scene_config.get("name", "Name not found")

global_scene_manager = SceneManager()
