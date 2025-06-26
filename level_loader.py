from util.file_parser import FileParser
from entity import Entity
from level_manager import LEVEL_MANAGER as lm
import raylibpy as rl

COLOR_MAP = {
    'GREEN': rl.GREEN,
    'BLUE': rl.BLUE,
    'RED': rl.RED,
    'YELLOW': rl.YELLOW,
    'LIGHTGRAY': rl.LIGHTGRAY,
    'DARKGRAY': rl.DARKGRAY,
    'BLACK': rl.BLACK,
    'WHITE': rl.WHITE,
}

def load_level(yaml_path):
    # Clear any existing level data
    lm.clear()

    parser = FileParser(yaml_path)
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

        lm.add_object(entity)

    return parser.get_level_name()
