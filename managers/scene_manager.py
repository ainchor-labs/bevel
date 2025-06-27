import yaml
from util.funcs import load_game_config

class SceneManager():
    def __init__(self):
        self.curr_level = None
        self.curr_level_config = None

    def load_level(self, level_file):
        self.curr_scene = level_file
        self.curr_level_config = load_game_config(level_file)

    def get_scene_name(self):
        return self.curr_level_config.get("name", "Name not found")
    
