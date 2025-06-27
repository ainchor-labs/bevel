import yaml
import raylibpy as rl
from .consts import COLOR_MAP

class FileParser:
    def __init__(self, filepath):
        self.filepath = filepath
        self.data = None

    def load(self):
        """Load and parse the YAML file."""
        with open(self.filepath, 'r') as file:
            self.data = yaml.safe_load(file)
        return self.data

    def get_level_name(self):
        """Return the level name, or 'Unnamed Level' if missing."""
        if self.data is None:
            raise ValueError("File not loaded yet. Call load() first.")
        return self.data.get('name', 'My Awseome Level')

    def get_objects(self):
        """Return the dictionary of objects in the level."""
        if self.data is None:
            raise ValueError("File not loaded yet. Call load() first.")
        return self.data.get('objects', {})
