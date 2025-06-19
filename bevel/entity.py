import importlib.util
import raylibpy as rl
from util.script_wrapper import ScriptWrapper


class Entity:
    def __init__(self, name, position, size, color, scripts):
        self.name = name
        self.position = rl.Vector2(position[0], position[1])
        self.size = rl.Vector2(size[0], size[1])
        self.color = color
        self.scripts = []
        self.load_scripts(scripts)
        self.debug = False  # default; can set later


    def load_scripts(self, script_files):
        for file in script_files:
            spec = importlib.util.spec_from_file_location(file, file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            # Wrap in ScriptWrapper
            wrapped_script = ScriptWrapper(module, self)
            self.scripts.append(wrapped_script)

    def start(self):
        for script in self.scripts:
            if hasattr(script, "start"):
                script.start()

    def update(self, dt):
        for script in self.scripts:
            if hasattr(script, "update"):
                script.update(dt)

    def physics_update(self, dt):
        for script in self.scripts:
            if hasattr(script, "physics_update"):
                script.physics_update(dt)
