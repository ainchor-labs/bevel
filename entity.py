import importlib.util
import raylibpy as rl
from util.script_wrapper import ScriptWrapper
from rendering.entity_renderer import EntityRenderer


class Entity:
    def __init__(self, name, position, size, color, scripts, entity_type='shape', sprite_path=None, x_scale=1.0, y_scale=1.0):
        self.name = name
        self.position = rl.Vector2(position[0], position[1])
        self.size = rl.Vector2(size[0], size[1])
        self.color = color
        self.scripts = []
        self.debug = False
        
        # Entity type and rendering properties
        self.entity_type = entity_type
        self.x_scale = x_scale
        self.y_scale = y_scale
        
        # Sprite-related properties
        self.sprite_path = sprite_path
        self.texture = None
        
        # Shape-related properties
        self.shape = 'rectangle'  # Default shape
        
        # Custom properties from YAML
        self.tiled_properties = {}
        
        # Load scripts and sprite if applicable
        self.load_scripts(scripts)
        if self.entity_type == 'sprite' and self.sprite_path:
            self.load_sprite()

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

    def render(self):
        """Render the entity using the EntityRenderer."""
        EntityRenderer.render_entity(self)
        
        # Call render on scripts if they have it
        for script in self.scripts:
            if hasattr(script, "render"):
                script.render()

    def load_sprite(self):
        """Load the sprite texture from file."""
        if not self.sprite_path:
            print(f"No sprite path provided for entity {self.name}")
            return
            
        try:
            self.texture = rl.load_texture(self.sprite_path.encode())
            
            # If size wasn't specified, use the texture's natural size
            if self.size.x == 0:
                self.size.x = self.texture.width
            if self.size.y == 0:
                self.size.y = self.texture.height
                
            print(f"Loaded sprite: {self.sprite_path} (texture: {self.texture.width}x{self.texture.height}, entity size: {self.size.x}x{self.size.y})")
        except Exception as e:
            print(f"Failed to load sprite {self.sprite_path} for entity {self.name}: {e}")
            self.texture = None

    def cleanup(self):
        """Clean up resources when entity is destroyed."""
        if self.texture:
            rl.unload_texture(self.texture)
            self.texture = None