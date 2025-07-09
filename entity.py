import importlib.util
import raylibpy as rl
from util.script_wrapper import ScriptWrapper


class Entity:
    def __init__(self, name, position, size, color, scripts, entity_type='shape', sprite_path=None, x_scale=1.0, y_scale=1.0):
        self.name = name
        self.position = rl.Vector2(position[0], position[1])
        self.size = rl.Vector2(size[0], size[1])
        self.color = color
        self.scripts = []
        self.load_scripts(scripts)
        self.debug = False  # default; can set later
        
        # Sprite-related properties
        self.entity_type = entity_type
        self.sprite_path = sprite_path
        self.texture = None
        
        # Scaling properties
        self.x_scale = x_scale
        self.y_scale = y_scale
        
        # Load sprite if this is a sprite entity
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
        """Render the entity based on its type (shape or sprite)."""
        if self.entity_type == 'sprite' and self.texture:
            # Render sprite with scaling
            scaled_width = self.size.x * self.x_scale
            scaled_height = self.size.y * self.y_scale
            
            dest_rect = rl.Rectangle(
                self.position.x, self.position.y,
                scaled_width, scaled_height
            )
            source_rect = rl.Rectangle(
                0, 0,
                self.texture.width, self.texture.height
            )
            rl.draw_texture_pro(self.texture, source_rect, dest_rect, rl.Vector2(0, 0), 0, self.color)
        else:
            # Render shape with scaling (existing shape rendering code)
            shape = getattr(self, 'shape', 'rectangle')
            
            scaled_width = int(self.size.x * self.x_scale)
            scaled_height = int(self.size.y * self.y_scale)
            
            if shape == 'circle':
                # For circles, use scaled width as diameter, calculate radius
                radius = int(scaled_width / 2)
                center_x = int(self.position.x + radius)
                center_y = int(self.position.y + radius)
                rl.draw_circle(center_x, center_y, radius, self.color)
            else:
                # Default to rectangle with scaling
                rl.draw_rectangle(int(self.position.x), int(self.position.y),
                                 scaled_width, scaled_height, self.color)
        
        # If debug is enabled, draw the entity name
        if self.debug:
            rl.draw_text(self.name.encode(), int(self.position.x), int(self.position.y) - 20, 10, rl.DARKGRAY)
        
        # Call render on scripts if they have it
        for script in self.scripts:
            if hasattr(script, "render"):
                script.render()

    def load_sprite(self):
        """Load the sprite texture from file."""
        try:
            self.texture = rl.load_texture(self.sprite_path.encode())
            # If size wasn't specified, use the texture's natural size
            if self.size.x == 0 or self.size.y == 0:
                self.size = rl.Vector2(self.texture.width, self.texture.height)
            print(f"Loaded sprite: {self.sprite_path} (size: {self.texture.width}x{self.texture.height})")
        except Exception as e:
            print(f"Failed to load sprite {self.sprite_path}: {e}")
            self.texture = None

    def cleanup(self):
        """Clean up resources when entity is destroyed."""
        if self.texture:
            rl.unload_texture(self.texture)
            self.texture = None