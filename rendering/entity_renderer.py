"""
Entity renderer for handling different rendering types.
"""
import raylibpy as rl


class EntityRenderer:
    """Handles rendering of different entity types."""
    
    @staticmethod
    def render_entity(entity):
        """Render an entity based on its type."""
        if entity.entity_type == 'sprite':
            EntityRenderer._render_sprite(entity)
        else:
            EntityRenderer._render_shape(entity)
        
        # Render debug info if enabled
        if entity.debug:
            EntityRenderer._render_debug_info(entity)
    
    @staticmethod
    def _render_sprite(entity):
        """Render a sprite entity."""
        if not entity.texture:
            # Fallback to rectangle if texture failed to load
            EntityRenderer._render_rectangle(entity, rl.MAGENTA)  # Magenta indicates missing texture
            return
        
        scaled_width = entity.size.x * entity.x_scale
        scaled_height = entity.size.y * entity.y_scale
        
        dest_rect = rl.Rectangle(
            entity.position.x, entity.position.y,
            scaled_width, scaled_height
        )
        source_rect = rl.Rectangle(
            0, 0,
            entity.texture.width, entity.texture.height
        )
        rl.draw_texture_pro(entity.texture, source_rect, dest_rect, rl.Vector2(0, 0), 0, entity.color)
    
    @staticmethod
    def _render_shape(entity):
        """Render a shape entity."""
        shape = getattr(entity, 'shape', 'rectangle')
        
        if shape == 'circle':
            EntityRenderer._render_circle(entity)
        else:
            EntityRenderer._render_rectangle(entity, entity.color)
    
    @staticmethod
    def _render_rectangle(entity, color):
        """Render a rectangle shape."""
        scaled_width = int(entity.size.x * entity.x_scale)
        scaled_height = int(entity.size.y * entity.y_scale)
        
        rl.draw_rectangle(
            int(entity.position.x), int(entity.position.y),
            scaled_width, scaled_height, color
        )
    
    @staticmethod
    def _render_circle(entity):
        """Render a circle shape."""
        scaled_width = entity.size.x * entity.x_scale
        radius = int(scaled_width / 2)
        center_x = int(entity.position.x + radius)
        center_y = int(entity.position.y + radius)
        rl.draw_circle(center_x, center_y, radius, entity.color)
    
    @staticmethod
    def _render_debug_info(entity):
        """Render debug information for an entity."""
        text = f"{entity.name} ({entity.entity_type})"
        text_y = int(entity.position.y) - 20
        rl.draw_text(text.encode(), int(entity.position.x), text_y, 10, rl.DARKGRAY)
