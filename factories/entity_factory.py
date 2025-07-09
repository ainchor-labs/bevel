"""
Entity factory for creating different types of entities from YAML configuration.
"""
import raylibpy as rl
from util.consts import COLOR_MAP
from entity import Entity


class EntityFactory:
    """Factory class for creating entities from YAML data."""
    
    @staticmethod
    def create_entity(name, data):
        """Create an entity from YAML data."""
        entity_type = data.get('type', 'shape')
        
        # Common properties
        position = [data.get('x', 0), data.get('y', 0)]
        scripts = data.get('scripts', [])
        debug = data.get('debug', False)
        x_scale = data.get('x_scale', 1.0)
        y_scale = data.get('y_scale', 1.0)
        properties = data.get('properties', {})
        
        if entity_type == 'sprite':
            return EntityFactory._create_sprite_entity(
                name, position, data, scripts, debug, x_scale, y_scale, properties
            )
        else:
            return EntityFactory._create_shape_entity(
                name, position, data, scripts, debug, x_scale, y_scale, properties
            )
    
    @staticmethod
    def _create_sprite_entity(name, position, data, scripts, debug, x_scale, y_scale, properties):
        """Create a sprite-based entity."""
        size = [data.get('width', 0), data.get('height', 0)]  # 0 means auto-detect
        sprite_path = data.get('path', '')
        color = EntityFactory._parse_color(data.get('color', 'WHITE'))
        
        entity = Entity(name, position, size, color, scripts, 'sprite', sprite_path, x_scale, y_scale)
        entity.debug = debug
        entity.tiled_properties = dict(properties)
        
        return entity
    
    @staticmethod
    def _create_shape_entity(name, position, data, scripts, debug, x_scale, y_scale, properties):
        """Create a shape-based entity."""
        size = [data.get('width', 50), data.get('height', 50)]
        color = EntityFactory._parse_color(data.get('color', 'BLACK'))
        shape = data.get('shape', 'rectangle')
        
        entity = Entity(name, position, size, color, scripts, 'shape', None, x_scale, y_scale)
        entity.debug = debug
        entity.shape = shape
        entity.tiled_properties = dict(properties)
        
        return entity
    
    @staticmethod
    def _parse_color(color_data):
        """Parse color from string or direct value."""
        if isinstance(color_data, str):
            return COLOR_MAP.get(color_data.upper(), rl.BLACK)
        return color_data
