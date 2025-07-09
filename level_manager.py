class LevelManager:
    def __init__(self):
        self.objects = {}

    def add_object(self, entity):
        """Register an entity by name."""
        self.objects[entity.name] = entity

    def get_object(self, name):
        """Get the entity object by name."""
        return self.objects.get(name)

    def get_object_position(self, name):
        """Get the position of an entity by name."""
        entity = self.get_object(name)
        if entity:
            return entity.position
        return None
    
    def clear(self):
        """Remove all objects from the manager."""
        self.objects.clear()

    def update(self):
        """Update all objects in the level."""
        for obj in self.objects.values():  # Changed from self.objects to self.objects.values()
            if hasattr(obj, 'update'):
                obj.update()

    def render(self):
        """Render all objects in the level."""
        for obj in self.objects.values():  # Changed from self.objects to self.objects.values()
            if hasattr(obj, 'render'):
                obj.render()

LEVEL_MANAGER = LevelManager()
