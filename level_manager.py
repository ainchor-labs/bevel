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

LEVEL_MANAGER = LevelManager()
