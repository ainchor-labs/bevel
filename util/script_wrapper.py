class ScriptWrapper:
    def __init__(self, module, entity):
        self.module = module
        self.entity = entity

        # Optionally: call an init hook if the script has one
        if hasattr(self.module, "bind_entity"):
            self.module.bind_entity(self.entity)

    def start(self):
        if hasattr(self.module, "start"):
            self.module.start(self.entity)

    def update(self, dt):
        if hasattr(self.module, "update"):
            self.module.update(dt, self.entity)

    def physics_update(self, dt):
        if hasattr(self.module, "physics_update"):
            self.module.physics_update(dt, self.entity)