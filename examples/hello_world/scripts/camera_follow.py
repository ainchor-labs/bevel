"""Camera follow script."""


class CameraFollow:
    """Makes the camera follow a target node."""

    def __init__(self):
        self.node = None
        self.target_name = "Player"
        self.target = None
        self.offset_y = -50  # Look slightly above the player

    def start(self):
        """Called before the first update."""
        # Find the player node
        if self.node._scene is not None:
            self.target = self.node._scene.find_node(self.target_name)
            if self.target:
                print(f"Camera following: {self.target.name}")
            else:
                print(f"Warning: Could not find target '{self.target_name}'")

    def update(self, delta: float):
        """Called every frame."""
        if self.target is None:
            return

        # Update camera position to follow target
        target_pos = self.target.global_position
        self.node.position.x = target_pos.x
        self.node.position.y = target_pos.y + self.offset_y
