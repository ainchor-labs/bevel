"""Platform script for visual rendering."""


class Platform:
    """A simple platform that renders itself as a colored rectangle."""

    def __init__(self):
        self.node = None
        self.color = (100, 80, 60, 255)  # Brown-ish color for platforms

    def awake(self):
        """Called when the node is created."""
        # Set color based on node name
        if "Ground" in self.node.name:
            self.color = (80, 160, 80, 255)  # Green for ground
        elif "Left" in self.node.name:
            self.color = (160, 80, 80, 255)  # Red-ish
        elif "Right" in self.node.name:
            self.color = (80, 80, 160, 255)  # Blue-ish
        elif "Middle" in self.node.name:
            self.color = (160, 160, 80, 255)  # Yellow-ish

    def update(self, delta: float):
        """Called every frame."""
        # Platforms are static, nothing to update
        pass
