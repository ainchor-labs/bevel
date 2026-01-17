"""Camera follow script for 3D demo.

Demonstrates:
- Finding nodes in the scene
- 3D transform operations
- Smooth following behavior
"""

from bevel.utils.math import Vector3


class CameraFollow:
    """Makes the camera follow a target node."""

    def __init__(self) -> None:
        self.node = None  # Set by engine
        self.target_name = "Player"
        self.offset = Vector3(0, 5, 10)
        self.smoothing = 5.0
        self._target = None

    def awake(self) -> None:
        """Called when node enters the scene."""
        print("CameraFollow initialized")

    def start(self) -> None:
        """Called before first update."""
        # Find the target node
        if self.node.scene is not None:
            self._target = self.node.scene.find_node(self.target_name)
            if self._target is not None:
                print(f"Camera following: {self._target.name}")
            else:
                print(f"Warning: Target '{self.target_name}' not found")

    def update(self, delta: float) -> None:
        """Called every frame."""
        if self._target is None:
            return

        # Get target position
        target_pos = self._target._transform_3d.position

        # Calculate desired camera position
        desired_pos = Vector3(
            target_pos.x + self.offset.x,
            target_pos.y + self.offset.y,
            target_pos.z + self.offset.z,
        )

        # Smoothly interpolate to desired position
        current_pos = self.node._transform_3d.position
        new_pos = current_pos.lerp(desired_pos, self.smoothing * delta)

        self.node._transform_3d.position = new_pos

        # Optionally: look at target
        # self.node._transform_3d.look_at(target_pos)

    def late_update(self, delta: float) -> None:
        """Called after all updates - good for camera to ensure smooth following."""
        pass  # Could move camera update here for smoother results

    # Hot reload support
    def preserve_state(self) -> dict:
        """Preserve state for hot reload."""
        return {
            "offset_x": self.offset.x,
            "offset_y": self.offset.y,
            "offset_z": self.offset.z,
            "smoothing": self.smoothing,
        }

    def restore_state(self, state: dict) -> None:
        """Restore state after hot reload."""
        self.offset = Vector3(
            state.get("offset_x", 0),
            state.get("offset_y", 5),
            state.get("offset_z", 10),
        )
        self.smoothing = state.get("smoothing", 5.0)
        # Re-find target after reload
        if self.node.scene is not None:
            self._target = self.node.scene.find_node(self.target_name)
