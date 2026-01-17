"""Player movement controller for 3D demo.

Demonstrates:
- 3D transform manipulation
- Input contexts
- Multi-script architecture
"""

from bevel.input import Input
from bevel.utils.math import Vector3


class PlayerMovement:
    """Handles player movement in 3D space."""

    def __init__(self) -> None:
        self.node = None  # Set by engine
        self.move_speed = 5.0
        self.sprint_multiplier = 1.5
        self.jump_force = 8.0
        self.gravity = 20.0
        self.velocity = Vector3.zero()
        self.is_grounded = True

    def awake(self) -> None:
        """Called when node enters the scene."""
        print(f"PlayerMovement attached to '{self.node.name}'")

    def start(self) -> None:
        """Called before first update."""
        print(f"Player starting at position: {self.node.position}")

    def update(self, delta: float) -> None:
        """Called every frame."""
        # Get input direction
        input_dir = Vector3.zero()

        if Input.is_action_pressed("move_forward"):
            input_dir.z -= 1
        if Input.is_action_pressed("move_backward"):
            input_dir.z += 1
        if Input.is_action_pressed("move_left"):
            input_dir.x -= 1
        if Input.is_action_pressed("move_right"):
            input_dir.x += 1

        # Normalize input
        if input_dir.length_squared > 0:
            input_dir = input_dir.normalized()

        # Calculate speed
        speed = self.move_speed
        if Input.is_action_pressed("sprint"):
            speed *= self.sprint_multiplier

        # Apply horizontal movement
        self.velocity.x = input_dir.x * speed
        self.velocity.z = input_dir.z * speed

        # Jumping
        if Input.is_action_just_pressed("jump") and self.is_grounded:
            self.velocity.y = self.jump_force
            self.is_grounded = False
            print("Jump!")

        # Apply gravity
        if not self.is_grounded:
            self.velocity.y -= self.gravity * delta

        # Move player
        transform = self.node._transform_3d
        new_pos = transform.position + self.velocity * delta

        # Simple ground check
        if new_pos.y <= 0:
            new_pos.y = 0
            self.velocity.y = 0
            self.is_grounded = True

        transform.position = new_pos

    def on_enable(self) -> None:
        """Called when node becomes active."""
        print("PlayerMovement enabled")

    def on_disable(self) -> None:
        """Called when node becomes inactive."""
        print("PlayerMovement disabled")

    # Hot reload support
    def preserve_state(self) -> dict:
        """Preserve state for hot reload."""
        return {
            "velocity_x": self.velocity.x,
            "velocity_y": self.velocity.y,
            "velocity_z": self.velocity.z,
            "is_grounded": self.is_grounded,
        }

    def restore_state(self, state: dict) -> None:
        """Restore state after hot reload."""
        self.velocity = Vector3(
            state.get("velocity_x", 0),
            state.get("velocity_y", 0),
            state.get("velocity_z", 0),
        )
        self.is_grounded = state.get("is_grounded", True)
