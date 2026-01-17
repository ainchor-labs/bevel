"""Player controller with physics-based movement."""

from bevel.input import Input
from bevel.utils.math import Vector2
from bevel import Time


class PlayerController:
    """Controls the player character with physics."""

    def __init__(self):
        self.node = None
        self.move_speed = 300.0
        self.jump_force = 25.0
        self.on_ground = False
        self.start_position = Vector2(640, 300)

    def awake(self):
        """Called when the node is created."""
        print(f"Player '{self.node.name}' ready!")
        print("Controls: A/D or Left/Right to move, Space/W/Up to jump, R to reset")

    def start(self):
        """Called before the first update."""
        self.start_position = self.node.position.copy()

    def update(self, delta: float):
        """Called every frame."""
        if self.node._body is None:
            return

        # Get current velocity
        velocity = self.node.get_linear_velocity()

        # Horizontal movement
        move_dir = 0.0
        if Input.is_action_pressed("move_left"):
            move_dir -= 1.0
        if Input.is_action_pressed("move_right"):
            move_dir += 1.0

        # Set horizontal velocity directly for responsive controls
        target_vx = move_dir * self.move_speed * Time.delta_time
        velocity.x = target_vx

        # Ground check (simple version - check if vertical velocity is near zero)
        # A more robust solution would use raycasts or contact checking
        self.on_ground = abs(velocity.y) < 1.0

        # Jumping
        if Input.is_action_just_pressed("jump") and self.on_ground:
            velocity.y = -self.jump_force
            print("Jump!")

        # Apply velocity
        self.node.set_linear_velocity(velocity)

        # Reset position if fallen off screen
        if self.node.position.y > 800 or Input.is_action_just_pressed("reset"):
            self._reset_position()

    def _reset_position(self):
        """Reset player to starting position."""
        self.node.position = self.start_position.copy()
        self.node.set_linear_velocity(Vector2.zero())
        # Sync the physics body
        if self.node._body is not None:
            self.node._body.position = (
                self.start_position.x / 32.0,  # Convert to meters
                self.start_position.y / 32.0
            )
            self.node._body.linearVelocity = (0, 0)
        print("Position reset!")

    def on_collision_enter(self, other):
        """Called when collision starts."""
        # Could be used for more precise ground detection
        pass

    def on_collision_exit(self, other):
        """Called when collision ends."""
        pass
