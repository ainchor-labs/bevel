"""2D node types for Bevel engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
from enum import Enum, auto

from bevel.core.node import Node
from bevel.utils.math import Vector2


@dataclass
class Node2D(Node):
    """Base class for 2D nodes.

    Extends Node with 2D-specific functionality.
    """

    def move_local(self, offset: Vector2) -> None:
        """Move by offset in local space (affected by rotation).

        Args:
            offset: The offset to move by
        """
        rotated = offset.rotated(self.rotation)
        self.position = self.position + rotated

    def move_global(self, offset: Vector2) -> None:
        """Move by offset in global space.

        Args:
            offset: The offset to move by
        """
        self.global_position = self.global_position + offset

    def look_at(self, target: Vector2) -> None:
        """Rotate to face a target position.

        Args:
            target: The position to look at
        """
        self.transform.look_at(target)

    def get_direction_to(self, target: Vector2) -> Vector2:
        """Get the normalized direction to a target.

        Args:
            target: The target position

        Returns:
            A normalized direction vector
        """
        return (target - self.global_position).normalized()


@dataclass
class Sprite(Node2D):
    """A node that displays a texture/sprite.

    Attributes:
        texture: Name of the texture resource to display
        centered: Whether the sprite is centered on its position
        flip_h: Flip the sprite horizontally
        flip_v: Flip the sprite vertically
        modulate: Color modulation (RGBA)
        region_enabled: Whether to use a region of the texture
        region_rect: The region rectangle (x, y, width, height)
    """

    texture: str = ""
    centered: bool = True
    flip_h: bool = False
    flip_v: bool = False
    modulate: tuple[int, int, int, int] = (255, 255, 255, 255)
    region_enabled: bool = False
    region_rect: tuple[int, int, int, int] = (0, 0, 0, 0)
    _texture_data: Any = field(default=None, repr=False)

    def get_texture_size(self) -> Vector2:
        """Get the size of the texture.

        Returns:
            The texture dimensions as a Vector2
        """
        if self._texture_data is not None:
            # Will be set by renderer when texture is loaded
            return Vector2(
                getattr(self._texture_data, "width", 0),
                getattr(self._texture_data, "height", 0)
            )
        return Vector2.zero()

    def get_rect(self) -> tuple[float, float, float, float]:
        """Get the bounding rectangle of the sprite.

        Returns:
            Tuple of (x, y, width, height)
        """
        size = self.get_texture_size()
        pos = self.global_position

        if self.centered:
            return (
                pos.x - size.x / 2,
                pos.y - size.y / 2,
                size.x,
                size.y
            )
        return (pos.x, pos.y, size.x, size.y)


@dataclass
class Camera2D(Node2D):
    """A 2D camera for viewing the scene.

    Attributes:
        current: Whether this is the active camera
        zoom: Camera zoom level (1.0 = 100%)
        offset: Screen offset from center
        smoothing_enabled: Enable position smoothing
        smoothing_speed: Speed of position smoothing (0-1)
        limit_left: Left boundary limit
        limit_right: Right boundary limit
        limit_top: Top boundary limit
        limit_bottom: Bottom boundary limit
    """

    current: bool = False
    zoom: float = 1.0
    offset: Vector2 = field(default_factory=Vector2.zero)
    smoothing_enabled: bool = False
    smoothing_speed: float = 0.1
    limit_left: float = float("-inf")
    limit_right: float = float("inf")
    limit_top: float = float("-inf")
    limit_bottom: float = float("inf")
    _target_position: Vector2 = field(default_factory=Vector2.zero, repr=False)
    _actual_position: Vector2 = field(default_factory=Vector2.zero, repr=False)

    def __post_init__(self) -> None:
        super().__post_init__()
        if isinstance(self.offset, (list, tuple)):
            self.offset = Vector2(self.offset[0], self.offset[1])
        self._target_position = self.global_position.copy()
        self._actual_position = self.global_position.copy()

    def make_current(self) -> None:
        """Make this the active camera."""
        # Deactivate other cameras in scene
        if self._scene is not None and self._scene.root is not None:
            for cam in self._scene.root.find_nodes_by_type(Camera2D):
                cam.current = False
        self.current = True

    def get_camera_position(self) -> Vector2:
        """Get the actual camera position (with smoothing applied).

        Returns:
            The camera's effective position
        """
        if self.smoothing_enabled:
            return self._actual_position
        return self.global_position

    def update_smoothing(self, delta: float) -> None:
        """Update camera position smoothing.

        Args:
            delta: Time since last frame
        """
        if not self.smoothing_enabled:
            self._actual_position = self.global_position.copy()
            return

        self._target_position = self.global_position.copy()
        self._actual_position = self._actual_position.lerp(
            self._target_position,
            min(1.0, self.smoothing_speed * delta * 60)
        )

    def apply_limits(self, screen_size: Vector2) -> Vector2:
        """Apply camera limits to position.

        Args:
            screen_size: The screen dimensions

        Returns:
            The clamped camera position
        """
        pos = self.get_camera_position()
        half_width = screen_size.x / (2 * self.zoom)
        half_height = screen_size.y / (2 * self.zoom)

        x = max(self.limit_left + half_width, min(self.limit_right - half_width, pos.x))
        y = max(self.limit_top + half_height, min(self.limit_bottom - half_height, pos.y))

        return Vector2(x, y)

    def screen_to_world(self, screen_pos: Vector2, screen_size: Vector2) -> Vector2:
        """Convert screen coordinates to world coordinates.

        Args:
            screen_pos: Position on screen
            screen_size: The screen dimensions

        Returns:
            The world position
        """
        cam_pos = self.get_camera_position()
        center = Vector2(screen_size.x / 2, screen_size.y / 2)
        relative = (screen_pos - center) / self.zoom
        return cam_pos + relative + self.offset

    def world_to_screen(self, world_pos: Vector2, screen_size: Vector2) -> Vector2:
        """Convert world coordinates to screen coordinates.

        Args:
            world_pos: Position in world
            screen_size: The screen dimensions

        Returns:
            The screen position
        """
        cam_pos = self.get_camera_position()
        center = Vector2(screen_size.x / 2, screen_size.y / 2)
        relative = world_pos - cam_pos - self.offset
        return center + relative * self.zoom


class BodyType(Enum):
    """Physics body types."""

    STATIC = auto()
    KINEMATIC = auto()
    DYNAMIC = auto()


class ShapeType(Enum):
    """Collision shape types."""

    RECTANGLE = auto()
    CIRCLE = auto()
    POLYGON = auto()  # Arbitrary convex polygon with custom vertices


@dataclass
class PhysicsConfig:
    """Physics configuration for a collision shape."""

    body_type: BodyType = BodyType.DYNAMIC
    mass: float = 1.0
    friction: float = 0.3
    restitution: float = 0.0
    linear_damping: float = 0.0
    angular_damping: float = 0.0
    fixed_rotation: bool = False
    is_sensor: bool = False
    gravity_scale: float = 1.0
    is_bullet: bool = False  # Enable CCD for fast-moving objects


@dataclass
class CollisionShape2D(Node2D):
    """A collision shape for physics interactions.

    Attributes:
        shape: The type of collision shape
        size: Size for rectangle shapes (width, height)
        radius: Radius for circle shapes
        vertices: Vertices for polygon shapes (list of Vector2, max 8 vertices)
        physics: Physics configuration
        collision_layer: Layers this shape is on
        collision_mask: Layers this shape collides with
    """

    shape: ShapeType = ShapeType.RECTANGLE
    size: Vector2 = field(default_factory=lambda: Vector2(32, 32))
    radius: float = 16.0
    vertices: list[Vector2] = field(default_factory=list)  # For polygon shapes
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    collision_layer: int = 1
    collision_mask: int = 1
    _body: Any = field(default=None, repr=False)  # Box2D body reference

    def __post_init__(self) -> None:
        super().__post_init__()
        if isinstance(self.size, (list, tuple)):
            self.size = Vector2(self.size[0], self.size[1])
        if isinstance(self.shape, str):
            self.shape = ShapeType[self.shape.upper()]
        if isinstance(self.physics, dict):
            physics_dict = self.physics
            if "body_type" in physics_dict and isinstance(physics_dict["body_type"], str):
                physics_dict["body_type"] = BodyType[physics_dict["body_type"].upper()]
            self.physics = PhysicsConfig(**physics_dict)
        # Convert vertices from list of lists/tuples to Vector2
        if self.vertices:
            self.vertices = [
                Vector2(v[0], v[1]) if isinstance(v, (list, tuple)) else v
                for v in self.vertices
            ]

    @property
    def body(self) -> Any:
        """Get the physics body."""
        return self._body

    def apply_force(self, force: Vector2) -> None:
        """Apply a force to the physics body.

        Args:
            force: The force vector to apply
        """
        if self._body is not None:
            self._body.ApplyForceToCenter((force.x, force.y), True)

    def apply_impulse(self, impulse: Vector2) -> None:
        """Apply an impulse to the physics body.

        Args:
            impulse: The impulse vector to apply
        """
        if self._body is not None:
            pos = self._body.position
            self._body.ApplyLinearImpulse((impulse.x, impulse.y), pos, True)

    def set_linear_velocity(self, velocity: Vector2) -> None:
        """Set the linear velocity of the physics body.

        Args:
            velocity: The new velocity
        """
        if self._body is not None:
            self._body.linearVelocity = (velocity.x, velocity.y)

    def get_linear_velocity(self) -> Vector2:
        """Get the linear velocity of the physics body.

        Returns:
            The current velocity
        """
        if self._body is not None:
            vel = self._body.linearVelocity
            return Vector2(vel[0], vel[1])
        return Vector2.zero()

    def set_angular_velocity(self, velocity: float) -> None:
        """Set the angular velocity of the physics body.

        Args:
            velocity: The new angular velocity in radians/second
        """
        if self._body is not None:
            self._body.angularVelocity = velocity

    def get_angular_velocity(self) -> float:
        """Get the angular velocity of the physics body.

        Returns:
            The current angular velocity
        """
        if self._body is not None:
            return self._body.angularVelocity
        return 0.0


# Type registry for scene loader
NODE_TYPES: dict[str, type[Node]] = {
    "Node": Node,
    "Node2D": Node2D,
    "Sprite": Sprite,
    "Camera2D": Camera2D,
    "CollisionShape2D": CollisionShape2D,
}


def register_node_type(name: str, node_class: type[Node]) -> None:
    """Register a custom node type.

    Args:
        name: The name to use in YAML files
        node_class: The node class
    """
    NODE_TYPES[name] = node_class


def get_node_type(name: str) -> type[Node] | None:
    """Get a node type by name.

    Args:
        name: The type name

    Returns:
        The node class or None if not found
    """
    return NODE_TYPES.get(name)
