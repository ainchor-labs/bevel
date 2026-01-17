"""3D Node types for Bevel engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, TYPE_CHECKING

from bevel.core.node import Node
from bevel.core.transform3d import Transform3D
from bevel.utils.math import Vector3, Quaternion

if TYPE_CHECKING:
    pass


class ProjectionType(Enum):
    """Camera projection types."""

    PERSPECTIVE = auto()
    ORTHOGRAPHIC = auto()


class LightType(Enum):
    """Light source types."""

    DIRECTIONAL = auto()
    POINT = auto()
    SPOT = auto()


class ShapeType3D(Enum):
    """3D collision shape types."""

    BOX = auto()
    SPHERE = auto()
    CAPSULE = auto()
    CONVEX_MESH = auto()
    TRIMESH = auto()


class BodyType3D(Enum):
    """3D physics body types."""

    STATIC = 0
    KINEMATIC = 1
    DYNAMIC = 2


@dataclass
class PhysicsConfig3D:
    """Configuration for 3D physics bodies."""

    body_type: BodyType3D = BodyType3D.DYNAMIC
    mass: float = 1.0
    friction: float = 0.5
    restitution: float = 0.0
    linear_damping: float = 0.0
    angular_damping: float = 0.05
    gravity_scale: float = 1.0
    is_sensor: bool = False


@dataclass
class Node3D(Node):
    """Base class for 3D nodes.

    Extends Node with 3D-specific transformation support.
    """

    _transform_3d: Transform3D = field(default_factory=Transform3D)

    def __post_init__(self) -> None:
        super().__post_init__()
        # Link 3D transform to this node
        self._transform_3d._node = self

    # Position shortcuts

    @property
    def position(self) -> Vector3:  # type: ignore[override]
        """Get the local position."""
        return self._transform_3d.position

    @position.setter
    def position(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local position."""
        self._transform_3d.position = value

    @property
    def global_position(self) -> Vector3:  # type: ignore[override]
        """Get the global position."""
        return self._transform_3d.global_position

    @global_position.setter
    def global_position(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the global position."""
        self._transform_3d.global_position = value

    # Rotation shortcuts (quaternion)

    @property
    def rotation(self) -> Quaternion:  # type: ignore[override]
        """Get the local rotation as a quaternion."""
        return self._transform_3d.rotation

    @rotation.setter
    def rotation(self, value: Quaternion) -> None:
        """Set the local rotation as a quaternion."""
        self._transform_3d.rotation = value

    @property
    def global_rotation(self) -> Quaternion:
        """Get the global rotation as a quaternion."""
        return self._transform_3d.global_rotation

    @global_rotation.setter
    def global_rotation(self, value: Quaternion) -> None:
        """Set the global rotation as a quaternion."""
        self._transform_3d.global_rotation = value

    # Euler angle shortcuts

    @property
    def euler_angles(self) -> Vector3:
        """Get the local rotation as Euler angles (radians)."""
        return self._transform_3d.euler_angles

    @euler_angles.setter
    def euler_angles(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local rotation from Euler angles (radians)."""
        self._transform_3d.euler_angles = value

    @property
    def rotation_degrees(self) -> Vector3:  # type: ignore[override]
        """Get the local rotation as Euler angles (degrees)."""
        return self._transform_3d.euler_angles_degrees

    @rotation_degrees.setter
    def rotation_degrees(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local rotation from Euler angles (degrees)."""
        self._transform_3d.euler_angles_degrees = value

    # Scale shortcuts

    @property
    def scale(self) -> Vector3:  # type: ignore[override]
        """Get the local scale."""
        return self._transform_3d.scale

    @scale.setter
    def scale(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local scale."""
        self._transform_3d.scale = value

    @property
    def global_scale(self) -> Vector3:
        """Get the global scale."""
        return self._transform_3d.global_scale

    @global_scale.setter
    def global_scale(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the global scale."""
        self._transform_3d.global_scale = value

    # Basis vectors

    @property
    def forward(self) -> Vector3:
        """Get the forward direction."""
        return self._transform_3d.forward

    @property
    def back(self) -> Vector3:
        """Get the back direction."""
        return self._transform_3d.back

    @property
    def right(self) -> Vector3:
        """Get the right direction."""
        return self._transform_3d.right

    @property
    def left(self) -> Vector3:
        """Get the left direction."""
        return self._transform_3d.left

    @property
    def up(self) -> Vector3:
        """Get the up direction."""
        return self._transform_3d.up

    @property
    def down(self) -> Vector3:
        """Get the down direction."""
        return self._transform_3d.down

    # Methods

    def look_at(self, target: Vector3, up: Vector3 | None = None) -> None:
        """Rotate to look at a target position.

        Args:
            target: The position to look at
            up: The up vector (defaults to Vector3.up())
        """
        self._transform_3d.look_at(target, up)

    def translate_local(self, offset: Vector3) -> None:
        """Move in local space.

        Args:
            offset: The offset in local coordinates
        """
        self._transform_3d.translate(offset)

    def translate_global(self, offset: Vector3) -> None:
        """Move in world space.

        Args:
            offset: The offset in world coordinates
        """
        self._transform_3d.translate_global(offset)

    def rotate_local(self, axis: Vector3, angle: float) -> None:
        """Rotate around an axis in local space.

        Args:
            axis: The rotation axis
            angle: The angle in radians
        """
        self._transform_3d.rotate(axis, angle)

    def rotate_global(self, axis: Vector3, angle: float) -> None:
        """Rotate around an axis in world space.

        Args:
            axis: The rotation axis
            angle: The angle in radians
        """
        self._transform_3d.rotate_global(axis, angle)


@dataclass
class MeshInstance(Node3D):
    """A node that renders a 3D mesh.

    Renders a mesh loaded from the resource manager.
    """

    mesh: str = ""  # Resource name
    material: str = ""  # Material/shader name
    cast_shadows: bool = True
    receive_shadows: bool = True
    _mesh_data: Any = field(default=None, repr=False)  # Loaded mesh data


@dataclass
class Camera3D(Node3D):
    """A 3D camera for viewing the scene.

    Supports both perspective and orthographic projections.
    """

    current: bool = False
    projection: ProjectionType = ProjectionType.PERSPECTIVE
    fov: float = 70.0  # Field of view in degrees (perspective only)
    near: float = 0.1
    far: float = 1000.0
    ortho_size: float = 10.0  # Half-height of view (orthographic only)

    # Class variable to track current camera
    _current_camera: "Camera3D | None" = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.current:
            self.make_current()

    def make_current(self) -> None:
        """Make this camera the active camera."""
        # Clear previous current camera
        if Camera3D._current_camera is not None and Camera3D._current_camera is not self:
            Camera3D._current_camera.current = False
        Camera3D._current_camera = self
        self.current = True

    @staticmethod
    def get_current() -> "Camera3D | None":
        """Get the currently active camera."""
        return Camera3D._current_camera

    def get_fov_radians(self) -> float:
        """Get the field of view in radians."""
        import math

        return math.radians(self.fov)

    def screen_to_world(
        self, screen_pos: tuple[float, float], screen_size: tuple[int, int], depth: float
    ) -> Vector3:
        """Convert screen coordinates to world position.

        Args:
            screen_pos: Screen position (x, y)
            screen_size: Screen dimensions (width, height)
            depth: Distance from camera

        Returns:
            World position
        """
        from bevel.utils.math import Matrix4
        import math

        # Normalize screen coordinates to [-1, 1]
        ndc_x = (2.0 * screen_pos[0] / screen_size[0]) - 1.0
        ndc_y = 1.0 - (2.0 * screen_pos[1] / screen_size[1])

        if self.projection == ProjectionType.PERSPECTIVE:
            # Calculate ray direction from NDC
            aspect = screen_size[0] / screen_size[1]
            tan_half_fov = math.tan(math.radians(self.fov) / 2)

            # Local ray direction
            local_dir = Vector3(
                ndc_x * tan_half_fov * aspect,
                ndc_y * tan_half_fov,
                -1.0,
            ).normalized()

            # Transform to world space
            world_dir = self.rotation.rotate_vector(local_dir)
            return self.global_position + world_dir * depth
        else:
            # Orthographic
            local_pos = Vector3(
                ndc_x * self.ortho_size * (screen_size[0] / screen_size[1]),
                ndc_y * self.ortho_size,
                -depth,
            )
            return self.global_position + self.rotation.rotate_vector(local_pos)

    def world_to_screen(
        self, world_pos: Vector3, screen_size: tuple[int, int]
    ) -> tuple[float, float] | None:
        """Convert world position to screen coordinates.

        Args:
            world_pos: Position in world space
            screen_size: Screen dimensions (width, height)

        Returns:
            Screen position (x, y) or None if behind camera
        """
        from bevel.utils.math import Matrix4
        import math

        # Transform to camera space
        relative = world_pos - self.global_position
        cam_space = self.rotation.inverse().rotate_vector(relative)

        # Check if behind camera
        if cam_space.z >= 0:
            return None

        if self.projection == ProjectionType.PERSPECTIVE:
            aspect = screen_size[0] / screen_size[1]
            tan_half_fov = math.tan(math.radians(self.fov) / 2)

            ndc_x = cam_space.x / (-cam_space.z * tan_half_fov * aspect)
            ndc_y = cam_space.y / (-cam_space.z * tan_half_fov)
        else:
            aspect = screen_size[0] / screen_size[1]
            ndc_x = cam_space.x / (self.ortho_size * aspect)
            ndc_y = cam_space.y / self.ortho_size

        # Convert to screen coordinates
        screen_x = (ndc_x + 1.0) * 0.5 * screen_size[0]
        screen_y = (1.0 - ndc_y) * 0.5 * screen_size[1]

        return (screen_x, screen_y)


@dataclass
class Light3D(Node3D):
    """A light source in 3D space.

    Supports directional, point, and spot lights.
    """

    light_type: LightType = LightType.POINT
    color: tuple[int, int, int, int] = (255, 255, 255, 255)
    intensity: float = 1.0
    range: float = 10.0  # For point and spot lights
    spot_angle: float = 45.0  # Degrees, for spot lights
    cast_shadows: bool = False

    @property
    def color_normalized(self) -> tuple[float, float, float, float]:
        """Get color as normalized floats (0-1)."""
        return (
            self.color[0] / 255.0,
            self.color[1] / 255.0,
            self.color[2] / 255.0,
            self.color[3] / 255.0,
        )


@dataclass
class CollisionShape3D(Node3D):
    """A 3D collision shape for physics interactions.

    Works with the Jolt physics system.
    """

    shape: ShapeType3D = ShapeType3D.BOX
    size: Vector3 = field(default_factory=lambda: Vector3(1, 1, 1))  # For box
    radius: float = 0.5  # For sphere and capsule
    height: float = 1.0  # For capsule
    mesh: str = ""  # For convex mesh and trimesh
    physics: PhysicsConfig3D = field(default_factory=PhysicsConfig3D)
    collision_layer: int = 1
    collision_mask: int = 1
    _body: Any = field(default=None, repr=False)  # Jolt body reference

    # Physics methods (implemented by physics system)

    def apply_force(self, force: Vector3) -> None:
        """Apply a force to the physics body.

        Args:
            force: Force vector in world space
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "apply_force"):
                physics.apply_force(self._body, force)

    def apply_impulse(self, impulse: Vector3) -> None:
        """Apply an impulse to the physics body.

        Args:
            impulse: Impulse vector in world space
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "apply_impulse"):
                physics.apply_impulse(self._body, impulse)

    def apply_torque(self, torque: Vector3) -> None:
        """Apply a torque to the physics body.

        Args:
            torque: Torque vector
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "apply_torque"):
                physics.apply_torque(self._body, torque)

    def set_linear_velocity(self, velocity: Vector3) -> None:
        """Set the linear velocity of the physics body.

        Args:
            velocity: Velocity vector
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "set_linear_velocity"):
                physics.set_linear_velocity(self._body, velocity)

    def get_linear_velocity(self) -> Vector3:
        """Get the linear velocity of the physics body.

        Returns:
            Velocity vector
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "get_linear_velocity"):
                return physics.get_linear_velocity(self._body)
        return Vector3.zero()

    def set_angular_velocity(self, velocity: Vector3) -> None:
        """Set the angular velocity of the physics body.

        Args:
            velocity: Angular velocity vector
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "set_angular_velocity"):
                physics.set_angular_velocity(self._body, velocity)

    def get_angular_velocity(self) -> Vector3:
        """Get the angular velocity of the physics body.

        Returns:
            Angular velocity vector
        """
        if self._body is not None and self._scene is not None:
            physics = getattr(self._scene, "_physics_3d", None)
            if physics is not None and hasattr(physics, "get_angular_velocity"):
                return physics.get_angular_velocity(self._body)
        return Vector3.zero()


# Node type registry for 3D nodes
NODE_TYPES_3D: dict[str, type[Node]] = {
    "Node3D": Node3D,
    "MeshInstance": MeshInstance,
    "Camera3D": Camera3D,
    "Light3D": Light3D,
    "CollisionShape3D": CollisionShape3D,
}


def register_3d_types() -> None:
    """Register 3D node types with the main registry."""
    from bevel.nodes.node_2d import NODE_TYPES

    NODE_TYPES.update(NODE_TYPES_3D)


def get_node_type_3d(name: str) -> type[Node] | None:
    """Get a 3D node type by name.

    Args:
        name: The node type name

    Returns:
        The node class or None if not found
    """
    return NODE_TYPES_3D.get(name)
