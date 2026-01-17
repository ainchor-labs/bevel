"""3D Transform component for nodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from bevel.utils.math import Vector3, Quaternion, Matrix4

if TYPE_CHECKING:
    from bevel.nodes.node_3d import Node3D


@dataclass
class Transform3D:
    """A 3D transformation containing position, rotation (as quaternion), and scale.

    Supports both local and global coordinate systems with parent-relative
    transformations.
    """

    _position: Vector3 = field(default_factory=Vector3.zero)
    _rotation: Quaternion = field(default_factory=Quaternion.identity)
    _scale: Vector3 = field(default_factory=Vector3.one)
    _node: "Node3D | None" = field(default=None, repr=False)

    @property
    def position(self) -> Vector3:
        """Get the local position."""
        return self._position

    @position.setter
    def position(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local position."""
        if isinstance(value, tuple):
            self._position = Vector3(value[0], value[1], value[2])
        else:
            self._position = value

    @property
    def rotation(self) -> Quaternion:
        """Get the local rotation as a quaternion."""
        return self._rotation

    @rotation.setter
    def rotation(self, value: Quaternion) -> None:
        """Set the local rotation as a quaternion."""
        self._rotation = value

    @property
    def euler_angles(self) -> Vector3:
        """Get the local rotation as Euler angles (in radians)."""
        return self._rotation.to_euler()

    @euler_angles.setter
    def euler_angles(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local rotation from Euler angles (in radians)."""
        if isinstance(value, tuple):
            self._rotation = Quaternion.from_euler(value[0], value[1], value[2])
        else:
            self._rotation = Quaternion.from_euler(value.x, value.y, value.z)

    @property
    def euler_angles_degrees(self) -> Vector3:
        """Get the local rotation as Euler angles (in degrees)."""
        return self._rotation.to_euler_degrees()

    @euler_angles_degrees.setter
    def euler_angles_degrees(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local rotation from Euler angles (in degrees)."""
        if isinstance(value, tuple):
            self._rotation = Quaternion.from_euler_degrees(value[0], value[1], value[2])
        else:
            self._rotation = Quaternion.from_euler_degrees(value.x, value.y, value.z)

    @property
    def scale(self) -> Vector3:
        """Get the local scale."""
        return self._scale

    @scale.setter
    def scale(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the local scale."""
        if isinstance(value, tuple):
            self._scale = Vector3(value[0], value[1], value[2])
        else:
            self._scale = value

    @property
    def global_position(self) -> Vector3:
        """Get the global (world) position."""
        if self._node is None or self._node.parent is None:
            return self._position

        parent = self._node.parent
        if hasattr(parent, "_transform_3d"):
            parent_matrix = parent._transform_3d.to_matrix()
            return parent_matrix * self._position
        return self._position

    @global_position.setter
    def global_position(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the global (world) position."""
        if isinstance(value, tuple):
            value = Vector3(value[0], value[1], value[2])

        if self._node is None or self._node.parent is None:
            self._position = value
            return

        parent = self._node.parent
        if hasattr(parent, "_transform_3d"):
            parent_matrix = parent._transform_3d.to_matrix().inverse()
            self._position = parent_matrix * value
        else:
            self._position = value

    @property
    def global_rotation(self) -> Quaternion:
        """Get the global (world) rotation."""
        if self._node is None or self._node.parent is None:
            return self._rotation

        parent = self._node.parent
        if hasattr(parent, "_transform_3d"):
            return parent._transform_3d.global_rotation * self._rotation
        return self._rotation

    @global_rotation.setter
    def global_rotation(self, value: Quaternion) -> None:
        """Set the global (world) rotation."""
        if self._node is None or self._node.parent is None:
            self._rotation = value
            return

        parent = self._node.parent
        if hasattr(parent, "_transform_3d"):
            parent_global = parent._transform_3d.global_rotation
            self._rotation = parent_global.inverse() * value
        else:
            self._rotation = value

    @property
    def global_scale(self) -> Vector3:
        """Get the global (world) scale."""
        if self._node is None or self._node.parent is None:
            return self._scale

        parent = self._node.parent
        if hasattr(parent, "_transform_3d"):
            parent_scale = parent._transform_3d.global_scale
            return Vector3(
                parent_scale.x * self._scale.x,
                parent_scale.y * self._scale.y,
                parent_scale.z * self._scale.z,
            )
        return self._scale

    @global_scale.setter
    def global_scale(self, value: Vector3 | tuple[float, float, float]) -> None:
        """Set the global (world) scale."""
        if isinstance(value, tuple):
            value = Vector3(value[0], value[1], value[2])

        if self._node is None or self._node.parent is None:
            self._scale = value
            return

        parent = self._node.parent
        if hasattr(parent, "_transform_3d"):
            parent_scale = parent._transform_3d.global_scale
            self._scale = Vector3(
                value.x / parent_scale.x if parent_scale.x != 0 else value.x,
                value.y / parent_scale.y if parent_scale.y != 0 else value.y,
                value.z / parent_scale.z if parent_scale.z != 0 else value.z,
            )
        else:
            self._scale = value

    # Basis vectors (local axes)

    @property
    def forward(self) -> Vector3:
        """Get the forward direction vector (local -Z axis in world space)."""
        return self._rotation.rotate_vector(Vector3.forward())

    @property
    def back(self) -> Vector3:
        """Get the back direction vector (local +Z axis in world space)."""
        return self._rotation.rotate_vector(Vector3.back())

    @property
    def right(self) -> Vector3:
        """Get the right direction vector (local +X axis in world space)."""
        return self._rotation.rotate_vector(Vector3.right())

    @property
    def left(self) -> Vector3:
        """Get the left direction vector (local -X axis in world space)."""
        return self._rotation.rotate_vector(Vector3.left())

    @property
    def up(self) -> Vector3:
        """Get the up direction vector (local +Y axis in world space)."""
        return self._rotation.rotate_vector(Vector3.up())

    @property
    def down(self) -> Vector3:
        """Get the down direction vector (local -Y axis in world space)."""
        return self._rotation.rotate_vector(Vector3.down())

    def look_at(self, target: Vector3, up: Vector3 | None = None) -> None:
        """Rotate to look at a target position.

        Args:
            target: The position to look at
            up: The up vector (defaults to Vector3.up())
        """
        if up is None:
            up = Vector3.up()

        direction = target - self.global_position
        if direction.length_squared < 0.0001:
            return

        self._rotation = Quaternion.look_at(direction, up)

    def translate(self, offset: Vector3) -> None:
        """Translate in local space.

        Args:
            offset: The offset to move by in local coordinates
        """
        # Rotate offset by current rotation to get local movement
        world_offset = self._rotation.rotate_vector(offset)
        self._position = self._position + world_offset

    def translate_global(self, offset: Vector3) -> None:
        """Translate in world space.

        Args:
            offset: The offset to move by in world coordinates
        """
        self._position = self._position + offset

    def rotate(self, axis: Vector3, angle: float) -> None:
        """Rotate around an axis in local space.

        Args:
            axis: The axis to rotate around (should be normalized)
            angle: The angle in radians
        """
        rotation = Quaternion.from_axis_angle(axis, angle)
        self._rotation = self._rotation * rotation

    def rotate_global(self, axis: Vector3, angle: float) -> None:
        """Rotate around an axis in world space.

        Args:
            axis: The axis to rotate around (should be normalized)
            angle: The angle in radians
        """
        rotation = Quaternion.from_axis_angle(axis, angle)
        self._rotation = rotation * self._rotation

    def to_matrix(self) -> Matrix4:
        """Convert to a transformation matrix."""
        return Matrix4.from_trs(self._position, self._rotation, self._scale)

    def to_global_matrix(self) -> Matrix4:
        """Convert to a global transformation matrix."""
        return Matrix4.from_trs(
            self.global_position, self.global_rotation, self.global_scale
        )

    def copy(self) -> Transform3D:
        """Return a copy of this transform."""
        return Transform3D(
            _position=self._position.copy(),
            _rotation=self._rotation.copy(),
            _scale=self._scale.copy(),
        )
