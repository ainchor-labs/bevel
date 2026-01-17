"""Transform2D class for 2D transformations."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from bevel.utils.math import Vector2

if TYPE_CHECKING:
    from bevel.core.node import Node


@dataclass
class Transform2D:
    """A 2D transformation containing position, rotation, and scale."""

    _position: Vector2 = field(default_factory=Vector2.zero)
    _rotation: float = 0.0  # In radians
    _scale: Vector2 = field(default_factory=Vector2.one)
    _node: "Node | None" = field(default=None, repr=False)

    def __post_init__(self) -> None:
        # Ensure we have proper Vector2 instances
        if isinstance(self._position, (list, tuple)):
            self._position = Vector2(self._position[0], self._position[1])
        if isinstance(self._scale, (list, tuple)):
            self._scale = Vector2(self._scale[0], self._scale[1])

    @property
    def position(self) -> Vector2:
        """Get the local position."""
        return self._position

    @position.setter
    def position(self, value: Vector2 | tuple[float, float]) -> None:
        """Set the local position."""
        if isinstance(value, (list, tuple)):
            self._position = Vector2(value[0], value[1])
        else:
            self._position = value

    @property
    def rotation(self) -> float:
        """Get the local rotation in radians."""
        return self._rotation

    @rotation.setter
    def rotation(self, value: float) -> None:
        """Set the local rotation in radians."""
        self._rotation = value

    @property
    def rotation_degrees(self) -> float:
        """Get the local rotation in degrees."""
        return math.degrees(self._rotation)

    @rotation_degrees.setter
    def rotation_degrees(self, value: float) -> None:
        """Set the local rotation in degrees."""
        self._rotation = math.radians(value)

    @property
    def scale(self) -> Vector2:
        """Get the local scale."""
        return self._scale

    @scale.setter
    def scale(self, value: Vector2 | tuple[float, float]) -> None:
        """Set the local scale."""
        if isinstance(value, (list, tuple)):
            self._scale = Vector2(value[0], value[1])
        else:
            self._scale = value

    @property
    def global_position(self) -> Vector2:
        """Get the global (world) position."""
        if self._node is None or self._node.parent is None:
            return self._position.copy()

        parent_transform = self._node.parent.transform
        parent_global = parent_transform.global_position
        parent_rotation = parent_transform.global_rotation
        parent_scale = parent_transform.global_scale

        # Apply parent's scale and rotation to local position
        scaled_pos = Vector2(
            self._position.x * parent_scale.x,
            self._position.y * parent_scale.y
        )
        rotated_pos = scaled_pos.rotated(parent_rotation)

        return parent_global + rotated_pos

    @global_position.setter
    def global_position(self, value: Vector2 | tuple[float, float]) -> None:
        """Set the global (world) position."""
        if isinstance(value, (list, tuple)):
            value = Vector2(value[0], value[1])

        if self._node is None or self._node.parent is None:
            self._position = value
            return

        parent_transform = self._node.parent.transform
        parent_global = parent_transform.global_position
        parent_rotation = parent_transform.global_rotation
        parent_scale = parent_transform.global_scale

        # Reverse the transformation
        relative_pos = value - parent_global
        unrotated_pos = relative_pos.rotated(-parent_rotation)

        if parent_scale.x != 0 and parent_scale.y != 0:
            self._position = Vector2(
                unrotated_pos.x / parent_scale.x,
                unrotated_pos.y / parent_scale.y
            )
        else:
            self._position = unrotated_pos

    @property
    def global_rotation(self) -> float:
        """Get the global (world) rotation in radians."""
        if self._node is None or self._node.parent is None:
            return self._rotation
        return self._node.parent.transform.global_rotation + self._rotation

    @global_rotation.setter
    def global_rotation(self, value: float) -> None:
        """Set the global (world) rotation in radians."""
        if self._node is None or self._node.parent is None:
            self._rotation = value
            return
        self._rotation = value - self._node.parent.transform.global_rotation

    @property
    def global_scale(self) -> Vector2:
        """Get the global (world) scale."""
        if self._node is None or self._node.parent is None:
            return self._scale.copy()

        parent_scale = self._node.parent.transform.global_scale
        return Vector2(
            self._scale.x * parent_scale.x,
            self._scale.y * parent_scale.y
        )

    @global_scale.setter
    def global_scale(self, value: Vector2 | tuple[float, float]) -> None:
        """Set the global (world) scale."""
        if isinstance(value, (list, tuple)):
            value = Vector2(value[0], value[1])

        if self._node is None or self._node.parent is None:
            self._scale = value
            return

        parent_scale = self._node.parent.transform.global_scale
        if parent_scale.x != 0 and parent_scale.y != 0:
            self._scale = Vector2(
                value.x / parent_scale.x,
                value.y / parent_scale.y
            )
        else:
            self._scale = value

    def look_at(self, target: Vector2) -> None:
        """Rotate to face the target position."""
        direction = target - self.global_position
        self.rotation = math.atan2(direction.y, direction.x)

    def translate(self, offset: Vector2) -> None:
        """Move by the given offset in local space."""
        self._position = self._position + offset

    def translate_global(self, offset: Vector2) -> None:
        """Move by the given offset in global space."""
        self.global_position = self.global_position + offset

    def copy(self) -> Transform2D:
        """Create a copy of this transform."""
        return Transform2D(
            _position=self._position.copy(),
            _rotation=self._rotation,
            _scale=self._scale.copy()
        )
