"""Math utilities for Bevel."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Union


@dataclass
class Vector2:
    """A 2D vector class."""

    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: Union[Vector2, tuple[float, float]]) -> Vector2:
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x, self.y + other.y)
        return Vector2(self.x + other[0], self.y + other[1])

    def __sub__(self, other: Union[Vector2, tuple[float, float]]) -> Vector2:
        if isinstance(other, Vector2):
            return Vector2(self.x - other.x, self.y - other.y)
        return Vector2(self.x - other[0], self.y - other[1])

    def __mul__(self, scalar: float) -> Vector2:
        return Vector2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> Vector2:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vector2:
        if scalar == 0:
            raise ValueError("Cannot divide by zero")
        return Vector2(self.x / scalar, self.y / scalar)

    def __neg__(self) -> Vector2:
        return Vector2(-self.x, -self.y)

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Vector2):
            return math.isclose(self.x, other.x) and math.isclose(self.y, other.y)
        return False

    def __iter__(self):
        yield self.x
        yield self.y

    @property
    def length(self) -> float:
        """Return the length (magnitude) of the vector."""
        return math.sqrt(self.x * self.x + self.y * self.y)

    @property
    def length_squared(self) -> float:
        """Return the squared length of the vector (faster than length)."""
        return self.x * self.x + self.y * self.y

    def normalized(self) -> Vector2:
        """Return a normalized (unit) vector."""
        length = self.length
        if length == 0:
            return Vector2(0, 0)
        return Vector2(self.x / length, self.y / length)

    def dot(self, other: Vector2) -> float:
        """Return the dot product with another vector."""
        return self.x * other.x + self.y * other.y

    def cross(self, other: Vector2) -> float:
        """Return the 2D cross product (z-component of 3D cross)."""
        return self.x * other.y - self.y * other.x

    def distance_to(self, other: Vector2) -> float:
        """Return the distance to another vector."""
        return (self - other).length

    def angle_to(self, other: Vector2) -> float:
        """Return the angle to another vector in radians."""
        return math.atan2(other.y - self.y, other.x - self.x)

    def rotated(self, angle: float) -> Vector2:
        """Return a vector rotated by angle (in radians)."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return Vector2(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a
        )

    def lerp(self, other: Vector2, t: float) -> Vector2:
        """Linear interpolation to another vector."""
        return Vector2(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t
        )

    def copy(self) -> Vector2:
        """Return a copy of this vector."""
        return Vector2(self.x, self.y)

    def to_tuple(self) -> tuple[float, float]:
        """Convert to a tuple."""
        return (self.x, self.y)

    def to_int_tuple(self) -> tuple[int, int]:
        """Convert to an integer tuple."""
        return (int(self.x), int(self.y))

    @staticmethod
    def from_tuple(t: tuple[float, float]) -> Vector2:
        """Create a Vector2 from a tuple."""
        return Vector2(t[0], t[1])

    @staticmethod
    def zero() -> Vector2:
        """Return a zero vector."""
        return Vector2(0, 0)

    @staticmethod
    def one() -> Vector2:
        """Return a (1, 1) vector."""
        return Vector2(1, 1)

    @staticmethod
    def up() -> Vector2:
        """Return an up vector (0, -1) in screen coordinates."""
        return Vector2(0, -1)

    @staticmethod
    def down() -> Vector2:
        """Return a down vector (0, 1) in screen coordinates."""
        return Vector2(0, 1)

    @staticmethod
    def left() -> Vector2:
        """Return a left vector (-1, 0)."""
        return Vector2(-1, 0)

    @staticmethod
    def right() -> Vector2:
        """Return a right vector (1, 0)."""
        return Vector2(1, 0)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value between min and max."""
    return max(min_val, min(max_val, value))


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation between two values."""
    return a + (b - a) * t


def deg_to_rad(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def rad_to_deg(radians: float) -> float:
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


@dataclass
class Vector3:
    """A 3D vector class."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: Union[Vector3, tuple[float, float, float]]) -> Vector3:
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        return Vector3(self.x + other[0], self.y + other[1], self.z + other[2])

    def __sub__(self, other: Union[Vector3, tuple[float, float, float]]) -> Vector3:
        if isinstance(other, Vector3):
            return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        return Vector3(self.x - other[0], self.y - other[1], self.z - other[2])

    def __mul__(self, scalar: float) -> Vector3:
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> Vector3:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vector3:
        if scalar == 0:
            raise ValueError("Cannot divide by zero")
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> Vector3:
        return Vector3(-self.x, -self.y, -self.z)

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Vector3):
            return (
                math.isclose(self.x, other.x)
                and math.isclose(self.y, other.y)
                and math.isclose(self.z, other.z)
            )
        return False

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    @property
    def length(self) -> float:
        """Return the length (magnitude) of the vector."""
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    @property
    def length_squared(self) -> float:
        """Return the squared length of the vector (faster than length)."""
        return self.x * self.x + self.y * self.y + self.z * self.z

    def normalized(self) -> Vector3:
        """Return a normalized (unit) vector."""
        length = self.length
        if length == 0:
            return Vector3(0, 0, 0)
        return Vector3(self.x / length, self.y / length, self.z / length)

    def dot(self, other: Vector3) -> float:
        """Return the dot product with another vector."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vector3) -> Vector3:
        """Return the cross product with another vector."""
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def distance_to(self, other: Vector3) -> float:
        """Return the distance to another vector."""
        return (self - other).length

    def angle_to(self, other: Vector3) -> float:
        """Return the angle to another vector in radians."""
        d = self.dot(other)
        len_product = self.length * other.length
        if len_product == 0:
            return 0.0
        return math.acos(clamp(d / len_product, -1.0, 1.0))

    def lerp(self, other: Vector3, t: float) -> Vector3:
        """Linear interpolation to another vector."""
        return Vector3(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.z + (other.z - self.z) * t,
        )

    def reflect(self, normal: Vector3) -> Vector3:
        """Reflect this vector off a surface with the given normal."""
        return self - normal * 2.0 * self.dot(normal)

    def project(self, onto: Vector3) -> Vector3:
        """Project this vector onto another vector."""
        onto_len_sq = onto.length_squared
        if onto_len_sq == 0:
            return Vector3.zero()
        return onto * (self.dot(onto) / onto_len_sq)

    def copy(self) -> Vector3:
        """Return a copy of this vector."""
        return Vector3(self.x, self.y, self.z)

    def to_tuple(self) -> tuple[float, float, float]:
        """Convert to a tuple."""
        return (self.x, self.y, self.z)

    def to_int_tuple(self) -> tuple[int, int, int]:
        """Convert to an integer tuple."""
        return (int(self.x), int(self.y), int(self.z))

    @staticmethod
    def from_tuple(t: tuple[float, float, float]) -> Vector3:
        """Create a Vector3 from a tuple."""
        return Vector3(t[0], t[1], t[2])

    @staticmethod
    def zero() -> Vector3:
        """Return a zero vector."""
        return Vector3(0, 0, 0)

    @staticmethod
    def one() -> Vector3:
        """Return a (1, 1, 1) vector."""
        return Vector3(1, 1, 1)

    @staticmethod
    def up() -> Vector3:
        """Return an up vector (0, 1, 0) in Y-up convention."""
        return Vector3(0, 1, 0)

    @staticmethod
    def down() -> Vector3:
        """Return a down vector (0, -1, 0)."""
        return Vector3(0, -1, 0)

    @staticmethod
    def left() -> Vector3:
        """Return a left vector (-1, 0, 0)."""
        return Vector3(-1, 0, 0)

    @staticmethod
    def right() -> Vector3:
        """Return a right vector (1, 0, 0)."""
        return Vector3(1, 0, 0)

    @staticmethod
    def forward() -> Vector3:
        """Return a forward vector (0, 0, -1) in OpenGL convention."""
        return Vector3(0, 0, -1)

    @staticmethod
    def back() -> Vector3:
        """Return a back vector (0, 0, 1)."""
        return Vector3(0, 0, 1)


@dataclass
class Quaternion:
    """A quaternion for 3D rotations.

    Uses the convention (x, y, z, w) where w is the scalar component.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0  # Identity quaternion by default

    def __mul__(self, other: Union[Quaternion, float]) -> Quaternion:
        """Multiply quaternions (rotation composition) or by scalar."""
        if isinstance(other, (int, float)):
            return Quaternion(
                self.x * other, self.y * other, self.z * other, self.w * other
            )
        # Quaternion multiplication (Hamilton product)
        return Quaternion(
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        )

    def __rmul__(self, scalar: float) -> Quaternion:
        return self.__mul__(scalar)

    def __neg__(self) -> Quaternion:
        return Quaternion(-self.x, -self.y, -self.z, -self.w)

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Quaternion):
            return (
                math.isclose(self.x, other.x)
                and math.isclose(self.y, other.y)
                and math.isclose(self.z, other.z)
                and math.isclose(self.w, other.w)
            )
        return False

    @property
    def length(self) -> float:
        """Return the length (magnitude) of the quaternion."""
        return math.sqrt(
            self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
        )

    @property
    def length_squared(self) -> float:
        """Return the squared length of the quaternion."""
        return self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w

    def normalized(self) -> Quaternion:
        """Return a normalized quaternion."""
        length = self.length
        if length == 0:
            return Quaternion.identity()
        return Quaternion(
            self.x / length, self.y / length, self.z / length, self.w / length
        )

    def conjugate(self) -> Quaternion:
        """Return the conjugate of this quaternion."""
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def inverse(self) -> Quaternion:
        """Return the inverse of this quaternion."""
        len_sq = self.length_squared
        if len_sq == 0:
            return Quaternion.identity()
        conj = self.conjugate()
        return Quaternion(
            conj.x / len_sq, conj.y / len_sq, conj.z / len_sq, conj.w / len_sq
        )

    def rotate_vector(self, v: Vector3) -> Vector3:
        """Rotate a vector by this quaternion."""
        # q * v * q^-1
        q_v = Quaternion(v.x, v.y, v.z, 0)
        result = self * q_v * self.conjugate()
        return Vector3(result.x, result.y, result.z)

    def to_euler(self) -> Vector3:
        """Convert to Euler angles (pitch, yaw, roll) in radians.

        Uses YXZ rotation order (yaw-pitch-roll).
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return Vector3(pitch, yaw, roll)

    def to_euler_degrees(self) -> Vector3:
        """Convert to Euler angles in degrees."""
        euler = self.to_euler()
        return Vector3(
            rad_to_deg(euler.x), rad_to_deg(euler.y), rad_to_deg(euler.z)
        )

    def dot(self, other: Quaternion) -> float:
        """Return the dot product with another quaternion."""
        return self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w

    def slerp(self, other: Quaternion, t: float) -> Quaternion:
        """Spherical linear interpolation to another quaternion."""
        # Compute cosine of angle between quaternions
        dot = self.dot(other)

        # If dot is negative, negate one quaternion to take shorter path
        other_q = other
        if dot < 0:
            other_q = -other
            dot = -dot

        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = Quaternion(
                self.x + (other_q.x - self.x) * t,
                self.y + (other_q.y - self.y) * t,
                self.z + (other_q.z - self.z) * t,
                self.w + (other_q.w - self.w) * t,
            )
            return result.normalized()

        # Perform slerp
        theta_0 = math.acos(dot)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        sin_theta_0 = math.sin(theta_0)

        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return Quaternion(
            self.x * s0 + other_q.x * s1,
            self.y * s0 + other_q.y * s1,
            self.z * s0 + other_q.z * s1,
            self.w * s0 + other_q.w * s1,
        )

    def copy(self) -> Quaternion:
        """Return a copy of this quaternion."""
        return Quaternion(self.x, self.y, self.z, self.w)

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Convert to a tuple (x, y, z, w)."""
        return (self.x, self.y, self.z, self.w)

    @staticmethod
    def identity() -> Quaternion:
        """Return the identity quaternion."""
        return Quaternion(0, 0, 0, 1)

    @staticmethod
    def from_euler(pitch: float, yaw: float, roll: float) -> Quaternion:
        """Create from Euler angles (in radians).

        Args:
            pitch: Rotation around X axis
            yaw: Rotation around Y axis
            roll: Rotation around Z axis
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return Quaternion(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    @staticmethod
    def from_euler_degrees(pitch: float, yaw: float, roll: float) -> Quaternion:
        """Create from Euler angles in degrees."""
        return Quaternion.from_euler(
            deg_to_rad(pitch), deg_to_rad(yaw), deg_to_rad(roll)
        )

    @staticmethod
    def from_axis_angle(axis: Vector3, angle: float) -> Quaternion:
        """Create from axis-angle representation.

        Args:
            axis: The axis of rotation (should be normalized)
            angle: The angle of rotation in radians
        """
        half_angle = angle * 0.5
        s = math.sin(half_angle)
        return Quaternion(axis.x * s, axis.y * s, axis.z * s, math.cos(half_angle))

    @staticmethod
    def look_at(forward: Vector3, up: Vector3 = None) -> Quaternion:
        """Create a quaternion that looks in the given direction.

        Args:
            forward: The forward direction
            up: The up vector (defaults to Vector3.up())
        """
        if up is None:
            up = Vector3.up()

        forward = forward.normalized()

        # Handle case where forward is parallel to up
        dot = up.dot(forward)
        if abs(dot) > 0.9999:
            # Use a different up vector
            up = Vector3.right() if abs(forward.y) < 0.9999 else Vector3.forward()

        right = up.cross(forward).normalized()
        up = forward.cross(right)

        # Build rotation matrix and convert to quaternion
        m00, m01, m02 = right.x, up.x, forward.x
        m10, m11, m12 = right.y, up.y, forward.y
        m20, m21, m22 = right.z, up.z, forward.z

        trace = m00 + m11 + m22

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            return Quaternion(
                (m21 - m12) * s,
                (m02 - m20) * s,
                (m10 - m01) * s,
                0.25 / s,
            )
        elif m00 > m11 and m00 > m22:
            s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
            return Quaternion(
                0.25 * s,
                (m01 + m10) / s,
                (m02 + m20) / s,
                (m21 - m12) / s,
            )
        elif m11 > m22:
            s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
            return Quaternion(
                (m01 + m10) / s,
                0.25 * s,
                (m12 + m21) / s,
                (m02 - m20) / s,
            )
        else:
            s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
            return Quaternion(
                (m02 + m20) / s,
                (m12 + m21) / s,
                0.25 * s,
                (m10 - m01) / s,
            )


@dataclass
class Matrix4:
    """A 4x4 transformation matrix.

    Stored in column-major order for OpenGL compatibility.
    Elements are stored as m[column][row], accessed as m00, m01, etc.
    """

    # Column 0
    m00: float = 1.0
    m01: float = 0.0
    m02: float = 0.0
    m03: float = 0.0
    # Column 1
    m10: float = 0.0
    m11: float = 1.0
    m12: float = 0.0
    m13: float = 0.0
    # Column 2
    m20: float = 0.0
    m21: float = 0.0
    m22: float = 1.0
    m23: float = 0.0
    # Column 3 (translation)
    m30: float = 0.0
    m31: float = 0.0
    m32: float = 0.0
    m33: float = 1.0

    def __mul__(self, other: Union[Matrix4, Vector3]) -> Union[Matrix4, Vector3]:
        """Multiply with another matrix or transform a vector."""
        if isinstance(other, Vector3):
            # Transform a point (w=1)
            return Vector3(
                self.m00 * other.x + self.m10 * other.y + self.m20 * other.z + self.m30,
                self.m01 * other.x + self.m11 * other.y + self.m21 * other.z + self.m31,
                self.m02 * other.x + self.m12 * other.y + self.m22 * other.z + self.m32,
            )
        # Matrix multiplication
        return Matrix4(
            # Column 0
            self.m00 * other.m00 + self.m10 * other.m01 + self.m20 * other.m02 + self.m30 * other.m03,
            self.m01 * other.m00 + self.m11 * other.m01 + self.m21 * other.m02 + self.m31 * other.m03,
            self.m02 * other.m00 + self.m12 * other.m01 + self.m22 * other.m02 + self.m32 * other.m03,
            self.m03 * other.m00 + self.m13 * other.m01 + self.m23 * other.m02 + self.m33 * other.m03,
            # Column 1
            self.m00 * other.m10 + self.m10 * other.m11 + self.m20 * other.m12 + self.m30 * other.m13,
            self.m01 * other.m10 + self.m11 * other.m11 + self.m21 * other.m12 + self.m31 * other.m13,
            self.m02 * other.m10 + self.m12 * other.m11 + self.m22 * other.m12 + self.m32 * other.m13,
            self.m03 * other.m10 + self.m13 * other.m11 + self.m23 * other.m12 + self.m33 * other.m13,
            # Column 2
            self.m00 * other.m20 + self.m10 * other.m21 + self.m20 * other.m22 + self.m30 * other.m23,
            self.m01 * other.m20 + self.m11 * other.m21 + self.m21 * other.m22 + self.m31 * other.m23,
            self.m02 * other.m20 + self.m12 * other.m21 + self.m22 * other.m22 + self.m32 * other.m23,
            self.m03 * other.m20 + self.m13 * other.m21 + self.m23 * other.m22 + self.m33 * other.m23,
            # Column 3
            self.m00 * other.m30 + self.m10 * other.m31 + self.m20 * other.m32 + self.m30 * other.m33,
            self.m01 * other.m30 + self.m11 * other.m31 + self.m21 * other.m32 + self.m31 * other.m33,
            self.m02 * other.m30 + self.m12 * other.m31 + self.m22 * other.m32 + self.m32 * other.m33,
            self.m03 * other.m30 + self.m13 * other.m31 + self.m23 * other.m32 + self.m33 * other.m33,
        )

    def transform_direction(self, direction: Vector3) -> Vector3:
        """Transform a direction vector (ignores translation)."""
        return Vector3(
            self.m00 * direction.x + self.m10 * direction.y + self.m20 * direction.z,
            self.m01 * direction.x + self.m11 * direction.y + self.m21 * direction.z,
            self.m02 * direction.x + self.m12 * direction.y + self.m22 * direction.z,
        )

    def get_translation(self) -> Vector3:
        """Get the translation component."""
        return Vector3(self.m30, self.m31, self.m32)

    def get_scale(self) -> Vector3:
        """Get the scale component (approximate, assumes no shear)."""
        return Vector3(
            Vector3(self.m00, self.m01, self.m02).length,
            Vector3(self.m10, self.m11, self.m12).length,
            Vector3(self.m20, self.m21, self.m22).length,
        )

    def determinant(self) -> float:
        """Calculate the determinant of the matrix."""
        # Laplace expansion along first column
        sub00 = self.m11 * (self.m22 * self.m33 - self.m23 * self.m32) - \
                self.m21 * (self.m12 * self.m33 - self.m13 * self.m32) + \
                self.m31 * (self.m12 * self.m23 - self.m13 * self.m22)
        sub01 = self.m10 * (self.m22 * self.m33 - self.m23 * self.m32) - \
                self.m20 * (self.m12 * self.m33 - self.m13 * self.m32) + \
                self.m30 * (self.m12 * self.m23 - self.m13 * self.m22)
        sub02 = self.m10 * (self.m21 * self.m33 - self.m23 * self.m31) - \
                self.m20 * (self.m11 * self.m33 - self.m13 * self.m31) + \
                self.m30 * (self.m11 * self.m23 - self.m13 * self.m21)
        sub03 = self.m10 * (self.m21 * self.m32 - self.m22 * self.m31) - \
                self.m20 * (self.m11 * self.m32 - self.m12 * self.m31) + \
                self.m30 * (self.m11 * self.m22 - self.m12 * self.m21)

        return self.m00 * sub00 - self.m01 * sub01 + self.m02 * sub02 - self.m03 * sub03

    def inverse(self) -> Matrix4:
        """Return the inverse of this matrix."""
        det = self.determinant()
        if abs(det) < 1e-10:
            return Matrix4.identity()

        inv_det = 1.0 / det

        # Compute adjugate matrix and multiply by inverse determinant
        # This is a standard 4x4 matrix inversion using cofactors
        return Matrix4(
            inv_det * (self.m11 * (self.m22 * self.m33 - self.m23 * self.m32) -
                       self.m21 * (self.m12 * self.m33 - self.m13 * self.m32) +
                       self.m31 * (self.m12 * self.m23 - self.m13 * self.m22)),
            inv_det * -(self.m01 * (self.m22 * self.m33 - self.m23 * self.m32) -
                        self.m21 * (self.m02 * self.m33 - self.m03 * self.m32) +
                        self.m31 * (self.m02 * self.m23 - self.m03 * self.m22)),
            inv_det * (self.m01 * (self.m12 * self.m33 - self.m13 * self.m32) -
                       self.m11 * (self.m02 * self.m33 - self.m03 * self.m32) +
                       self.m31 * (self.m02 * self.m13 - self.m03 * self.m12)),
            inv_det * -(self.m01 * (self.m12 * self.m23 - self.m13 * self.m22) -
                        self.m11 * (self.m02 * self.m23 - self.m03 * self.m22) +
                        self.m21 * (self.m02 * self.m13 - self.m03 * self.m12)),

            inv_det * -(self.m10 * (self.m22 * self.m33 - self.m23 * self.m32) -
                        self.m20 * (self.m12 * self.m33 - self.m13 * self.m32) +
                        self.m30 * (self.m12 * self.m23 - self.m13 * self.m22)),
            inv_det * (self.m00 * (self.m22 * self.m33 - self.m23 * self.m32) -
                       self.m20 * (self.m02 * self.m33 - self.m03 * self.m32) +
                       self.m30 * (self.m02 * self.m23 - self.m03 * self.m22)),
            inv_det * -(self.m00 * (self.m12 * self.m33 - self.m13 * self.m32) -
                        self.m10 * (self.m02 * self.m33 - self.m03 * self.m32) +
                        self.m30 * (self.m02 * self.m13 - self.m03 * self.m12)),
            inv_det * (self.m00 * (self.m12 * self.m23 - self.m13 * self.m22) -
                       self.m10 * (self.m02 * self.m23 - self.m03 * self.m22) +
                       self.m20 * (self.m02 * self.m13 - self.m03 * self.m12)),

            inv_det * (self.m10 * (self.m21 * self.m33 - self.m23 * self.m31) -
                       self.m20 * (self.m11 * self.m33 - self.m13 * self.m31) +
                       self.m30 * (self.m11 * self.m23 - self.m13 * self.m21)),
            inv_det * -(self.m00 * (self.m21 * self.m33 - self.m23 * self.m31) -
                        self.m20 * (self.m01 * self.m33 - self.m03 * self.m31) +
                        self.m30 * (self.m01 * self.m23 - self.m03 * self.m21)),
            inv_det * (self.m00 * (self.m11 * self.m33 - self.m13 * self.m31) -
                       self.m10 * (self.m01 * self.m33 - self.m03 * self.m31) +
                       self.m30 * (self.m01 * self.m13 - self.m03 * self.m11)),
            inv_det * -(self.m00 * (self.m11 * self.m23 - self.m13 * self.m21) -
                        self.m10 * (self.m01 * self.m23 - self.m03 * self.m21) +
                        self.m20 * (self.m01 * self.m13 - self.m03 * self.m11)),

            inv_det * -(self.m10 * (self.m21 * self.m32 - self.m22 * self.m31) -
                        self.m20 * (self.m11 * self.m32 - self.m12 * self.m31) +
                        self.m30 * (self.m11 * self.m22 - self.m12 * self.m21)),
            inv_det * (self.m00 * (self.m21 * self.m32 - self.m22 * self.m31) -
                       self.m20 * (self.m01 * self.m32 - self.m02 * self.m31) +
                       self.m30 * (self.m01 * self.m22 - self.m02 * self.m21)),
            inv_det * -(self.m00 * (self.m11 * self.m32 - self.m12 * self.m31) -
                        self.m10 * (self.m01 * self.m32 - self.m02 * self.m31) +
                        self.m30 * (self.m01 * self.m12 - self.m02 * self.m11)),
            inv_det * (self.m00 * (self.m11 * self.m22 - self.m12 * self.m21) -
                       self.m10 * (self.m01 * self.m22 - self.m02 * self.m21) +
                       self.m20 * (self.m01 * self.m12 - self.m02 * self.m11)),
        )

    def transpose(self) -> Matrix4:
        """Return the transposed matrix."""
        return Matrix4(
            self.m00, self.m10, self.m20, self.m30,
            self.m01, self.m11, self.m21, self.m31,
            self.m02, self.m12, self.m22, self.m32,
            self.m03, self.m13, self.m23, self.m33,
        )

    def copy(self) -> Matrix4:
        """Return a copy of this matrix."""
        return Matrix4(
            self.m00, self.m01, self.m02, self.m03,
            self.m10, self.m11, self.m12, self.m13,
            self.m20, self.m21, self.m22, self.m23,
            self.m30, self.m31, self.m32, self.m33,
        )

    def to_tuple(self) -> tuple[float, ...]:
        """Convert to a flat tuple (column-major order)."""
        return (
            self.m00, self.m01, self.m02, self.m03,
            self.m10, self.m11, self.m12, self.m13,
            self.m20, self.m21, self.m22, self.m23,
            self.m30, self.m31, self.m32, self.m33,
        )

    def to_list(self) -> list[list[float]]:
        """Convert to a 4x4 list of lists (row-major for readability)."""
        return [
            [self.m00, self.m10, self.m20, self.m30],
            [self.m01, self.m11, self.m21, self.m31],
            [self.m02, self.m12, self.m22, self.m32],
            [self.m03, self.m13, self.m23, self.m33],
        ]

    @staticmethod
    def identity() -> Matrix4:
        """Return the identity matrix."""
        return Matrix4()

    @staticmethod
    def from_translation(translation: Vector3) -> Matrix4:
        """Create a translation matrix."""
        return Matrix4(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            translation.x, translation.y, translation.z, 1,
        )

    @staticmethod
    def from_scale(scale: Vector3) -> Matrix4:
        """Create a scale matrix."""
        return Matrix4(
            scale.x, 0, 0, 0,
            0, scale.y, 0, 0,
            0, 0, scale.z, 0,
            0, 0, 0, 1,
        )

    @staticmethod
    def from_rotation(quaternion: Quaternion) -> Matrix4:
        """Create a rotation matrix from a quaternion."""
        q = quaternion.normalized()
        xx = q.x * q.x
        yy = q.y * q.y
        zz = q.z * q.z
        xy = q.x * q.y
        xz = q.x * q.z
        yz = q.y * q.z
        wx = q.w * q.x
        wy = q.w * q.y
        wz = q.w * q.z

        return Matrix4(
            1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy), 0,
            2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx), 0,
            2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy), 0,
            0, 0, 0, 1,
        )

    @staticmethod
    def from_trs(
        translation: Vector3, rotation: Quaternion, scale: Vector3
    ) -> Matrix4:
        """Create a transformation matrix from translation, rotation, and scale."""
        # T * R * S
        t = Matrix4.from_translation(translation)
        r = Matrix4.from_rotation(rotation)
        s = Matrix4.from_scale(scale)
        return t * r * s

    @staticmethod
    def perspective(fov: float, aspect: float, near: float, far: float) -> Matrix4:
        """Create a perspective projection matrix.

        Args:
            fov: Field of view in radians
            aspect: Aspect ratio (width / height)
            near: Near clipping plane
            far: Far clipping plane
        """
        tan_half_fov = math.tan(fov / 2)
        return Matrix4(
            1 / (aspect * tan_half_fov), 0, 0, 0,
            0, 1 / tan_half_fov, 0, 0,
            0, 0, -(far + near) / (far - near), -1,
            0, 0, -(2 * far * near) / (far - near), 0,
        )

    @staticmethod
    def orthographic(
        left: float, right: float, bottom: float, top: float, near: float, far: float
    ) -> Matrix4:
        """Create an orthographic projection matrix."""
        return Matrix4(
            2 / (right - left), 0, 0, 0,
            0, 2 / (top - bottom), 0, 0,
            0, 0, -2 / (far - near), 0,
            -(right + left) / (right - left),
            -(top + bottom) / (top - bottom),
            -(far + near) / (far - near),
            1,
        )

    @staticmethod
    def look_at(eye: Vector3, target: Vector3, up: Vector3 = None) -> Matrix4:
        """Create a view matrix that looks at a target from eye position.

        Args:
            eye: Camera position
            target: Target position to look at
            up: Up vector (defaults to Vector3.up())
        """
        if up is None:
            up = Vector3.up()

        forward = (target - eye).normalized()
        right = forward.cross(up).normalized()
        up = right.cross(forward)

        return Matrix4(
            right.x, up.x, -forward.x, 0,
            right.y, up.y, -forward.y, 0,
            right.z, up.z, -forward.z, 0,
            -right.dot(eye), -up.dot(eye), forward.dot(eye), 1,
        )
