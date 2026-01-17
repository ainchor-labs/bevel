"""Box2D physics integration for Bevel engine."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Callable, TYPE_CHECKING

from bevel.utils.math import Vector2
from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.nodes.node_2d import CollisionShape2D
    from bevel.core.scene import Scene

logger = get_logger("physics")

# Conversion factor: pixels to meters (Box2D works in meters)
PIXELS_PER_METER = 32.0


def pixels_to_meters(pixels: float) -> float:
    """Convert pixels to meters."""
    return pixels / PIXELS_PER_METER


def meters_to_pixels(meters: float) -> float:
    """Convert meters to pixels."""
    return meters * PIXELS_PER_METER


def vec_pixels_to_meters(vec: Vector2) -> tuple[float, float]:
    """Convert a Vector2 from pixels to meters."""
    return (vec.x / PIXELS_PER_METER, vec.y / PIXELS_PER_METER)


def vec_meters_to_pixels(vec: tuple[float, float]) -> Vector2:
    """Convert a tuple from meters to pixels."""
    return Vector2(vec[0] * PIXELS_PER_METER, vec[1] * PIXELS_PER_METER)


# =============================================================================
# Joint Types
# =============================================================================


class JointType(Enum):
    """Types of physics joints."""

    REVOLUTE = auto()    # Hinge/pin joint
    DISTANCE = auto()    # Spring/rope joint
    PRISMATIC = auto()   # Slider joint
    PULLEY = auto()      # Pulley system
    WELD = auto()        # Rigid connection
    WHEEL = auto()       # Vehicle wheel joint
    FRICTION = auto()    # Top-down friction
    ROPE = auto()        # Max distance constraint
    MOTOR = auto()       # Motor joint for control
    GEAR = auto()        # Gear connection between joints


@dataclass
class JointDef:
    """Base joint definition."""

    body_a: "CollisionShape2D"
    body_b: "CollisionShape2D"
    collide_connected: bool = False


@dataclass
class RevoluteJointDef(JointDef):
    """Revolute (hinge) joint definition."""

    anchor: Vector2 = field(default_factory=Vector2.zero)
    lower_angle: float = 0.0  # Radians
    upper_angle: float = 0.0
    enable_limit: bool = False
    motor_speed: float = 0.0  # Radians/second
    max_motor_torque: float = 0.0
    enable_motor: bool = False


@dataclass
class DistanceJointDef(JointDef):
    """Distance (spring) joint definition."""

    anchor_a: Vector2 = field(default_factory=Vector2.zero)
    anchor_b: Vector2 = field(default_factory=Vector2.zero)
    length: float = 0.0  # 0 = auto-calculate
    min_length: float = 0.0
    max_length: float = 0.0
    stiffness: float = 0.0  # Frequency in Hz (0 = rigid)
    damping: float = 0.0  # Damping ratio


@dataclass
class PrismaticJointDef(JointDef):
    """Prismatic (slider) joint definition."""

    anchor: Vector2 = field(default_factory=Vector2.zero)
    axis: Vector2 = field(default_factory=lambda: Vector2(1, 0))
    lower_translation: float = 0.0
    upper_translation: float = 0.0
    enable_limit: bool = False
    motor_speed: float = 0.0
    max_motor_force: float = 0.0
    enable_motor: bool = False


@dataclass
class PulleyJointDef(JointDef):
    """Pulley joint definition."""

    ground_anchor_a: Vector2 = field(default_factory=Vector2.zero)
    ground_anchor_b: Vector2 = field(default_factory=Vector2.zero)
    anchor_a: Vector2 = field(default_factory=Vector2.zero)
    anchor_b: Vector2 = field(default_factory=Vector2.zero)
    ratio: float = 1.0


@dataclass
class WeldJointDef(JointDef):
    """Weld (rigid) joint definition."""

    anchor: Vector2 = field(default_factory=Vector2.zero)
    reference_angle: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.0


@dataclass
class WheelJointDef(JointDef):
    """Wheel joint definition (for vehicles)."""

    anchor: Vector2 = field(default_factory=Vector2.zero)
    axis: Vector2 = field(default_factory=lambda: Vector2(0, 1))
    enable_limit: bool = False
    lower_translation: float = 0.0
    upper_translation: float = 0.0
    enable_motor: bool = False
    motor_speed: float = 0.0
    max_motor_torque: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.0


@dataclass
class FrictionJointDef(JointDef):
    """Friction joint definition (for top-down games)."""

    anchor: Vector2 = field(default_factory=Vector2.zero)
    max_force: float = 0.0
    max_torque: float = 0.0


@dataclass
class RopeJointDef(JointDef):
    """Rope joint definition (max distance constraint)."""

    anchor_a: Vector2 = field(default_factory=Vector2.zero)
    anchor_b: Vector2 = field(default_factory=Vector2.zero)
    max_length: float = 0.0  # 0 = auto-calculate


@dataclass
class MotorJointDef(JointDef):
    """Motor joint definition (for programmatic control)."""

    linear_offset: Vector2 = field(default_factory=Vector2.zero)
    angular_offset: float = 0.0
    max_force: float = 1.0
    max_torque: float = 1.0
    correction_factor: float = 0.3


@dataclass
class GearJointDef:
    """Gear joint definition (connects two other joints)."""

    joint_a: Any  # PhysicsJoint
    joint_b: Any  # PhysicsJoint
    ratio: float = 1.0


# =============================================================================
# Contact Information
# =============================================================================


@dataclass
class ContactInfo:
    """Information about a collision contact."""

    node_a: "CollisionShape2D"
    node_b: "CollisionShape2D"
    normal: Vector2
    contact_points: list[Vector2]
    impulse: float = 0.0  # Total impulse from PostSolve


@dataclass
class PreSolveInfo:
    """Information available during PreSolve for contact modification."""

    node_a: "CollisionShape2D"
    node_b: "CollisionShape2D"
    normal: Vector2
    contact_points: list[Vector2]
    _contact: Any = field(default=None, repr=False)

    def disable_contact(self) -> None:
        """Disable this contact for the current time step."""
        if self._contact is not None:
            self._contact.enabled = False

    def set_friction(self, friction: float) -> None:
        """Override friction for this contact."""
        if self._contact is not None:
            self._contact.friction = friction

    def set_restitution(self, restitution: float) -> None:
        """Override restitution for this contact."""
        if self._contact is not None:
            self._contact.restitution = restitution


@dataclass
class PostSolveInfo:
    """Information available during PostSolve with impulse data."""

    node_a: "CollisionShape2D"
    node_b: "CollisionShape2D"
    normal_impulse: float
    tangent_impulse: float


# =============================================================================
# Raycast Options
# =============================================================================


@dataclass
class RaycastOptions:
    """Options for raycasting."""

    collision_mask: int = 0xFFFF  # Which layers to hit
    first_only: bool = False  # Stop at first hit
    sort_by_distance: bool = True  # Sort results by distance


@dataclass
class RaycastHit:
    """Result of a raycast hit."""

    node: "CollisionShape2D"
    point: Vector2
    normal: Vector2
    fraction: float  # 0-1 along the ray


# =============================================================================
# AABB Query
# =============================================================================


@dataclass
class AABBQueryResult:
    """Result of an AABB query."""

    nodes: list["CollisionShape2D"]


# =============================================================================
# Debug Draw Options
# =============================================================================


@dataclass
class DebugDrawFlags:
    """Flags for what to draw in debug mode."""

    shapes: bool = True
    joints: bool = True
    aabbs: bool = False
    center_of_mass: bool = False
    velocities: bool = False
    contact_points: bool = False
    contact_normals: bool = False


# =============================================================================
# Contact Listener
# =============================================================================


class ContactListener:
    """Handles collision callbacks from Box2D."""

    def __init__(self) -> None:
        self._on_begin_contact: list[Callable[[ContactInfo], None]] = []
        self._on_end_contact: list[Callable[[ContactInfo], None]] = []
        self._on_pre_solve: list[Callable[[PreSolveInfo], None]] = []
        self._on_post_solve: list[Callable[[PostSolveInfo], None]] = []
        self._body_to_node: dict[Any, "CollisionShape2D"] = {}

    def register_body(self, body: Any, node: "CollisionShape2D") -> None:
        """Register a body-node mapping."""
        self._body_to_node[body] = node

    def unregister_body(self, body: Any) -> None:
        """Unregister a body-node mapping."""
        self._body_to_node.pop(body, None)

    def on_begin_contact(self, callback: Callable[[ContactInfo], None]) -> None:
        """Register a callback for collision start."""
        self._on_begin_contact.append(callback)

    def on_end_contact(self, callback: Callable[[ContactInfo], None]) -> None:
        """Register a callback for collision end."""
        self._on_end_contact.append(callback)

    def on_pre_solve(self, callback: Callable[[PreSolveInfo], None]) -> None:
        """Register a callback for pre-solve (contact modification)."""
        self._on_pre_solve.append(callback)

    def on_post_solve(self, callback: Callable[[PostSolveInfo], None]) -> None:
        """Register a callback for post-solve (impulse data)."""
        self._on_post_solve.append(callback)

    def BeginContact(self, contact: Any) -> None:
        """Called by Box2D when a collision begins."""
        body_a = contact.fixtureA.body
        body_b = contact.fixtureB.body

        node_a = self._body_to_node.get(body_a)
        node_b = self._body_to_node.get(body_b)

        if node_a is None or node_b is None:
            return

        # Get contact info
        manifold = contact.worldManifold
        normal = Vector2(manifold.normal[0], manifold.normal[1])
        points = [
            vec_meters_to_pixels((p[0], p[1]))
            for p in manifold.points[:contact.manifold.pointCount]
        ]

        info = ContactInfo(
            node_a=node_a,
            node_b=node_b,
            normal=normal,
            contact_points=points,
        )

        # Call script methods
        for script in node_a.scripts:
            if hasattr(script, "on_collision_enter"):
                try:
                    script.on_collision_enter(node_b)
                except Exception as e:
                    logger.error(f"Error in on_collision_enter: {e}")

        for script in node_b.scripts:
            if hasattr(script, "on_collision_enter"):
                try:
                    script.on_collision_enter(node_a)
                except Exception as e:
                    logger.error(f"Error in on_collision_enter: {e}")

        # Call registered callbacks
        for callback in self._on_begin_contact:
            try:
                callback(info)
            except Exception as e:
                logger.error(f"Error in begin contact callback: {e}")

    def EndContact(self, contact: Any) -> None:
        """Called by Box2D when a collision ends."""
        body_a = contact.fixtureA.body
        body_b = contact.fixtureB.body

        node_a = self._body_to_node.get(body_a)
        node_b = self._body_to_node.get(body_b)

        if node_a is None or node_b is None:
            return

        info = ContactInfo(
            node_a=node_a,
            node_b=node_b,
            normal=Vector2.zero(),
            contact_points=[],
        )

        # Call script methods
        for script in node_a.scripts:
            if hasattr(script, "on_collision_exit"):
                try:
                    script.on_collision_exit(node_b)
                except Exception as e:
                    logger.error(f"Error in on_collision_exit: {e}")

        for script in node_b.scripts:
            if hasattr(script, "on_collision_exit"):
                try:
                    script.on_collision_exit(node_a)
                except Exception as e:
                    logger.error(f"Error in on_collision_exit: {e}")

        # Call registered callbacks
        for callback in self._on_end_contact:
            try:
                callback(info)
            except Exception as e:
                logger.error(f"Error in end contact callback: {e}")

    def PreSolve(self, contact: Any, old_manifold: Any) -> None:
        """Called before collision solving - allows contact modification."""
        if not self._on_pre_solve:
            return

        body_a = contact.fixtureA.body
        body_b = contact.fixtureB.body

        node_a = self._body_to_node.get(body_a)
        node_b = self._body_to_node.get(body_b)

        if node_a is None or node_b is None:
            return

        manifold = contact.worldManifold
        normal = Vector2(manifold.normal[0], manifold.normal[1])
        points = [
            vec_meters_to_pixels((p[0], p[1]))
            for p in manifold.points[:contact.manifold.pointCount]
        ]

        info = PreSolveInfo(
            node_a=node_a,
            node_b=node_b,
            normal=normal,
            contact_points=points,
            _contact=contact,
        )

        # Call script methods
        for script in node_a.scripts:
            if hasattr(script, "on_pre_solve"):
                try:
                    script.on_pre_solve(info, node_b)
                except Exception as e:
                    logger.error(f"Error in on_pre_solve: {e}")

        for script in node_b.scripts:
            if hasattr(script, "on_pre_solve"):
                try:
                    script.on_pre_solve(info, node_a)
                except Exception as e:
                    logger.error(f"Error in on_pre_solve: {e}")

        # Call registered callbacks
        for callback in self._on_pre_solve:
            try:
                callback(info)
            except Exception as e:
                logger.error(f"Error in pre solve callback: {e}")

    def PostSolve(self, contact: Any, impulse: Any) -> None:
        """Called after collision solving - provides impulse data."""
        if not self._on_post_solve:
            return

        body_a = contact.fixtureA.body
        body_b = contact.fixtureB.body

        node_a = self._body_to_node.get(body_a)
        node_b = self._body_to_node.get(body_b)

        if node_a is None or node_b is None:
            return

        # Get impulse data
        normal_impulse = sum(impulse.normalImpulses)
        tangent_impulse = sum(impulse.tangentImpulses)

        info = PostSolveInfo(
            node_a=node_a,
            node_b=node_b,
            normal_impulse=normal_impulse,
            tangent_impulse=tangent_impulse,
        )

        # Call script methods
        for script in node_a.scripts:
            if hasattr(script, "on_post_solve"):
                try:
                    script.on_post_solve(info, node_b)
                except Exception as e:
                    logger.error(f"Error in on_post_solve: {e}")

        for script in node_b.scripts:
            if hasattr(script, "on_post_solve"):
                try:
                    script.on_post_solve(info, node_a)
                except Exception as e:
                    logger.error(f"Error in on_post_solve: {e}")

        # Call registered callbacks
        for callback in self._on_post_solve:
            try:
                callback(info)
            except Exception as e:
                logger.error(f"Error in post solve callback: {e}")


# =============================================================================
# Physics Joint Wrapper
# =============================================================================


class PhysicsJoint:
    """Wrapper for a Box2D joint."""

    def __init__(
        self,
        joint: Any,
        joint_type: JointType,
        body_a: "CollisionShape2D",
        body_b: "CollisionShape2D"
    ) -> None:
        self._joint = joint
        self._type = joint_type
        self._body_a = body_a
        self._body_b = body_b

    @property
    def joint_type(self) -> JointType:
        """Get the joint type."""
        return self._type

    @property
    def body_a(self) -> "CollisionShape2D":
        """Get body A."""
        return self._body_a

    @property
    def body_b(self) -> "CollisionShape2D":
        """Get body B."""
        return self._body_b

    @property
    def anchor_a(self) -> Vector2:
        """Get anchor point on body A in world coordinates."""
        pos = self._joint.anchorA
        return vec_meters_to_pixels((pos[0], pos[1]))

    @property
    def anchor_b(self) -> Vector2:
        """Get anchor point on body B in world coordinates."""
        pos = self._joint.anchorB
        return vec_meters_to_pixels((pos[0], pos[1]))

    # Revolute joint properties
    def get_joint_angle(self) -> float:
        """Get joint angle (revolute joints)."""
        if self._type == JointType.REVOLUTE:
            return self._joint.angle
        return 0.0

    def get_joint_speed(self) -> float:
        """Get joint angular speed (revolute joints)."""
        if self._type == JointType.REVOLUTE:
            return self._joint.speed
        return 0.0

    def set_motor_speed(self, speed: float) -> None:
        """Set motor speed (revolute/wheel joints)."""
        if self._type in (JointType.REVOLUTE, JointType.WHEEL, JointType.PRISMATIC):
            self._joint.motorSpeed = speed

    def set_max_motor_torque(self, torque: float) -> None:
        """Set max motor torque (revolute/wheel joints)."""
        if self._type in (JointType.REVOLUTE, JointType.WHEEL):
            self._joint.maxMotorTorque = torque

    def enable_motor(self, enabled: bool) -> None:
        """Enable/disable motor (revolute/wheel/prismatic joints)."""
        if self._type in (JointType.REVOLUTE, JointType.WHEEL, JointType.PRISMATIC):
            self._joint.motorEnabled = enabled

    def enable_limit(self, enabled: bool) -> None:
        """Enable/disable limits (revolute/prismatic/wheel joints)."""
        if self._type in (JointType.REVOLUTE, JointType.PRISMATIC, JointType.WHEEL):
            self._joint.limitEnabled = enabled

    def set_limits(self, lower: float, upper: float) -> None:
        """Set joint limits (revolute: radians, prismatic: meters)."""
        if self._type == JointType.REVOLUTE:
            self._joint.limits = (lower, upper)
        elif self._type in (JointType.PRISMATIC, JointType.WHEEL):
            self._joint.limits = (pixels_to_meters(lower), pixels_to_meters(upper))

    # Distance joint properties
    def get_length(self) -> float:
        """Get distance joint length."""
        if self._type == JointType.DISTANCE:
            return meters_to_pixels(self._joint.length)
        return 0.0

    def set_length(self, length: float) -> None:
        """Set distance joint length."""
        if self._type == JointType.DISTANCE:
            self._joint.length = pixels_to_meters(length)

    def set_stiffness(self, stiffness: float) -> None:
        """Set joint stiffness (distance/weld/wheel joints)."""
        if self._type in (JointType.DISTANCE, JointType.WELD, JointType.WHEEL):
            self._joint.stiffness = stiffness

    def set_damping(self, damping: float) -> None:
        """Set joint damping (distance/weld/wheel joints)."""
        if self._type in (JointType.DISTANCE, JointType.WELD, JointType.WHEEL):
            self._joint.damping = damping

    # Motor joint properties
    def set_linear_offset(self, offset: Vector2) -> None:
        """Set motor joint linear offset."""
        if self._type == JointType.MOTOR:
            self._joint.linearOffset = vec_pixels_to_meters(offset)

    def set_angular_offset(self, offset: float) -> None:
        """Set motor joint angular offset."""
        if self._type == JointType.MOTOR:
            self._joint.angularOffset = offset

    def set_max_force(self, force: float) -> None:
        """Set max force (motor/friction joints)."""
        if self._type in (JointType.MOTOR, JointType.FRICTION):
            self._joint.maxForce = force

    def set_max_torque(self, torque: float) -> None:
        """Set max torque (motor/friction joints)."""
        if self._type in (JointType.MOTOR, JointType.FRICTION):
            self._joint.maxTorque = torque


# =============================================================================
# Physics Configuration
# =============================================================================


@dataclass
class PhysicsConfig:
    """Physics world configuration."""

    gravity: Vector2 = field(default_factory=lambda: Vector2(0, 9.8 * PIXELS_PER_METER))
    velocity_iterations: int = 8
    position_iterations: int = 3
    time_step: float = 1.0 / 60.0
    allow_sleeping: bool = True
    continuous_physics: bool = True  # Enable CCD


# =============================================================================
# Physics World
# =============================================================================


class PhysicsWorld:
    """Manages the Box2D physics world."""

    def __init__(self, config: PhysicsConfig | None = None) -> None:
        """Initialize the physics world.

        Args:
            config: Physics configuration
        """
        self.config = config or PhysicsConfig()
        self._world: Any = None
        self._contact_listener = ContactListener()
        # Use id(node) as key since dataclasses aren't hashable
        self._bodies: dict[int, Any] = {}
        self._node_ids: dict[int, "CollisionShape2D"] = {}  # Reverse lookup
        self._joints: list[PhysicsJoint] = []
        self._initialized = False
        self._accumulator = 0.0
        self._paused = False
        self._debug_draw_flags = DebugDrawFlags()
        self._debug_contacts: list[tuple[Vector2, Vector2]] = []  # point, normal pairs

    def initialize(self) -> bool:
        """Initialize the physics world.

        Returns:
            True if successful
        """
        try:
            from Box2D import b2World, b2ContactListener

            # Create world with gravity
            gravity = vec_pixels_to_meters(self.config.gravity)
            self._world = b2World(gravity=gravity)

            # Configure world settings
            self._world.allowSleeping = self.config.allow_sleeping
            self._world.continuousPhysics = self.config.continuous_physics

            # Set up contact listener
            class BevelContactListener(b2ContactListener):
                def __init__(self, listener: ContactListener):
                    b2ContactListener.__init__(self)
                    self.listener = listener

                def BeginContact(self, contact: Any) -> None:
                    self.listener.BeginContact(contact)

                def EndContact(self, contact: Any) -> None:
                    self.listener.EndContact(contact)

                def PreSolve(self, contact: Any, old_manifold: Any) -> None:
                    self.listener.PreSolve(contact, old_manifold)

                def PostSolve(self, contact: Any, impulse: Any) -> None:
                    self.listener.PostSolve(contact, impulse)

            self._b2_listener = BevelContactListener(self._contact_listener)
            self._world.contactListener = self._b2_listener

            self._initialized = True
            logger.info("Physics world initialized")
            return True

        except ImportError as e:
            logger.error(f"Failed to import Box2D: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to initialize physics: {e}")
            return False

    def shutdown(self) -> None:
        """Shut down the physics world."""
        if not self._initialized:
            return

        # Remove all joints first
        for joint in self._joints:
            try:
                self._world.DestroyJoint(joint._joint)
            except Exception:
                pass
        self._joints.clear()

        # Remove all bodies
        for body in list(self._bodies.values()):
            self._world.DestroyBody(body)
        self._bodies.clear()
        self._node_ids.clear()

        self._world = None
        self._initialized = False
        logger.info("Physics world shut down")

    def create_body(self, node: "CollisionShape2D") -> bool:
        """Create a physics body for a collision shape node.

        Args:
            node: The CollisionShape2D node

        Returns:
            True if successful
        """
        if not self._initialized:
            return False

        node_id = id(node)
        if node_id in self._bodies:
            return True

        try:
            from Box2D import b2BodyDef, b2FixtureDef, b2PolygonShape, b2CircleShape
            from bevel.nodes.node_2d import BodyType, ShapeType

            # Create body definition
            body_def = b2BodyDef()

            # Set body type
            if node.physics.body_type == BodyType.STATIC:
                body_def.type = 0  # b2_staticBody
            elif node.physics.body_type == BodyType.KINEMATIC:
                body_def.type = 1  # b2_kinematicBody
            else:  # DYNAMIC
                body_def.type = 2  # b2_dynamicBody

            # Set position
            pos = vec_pixels_to_meters(node.global_position)
            body_def.position = pos
            body_def.angle = node.rotation
            body_def.linearDamping = node.physics.linear_damping
            body_def.angularDamping = node.physics.angular_damping
            body_def.fixedRotation = node.physics.fixed_rotation
            body_def.gravityScale = node.physics.gravity_scale
            body_def.bullet = node.physics.is_bullet  # Enable CCD for fast objects

            # Create the body
            body = self._world.CreateBody(body_def)

            # Create shape(s)
            if node.shape == ShapeType.RECTANGLE:
                shape = b2PolygonShape()
                half_width = pixels_to_meters(node.size.x / 2)
                half_height = pixels_to_meters(node.size.y / 2)
                shape.SetAsBox(half_width, half_height)
            elif node.shape == ShapeType.CIRCLE:
                shape = b2CircleShape()
                shape.radius = pixels_to_meters(node.radius)
            elif node.shape == ShapeType.POLYGON:
                shape = b2PolygonShape()
                # Convert vertices from pixels to meters
                vertices = [
                    (pixels_to_meters(v.x), pixels_to_meters(v.y))
                    for v in node.vertices
                ]
                shape.vertices = vertices
            else:
                logger.error(f"Unsupported shape type: {node.shape}")
                return False

            # Create fixture
            fixture_def = b2FixtureDef()
            fixture_def.shape = shape
            fixture_def.density = node.physics.mass
            fixture_def.friction = node.physics.friction
            fixture_def.restitution = node.physics.restitution
            fixture_def.isSensor = node.physics.is_sensor

            # Set collision filtering
            fixture_def.filter.categoryBits = node.collision_layer
            fixture_def.filter.maskBits = node.collision_mask

            body.CreateFixture(fixture_def)

            # Store references using id(node) as key
            self._bodies[node_id] = body
            self._node_ids[node_id] = node
            node._body = body
            self._contact_listener.register_body(body, node)

            logger.debug(f"Created physics body for '{node.name}'")
            return True

        except Exception as e:
            logger.error(f"Failed to create physics body: {e}")
            import traceback
            traceback.print_exc()
            return False

    def remove_body(self, node: "CollisionShape2D") -> bool:
        """Remove a physics body.

        Args:
            node: The CollisionShape2D node

        Returns:
            True if removed
        """
        if not self._initialized:
            return False

        node_id = id(node)
        if node_id not in self._bodies:
            return False

        body = self._bodies[node_id]

        # Remove any joints connected to this body
        joints_to_remove = [
            j for j in self._joints
            if j._body_a is node or j._body_b is node
        ]
        for joint in joints_to_remove:
            self.destroy_joint(joint)

        self._contact_listener.unregister_body(body)
        self._world.DestroyBody(body)
        del self._bodies[node_id]
        del self._node_ids[node_id]
        node._body = None

        logger.debug(f"Removed physics body for '{node.name}'")
        return True

    # =========================================================================
    # Joint Creation
    # =========================================================================

    def create_revolute_joint(self, joint_def: RevoluteJointDef) -> PhysicsJoint | None:
        """Create a revolute (hinge) joint.

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2RevoluteJointDef

            jd = b2RevoluteJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            anchor = vec_pixels_to_meters(joint_def.anchor)
            jd.localAnchorA = body_a.GetLocalPoint(anchor)
            jd.localAnchorB = body_b.GetLocalPoint(anchor)
            jd.referenceAngle = body_b.angle - body_a.angle

            jd.enableLimit = joint_def.enable_limit
            jd.lowerAngle = joint_def.lower_angle
            jd.upperAngle = joint_def.upper_angle
            jd.enableMotor = joint_def.enable_motor
            jd.motorSpeed = joint_def.motor_speed
            jd.maxMotorTorque = joint_def.max_motor_torque

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.REVOLUTE, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create revolute joint: {e}")
            return None

    def create_distance_joint(self, joint_def: DistanceJointDef) -> PhysicsJoint | None:
        """Create a distance (spring) joint.

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2DistanceJointDef

            jd = b2DistanceJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            anchor_a = vec_pixels_to_meters(joint_def.anchor_a)
            anchor_b = vec_pixels_to_meters(joint_def.anchor_b)
            jd.localAnchorA = body_a.GetLocalPoint(anchor_a)
            jd.localAnchorB = body_b.GetLocalPoint(anchor_b)

            if joint_def.length > 0:
                jd.length = pixels_to_meters(joint_def.length)
            else:
                # Auto-calculate from current positions
                jd.length = math.sqrt(
                    (anchor_b[0] - anchor_a[0]) ** 2 +
                    (anchor_b[1] - anchor_a[1]) ** 2
                )

            jd.minLength = pixels_to_meters(joint_def.min_length) if joint_def.min_length > 0 else jd.length
            jd.maxLength = pixels_to_meters(joint_def.max_length) if joint_def.max_length > 0 else jd.length
            jd.stiffness = joint_def.stiffness
            jd.damping = joint_def.damping

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.DISTANCE, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create distance joint: {e}")
            return None

    def create_prismatic_joint(self, joint_def: PrismaticJointDef) -> PhysicsJoint | None:
        """Create a prismatic (slider) joint.

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2PrismaticJointDef

            jd = b2PrismaticJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            anchor = vec_pixels_to_meters(joint_def.anchor)
            jd.localAnchorA = body_a.GetLocalPoint(anchor)
            jd.localAnchorB = body_b.GetLocalPoint(anchor)
            jd.localAxisA = (joint_def.axis.x, joint_def.axis.y)
            jd.referenceAngle = body_b.angle - body_a.angle

            jd.enableLimit = joint_def.enable_limit
            jd.lowerTranslation = pixels_to_meters(joint_def.lower_translation)
            jd.upperTranslation = pixels_to_meters(joint_def.upper_translation)
            jd.enableMotor = joint_def.enable_motor
            jd.motorSpeed = joint_def.motor_speed
            jd.maxMotorForce = joint_def.max_motor_force

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.PRISMATIC, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create prismatic joint: {e}")
            return None

    def create_pulley_joint(self, joint_def: PulleyJointDef) -> PhysicsJoint | None:
        """Create a pulley joint.

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2PulleyJointDef

            jd = b2PulleyJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            jd.groundAnchorA = vec_pixels_to_meters(joint_def.ground_anchor_a)
            jd.groundAnchorB = vec_pixels_to_meters(joint_def.ground_anchor_b)
            jd.localAnchorA = body_a.GetLocalPoint(vec_pixels_to_meters(joint_def.anchor_a))
            jd.localAnchorB = body_b.GetLocalPoint(vec_pixels_to_meters(joint_def.anchor_b))
            jd.ratio = joint_def.ratio

            # Calculate lengths
            anchor_a = vec_pixels_to_meters(joint_def.anchor_a)
            anchor_b = vec_pixels_to_meters(joint_def.anchor_b)
            ground_a = vec_pixels_to_meters(joint_def.ground_anchor_a)
            ground_b = vec_pixels_to_meters(joint_def.ground_anchor_b)

            jd.lengthA = math.sqrt(
                (anchor_a[0] - ground_a[0]) ** 2 +
                (anchor_a[1] - ground_a[1]) ** 2
            )
            jd.lengthB = math.sqrt(
                (anchor_b[0] - ground_b[0]) ** 2 +
                (anchor_b[1] - ground_b[1]) ** 2
            )

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.PULLEY, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create pulley joint: {e}")
            return None

    def create_weld_joint(self, joint_def: WeldJointDef) -> PhysicsJoint | None:
        """Create a weld (rigid) joint.

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2WeldJointDef

            jd = b2WeldJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            anchor = vec_pixels_to_meters(joint_def.anchor)
            jd.localAnchorA = body_a.GetLocalPoint(anchor)
            jd.localAnchorB = body_b.GetLocalPoint(anchor)
            jd.referenceAngle = joint_def.reference_angle if joint_def.reference_angle != 0 else (body_b.angle - body_a.angle)
            jd.stiffness = joint_def.stiffness
            jd.damping = joint_def.damping

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.WELD, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create weld joint: {e}")
            return None

    def create_wheel_joint(self, joint_def: WheelJointDef) -> PhysicsJoint | None:
        """Create a wheel joint (for vehicles).

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2WheelJointDef

            jd = b2WheelJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            anchor = vec_pixels_to_meters(joint_def.anchor)
            jd.localAnchorA = body_a.GetLocalPoint(anchor)
            jd.localAnchorB = body_b.GetLocalPoint(anchor)
            jd.localAxisA = (joint_def.axis.x, joint_def.axis.y)

            jd.enableLimit = joint_def.enable_limit
            jd.lowerTranslation = pixels_to_meters(joint_def.lower_translation)
            jd.upperTranslation = pixels_to_meters(joint_def.upper_translation)
            jd.enableMotor = joint_def.enable_motor
            jd.motorSpeed = joint_def.motor_speed
            jd.maxMotorTorque = joint_def.max_motor_torque
            jd.stiffness = joint_def.stiffness
            jd.damping = joint_def.damping

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.WHEEL, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create wheel joint: {e}")
            return None

    def create_friction_joint(self, joint_def: FrictionJointDef) -> PhysicsJoint | None:
        """Create a friction joint (for top-down games).

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2FrictionJointDef

            jd = b2FrictionJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            anchor = vec_pixels_to_meters(joint_def.anchor)
            jd.localAnchorA = body_a.GetLocalPoint(anchor)
            jd.localAnchorB = body_b.GetLocalPoint(anchor)
            jd.maxForce = joint_def.max_force
            jd.maxTorque = joint_def.max_torque

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.FRICTION, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create friction joint: {e}")
            return None

    def create_rope_joint(self, joint_def: RopeJointDef) -> PhysicsJoint | None:
        """Create a rope joint (max distance constraint).

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2RopeJointDef

            jd = b2RopeJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            jd.localAnchorA = body_a.GetLocalPoint(vec_pixels_to_meters(joint_def.anchor_a))
            jd.localAnchorB = body_b.GetLocalPoint(vec_pixels_to_meters(joint_def.anchor_b))

            if joint_def.max_length > 0:
                jd.maxLength = pixels_to_meters(joint_def.max_length)
            else:
                # Auto-calculate from current positions
                anchor_a = vec_pixels_to_meters(joint_def.anchor_a)
                anchor_b = vec_pixels_to_meters(joint_def.anchor_b)
                jd.maxLength = math.sqrt(
                    (anchor_b[0] - anchor_a[0]) ** 2 +
                    (anchor_b[1] - anchor_a[1]) ** 2
                )

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.ROPE, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create rope joint: {e}")
            return None

    def create_motor_joint(self, joint_def: MotorJointDef) -> PhysicsJoint | None:
        """Create a motor joint (for programmatic control).

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        body_a = joint_def.body_a._body
        body_b = joint_def.body_b._body

        if body_a is None or body_b is None:
            logger.error("Cannot create joint: bodies not initialized")
            return None

        try:
            from Box2D import b2MotorJointDef

            jd = b2MotorJointDef()
            jd.bodyA = body_a
            jd.bodyB = body_b
            jd.collideConnected = joint_def.collide_connected

            jd.linearOffset = vec_pixels_to_meters(joint_def.linear_offset)
            jd.angularOffset = joint_def.angular_offset
            jd.maxForce = joint_def.max_force
            jd.maxTorque = joint_def.max_torque
            jd.correctionFactor = joint_def.correction_factor

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.MOTOR, joint_def.body_a, joint_def.body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create motor joint: {e}")
            return None

    def create_gear_joint(self, joint_def: GearJointDef) -> PhysicsJoint | None:
        """Create a gear joint (connects two other joints).

        Args:
            joint_def: Joint definition

        Returns:
            PhysicsJoint wrapper or None on failure
        """
        if not self._initialized:
            return None

        joint_a = joint_def.joint_a
        joint_b = joint_def.joint_b

        if not isinstance(joint_a, PhysicsJoint) or not isinstance(joint_b, PhysicsJoint):
            logger.error("Gear joint requires two PhysicsJoint instances")
            return None

        if joint_a._type not in (JointType.REVOLUTE, JointType.PRISMATIC):
            logger.error("Gear joint A must be revolute or prismatic")
            return None

        if joint_b._type not in (JointType.REVOLUTE, JointType.PRISMATIC):
            logger.error("Gear joint B must be revolute or prismatic")
            return None

        try:
            from Box2D import b2GearJointDef

            jd = b2GearJointDef()
            jd.bodyA = joint_a._body_b._body
            jd.bodyB = joint_b._body_b._body
            jd.joint1 = joint_a._joint
            jd.joint2 = joint_b._joint
            jd.ratio = joint_def.ratio

            joint = self._world.CreateJoint(jd)
            wrapper = PhysicsJoint(joint, JointType.GEAR, joint_a._body_b, joint_b._body_b)
            self._joints.append(wrapper)
            return wrapper

        except Exception as e:
            logger.error(f"Failed to create gear joint: {e}")
            return None

    def destroy_joint(self, joint: PhysicsJoint) -> bool:
        """Destroy a joint.

        Args:
            joint: The joint to destroy

        Returns:
            True if destroyed
        """
        if not self._initialized:
            return False

        if joint not in self._joints:
            return False

        try:
            self._world.DestroyJoint(joint._joint)
            self._joints.remove(joint)
            return True
        except Exception as e:
            logger.error(f"Failed to destroy joint: {e}")
            return False

    def get_joints(self) -> list[PhysicsJoint]:
        """Get all joints.

        Returns:
            List of all joints
        """
        return list(self._joints)

    # =========================================================================
    # Simulation
    # =========================================================================

    def step(self, delta: float) -> None:
        """Step the physics simulation.

        Args:
            delta: Time since last frame
        """
        if not self._initialized or self._paused:
            return

        # Use fixed time step with accumulator for stability
        self._accumulator += delta

        # Clear debug contacts
        self._debug_contacts.clear()

        while self._accumulator >= self.config.time_step:
            self._world.Step(
                self.config.time_step,
                self.config.velocity_iterations,
                self.config.position_iterations
            )
            self._accumulator -= self.config.time_step

            # Collect debug contact info
            if self._debug_draw_flags.contact_points or self._debug_draw_flags.contact_normals:
                for contact in self._world.contacts:
                    if contact.touching:
                        manifold = contact.worldManifold
                        for i in range(contact.manifold.pointCount):
                            point = vec_meters_to_pixels((manifold.points[i][0], manifold.points[i][1]))
                            normal = Vector2(manifold.normal[0], manifold.normal[1])
                            self._debug_contacts.append((point, normal))

        # Sync positions back to nodes
        self._sync_transforms()

    def _sync_transforms(self) -> None:
        """Sync Box2D body transforms to node transforms."""
        for node_id, body in self._bodies.items():
            if body.type == 0:  # Static bodies don't move
                continue

            node = self._node_ids.get(node_id)
            if node is None:
                continue

            # Get position and rotation from Box2D
            pos = body.position
            angle = body.angle

            # Convert and apply to node
            node._transform.global_position = vec_meters_to_pixels((pos[0], pos[1]))
            node._transform.rotation = angle

    def sync_node_to_body(self, node: "CollisionShape2D") -> None:
        """Sync a node's transform to its physics body.

        Args:
            node: The node to sync
        """
        node_id = id(node)
        if node_id not in self._bodies:
            return

        body = self._bodies[node_id]
        pos = vec_pixels_to_meters(node.global_position)
        body.transform = (pos, node.rotation)

    # =========================================================================
    # Pausing
    # =========================================================================

    def pause(self) -> None:
        """Pause physics simulation."""
        self._paused = True

    def resume(self) -> None:
        """Resume physics simulation."""
        self._paused = False

    @property
    def is_paused(self) -> bool:
        """Check if physics is paused."""
        return self._paused

    # =========================================================================
    # Body Sleeping
    # =========================================================================

    def set_body_awake(self, node: "CollisionShape2D", awake: bool) -> None:
        """Set whether a body is awake.

        Args:
            node: The node
            awake: Whether to wake or sleep the body
        """
        node_id = id(node)
        if node_id in self._bodies:
            self._bodies[node_id].awake = awake

    def is_body_awake(self, node: "CollisionShape2D") -> bool:
        """Check if a body is awake.

        Args:
            node: The node

        Returns:
            True if the body is awake
        """
        node_id = id(node)
        if node_id in self._bodies:
            return self._bodies[node_id].awake
        return False

    def set_sleeping_allowed(self, node: "CollisionShape2D", allowed: bool) -> None:
        """Set whether a body is allowed to sleep.

        Args:
            node: The node
            allowed: Whether sleeping is allowed
        """
        node_id = id(node)
        if node_id in self._bodies:
            self._bodies[node_id].sleepingAllowed = allowed

    # =========================================================================
    # Scene Setup
    # =========================================================================

    def setup_scene(self, scene: "Scene") -> None:
        """Set up physics bodies for all CollisionShape2D nodes in a scene.

        Args:
            scene: The scene to set up
        """
        if scene.root is None:
            return

        from bevel.nodes.node_2d import CollisionShape2D

        for node in scene.root.find_nodes_by_type(CollisionShape2D):
            self.create_body(node)

        logger.info(f"Set up {len(self._bodies)} physics bodies")

    # =========================================================================
    # Raycasting
    # =========================================================================

    def raycast(
        self,
        start: Vector2,
        end: Vector2,
        options: RaycastOptions | None = None
    ) -> list[RaycastHit]:
        """Cast a ray and return hits.

        Args:
            start: Ray start position (pixels)
            end: Ray end position (pixels)
            options: Raycast options

        Returns:
            List of RaycastHit results
        """
        if not self._initialized:
            return []

        if options is None:
            options = RaycastOptions()

        results: list[RaycastHit] = []

        class RaycastCallback:
            def __init__(self, listener: ContactListener, mask: int, first_only: bool):
                self.listener = listener
                self.mask = mask
                self.first_only = first_only
                self.hits: list[tuple[Any, tuple, tuple, float]] = []

            def ReportFixture(self, fixture: Any, point: tuple, normal: tuple, fraction: float) -> float:
                # Check collision mask
                if (fixture.filterData.categoryBits & self.mask) == 0:
                    return 1.0  # Continue (ignore this fixture)

                self.hits.append((fixture.body, point, normal, fraction))

                if self.first_only:
                    return fraction  # Stop at this hit
                return 1.0  # Continue casting

        callback = RaycastCallback(
            self._contact_listener,
            options.collision_mask,
            options.first_only
        )

        start_m = vec_pixels_to_meters(start)
        end_m = vec_pixels_to_meters(end)

        self._world.RayCast(callback, start_m, end_m)

        for body, point, normal, fraction in callback.hits:
            node = self._contact_listener._body_to_node.get(body)
            if node is not None:
                results.append(RaycastHit(
                    node=node,
                    point=vec_meters_to_pixels(point),
                    normal=Vector2(normal[0], normal[1]),
                    fraction=fraction,
                ))

        if options.sort_by_distance:
            results.sort(key=lambda h: h.fraction)

        return results

    def raycast_first(
        self,
        start: Vector2,
        end: Vector2,
        collision_mask: int = 0xFFFF
    ) -> RaycastHit | None:
        """Cast a ray and return only the first hit.

        Args:
            start: Ray start position (pixels)
            end: Ray end position (pixels)
            collision_mask: Which layers to hit

        Returns:
            First RaycastHit or None
        """
        options = RaycastOptions(
            collision_mask=collision_mask,
            first_only=True,
            sort_by_distance=True
        )
        results = self.raycast(start, end, options)
        return results[0] if results else None

    # =========================================================================
    # AABB Query
    # =========================================================================

    def query_aabb(
        self,
        lower_bound: Vector2,
        upper_bound: Vector2,
        collision_mask: int = 0xFFFF
    ) -> list["CollisionShape2D"]:
        """Query all bodies overlapping an AABB.

        Args:
            lower_bound: Lower corner of AABB (pixels)
            upper_bound: Upper corner of AABB (pixels)
            collision_mask: Which layers to include

        Returns:
            List of overlapping nodes
        """
        if not self._initialized:
            return []

        results: list["CollisionShape2D"] = []

        class AABBCallback:
            def __init__(self, listener: ContactListener, mask: int):
                self.listener = listener
                self.mask = mask
                self.bodies: set[Any] = set()

            def ReportFixture(self, fixture: Any) -> bool:
                # Check collision mask
                if (fixture.filterData.categoryBits & self.mask) == 0:
                    return True  # Continue (ignore this fixture)

                self.bodies.add(fixture.body)
                return True  # Continue querying

        callback = AABBCallback(self._contact_listener, collision_mask)

        from Box2D import b2AABB
        aabb = b2AABB()
        aabb.lowerBound = vec_pixels_to_meters(lower_bound)
        aabb.upperBound = vec_pixels_to_meters(upper_bound)

        self._world.QueryAABB(callback, aabb)

        for body in callback.bodies:
            node = self._contact_listener._body_to_node.get(body)
            if node is not None:
                results.append(node)

        return results

    def query_point(
        self,
        point: Vector2,
        collision_mask: int = 0xFFFF
    ) -> list["CollisionShape2D"]:
        """Query all bodies containing a point.

        Args:
            point: Point to query (pixels)
            collision_mask: Which layers to include

        Returns:
            List of containing nodes
        """
        # Query a tiny AABB around the point
        epsilon = 0.001
        lower = Vector2(point.x - epsilon, point.y - epsilon)
        upper = Vector2(point.x + epsilon, point.y + epsilon)

        candidates = self.query_aabb(lower, upper, collision_mask)

        # Filter to only those actually containing the point
        results: list["CollisionShape2D"] = []
        point_m = vec_pixels_to_meters(point)

        for node in candidates:
            if node._body is not None:
                for fixture in node._body.fixtures:
                    if fixture.TestPoint(point_m):
                        results.append(node)
                        break

        return results

    # =========================================================================
    # Gravity
    # =========================================================================

    def set_gravity(self, gravity: Vector2) -> None:
        """Set the world gravity.

        Args:
            gravity: New gravity vector (pixels/second^2)
        """
        if not self._initialized:
            return

        self.config.gravity = gravity
        self._world.gravity = vec_pixels_to_meters(gravity)

    def get_gravity(self) -> Vector2:
        """Get the world gravity.

        Returns:
            Gravity vector (pixels/second^2)
        """
        return self.config.gravity.copy()

    # =========================================================================
    # Debug Draw
    # =========================================================================

    @property
    def debug_draw_flags(self) -> DebugDrawFlags:
        """Get debug draw flags."""
        return self._debug_draw_flags

    def set_debug_draw_flags(self, flags: DebugDrawFlags) -> None:
        """Set debug draw flags."""
        self._debug_draw_flags = flags

    def get_debug_draw_data(self) -> dict[str, Any]:
        """Get data for debug rendering.

        Returns:
            Dictionary with debug data for the renderer
        """
        if not self._initialized:
            return {}

        data: dict[str, Any] = {
            "flags": self._debug_draw_flags,
            "bodies": [],
            "joints": [],
            "contacts": self._debug_contacts.copy(),
        }

        # Collect body data
        for node_id, body in self._bodies.items():
            node = self._node_ids.get(node_id)
            if node is None:
                continue

            body_data = {
                "node": node,
                "position": vec_meters_to_pixels((body.position[0], body.position[1])),
                "angle": body.angle,
                "type": body.type,
                "awake": body.awake,
            }

            if self._debug_draw_flags.velocities:
                vel = body.linearVelocity
                body_data["linear_velocity"] = vec_meters_to_pixels((vel[0], vel[1]))
                body_data["angular_velocity"] = body.angularVelocity

            if self._debug_draw_flags.center_of_mass:
                com = body.worldCenter
                body_data["center_of_mass"] = vec_meters_to_pixels((com[0], com[1]))

            if self._debug_draw_flags.aabbs:
                # Get AABB from fixtures
                for fixture in body.fixtures:
                    aabb = fixture.GetAABB(0)
                    body_data["aabb"] = {
                        "lower": vec_meters_to_pixels((aabb.lowerBound[0], aabb.lowerBound[1])),
                        "upper": vec_meters_to_pixels((aabb.upperBound[0], aabb.upperBound[1])),
                    }
                    break  # Just use first fixture's AABB

            data["bodies"].append(body_data)

        # Collect joint data
        if self._debug_draw_flags.joints:
            for joint in self._joints:
                joint_data = {
                    "type": joint.joint_type,
                    "anchor_a": joint.anchor_a,
                    "anchor_b": joint.anchor_b,
                }
                data["joints"].append(joint_data)

        return data

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def contact_listener(self) -> ContactListener:
        """Get the contact listener."""
        return self._contact_listener

    @property
    def body_count(self) -> int:
        """Get the number of physics bodies."""
        return len(self._bodies)

    @property
    def joint_count(self) -> int:
        """Get the number of joints."""
        return len(self._joints)

    @property
    def world(self) -> Any:
        """Get the underlying Box2D world (for advanced usage)."""
        return self._world