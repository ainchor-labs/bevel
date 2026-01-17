"""Pythonic wrapper for Jolt Physics bindings.

This module provides a high-level Python interface for the Jolt Physics
3D physics engine.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, TYPE_CHECKING

from bevel.utils.math import Vector3, Quaternion
from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.nodes.node_3d import CollisionShape3D
    from bevel.core.scene import Scene

logger = get_logger("physics3d")

# Import the C++ extension
JOLT_AVAILABLE = False
_JoltWorld = None

try:
    from bevel.physics.jolt_bindings._jolt import JoltPhysicsWorld as _JoltWorld
    JOLT_AVAILABLE = True
except ImportError as e:
    logger.warning(f"Jolt Physics bindings not available: {e}")


@dataclass
class ContactInfo3D:
    """Information about a 3D collision contact."""

    node_a: "CollisionShape3D"
    node_b: "CollisionShape3D"
    contact_point: Vector3
    contact_normal: Vector3
    penetration_depth: float


@dataclass
class PhysicsConfig3D:
    """3D Physics world configuration."""

    gravity: Vector3 = field(default_factory=lambda: Vector3(0, -9.81, 0))
    time_step: float = 1.0 / 60.0

@dataclass
class Joint3D:
    """Base class for 3D physics joints."""
    
    body_a: "CollisionShape3D"
    body_b: "CollisionShape3D"
    _joint_id: int = field(default=0, repr=False)


@dataclass
class FixedJoint3D(Joint3D):
    """Fixed joint - locks bodies together."""
    
    anchor: Vector3 = field(default_factory=Vector3.zero)


@dataclass
class HingeJoint3D(Joint3D):
    """Hinge joint - rotation around an axis."""
    
    anchor: Vector3 = field(default_factory=Vector3.zero)
    axis: Vector3 = field(default_factory=lambda: Vector3(0, 1, 0))
    min_angle: float = -float('inf')
    max_angle: float = float('inf')


@dataclass
class SliderJoint3D(Joint3D):
    """Slider joint - translation along an axis."""
    
    anchor: Vector3 = field(default_factory=Vector3.zero)
    axis: Vector3 = field(default_factory=lambda: Vector3(0, 1, 0))
    min_distance: float = -float('inf')
    max_distance: float = float('inf')


@dataclass
class DistanceJoint3D(Joint3D):
    """Distance joint - maintains distance between two points."""
    
    anchor_a: Vector3 = field(default_factory=Vector3.zero)
    anchor_b: Vector3 = field(default_factory=Vector3.zero)
    min_distance: float = 0.0
    max_distance: float = float('inf')


@dataclass
class PointJoint3D(Joint3D):
    """Point joint (ball and socket) - rotation around a point."""
    
    anchor: Vector3 = field(default_factory=Vector3.zero)

class PhysicsWorld3D:
    """Manages the Jolt physics world.

    Provides a Pythonic interface to the underlying C++ Jolt Physics bindings.
    """

    def __init__(self, config: PhysicsConfig3D | None = None) -> None:
        """Initialize the physics world.

        Args:
            config: Physics configuration (optional)
        """
        self.config = config or PhysicsConfig3D()
        self._world: Any = None  # _JoltWorld instance
        self._bodies: dict[int, int] = {}  # node_id -> body_id
        self._node_ids: dict[int, "CollisionShape3D"] = {}  # body_id -> node
        self._initialized = False
        self._on_collision: list[Callable[[ContactInfo3D], None]] = []
        self._joints: list[Joint3D] = []

    def initialize(self) -> bool:
        """Initialize the physics world.

        Returns:
            True if initialization successful
        """
        if not JOLT_AVAILABLE or _JoltWorld is None:
            logger.error("Jolt Physics not available - bindings not compiled")
            return False

        try:
            self._world = _JoltWorld()
            if not self._world.initialize():
                logger.error("Failed to initialize Jolt physics world")
                return False

            # Set gravity
            self._world.set_gravity(self.config.gravity.to_tuple())

            # Set collision callback
            self._world.set_collision_callback(self._handle_collision)

            self._initialized = True
            logger.info("3D Physics world initialized")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize 3D physics: {e}")
            return False

    def shutdown(self) -> None:
        """Shut down the physics world."""
        if self._world is not None:
            self._world.shutdown()
            self._world = None
        self._bodies.clear()
        self._node_ids.clear()
        self._initialized = False
        logger.info("3D Physics world shut down")

    def create_body(self, node: "CollisionShape3D") -> bool:
        """Create a physics body for a collision shape node.

        Args:
            node: The CollisionShape3D node to create a body for

        Returns:
            True if body was created successfully
        """
        if not self._initialized:
            return False

        from bevel.nodes.node_3d import ShapeType3D, BodyType3D

        node_id = id(node)
        if node_id in self._bodies:
            return True  # Already created

        # Get transform
        pos = node.global_position.to_tuple()
        rot = node.global_rotation.to_tuple()

        # Map body type
        body_type = {
            BodyType3D.STATIC: 0,
            BodyType3D.KINEMATIC: 1,
            BodyType3D.DYNAMIC: 2,
        }.get(node.physics.body_type, 2)

        body_id = None

        if node.shape == ShapeType3D.BOX:
            half_extents = (node.size.x / 2, node.size.y / 2, node.size.z / 2)
            body_id = self._world.add_box_body(
                pos,
                rot,
                half_extents,
                body_type,
                node.physics.mass,
                node.physics.friction,
                node.physics.restitution,
            )
        elif node.shape == ShapeType3D.SPHERE:
            body_id = self._world.add_sphere_body(
                pos, node.radius, body_type, node.physics.mass
            )
        elif node.shape == ShapeType3D.CAPSULE:
            body_id = self._world.add_capsule_body(
                pos, rot, node.radius, node.height / 2, body_type, node.physics.mass
            )
        else:
            logger.warning(f"Unsupported shape type: {node.shape}")
            return False

        if body_id is not None and body_id != 0:
            self._bodies[node_id] = body_id
            self._node_ids[body_id] = node
            node._body = body_id
            logger.debug(f"Created 3D physics body for '{node.name}' (id={body_id})")
            return True

        logger.warning(f"Failed to create 3D physics body for '{node.name}'")
        return False

    def remove_body(self, node: "CollisionShape3D") -> bool:
        """Remove a physics body.

        Args:
            node: The node whose body should be removed

        Returns:
            True if body was removed
        """
        node_id = id(node)
        if node_id not in self._bodies:
            return False

        body_id = self._bodies[node_id]
        self._world.remove_body(body_id)
        del self._bodies[node_id]
        del self._node_ids[body_id]
        node._body = None
        return True

    def raycast(
        self, 
        start: Vector3, 
        end: Vector3
    ) -> list[tuple[CollisionShape3D, Vector3, Vector3]]:
        """Perform a raycast.
        
        Args:
            start: Ray start position
            end: Ray end position
            
        Returns:
            List of (node, hit_point, hit_normal) tuples
        """
        if not self._initialized:
            return []
            
        results = self._world.raycast(start.to_tuple(), end.to_tuple())
        
        hits = []
        for body_id, point, normal in results:
            node = self._node_ids.get(body_id)
            if node:
                hits.append((
                    node,
                    Vector3(*point),
                    Vector3(*normal)
                ))
        
        return hits

    def step(self, delta: float) -> None:
        """Step the physics simulation.

        Args:
            delta: Time step in seconds
        """
        if not self._initialized:
            return

        self._world.step(delta)
        self._sync_transforms()

    def _sync_transforms(self) -> None:
        """Sync Jolt body transforms to node transforms."""
        from bevel.nodes.node_3d import BodyType3D

        for body_id, node in self._node_ids.items():
            if node.physics.body_type == BodyType3D.STATIC:
                continue

            pos = self._world.get_body_position(body_id)
            rot = self._world.get_body_rotation(body_id)

            node._transform_3d.global_position = Vector3(*pos)
            node._transform_3d.global_rotation = Quaternion(*rot)

    def _handle_collision(
        self,
        body_a_id: int,
        body_b_id: int,
        point: tuple,
        normal: tuple,
        depth: float,
    ) -> None:
        """Handle collision callback from C++.

        Args:
            body_a_id: First body ID
            body_b_id: Second body ID
            point: Contact point
            normal: Contact normal
            depth: Penetration depth
        """
        node_a = self._node_ids.get(body_a_id)
        node_b = self._node_ids.get(body_b_id)

        if node_a is None or node_b is None:
            return

        info = ContactInfo3D(
            node_a=node_a,
            node_b=node_b,
            contact_point=Vector3(*point),
            contact_normal=Vector3(*normal),
            penetration_depth=depth,
        )

        # Call script methods
        for script in node_a.scripts:
            if hasattr(script, "on_collision_enter_3d"):
                try:
                    script.on_collision_enter_3d(node_b, info)
                except Exception as e:
                    logger.error(f"Error in on_collision_enter_3d: {e}")

        for script in node_b.scripts:
            if hasattr(script, "on_collision_enter_3d"):
                try:
                    script.on_collision_enter_3d(node_a, info)
                except Exception as e:
                    logger.error(f"Error in on_collision_enter_3d: {e}")

        # Call registered callbacks
        for callback in self._on_collision:
            try:
                callback(info)
            except Exception as e:
                logger.error(f"Error in collision callback: {e}")

    def setup_scene(self, scene: "Scene") -> None:
        """Set up physics bodies for all CollisionShape3D nodes in a scene.

        Args:
            scene: The scene to set up physics for
        """
        if scene.root is None:
            return

        from bevel.nodes.node_3d import CollisionShape3D

        count = 0
        for node in scene.root.find_nodes_by_type(CollisionShape3D):
            if self.create_body(node):
                count += 1

        logger.info(f"Set up {count} 3D physics bodies")

    def set_gravity(self, gravity: Vector3) -> None:
        """Set the world gravity.

        Args:
            gravity: Gravity vector
        """
        if not self._initialized:
            return
        self._world.set_gravity(gravity.to_tuple())
        self.config.gravity = gravity

    def get_gravity(self) -> Vector3:
        """Get the world gravity.

        Returns:
            Gravity vector
        """
        if not self._initialized:
            return self.config.gravity
        g = self._world.get_gravity()
        return Vector3(*g)

    def apply_force(self, body_id: int, force: Vector3) -> None:
        """Apply a force to a body.

        Args:
            body_id: The body ID
            force: Force vector
        """
        if not self._initialized:
            return
        self._world.apply_force(body_id, force.to_tuple())

    def apply_impulse(self, body_id: int, impulse: Vector3) -> None:
        """Apply an impulse to a body.

        Args:
            body_id: The body ID
            impulse: Impulse vector
        """
        if not self._initialized:
            return
        self._world.apply_impulse(body_id, impulse.to_tuple())

    def apply_torque(self, body_id: int, torque: Vector3) -> None:
        """Apply a torque to a body.

        Args:
            body_id: The body ID
            torque: Torque vector
        """
        if not self._initialized:
            return
        self._world.apply_torque(body_id, torque.to_tuple())

    def set_linear_velocity(self, body_id: int, velocity: Vector3) -> None:
        """Set a body's linear velocity.

        Args:
            body_id: The body ID
            velocity: Velocity vector
        """
        if not self._initialized:
            return
        self._world.set_linear_velocity(body_id, velocity.to_tuple())

    def get_linear_velocity(self, body_id: int) -> Vector3:
        """Get a body's linear velocity.

        Args:
            body_id: The body ID

        Returns:
            Velocity vector
        """
        if not self._initialized:
            return Vector3.zero()
        vel = self._world.get_linear_velocity(body_id)
        return Vector3(*vel)

    def set_angular_velocity(self, body_id: int, velocity: Vector3) -> None:
        """Set a body's angular velocity.

        Args:
            body_id: The body ID
            velocity: Angular velocity vector
        """
        if not self._initialized:
            return
        self._world.set_angular_velocity(body_id, velocity.to_tuple())

    def get_angular_velocity(self, body_id: int) -> Vector3:
        """Get a body's angular velocity.

        Args:
            body_id: The body ID

        Returns:
            Angular velocity vector
        """
        if not self._initialized:
            return Vector3.zero()
        vel = self._world.get_angular_velocity(body_id)
        return Vector3(*vel)

    def on_collision(self, callback: Callable[[ContactInfo3D], None]) -> None:
        """Register a collision callback.

        Args:
            callback: Function to call on collision
        """
        self._on_collision.append(callback)

    def create_fixed_joint(
        self,
        body_a: "CollisionShape3D",
        body_b: "CollisionShape3D",
        anchor: Vector3
    ) -> FixedJoint3D | None:
        """Create a fixed joint."""
        if not self._initialized:
            return None
            
        body_a_id = self._bodies.get(id(body_a))
        body_b_id = self._bodies.get(id(body_b))
        
        if body_a_id is None or body_b_id is None:
            return None
            
        joint_id = self._world.create_fixed_joint(
            body_a_id,
            body_b_id,
            anchor.to_tuple()
        )
        
        if joint_id == 0:
            return None
            
        joint = FixedJoint3D(body_a=body_a, body_b=body_b, anchor=anchor)
        joint._joint_id = joint_id
        self._joints.append(joint)
        return joint

    def create_hinge_joint(
        self,
        body_a: "CollisionShape3D",
        body_b: "CollisionShape3D",
        anchor: Vector3,
        axis: Vector3,
        min_angle: float = -float('inf'),
        max_angle: float = float('inf')
    ) -> HingeJoint3D | None:
        """Create a hinge joint."""
        if not self._initialized:
            return None
            
        body_a_id = self._bodies.get(id(body_a))
        body_b_id = self._bodies.get(id(body_b))
        
        if body_a_id is None or body_b_id is None:
            return None
            
        joint_id = self._world.create_hinge_joint(
            body_a_id,
            body_b_id,
            anchor.to_tuple(),
            axis.normalized().to_tuple(),
            min_angle,
            max_angle
        )
        
        if joint_id == 0:
            return None
            
        joint = HingeJoint3D(
            body_a=body_a,
            body_b=body_b,
            anchor=anchor,
            axis=axis,
            min_angle=min_angle,
            max_angle=max_angle
        )
        joint._joint_id = joint_id
        self._joints.append(joint)
        return joint

    def create_slider_joint(
        self,
        body_a: "CollisionShape3D",
        body_b: "CollisionShape3D",
        anchor: Vector3,
        axis: Vector3,
        min_distance: float = -float('inf'),
        max_distance: float = float('inf')
    ) -> SliderJoint3D | None:
        """Create a slider joint."""
        if not self._initialized:
            return None
            
        body_a_id = self._bodies.get(id(body_a))
        body_b_id = self._bodies.get(id(body_b))
        
        if body_a_id is None or body_b_id is None:
            return None
            
        joint_id = self._world.create_slider_joint(
            body_a_id,
            body_b_id,
            anchor.to_tuple(),
            axis.normalized().to_tuple(),
            min_distance,
            max_distance
        )
        
        if joint_id == 0:
            return None
            
        joint = SliderJoint3D(
            body_a=body_a,
            body_b=body_b,
            anchor=anchor,
            axis=axis,
            min_distance=min_distance,
            max_distance=max_distance
        )
        joint._joint_id = joint_id
        self._joints.append(joint)
        return joint

    def create_distance_joint(
        self,
        body_a: "CollisionShape3D",
        body_b: "CollisionShape3D",
        anchor_a: Vector3,
        anchor_b: Vector3,
        min_distance: float = 0.0,
        max_distance: float = float('inf')
    ) -> DistanceJoint3D | None:
        """Create a distance joint."""
        if not self._initialized:
            return None
            
        body_a_id = self._bodies.get(id(body_a))
        body_b_id = self._bodies.get(id(body_b))
        
        if body_a_id is None or body_b_id is None:
            return None
            
        joint_id = self._world.create_distance_joint(
            body_a_id,
            body_b_id,
            anchor_a.to_tuple(),
            anchor_b.to_tuple(),
            min_distance,
            max_distance
        )
        
        if joint_id == 0:
            return None
            
        joint = DistanceJoint3D(
            body_a=body_a,
            body_b=body_b,
            anchor_a=anchor_a,
            anchor_b=anchor_b,
            min_distance=min_distance,
            max_distance=max_distance
        )
        joint._joint_id = joint_id
        self._joints.append(joint)
        return joint

    def create_point_joint(
        self,
        body_a: "CollisionShape3D",
        body_b: "CollisionShape3D",
        anchor: Vector3
    ) -> PointJoint3D | None:
        """Create a point (ball and socket) joint."""
        if not self._initialized:
            return None
            
        body_a_id = self._bodies.get(id(body_a))
        body_b_id = self._bodies.get(id(body_b))
        
        if body_a_id is None or body_b_id is None:
            return None
            
        joint_id = self._world.create_point_joint(
            body_a_id,
            body_b_id,
            anchor.to_tuple()
        )
        
        if joint_id == 0:
            return None
            
        joint = PointJoint3D(body_a=body_a, body_b=body_b, anchor=anchor)
        joint._joint_id = joint_id
        self._joints.append(joint)
        return joint

    def destroy_joint(self, joint: Joint3D) -> bool:
        """Destroy a joint."""
        if not self._initialized:
            return False
            
        if joint not in self._joints:
            return False
            
        try:
            self._world.destroy_joint(joint._joint_id)
            self._joints.remove(joint)
            return True
        except Exception as e:
            logger.error(f"Failed to destroy joint: {e}")
            return False

    def get_joints(self) -> list[Joint3D]:
        """Get all joints."""
        return list(self._joints)

    def query_aabb(
        self,
        min_point: Vector3,
        max_point: Vector3
    ) -> list["CollisionShape3D"]:
        """Query all bodies in an axis-aligned bounding box.
        
        Args:
            min_point: Minimum corner of the box
            max_point: Maximum corner of the box
            
        Returns:
            List of collision shapes in the box
        """
        if not self._initialized:
            return []
            
        body_ids = self._world.query_aabb(min_point.to_tuple(), max_point.to_tuple())
        
        nodes = []
        for body_id in body_ids:
            node = self._node_ids.get(body_id)
            if node:
                nodes.append(node)
        
        return nodes
    
    def query_sphere(
        self,
        center: Vector3,
        radius: float
    ) -> list["CollisionShape3D"]:
        """Query all bodies in a sphere.
        
        Args:
            center: Center of the sphere
            radius: Radius of the sphere
            
        Returns:
            List of collision shapes in the sphere
        """
        if not self._initialized:
            return []
            
        body_ids = self._world.query_sphere(center.to_tuple(), radius)
        
        nodes = []
        for body_id in body_ids:
            node = self._node_ids.get(body_id)
            if node:
                nodes.append(node)
        
        return nodes