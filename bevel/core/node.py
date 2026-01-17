"""Base Node class for the scene tree."""

from __future__ import annotations

from typing import Any, Callable, TypeVar, Iterator, TYPE_CHECKING
from dataclasses import dataclass, field

from bevel.core.transform import Transform2D
from bevel.core.signals import SignalManager, Signal
from bevel.utils.math import Vector2
from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    pass

logger = get_logger("node")

T = TypeVar("T", bound="Node")


@dataclass
class Node:
    """Base class for all nodes in the scene tree.

    Nodes form a hierarchical tree structure. Each node can have:
    - A parent node (except the root)
    - Multiple child nodes
    - A transform (position, rotation, scale)
    - Custom properties
    - Attached scripts
    """

    name: str = "Node"
    _parent: "Node | None" = field(default=None, repr=False)
    _children: list["Node"] = field(default_factory=list, repr=False)
    _transform: Transform2D = field(default_factory=Transform2D)
    _active: bool = True
    _visible: bool = True
    _properties: dict[str, Any] = field(default_factory=dict)
    _scripts: list[Any] = field(default_factory=list, repr=False)
    _groups: set[str] = field(default_factory=set)
    z_index: int = 0
    layer: str = "world"  # Render layer name
    _scene: Any = field(default=None, repr=False)  # Reference to parent Scene
    _started: bool = field(default=False, repr=False)  # Track if start() was called
    _signal_manager: SignalManager = field(default_factory=SignalManager, repr=False)

    def __post_init__(self) -> None:
        # Link transform to this node
        self._transform._node = self
        # Link signal manager to this node
        self._signal_manager.set_owner(self)

    @property
    def parent(self) -> "Node | None":
        """Get the parent node."""
        return self._parent

    @property
    def children(self) -> list["Node"]:
        """Get the list of child nodes (read-only copy)."""
        return list(self._children)

    @property
    def transform(self) -> Transform2D:
        """Get the transform component."""
        return self._transform

    @property
    def position(self) -> Vector2:
        """Shortcut to get the local position."""
        return self._transform.position

    @position.setter
    def position(self, value: Vector2 | tuple[float, float]) -> None:
        """Shortcut to set the local position."""
        self._transform.position = value

    @property
    def global_position(self) -> Vector2:
        """Shortcut to get the global position."""
        return self._transform.global_position

    @global_position.setter
    def global_position(self, value: Vector2 | tuple[float, float]) -> None:
        """Shortcut to set the global position."""
        self._transform.global_position = value

    @property
    def rotation(self) -> float:
        """Shortcut to get the local rotation (radians)."""
        return self._transform.rotation

    @rotation.setter
    def rotation(self, value: float) -> None:
        """Shortcut to set the local rotation (radians)."""
        self._transform.rotation = value

    @property
    def rotation_degrees(self) -> float:
        """Shortcut to get the local rotation (degrees)."""
        return self._transform.rotation_degrees

    @rotation_degrees.setter
    def rotation_degrees(self, value: float) -> None:
        """Shortcut to set the local rotation (degrees)."""
        self._transform.rotation_degrees = value

    @property
    def scale(self) -> Vector2:
        """Shortcut to get the local scale."""
        return self._transform.scale

    @scale.setter
    def scale(self, value: Vector2 | tuple[float, float]) -> None:
        """Shortcut to set the local scale."""
        self._transform.scale = value

    @property
    def active(self) -> bool:
        """Check if the node is active."""
        return self._active

    @active.setter
    def active(self, value: bool) -> None:
        """Set the node's active state.

        Dispatches on_enable/on_disable events when state changes.
        """
        if self._active == value:
            return

        old_active = self._active
        self._active = value

        # Dispatch lifecycle events if scene is available
        if self._scene is not None and hasattr(self._scene, "_lifecycle"):
            if value and not old_active:
                # Becoming active - dispatch on_enable
                self._scene._lifecycle.dispatch_enable(self)
            elif not value and old_active:
                # Becoming inactive - dispatch on_disable
                self._scene._lifecycle.dispatch_disable(self)

    @property
    def visible(self) -> bool:
        """Check if the node is visible."""
        return self._visible

    @visible.setter
    def visible(self, value: bool) -> None:
        """Set the node's visibility."""
        self._visible = value

    @property
    def scripts(self) -> list[Any]:
        """Get the list of attached scripts."""
        return self._scripts

    @property
    def scene(self) -> Any:
        """Get the scene this node belongs to."""
        return self._scene

    def add_child(self, child: "Node") -> None:
        """Add a child node.

        Args:
            child: The node to add as a child
        """
        if child._parent is not None:
            child._parent.remove_child(child)

        child._parent = self
        child._scene = self._scene
        self._children.append(child)

        # Propagate scene reference to all descendants
        for descendant in child.get_descendants():
            descendant._scene = self._scene

        logger.debug(f"Added child '{child.name}' to '{self.name}'")

    def remove_child(self, child: "Node") -> bool:
        """Remove a child node.

        Args:
            child: The node to remove

        Returns:
            True if the child was removed, False if not found
        """
        if child in self._children:
            self._children.remove(child)
            child._parent = None
            child._scene = None
            logger.debug(f"Removed child '{child.name}' from '{self.name}'")
            return True
        return False

    def get_child(self, name: str) -> "Node | None":
        """Get a direct child by name.

        Args:
            name: The name of the child to find

        Returns:
            The child node or None if not found
        """
        for child in self._children:
            if child.name == name:
                return child
        return None

    def get_node(self, path: str) -> "Node | None":
        """Get a node by path relative to this node.

        Args:
            path: A path like "Child/Grandchild" or "../Sibling"

        Returns:
            The node or None if not found
        """
        parts = path.split("/")
        current: Node | None = self

        for part in parts:
            if current is None:
                return None
            if part == "..":
                current = current._parent
            elif part == ".":
                continue
            else:
                current = current.get_child(part)

        return current

    def find_node(self, name: str) -> "Node | None":
        """Find a node by name in the entire subtree.

        Args:
            name: The name to search for

        Returns:
            The first matching node or None
        """
        if self.name == name:
            return self
        for child in self._children:
            found = child.find_node(name)
            if found is not None:
                return found
        return None

    def find_nodes_by_type(self, node_type: type[T]) -> list[T]:
        """Find all nodes of a specific type in the subtree.

        Args:
            node_type: The type of node to find

        Returns:
            A list of matching nodes
        """
        result: list[T] = []
        if isinstance(self, node_type):
            result.append(self)
        for child in self._children:
            result.extend(child.find_nodes_by_type(node_type))
        return result

    def get_descendants(self) -> Iterator["Node"]:
        """Iterate over all descendants (depth-first).

        Yields:
            Each descendant node
        """
        for child in self._children:
            yield child
            yield from child.get_descendants()

    def get_ancestors(self) -> Iterator["Node"]:
        """Iterate over all ancestors (from parent to root).

        Yields:
            Each ancestor node
        """
        current = self._parent
        while current is not None:
            yield current
            current = current._parent

    def get_root(self) -> "Node":
        """Get the root node of this tree.

        Returns:
            The root node
        """
        current = self
        while current._parent is not None:
            current = current._parent
        return current

    def is_ancestor_of(self, node: "Node") -> bool:
        """Check if this node is an ancestor of another.

        Args:
            node: The potential descendant

        Returns:
            True if this is an ancestor of node
        """
        return self in node.get_ancestors()

    def add_to_group(self, group: str) -> None:
        """Add this node to a group.

        Args:
            group: The group name
        """
        self._groups.add(group)

    def remove_from_group(self, group: str) -> None:
        """Remove this node from a group.

        Args:
            group: The group name
        """
        self._groups.discard(group)

    def is_in_group(self, group: str) -> bool:
        """Check if this node is in a group.

        Args:
            group: The group name

        Returns:
            True if in the group
        """
        return group in self._groups

    def set_property(self, key: str, value: Any) -> None:
        """Set a custom property.

        Args:
            key: The property name
            value: The property value
        """
        self._properties[key] = value

    def get_property(self, key: str, default: Any = None) -> Any:
        """Get a custom property.

        Args:
            key: The property name
            default: Default value if not found

        Returns:
            The property value or default
        """
        return self._properties.get(key, default)

    def has_property(self, key: str) -> bool:
        """Check if a property exists.

        Args:
            key: The property name

        Returns:
            True if the property exists
        """
        return key in self._properties

    def attach_script(self, script: Any, priority: int = 0) -> None:
        """Attach a script to this node.

        Args:
            script: The script instance to attach
            priority: Execution priority (lower runs first, default 0)
        """
        script.node = self
        script._priority = priority
        self._scripts.append(script)
        # Sort scripts by priority (stable sort preserves order for equal priorities)
        self._scripts.sort(key=lambda s: getattr(s, "_priority", 0))
        logger.debug(f"Attached script '{script.__class__.__name__}' to '{self.name}'")

    def detach_script(self, script: Any) -> bool:
        """Detach a script from this node.

        Args:
            script: The script to detach

        Returns:
            True if detached, False if not found
        """
        if script in self._scripts:
            self._scripts.remove(script)
            script.node = None
            return True
        return False

    def get_script(self, name: str) -> Any | None:
        """Get a script by class name.

        Args:
            name: The script class name (e.g., "PlayerMovement")

        Returns:
            The script instance or None if not found
        """
        for script in self._scripts:
            if script.__class__.__name__ == name:
                return script
        return None

    def get_script_by_type(self, script_type: type[T]) -> T | None:
        """Get a script by type.

        Args:
            script_type: The script class type

        Returns:
            The script instance or None if not found
        """
        for script in self._scripts:
            if isinstance(script, script_type):
                return script  # type: ignore
        return None

    def get_scripts_by_type(self, script_type: type[T]) -> list[T]:
        """Get all scripts of a specific type.

        Args:
            script_type: The script class type

        Returns:
            List of matching script instances
        """
        return [s for s in self._scripts if isinstance(s, script_type)]

    def has_script(self, name: str) -> bool:
        """Check if a script is attached by class name.

        Args:
            name: The script class name

        Returns:
            True if the script is attached
        """
        return self.get_script(name) is not None

    # Signal system methods

    def define_signal(
        self,
        name: str,
        parameters: list[str] | None = None,
        description: str = "",
    ) -> Signal | None:
        """Define a signal on this node.

        Args:
            name: The signal name
            parameters: Expected parameter names (for documentation)
            description: Signal description

        Returns:
            The created Signal object
        """
        return self._signal_manager.define_signal(name, parameters, description)

    def emit_signal(self, name: str, *args: Any) -> bool:
        """Emit a signal by name.

        Args:
            name: The signal name
            *args: Arguments to pass to callbacks

        Returns:
            True if signal exists and was emitted
        """
        return self._signal_manager.emit_signal(name, *args)

    def connect(
        self,
        signal_name: str,
        target_node: "Node",
        method_name: str,
        one_shot: bool = False,
        deferred: bool = False,
    ) -> bool:
        """Connect to a signal on this node.

        Args:
            signal_name: The signal name to connect to
            target_node: The node containing the callback method
            method_name: Name of the method to call when signal emits
            one_shot: Auto-disconnect after first emit
            deferred: Call at end of frame

        Returns:
            True if connected successfully
        """
        return self._signal_manager.connect_signal(
            signal_name, target_node, method_name, one_shot, deferred
        )

    def disconnect(
        self,
        signal_name: str,
        target_node: "Node",
        method_name: str,
    ) -> bool:
        """Disconnect from a signal on this node.

        Args:
            signal_name: The signal name
            target_node: The target node
            method_name: The method name

        Returns:
            True if disconnected
        """
        return self._signal_manager.disconnect_signal(
            signal_name, target_node, method_name
        )

    def has_signal(self, name: str) -> bool:
        """Check if a signal is defined.

        Args:
            name: The signal name

        Returns:
            True if the signal exists
        """
        return self._signal_manager.has_signal(name)

    def get_signal(self, name: str) -> Signal | None:
        """Get a signal by name.

        Args:
            name: The signal name

        Returns:
            The Signal object or None if not found
        """
        return self._signal_manager.get_signal(name)

    def get_signal_list(self) -> list[str]:
        """Get list of defined signal names.

        Returns:
            List of signal names
        """
        return self._signal_manager.get_signal_list()

    def disconnect_all_signals(self) -> None:
        """Disconnect all signal connections (for cleanup)."""
        self._signal_manager.disconnect_all_signals()

    def queue_free(self) -> None:
        """Mark this node for deletion at the end of the frame."""
        if self._scene is not None:
            self._scene.queue_free(self)
        elif self._parent is not None:
            self._parent.remove_child(self)

    def reparent(self, new_parent: "Node") -> None:
        """Move this node to a new parent while preserving global transform.

        Args:
            new_parent: The new parent node
        """
        # Store global transform
        global_pos = self.global_position
        global_rot = self._transform.global_rotation
        global_scale = self._transform.global_scale

        # Reparent
        if self._parent is not None:
            self._parent.remove_child(self)
        new_parent.add_child(self)

        # Restore global transform
        self._transform.global_position = global_pos
        self._transform.global_rotation = global_rot
        self._transform.global_scale = global_scale

    def duplicate(self) -> "Node":
        """Create a deep copy of this node and its children.

        Returns:
            A new Node instance
        """
        # Create a new instance of the same type
        new_node = self.__class__(name=f"{self.name}_copy")
        new_node._transform = self._transform.copy()
        new_node._transform._node = new_node
        new_node._active = self._active
        new_node._visible = self._visible
        new_node._properties = dict(self._properties)
        new_node._groups = set(self._groups)
        new_node.z_index = self.z_index

        # Duplicate children
        for child in self._children:
            new_node.add_child(child.duplicate())

        return new_node

    def print_tree(self, indent: int = 0) -> str:
        """Generate a string representation of the node tree.

        Args:
            indent: Current indentation level

        Returns:
            A formatted string showing the tree structure
        """
        result = "  " * indent + f"- {self.name} ({self.__class__.__name__})\n"
        for child in self._children:
            result += child.print_tree(indent + 1)
        return result

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}')"
