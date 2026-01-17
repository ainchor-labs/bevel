"""Signal system for Bevel engine.

Provides Godot-style signals for decoupled inter-node communication.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, TYPE_CHECKING
from weakref import ref, ReferenceType

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.node import Node

logger = get_logger("signals")


@dataclass
class SignalDefinition:
    """Defines a signal's metadata."""

    name: str
    parameters: list[str] = field(default_factory=list)
    description: str = ""


@dataclass
class SignalConnection:
    """Represents a connection from a signal to a callback."""

    target_node_ref: ReferenceType["Node"]  # Weak reference to target node
    method_name: str
    one_shot: bool = False
    deferred: bool = False  # Call at end of frame

    @property
    def target_node(self) -> "Node | None":
        """Get the target node, or None if deallocated."""
        return self.target_node_ref()

    @property
    def is_valid(self) -> bool:
        """Check if this connection is still valid."""
        return self.target_node_ref() is not None


class Signal:
    """A signal that can be emitted and connected to callbacks."""

    def __init__(self, definition: SignalDefinition, owner: "Node") -> None:
        """Initialize a signal.

        Args:
            definition: The signal definition with metadata
            owner: The node that owns this signal
        """
        self.definition = definition
        self._owner_ref: ReferenceType["Node"] = ref(owner)
        self._connections: list[SignalConnection] = []

    @property
    def owner(self) -> "Node | None":
        """Get the node that owns this signal."""
        return self._owner_ref()

    @property
    def name(self) -> str:
        """Get the signal name."""
        return self.definition.name

    def connect(
        self,
        target_node: "Node",
        method_name: str,
        one_shot: bool = False,
        deferred: bool = False,
    ) -> bool:
        """Connect to a target node's method.

        Args:
            target_node: The node containing the callback method
            method_name: Name of the method to call
            one_shot: Auto-disconnect after first emit
            deferred: Call at end of frame

        Returns:
            True if connected successfully
        """
        # Validate target has the method (on node or its scripts)
        method_found = hasattr(target_node, method_name)
        if not method_found:
            for script in getattr(target_node, "scripts", []):
                if hasattr(script, method_name):
                    method_found = True
                    break

        if not method_found:
            logger.warning(
                f"Cannot connect signal '{self.name}': "
                f"'{target_node.name}' has no method '{method_name}'"
            )
            return False

        # Check for duplicate connection
        for conn in self._connections:
            if conn.target_node is target_node and conn.method_name == method_name:
                logger.debug(
                    f"Signal '{self.name}' already connected to "
                    f"{target_node.name}.{method_name}"
                )
                return True

        connection = SignalConnection(
            target_node_ref=ref(target_node),
            method_name=method_name,
            one_shot=one_shot,
            deferred=deferred,
        )
        self._connections.append(connection)
        logger.debug(f"Connected {self.name} -> {target_node.name}.{method_name}")
        return True

    def disconnect(self, target_node: "Node", method_name: str) -> bool:
        """Disconnect from a target node's method.

        Args:
            target_node: The target node
            method_name: The method name

        Returns:
            True if disconnected, False if not found
        """
        for i, conn in enumerate(self._connections):
            if conn.target_node is target_node and conn.method_name == method_name:
                del self._connections[i]
                logger.debug(
                    f"Disconnected {self.name} from {target_node.name}.{method_name}"
                )
                return True
        return False

    def disconnect_all(self) -> None:
        """Disconnect all connections."""
        self._connections.clear()

    def emit(self, *args: Any) -> None:
        """Emit the signal, calling all connected callbacks.

        Args:
            *args: Arguments to pass to callbacks
        """
        # Clean up dead references
        self._connections = [c for c in self._connections if c.is_valid]

        to_disconnect: list[tuple["Node", str]] = []
        deferred_calls: list[tuple[Callable[..., Any], tuple[Any, ...]]] = []

        for conn in self._connections:
            target = conn.target_node
            if target is None:
                continue

            # Find the method (on node or scripts)
            method: Callable[..., Any] | None = getattr(target, conn.method_name, None)
            if method is None:
                for script in getattr(target, "scripts", []):
                    method = getattr(script, conn.method_name, None)
                    if method is not None:
                        break

            if method is None:
                logger.warning(
                    f"Signal '{self.name}' target method '{conn.method_name}' not found"
                )
                continue

            try:
                if conn.deferred:
                    # Queue for deferred execution
                    deferred_calls.append((method, args))
                else:
                    method(*args)
            except Exception as e:
                logger.error(
                    f"Error in signal '{self.name}' callback "
                    f"'{conn.method_name}': {e}"
                )

            if conn.one_shot:
                to_disconnect.append((target, conn.method_name))

        # Handle deferred calls
        if deferred_calls:
            owner = self._owner_ref()
            if owner is not None and hasattr(owner, "_scene") and owner._scene is not None:
                scene = owner._scene
                if hasattr(scene, "queue_deferred_call"):
                    for method, call_args in deferred_calls:
                        scene.queue_deferred_call(method, call_args)
                else:
                    # Fallback: execute immediately
                    for method, call_args in deferred_calls:
                        try:
                            method(*call_args)
                        except Exception as e:
                            logger.error(f"Error in deferred call: {e}")

        # Remove one-shot connections
        for target, method_name in to_disconnect:
            self.disconnect(target, method_name)

    @property
    def connection_count(self) -> int:
        """Get number of active connections."""
        self._connections = [c for c in self._connections if c.is_valid]
        return len(self._connections)


class SignalManager:
    """Manages signals for a node using composition."""

    def __init__(self) -> None:
        """Initialize the signal manager."""
        self._signals: dict[str, Signal] = {}
        self._signal_definitions: dict[str, SignalDefinition] = {}
        self._owner: "Node | None" = None

    def set_owner(self, owner: "Node") -> None:
        """Set the owner node.

        Args:
            owner: The node that owns these signals
        """
        self._owner = owner

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
            The created Signal object, or None if no owner set
        """
        if self._owner is None:
            logger.error("Cannot define signal: no owner set")
            return None

        if name in self._signals:
            return self._signals[name]

        definition = SignalDefinition(
            name=name,
            parameters=parameters or [],
            description=description,
        )
        self._signal_definitions[name] = definition

        signal = Signal(definition, self._owner)
        self._signals[name] = signal
        return signal

    def get_signal(self, name: str) -> Signal | None:
        """Get a signal by name.

        Args:
            name: The signal name

        Returns:
            The Signal object or None if not found
        """
        return self._signals.get(name)

    def has_signal(self, name: str) -> bool:
        """Check if a signal is defined.

        Args:
            name: The signal name

        Returns:
            True if the signal exists
        """
        return name in self._signals

    def emit_signal(self, name: str, *args: Any) -> bool:
        """Emit a signal by name.

        Args:
            name: The signal name
            *args: Arguments to pass to callbacks

        Returns:
            True if signal exists and was emitted
        """
        signal = self._signals.get(name)
        if signal is None:
            logger.warning(f"Cannot emit undefined signal: {name}")
            return False

        signal.emit(*args)
        return True

    def connect_signal(
        self,
        signal_name: str,
        target_node: "Node",
        method_name: str,
        one_shot: bool = False,
        deferred: bool = False,
    ) -> bool:
        """Connect to a signal on this node.

        Convenience method that combines get_signal + connect.

        Args:
            signal_name: The signal name
            target_node: The target node
            method_name: The method to call
            one_shot: Auto-disconnect after first emit
            deferred: Call at end of frame

        Returns:
            True if connected successfully
        """
        signal = self._signals.get(signal_name)
        if signal is None:
            logger.warning(f"Cannot connect to undefined signal: {signal_name}")
            return False

        return signal.connect(target_node, method_name, one_shot, deferred)

    def disconnect_signal(
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
        signal = self._signals.get(signal_name)
        if signal is None:
            return False
        return signal.disconnect(target_node, method_name)

    def disconnect_all_signals(self) -> None:
        """Disconnect all signal connections (for cleanup)."""
        for signal in self._signals.values():
            signal.disconnect_all()

    def get_signal_list(self) -> list[str]:
        """Get list of defined signal names.

        Returns:
            List of signal names
        """
        return list(self._signals.keys())
