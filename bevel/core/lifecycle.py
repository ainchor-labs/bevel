"""Lifecycle event dispatcher for nodes and scripts."""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING, Any

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.node import Node

logger = get_logger("lifecycle")


class LifecycleEvent(Enum):
    """Lifecycle events that can be dispatched to nodes."""

    AWAKE = auto()  # Called when node is created/added to scene
    ON_ENABLE = auto()  # Called when node becomes active
    START = auto()  # Called before first update
    UPDATE = auto()  # Called every frame
    FIXED_UPDATE = auto()  # Called at fixed intervals (for physics)
    LATE_UPDATE = auto()  # Called after all updates
    ON_DISABLE = auto()  # Called when node becomes inactive
    DESTROY = auto()  # Called when node is destroyed


class LifecycleDispatcher:
    """Dispatches lifecycle events to nodes and their scripts."""

    def __init__(self) -> None:
        self._pending_start: list[Node] = []

    def dispatch_awake(self, node: "Node") -> None:
        """Dispatch awake event to a node and its scripts.

        Args:
            node: The node to dispatch to
        """
        for script in node.scripts:
            if hasattr(script, "awake"):
                try:
                    script.awake()
                except Exception as e:
                    logger.error(f"Error in awake() for '{node.name}': {e}")

        # If node is active, dispatch on_enable after awake
        if node.active:
            self.dispatch_enable(node)

        # Mark for start on next update
        self._pending_start.append(node)

    def dispatch_enable(self, node: "Node") -> None:
        """Dispatch on_enable event to a node and its scripts.

        Called when a node becomes active.

        Args:
            node: The node to dispatch to
        """
        for script in node.scripts:
            if hasattr(script, "on_enable"):
                try:
                    script.on_enable()
                except Exception as e:
                    logger.error(f"Error in on_enable() for '{node.name}': {e}")

    def dispatch_disable(self, node: "Node") -> None:
        """Dispatch on_disable event to a node and its scripts.

        Called when a node becomes inactive.

        Args:
            node: The node to dispatch to
        """
        for script in node.scripts:
            if hasattr(script, "on_disable"):
                try:
                    script.on_disable()
                except Exception as e:
                    logger.error(f"Error in on_disable() for '{node.name}': {e}")

    def dispatch_start(self, node: "Node") -> None:
        """Dispatch start event to a node and its scripts.

        Args:
            node: The node to dispatch to
        """
        if node._started:
            return

        for script in node.scripts:
            if hasattr(script, "start"):
                try:
                    script.start()
                except Exception as e:
                    logger.error(f"Error in start() for '{node.name}': {e}")

        node._started = True

    def dispatch_update(self, node: "Node", delta: float) -> None:
        """Dispatch update event to a node and its scripts.

        Args:
            node: The node to dispatch to
            delta: Time since last frame in seconds
        """
        if not node.active:
            return

        # Handle pending starts
        if node in self._pending_start:
            self.dispatch_start(node)
            self._pending_start.remove(node)

        for script in node.scripts:
            if hasattr(script, "update"):
                try:
                    script.update(delta)
                except Exception as e:
                    logger.error(f"Error in update() for '{node.name}': {e}")

    def dispatch_fixed_update(self, node: "Node", delta: float) -> None:
        """Dispatch fixed update event to a node and its scripts.

        Args:
            node: The node to dispatch to
            delta: Fixed time step in seconds
        """
        if not node.active:
            return

        for script in node.scripts:
            if hasattr(script, "fixed_update"):
                try:
                    script.fixed_update(delta)
                except Exception as e:
                    logger.error(f"Error in fixed_update() for '{node.name}': {e}")

    def dispatch_late_update(self, node: "Node", delta: float) -> None:
        """Dispatch late update event to a node and its scripts.

        Args:
            node: The node to dispatch to
            delta: Time since last frame in seconds
        """
        if not node.active:
            return

        for script in node.scripts:
            if hasattr(script, "late_update"):
                try:
                    script.late_update(delta)
                except Exception as e:
                    logger.error(f"Error in late_update() for '{node.name}': {e}")

    def dispatch_destroy(self, node: "Node") -> None:
        """Dispatch destroy event to a node and its scripts.

        Args:
            node: The node being destroyed
        """
        # Disconnect all signals first
        if hasattr(node, "disconnect_all_signals"):
            try:
                node.disconnect_all_signals()
            except Exception as e:
                logger.error(f"Error disconnecting signals for '{node.name}': {e}")

        for script in node.scripts:
            if hasattr(script, "on_destroy"):
                try:
                    script.on_destroy()
                except Exception as e:
                    logger.error(f"Error in on_destroy() for '{node.name}': {e}")

    def dispatch_tree_awake(self, root: "Node") -> None:
        """Dispatch awake to an entire node tree.

        Args:
            root: The root node of the tree
        """
        self.dispatch_awake(root)
        for child in root.children:
            self.dispatch_tree_awake(child)

    def dispatch_tree_update(self, root: "Node", delta: float) -> None:
        """Dispatch update to an entire node tree.

        Args:
            root: The root node of the tree
            delta: Time since last frame in seconds
        """
        self.dispatch_update(root, delta)
        for child in root.children:
            self.dispatch_tree_update(child, delta)

    def dispatch_tree_fixed_update(self, root: "Node", delta: float) -> None:
        """Dispatch fixed update to an entire node tree.

        Args:
            root: The root node of the tree
            delta: Fixed time step in seconds
        """
        self.dispatch_fixed_update(root, delta)
        for child in root.children:
            self.dispatch_tree_fixed_update(child, delta)

    def dispatch_tree_late_update(self, root: "Node", delta: float) -> None:
        """Dispatch late update to an entire node tree.

        Args:
            root: The root node of the tree
            delta: Time since last frame in seconds
        """
        self.dispatch_late_update(root, delta)
        for child in root.children:
            self.dispatch_tree_late_update(child, delta)

    def dispatch_tree_destroy(self, root: "Node") -> None:
        """Dispatch destroy to an entire node tree (children first).

        Args:
            root: The root node of the tree
        """
        # Destroy children first (depth-first, reverse order)
        for child in reversed(root.children):
            self.dispatch_tree_destroy(child)
        self.dispatch_destroy(root)

    def dispatch_tree_enable(self, root: "Node") -> None:
        """Dispatch on_enable to an entire node tree.

        Args:
            root: The root node of the tree
        """
        if root.active:
            self.dispatch_enable(root)
            for child in root.children:
                self.dispatch_tree_enable(child)

    def dispatch_tree_disable(self, root: "Node") -> None:
        """Dispatch on_disable to an entire node tree.

        Args:
            root: The root node of the tree
        """
        if not root.active:
            self.dispatch_disable(root)
            for child in root.children:
                self.dispatch_tree_disable(child)
