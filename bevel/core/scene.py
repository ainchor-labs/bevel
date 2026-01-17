"""Scene and SceneManager classes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Sequence, TYPE_CHECKING
from pathlib import Path

from bevel.core.node import Node
from bevel.core.lifecycle import LifecycleDispatcher
from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.transitions import Transition

logger = get_logger("scene")


@dataclass
class SceneConfig:
    """Configuration for a scene."""

    name: str = "Untitled Scene"
    type: str = "2D"
    background_color: tuple[int, int, int, int] = (0, 0, 0, 255)


@dataclass
class Scene:
    """A scene containing a tree of nodes."""

    config: SceneConfig = field(default_factory=SceneConfig)
    root: Node | None = None
    _lifecycle: LifecycleDispatcher = field(default_factory=LifecycleDispatcher, repr=False)
    _pending_free: list[Node] = field(default_factory=list, repr=False)
    _groups: dict[str, set[Node]] = field(default_factory=dict, repr=False)
    _deferred_calls: list[tuple[Callable[..., Any], Sequence[Any]]] = field(
        default_factory=list, repr=False
    )
    file_path: str | None = None

    def __post_init__(self) -> None:
        if self.root is not None:
            self._set_scene_recursive(self.root)

    def _set_scene_recursive(self, node: Node) -> None:
        """Set the scene reference for a node and its descendants."""
        node._scene = self
        for child in node.children:
            self._set_scene_recursive(child)

    def set_root(self, root: Node) -> None:
        """Set the root node of the scene.

        Args:
            root: The new root node
        """
        self.root = root
        self._set_scene_recursive(root)

    def awake(self) -> None:
        """Called when the scene is loaded. Dispatches awake to all nodes."""
        if self.root is not None:
            self._lifecycle.dispatch_tree_awake(self.root)
            logger.info(f"Scene '{self.config.name}' awakened")

    def update(self, delta: float) -> None:
        """Update the scene.

        Args:
            delta: Time since last frame in seconds
        """
        if self.root is not None:
            self._lifecycle.dispatch_tree_update(self.root, delta)

        # Process queued deletions
        self._process_free_queue()

    def fixed_update(self, delta: float) -> None:
        """Fixed update for physics.

        Args:
            delta: Fixed time step in seconds
        """
        if self.root is not None:
            self._lifecycle.dispatch_tree_fixed_update(self.root, delta)

    def late_update(self, delta: float) -> None:
        """Late update (after main update).

        Args:
            delta: Time since last frame in seconds
        """
        if self.root is not None:
            self._lifecycle.dispatch_tree_late_update(self.root, delta)

        # Process deferred calls at end of frame
        self._process_deferred_calls()

    def queue_deferred_call(
        self, callback: Callable[..., Any], args: Sequence[Any]
    ) -> None:
        """Queue a callback to be called at end of frame.

        Used by the signal system for deferred signal connections.

        Args:
            callback: The function to call
            args: Arguments to pass to the callback
        """
        self._deferred_calls.append((callback, args))

    def _process_deferred_calls(self) -> None:
        """Process all queued deferred calls."""
        if not self._deferred_calls:
            return

        calls = self._deferred_calls.copy()
        self._deferred_calls.clear()

        for callback, args in calls:
            try:
                callback(*args)
            except Exception as e:
                logger.error(f"Error in deferred call: {e}")

    def queue_free(self, node: Node) -> None:
        """Queue a node for deletion at the end of the frame.

        Args:
            node: The node to delete
        """
        if node not in self._pending_free:
            self._pending_free.append(node)

    def _process_free_queue(self) -> None:
        """Process all pending node deletions."""
        for node in self._pending_free:
            self._lifecycle.dispatch_tree_destroy(node)
            if node.parent is not None:
                node.parent.remove_child(node)
        self._pending_free.clear()

    def find_node(self, name: str) -> Node | None:
        """Find a node by name in the scene.

        Args:
            name: The node name to search for

        Returns:
            The node or None if not found
        """
        if self.root is not None:
            return self.root.find_node(name)
        return None

    def get_nodes_in_group(self, group: str) -> list[Node]:
        """Get all nodes in a group.

        Args:
            group: The group name

        Returns:
            List of nodes in the group
        """
        result: list[Node] = []
        if self.root is not None:
            for node in self.root.get_descendants():
                if node.is_in_group(group):
                    result.append(node)
            if self.root.is_in_group(group):
                result.append(self.root)
        return result

    def add_node(self, node: Node, parent: Node | None = None) -> None:
        """Add a node to the scene.

        Args:
            node: The node to add
            parent: Optional parent (defaults to root)
        """
        if parent is None:
            parent = self.root

        if parent is not None:
            parent.add_child(node)
        else:
            self.set_root(node)

        # Dispatch awake to the new node tree
        self._lifecycle.dispatch_tree_awake(node)

    def remove_node(self, node: Node) -> None:
        """Remove a node from the scene.

        Args:
            node: The node to remove
        """
        self.queue_free(node)


class SceneManager:
    """Manages scene loading, switching, and the scene stack."""

    def __init__(self) -> None:
        self._current_scene: Scene | None = None
        self._scene_stack: list[Scene] = []
        self._loader: Any = None  # Will be set by engine
        self._on_scene_changed: list[Callable[[Scene | None], None]] = []
        self._transition: "Transition | None" = None
        self._pending_scene: Scene | str | None = None
        self._renderer: Any = None  # For transition rendering

    @property
    def current_scene(self) -> Scene | None:
        """Get the currently active scene."""
        return self._current_scene

    def set_loader(self, loader: Any) -> None:
        """Set the scene loader.

        Args:
            loader: A SceneLoader instance
        """
        self._loader = loader

    def load_scene(self, path: str) -> Scene | None:
        """Load a scene from a file.

        Args:
            path: Path to the scene YAML file

        Returns:
            The loaded Scene or None if loading failed
        """
        if self._loader is None:
            logger.error("No scene loader set")
            return None

        scene = self._loader.load(path)
        if scene is not None:
            logger.info(f"Loaded scene: {path}")
        return scene

    def set_renderer(self, renderer: Any) -> None:
        """Set the renderer for transition effects.

        Args:
            renderer: The Renderer instance
        """
        self._renderer = renderer

    def change_scene(
        self,
        scene: Scene | str,
        transition: str | "Transition | None" = None,
        duration: float = 0.5,
        **transition_kwargs: Any,
    ) -> bool:
        """Change to a new scene with optional transition.

        Args:
            scene: A Scene instance or path to a scene file
            transition: Transition name, Transition instance, or None for immediate
            duration: Transition duration in seconds
            **transition_kwargs: Additional arguments for the transition

        Returns:
            True if successful, False otherwise
        """
        # If a transition is already in progress, queue this change
        if self._transition is not None and self._transition.is_active:
            logger.warning("Transition already in progress")
            return False

        # Handle transition
        if transition is not None:
            return self._change_with_transition(scene, transition, duration, **transition_kwargs)

        # Immediate change (no transition)
        return self._change_scene_immediate(scene)

    def _change_scene_immediate(self, scene: Scene | str) -> bool:
        """Change scene immediately without transition.

        Args:
            scene: A Scene instance or path to a scene file

        Returns:
            True if successful
        """
        # Load if path is provided
        if isinstance(scene, str):
            loaded = self.load_scene(scene)
            if loaded is None:
                return False
            scene = loaded

        # Unload current scene
        if self._current_scene is not None:
            self._unload_scene(self._current_scene)

        # Set new scene
        self._current_scene = scene
        scene.awake()

        # Notify listeners
        for callback in self._on_scene_changed:
            callback(scene)

        logger.info(f"Changed to scene: {scene.config.name}")
        return True

    def _change_with_transition(
        self,
        scene: Scene | str,
        transition: str | "Transition",
        duration: float,
        **kwargs: Any,
    ) -> bool:
        """Change scene with a visual transition.

        Args:
            scene: Target scene
            transition: Transition name or instance
            duration: Transition duration
            **kwargs: Additional transition arguments

        Returns:
            True if transition started
        """
        from bevel.core.transitions import create_transition, Transition as TransitionClass

        # Create transition if name provided
        if isinstance(transition, str):
            self._transition = create_transition(transition, duration=duration, **kwargs)
        else:
            self._transition = transition

        # Store pending scene
        self._pending_scene = scene

        # Start transition
        self._transition.start(
            on_switch=self._on_transition_switch,
            on_complete=self._on_transition_complete,
        )

        logger.debug(f"Started transition to new scene")
        return True

    def _on_transition_switch(self) -> None:
        """Called at the midpoint of a transition to switch scenes."""
        if self._pending_scene is not None:
            self._change_scene_immediate(self._pending_scene)
            self._pending_scene = None

    def _on_transition_complete(self) -> None:
        """Called when a transition completes."""
        self._transition = None
        logger.debug("Transition complete")

    @property
    def is_transitioning(self) -> bool:
        """Check if a transition is in progress."""
        return self._transition is not None and self._transition.is_active

    @property
    def current_transition(self) -> "Transition | None":
        """Get the current transition if active."""
        return self._transition if self.is_transitioning else None

    def push_scene(self, scene: Scene | str) -> bool:
        """Push a new scene onto the stack (pausing the current).

        Args:
            scene: A Scene instance or path to a scene file

        Returns:
            True if successful, False otherwise
        """
        if isinstance(scene, str):
            loaded = self.load_scene(scene)
            if loaded is None:
                return False
            scene = loaded

        if self._current_scene is not None:
            self._scene_stack.append(self._current_scene)

        self._current_scene = scene
        scene.awake()

        for callback in self._on_scene_changed:
            callback(scene)

        logger.info(f"Pushed scene: {scene.config.name}")
        return True

    def pop_scene(self) -> bool:
        """Pop the current scene and return to the previous one.

        Returns:
            True if successful, False if stack is empty
        """
        if not self._scene_stack:
            logger.warning("Cannot pop scene: stack is empty")
            return False

        if self._current_scene is not None:
            self._unload_scene(self._current_scene)

        self._current_scene = self._scene_stack.pop()

        for callback in self._on_scene_changed:
            callback(self._current_scene)

        logger.info(f"Popped to scene: {self._current_scene.config.name}")
        return True

    def reload_scene(self) -> bool:
        """Reload the current scene from its file.

        Returns:
            True if successful, False otherwise
        """
        if self._current_scene is None or self._current_scene.file_path is None:
            logger.warning("Cannot reload: no current scene or file path")
            return False

        return self.change_scene(self._current_scene.file_path)

    def _unload_scene(self, scene: Scene) -> None:
        """Unload a scene.

        Args:
            scene: The scene to unload
        """
        if scene.root is not None:
            scene._lifecycle.dispatch_tree_destroy(scene.root)
        logger.debug(f"Unloaded scene: {scene.config.name}")

    def on_scene_changed(self, callback: Callable[[Scene | None], None]) -> None:
        """Register a callback for scene changes.

        Args:
            callback: Function to call when scene changes
        """
        self._on_scene_changed.append(callback)

    def update(self, delta: float) -> None:
        """Update the current scene.

        Args:
            delta: Time since last frame in seconds
        """
        # Update transition if active
        if self._transition is not None and self._transition.is_active:
            self._transition.update(delta)

        if self._current_scene is not None:
            self._current_scene.update(delta)

    def fixed_update(self, delta: float) -> None:
        """Fixed update for the current scene.

        Args:
            delta: Fixed time step in seconds
        """
        if self._current_scene is not None:
            self._current_scene.fixed_update(delta)

    def late_update(self, delta: float) -> None:
        """Late update for the current scene.

        Args:
            delta: Time since last frame in seconds
        """
        if self._current_scene is not None:
            self._current_scene.late_update(delta)

    def render_transition(self) -> None:
        """Render the current transition overlay if active.

        Call this after rendering the scene.
        """
        if self._transition is not None and self._transition.is_active:
            if self._renderer is not None:
                self._transition.render(self._renderer)
