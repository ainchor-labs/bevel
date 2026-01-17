"""Hot reload system for Bevel engine.

Watches files for changes and reloads scripts/scenes while preserving state.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, TYPE_CHECKING
from threading import Thread

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.scene import Scene, SceneManager
    from bevel.core.node import Node

logger = get_logger("hot_reload")

# Try to import watchdog, but make it optional
try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler, FileModifiedEvent
    WATCHDOG_AVAILABLE = True
except ImportError:
    WATCHDOG_AVAILABLE = False
    Observer = None
    FileSystemEventHandler = object


@dataclass
class ReloadEvent:
    """Represents a file change that triggers a reload."""

    path: Path
    event_type: str  # "modified", "created", "deleted"
    timestamp: float = field(default_factory=time.time)


@dataclass
class PreservedState:
    """State preserved during a hot reload."""

    node_states: dict[str, dict[str, Any]] = field(default_factory=dict)
    global_state: dict[str, Any] = field(default_factory=dict)


class StatePreserver:
    """Preserves and restores node state during hot reload."""

    def __init__(self) -> None:
        self._preserved: PreservedState = PreservedState()
        self._preserve_callbacks: list[Callable[[Node], dict[str, Any]]] = []
        self._restore_callbacks: list[Callable[[Node, dict[str, Any]], None]] = []

    def register_preserve_callback(
        self, callback: Callable[["Node"], dict[str, Any]]
    ) -> None:
        """Register a callback to preserve custom state.

        Args:
            callback: Function that takes a node and returns state dict
        """
        self._preserve_callbacks.append(callback)

    def register_restore_callback(
        self, callback: Callable[["Node", dict[str, Any]], None]
    ) -> None:
        """Register a callback to restore custom state.

        Args:
            callback: Function that takes a node and state dict
        """
        self._restore_callbacks.append(callback)

    def preserve_scene(self, scene: "Scene") -> PreservedState:
        """Preserve the state of all nodes in a scene.

        Args:
            scene: The scene to preserve

        Returns:
            PreservedState object containing all state
        """
        self._preserved = PreservedState()

        if scene.root is not None:
            self._preserve_node_recursive(scene.root)

        logger.debug(f"Preserved state for {len(self._preserved.node_states)} nodes")
        return self._preserved

    def _preserve_node_recursive(self, node: "Node") -> None:
        """Preserve state for a node and its descendants.

        Args:
            node: The node to preserve
        """
        node_path = self._get_node_path(node)
        state: dict[str, Any] = {}

        # Preserve transform
        state["position"] = (node.position.x, node.position.y)
        state["rotation"] = node.rotation
        state["scale"] = (node.scale.x, node.scale.y)

        # Preserve common properties
        state["active"] = node.active
        state["visible"] = node.visible
        state["z_index"] = node.z_index
        state["layer"] = node.layer

        # Preserve custom properties
        state["properties"] = dict(node._properties)

        # Preserve script state
        state["scripts"] = {}
        for script in node._scripts:
            script_state = self._preserve_script_state(script)
            if script_state:
                state["scripts"][script.__class__.__name__] = script_state

        # Call custom preserve callbacks
        for callback in self._preserve_callbacks:
            try:
                custom_state = callback(node)
                if custom_state:
                    state.update(custom_state)
            except Exception as e:
                logger.warning(f"Error in preserve callback: {e}")

        self._preserved.node_states[node_path] = state

        # Recurse to children
        for child in node.children:
            self._preserve_node_recursive(child)

    def _preserve_script_state(self, script: Any) -> dict[str, Any]:
        """Preserve state from a script.

        Args:
            script: The script instance

        Returns:
            Dictionary of preserved state
        """
        state: dict[str, Any] = {}

        # Check if script has a custom preserve method
        if hasattr(script, "preserve_state"):
            try:
                return script.preserve_state()
            except Exception as e:
                logger.warning(f"Error in script.preserve_state(): {e}")

        # Otherwise preserve public attributes that aren't methods
        for name in dir(script):
            if name.startswith("_"):
                continue
            try:
                value = getattr(script, name)
                if callable(value):
                    continue
                # Only preserve simple types
                if isinstance(value, (int, float, str, bool, list, dict, tuple)):
                    state[name] = value
            except Exception:
                pass

        return state

    def restore_scene(self, scene: "Scene", preserved: PreservedState | None = None) -> None:
        """Restore preserved state to a scene.

        Args:
            scene: The scene to restore to
            preserved: State to restore (uses last preserved if None)
        """
        if preserved is None:
            preserved = self._preserved

        if scene.root is not None:
            self._restore_node_recursive(scene.root, preserved)

        logger.debug(f"Restored state to scene")

    def _restore_node_recursive(self, node: "Node", preserved: PreservedState) -> None:
        """Restore state to a node and its descendants.

        Args:
            node: The node to restore
            preserved: The preserved state
        """
        node_path = self._get_node_path(node)
        state = preserved.node_states.get(node_path)

        if state is not None:
            # Restore transform
            if "position" in state:
                node.position = state["position"]
            if "rotation" in state:
                node.rotation = state["rotation"]
            if "scale" in state:
                node.scale = state["scale"]

            # Restore common properties
            if "active" in state:
                node._active = state["active"]  # Direct set to avoid events
            if "visible" in state:
                node.visible = state["visible"]
            if "z_index" in state:
                node.z_index = state["z_index"]
            if "layer" in state:
                node.layer = state["layer"]

            # Restore custom properties
            if "properties" in state:
                node._properties.update(state["properties"])

            # Restore script state
            if "scripts" in state:
                for script in node._scripts:
                    script_name = script.__class__.__name__
                    if script_name in state["scripts"]:
                        self._restore_script_state(script, state["scripts"][script_name])

            # Call custom restore callbacks
            for callback in self._restore_callbacks:
                try:
                    callback(node, state)
                except Exception as e:
                    logger.warning(f"Error in restore callback: {e}")

        # Recurse to children
        for child in node.children:
            self._restore_node_recursive(child, preserved)

    def _restore_script_state(self, script: Any, state: dict[str, Any]) -> None:
        """Restore state to a script.

        Args:
            script: The script instance
            state: State to restore
        """
        # Check if script has a custom restore method
        if hasattr(script, "restore_state"):
            try:
                script.restore_state(state)
                return
            except Exception as e:
                logger.warning(f"Error in script.restore_state(): {e}")

        # Otherwise restore attributes directly
        for name, value in state.items():
            if hasattr(script, name):
                try:
                    setattr(script, name, value)
                except Exception:
                    pass

    def _get_node_path(self, node: "Node") -> str:
        """Get a unique path for a node in the tree.

        Args:
            node: The node

        Returns:
            A path string like "Root/Child/Grandchild"
        """
        parts = [node.name]
        current = node.parent
        while current is not None:
            parts.insert(0, current.name)
            current = current.parent
        return "/".join(parts)

if WATCHDOG_AVAILABLE:
    class _FileChangeHandler(FileSystemEventHandler):
        """Handles file system events for hot reload."""

        def __init__(self, hot_reloader: "HotReloader") -> None:
            super().__init__()
            self._hot_reloader = hot_reloader

        def on_modified(self, event: Any) -> None:
            if event.is_directory:
                return

            path = Path(event.src_path)
            if self._should_handle(path):
                logger.debug(f"File modified detected: {path}")
                # Queue the reload instead of processing immediately
                # This ensures it happens on the main thread
                self._hot_reloader._queue_reload(
                    ReloadEvent(path=path, event_type="modified")
                )

        def _should_handle(self, path: Path) -> bool:
            """Check if this file should trigger a reload."""
            # Handle Python scripts
            if path.suffix == ".py":
                logger.debug(f"Python file change detected: {path.name}")
                return True
            # Handle YAML scenes
            if path.suffix in (".yaml", ".yml"):
                logger.debug(f"YAML file change detected: {path.name}")
                return True
            return False
else:
    _FileChangeHandler = None


class HotReloader:
    """Watches files and triggers reloads when they change."""

    def __init__(
        self,
        scene_manager: "SceneManager",
        project_root: Path | None = None,
    ) -> None:
        """Initialize the hot reloader.

        Args:
            scene_manager: The SceneManager to reload scenes with
            project_root: Root directory to watch
        """
        if not WATCHDOG_AVAILABLE:
            logger.warning(
                "watchdog not installed. Hot reload disabled. "
                "Install with: pip install bevel[hotreload]"
            )

        self._scene_manager = scene_manager
        self._project_root = project_root or Path.cwd()
        self._state_preserver = StatePreserver()
        self._observer: Any = None
        self._pending_reloads: list[ReloadEvent] = []
        self._reload_callbacks: list[Callable[[ReloadEvent], None]] = []
        self._enabled = False
        self._debounce_time = 0.1  # Seconds to wait before processing changes
        self._last_reload_time = 0.0
        self._watch_paths: list[Path] = []
        self._script_modules: dict[str, Any] = {}  # Track loaded script modules

    @property
    def enabled(self) -> bool:
        """Check if hot reload is enabled."""
        return self._enabled

    @property
    def state_preserver(self) -> StatePreserver:
        """Get the state preserver."""
        return self._state_preserver

    def start(self, watch_paths: list[str] | None = None) -> bool:
        """Start watching for file changes.

        Args:
            watch_paths: Paths to watch relative to project root

        Returns:
            True if started successfully
        """
        if not WATCHDOG_AVAILABLE:
            logger.error("Hot reload requires watchdog. Install with: pip install watchdog")
            return False

        if self._enabled:
            logger.warning("Hot reload already started")
            return True

        # Set up watch paths
        if watch_paths:
            self._watch_paths = [self._project_root / p for p in watch_paths]
        else:
            # Default watch paths - watch common directories
            self._watch_paths = [
                self._project_root / "scenes",
                self._project_root / "scripts",
                self._project_root / "prefabs",
            ]

        # Filter to existing paths
        existing_paths = [p for p in self._watch_paths if p.exists()]
        
        if not existing_paths:
            logger.warning(f"No watch paths found in {self._project_root}")
            logger.warning(f"Attempted paths: {', '.join(str(p.name) for p in self._watch_paths)}")
            return False
        
        self._watch_paths = existing_paths

        # Start observer
        self._observer = Observer()
        handler = _FileChangeHandler(self)

        for watch_path in self._watch_paths:
            try:
                self._observer.schedule(handler, str(watch_path), recursive=True)
                logger.info(f"Hot reload watching: {watch_path.relative_to(self._project_root)}")
            except Exception as e:
                logger.warning(f"Failed to watch {watch_path}: {e}")

        self._observer.start()
        self._enabled = True
        logger.info("Hot reload active - edit .py or .yaml files to see changes")
        return True

    def stop(self) -> None:
        """Stop watching for file changes."""
        if self._observer is not None:
            self._observer.stop()
            self._observer.join()
            self._observer = None
        self._enabled = False
        logger.info("Hot reload stopped")

    def _queue_reload(self, event: ReloadEvent) -> None:
        """Queue a reload event for processing.

        Args:
            event: The reload event
        """
        # Check if this file is already queued
        for pending in self._pending_reloads:
            if pending.path == event.path:
                pending.timestamp = event.timestamp
                logger.debug(f"Updated queued reload: {event.path.name}")
                return

        self._pending_reloads.append(event)
        logger.info(f"Queued reload for: {event.path.name}")

    def update(self) -> None:
        """Process pending reloads. Call this once per frame."""
        if not self._enabled or not self._pending_reloads:
            return

        current_time = time.time()

        # Check if debounce time has passed
        if current_time - self._last_reload_time < self._debounce_time:
            return

        # Check if all pending events are old enough
        oldest_event = min(self._pending_reloads, key=lambda e: e.timestamp)
        if current_time - oldest_event.timestamp < self._debounce_time:
            return

        # Process all pending reloads
        events = self._pending_reloads.copy()
        self._pending_reloads.clear()
        self._last_reload_time = current_time

        for event in events:
            self._process_reload(event)

    def _process_reload(self, event: ReloadEvent) -> None:
        """Process a single reload event.

        Args:
            event: The reload event
        """
        logger.info(f"Hot reloading: {event.path}")

        # Notify callbacks
        for callback in self._reload_callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Error in reload callback: {e}")

        if event.path.suffix == ".py":
            self._reload_script(event.path)
        elif event.path.suffix in (".yaml", ".yml"):
            self._reload_scene(event.path)

    def _reload_script(self, path: Path) -> None:
        """Reload a Python script.

        Args:
            path: Path to the script
        """
        # Preserve current scene state
        current_scene = self._scene_manager.current_scene
        preserved_state = None
        if current_scene is not None:
            preserved_state = self._state_preserver.preserve_scene(current_scene)

        try:
            # Find the module name
            module_name = self._path_to_module(path)

            if module_name in sys.modules:
                # Reload existing module
                module = sys.modules[module_name]
                importlib.reload(module)
                logger.info(f"Reloaded module: {module_name}")
            else:
                # Load new module
                spec = importlib.util.spec_from_file_location(module_name, path)
                if spec is not None and spec.loader is not None:
                    module = importlib.util.module_from_spec(spec)
                    sys.modules[module_name] = module
                    spec.loader.exec_module(module)
                    logger.info(f"Loaded new module: {module_name}")

            # Reload the current scene to apply script changes
            if current_scene is not None and current_scene.file_path is not None:
                # Use the immediate reload to avoid freezing
                scene_path = current_scene.file_path
                
                # Load the scene fresh
                new_scene = self._scene_manager._loader.load_scene(scene_path)
                if new_scene is not None:
                    # Unload old scene
                    self._scene_manager._unload_scene(current_scene)
                    
                    # Set new scene
                    self._scene_manager._current_scene = new_scene
                    new_scene.awake()
                    
                    # Restore state
                    if preserved_state is not None:
                        self._state_preserver.restore_scene(new_scene, preserved_state)
                    
                    # Notify listeners
                    for callback in self._scene_manager._on_scene_changed:
                        callback(new_scene)

        except Exception as e:
            logger.error(f"Failed to reload script {path}: {e}")
            import traceback
            traceback.print_exc()

    def _reload_scene(self, path: Path) -> None:
        """Reload a scene YAML file.

        Args:
            path: Path to the scene file
        """
        current_scene = self._scene_manager.current_scene

        # Check if this is the current scene
        if current_scene is not None and current_scene.file_path is not None:
            current_path = Path(current_scene.file_path)
            if current_path.resolve() == path.resolve():
                # Preserve state
                preserved_state = self._state_preserver.preserve_scene(current_scene)

                try:
                    # Load the scene fresh
                    new_scene = self._scene_manager._loader.load_scene(str(path))
                    if new_scene is not None:
                        # Unload old scene
                        self._scene_manager._unload_scene(current_scene)
                        
                        # Set new scene
                        self._scene_manager._current_scene = new_scene
                        new_scene.awake()
                        
                        # Restore state
                        self._state_preserver.restore_scene(new_scene, preserved_state)
                        
                        # Notify listeners
                        for callback in self._scene_manager._on_scene_changed:
                            callback(new_scene)

                    logger.info(f"Reloaded scene: {path}")
                except Exception as e:
                    logger.error(f"Failed to reload scene {path}: {e}")
                    import traceback
                    traceback.print_exc()

    def _path_to_module(self, path: Path) -> str:
        """Convert a file path to a Python module name.

        Args:
            path: The file path

        Returns:
            Module name like "scripts.player_movement"
        """
        try:
            relative = path.relative_to(self._project_root)
            parts = list(relative.parts)
            # Remove .py extension
            if parts[-1].endswith(".py"):
                parts[-1] = parts[-1][:-3]
            return ".".join(parts)
        except ValueError:
            # Path is not relative to project root
            return path.stem

    def on_reload(self, callback: Callable[[ReloadEvent], None]) -> None:
        """Register a callback for reload events.

        Args:
            callback: Function to call when a file is reloaded
        """
        self._reload_callbacks.append(callback)

    def force_reload(self) -> None:
        """Force reload the current scene."""
        current_scene = self._scene_manager.current_scene
        if current_scene is not None and current_scene.file_path is not None:
            preserved_state = self._state_preserver.preserve_scene(current_scene)

            try:
                self._scene_manager.reload_scene()

                new_scene = self._scene_manager.current_scene
                if new_scene is not None:
                    self._state_preserver.restore_scene(new_scene, preserved_state)

                logger.info("Forced scene reload")
            except Exception as e:
                logger.error(f"Force reload failed: {e}")


def is_hot_reload_available() -> bool:
    """Check if hot reload is available (watchdog installed).

    Returns:
        True if watchdog is available
    """
    return WATCHDOG_AVAILABLE
