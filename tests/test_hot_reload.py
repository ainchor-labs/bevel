"""Tests for the Hot Reload System.

Tests cover:
- ReloadEvent creation
- StatePreserver (preserve/restore node state)
- HotReloader configuration
- File change detection
- Module path conversion
"""

import pytest
from pathlib import Path
from unittest.mock import MagicMock, patch

from bevel.core.node import Node
from bevel.development.hot_reload import (
    ReloadEvent,
    PreservedState,
    StatePreserver,
    HotReloader,
    is_hot_reload_available,
)


class TestReloadEvent:
    """Tests for ReloadEvent dataclass."""

    def test_create_event(self) -> None:
        """Test creating a reload event."""
        event = ReloadEvent(
            path=Path("/project/scripts/player.py"),
            event_type="modified",
        )
        assert event.path == Path("/project/scripts/player.py")
        assert event.event_type == "modified"
        assert event.timestamp > 0


class TestPreservedState:
    """Tests for PreservedState dataclass."""

    def test_default_values(self) -> None:
        """Test default empty state."""
        state = PreservedState()
        assert state.node_states == {}
        assert state.global_state == {}


class TestStatePreserver:
    """Tests for StatePreserver class."""

    def test_preserve_node_transform(self, mock_scene: MagicMock) -> None:
        """Test preserving node transform."""
        preserver = StatePreserver()

        node = Node(name="Test")
        node.position = (100, 200)
        node.rotation = 1.5
        node.scale = (2, 2)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        assert "Test" in preserved.node_states
        state = preserved.node_states["Test"]
        assert state["position"] == (100, 200)
        assert state["rotation"] == 1.5
        assert state["scale"] == (2, 2)

    def test_preserve_node_properties(self, mock_scene: MagicMock) -> None:
        """Test preserving custom node properties."""
        preserver = StatePreserver()

        node = Node(name="Test")
        node.set_property("health", 50)
        node.set_property("score", 1000)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        state = preserved.node_states["Test"]
        assert state["properties"]["health"] == 50
        assert state["properties"]["score"] == 1000

    def test_preserve_node_visibility(self, mock_scene: MagicMock) -> None:
        """Test preserving node active and visible state."""
        preserver = StatePreserver()

        node = Node(name="Test")
        node._active = False
        node._visible = False
        node.z_index = 5
        node.layer = "ui"
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        state = preserved.node_states["Test"]
        assert state["active"] is False
        assert state["visible"] is False
        assert state["z_index"] == 5
        assert state["layer"] == "ui"

    def test_preserve_script_state(self, mock_scene: MagicMock) -> None:
        """Test preserving script state."""
        preserver = StatePreserver()

        class TestScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0
                self.health = 75
                self.name = "Player"
                self._private = "hidden"  # Should be ignored

        node = Node(name="Test")
        script = TestScript()
        node.attach_script(script)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        state = preserved.node_states["Test"]
        assert "TestScript" in state["scripts"]
        assert state["scripts"]["TestScript"]["health"] == 75
        assert state["scripts"]["TestScript"]["name"] == "Player"

    def test_preserve_custom_callback(self, mock_scene: MagicMock) -> None:
        """Test custom preserve callback."""
        preserver = StatePreserver()

        def custom_preserve(node: Node) -> dict:
            return {"custom_value": 42}

        preserver.register_preserve_callback(custom_preserve)

        node = Node(name="Test")
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        state = preserved.node_states["Test"]
        assert state["custom_value"] == 42

    def test_restore_node_transform(self, mock_scene: MagicMock) -> None:
        """Test restoring node transform."""
        preserver = StatePreserver()

        # Create and preserve
        node = Node(name="Test")
        node.position = (100, 200)
        node.rotation = 1.5
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        # Modify
        node.position = (0, 0)
        node.rotation = 0

        # Restore
        preserver.restore_scene(mock_scene, preserved)

        assert node.position.x == 100
        assert node.position.y == 200
        assert node.rotation == 1.5

    def test_restore_node_properties(self, mock_scene: MagicMock) -> None:
        """Test restoring custom properties."""
        preserver = StatePreserver()

        node = Node(name="Test")
        node.set_property("health", 100)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)
        node.set_property("health", 0)

        preserver.restore_scene(mock_scene, preserved)

        assert node.get_property("health") == 100

    def test_restore_script_state(self, mock_scene: MagicMock) -> None:
        """Test restoring script state."""
        preserver = StatePreserver()

        class TestScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0
                self.health = 100

        node = Node(name="Test")
        script = TestScript()
        node.attach_script(script)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)
        script.health = 0

        preserver.restore_scene(mock_scene, preserved)

        assert script.health == 100

    def test_restore_custom_callback(self, mock_scene: MagicMock) -> None:
        """Test custom restore callback."""
        preserver = StatePreserver()
        restored_value = None

        def custom_restore(node: Node, state: dict) -> None:
            nonlocal restored_value
            restored_value = state.get("custom_value")

        preserver.register_preserve_callback(lambda n: {"custom_value": 42})
        preserver.register_restore_callback(custom_restore)

        node = Node(name="Test")
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)
        preserver.restore_scene(mock_scene, preserved)

        assert restored_value == 42

    def test_preserve_hierarchy(self, mock_scene: MagicMock) -> None:
        """Test preserving entire node hierarchy."""
        preserver = StatePreserver()

        root = Node(name="Root")
        child = Node(name="Child")
        grandchild = Node(name="Grandchild")

        root.add_child(child)
        child.add_child(grandchild)

        root.position = (10, 20)
        child.position = (30, 40)
        grandchild.position = (50, 60)

        root._scene = mock_scene
        mock_scene.root = root

        preserved = preserver.preserve_scene(mock_scene)

        assert "Root" in preserved.node_states
        assert "Root/Child" in preserved.node_states
        assert "Root/Child/Grandchild" in preserved.node_states

    def test_script_preserve_method(self, mock_scene: MagicMock) -> None:
        """Test script with custom preserve_state method."""
        preserver = StatePreserver()

        class CustomScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0
                self.internal_state = {"complex": "data"}

            def preserve_state(self) -> dict:
                return {"custom": self.internal_state}

        node = Node(name="Test")
        script = CustomScript()
        node.attach_script(script)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)

        state = preserved.node_states["Test"]
        assert state["scripts"]["CustomScript"]["custom"] == {"complex": "data"}

    def test_script_restore_method(self, mock_scene: MagicMock) -> None:
        """Test script with custom restore_state method."""
        preserver = StatePreserver()

        class CustomScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0
                self.internal_state = {}

            def preserve_state(self) -> dict:
                return {"custom": {"key": "value"}}

            def restore_state(self, state: dict) -> None:
                self.internal_state = state.get("custom", {})

        node = Node(name="Test")
        script = CustomScript()
        node.attach_script(script)
        node._scene = mock_scene
        mock_scene.root = node

        preserved = preserver.preserve_scene(mock_scene)
        script.internal_state = {}

        preserver.restore_scene(mock_scene, preserved)

        assert script.internal_state == {"key": "value"}


class TestHotReloader:
    """Tests for HotReloader class."""

    def test_path_to_module(self) -> None:
        """Test converting file path to module name."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager, project_root=Path("/project"))

        module = reloader._path_to_module(Path("/project/scripts/player.py"))
        assert module == "scripts.player"

    def test_path_to_module_nested(self) -> None:
        """Test converting nested file path to module name."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager, project_root=Path("/project"))

        module = reloader._path_to_module(Path("/project/game/entities/player.py"))
        assert module == "game.entities.player"

    def test_path_to_module_outside_project(self) -> None:
        """Test path outside project returns stem."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager, project_root=Path("/project"))

        module = reloader._path_to_module(Path("/other/path/script.py"))
        assert module == "script"

    def test_enabled_property(self) -> None:
        """Test enabled property."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)

        assert reloader.enabled is False

    def test_state_preserver_property(self) -> None:
        """Test state_preserver property."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)

        assert isinstance(reloader.state_preserver, StatePreserver)

    def test_queue_reload(self) -> None:
        """Test queuing a reload event."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)

        event = ReloadEvent(path=Path("test.py"), event_type="modified")
        reloader._queue_reload(event)

        assert len(reloader._pending_reloads) == 1
        assert reloader._pending_reloads[0].path == Path("test.py")

    def test_queue_duplicate_updates_timestamp(self) -> None:
        """Test queuing duplicate path updates timestamp."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)

        event1 = ReloadEvent(path=Path("test.py"), event_type="modified")
        event1.timestamp = 100.0
        reloader._queue_reload(event1)

        event2 = ReloadEvent(path=Path("test.py"), event_type="modified")
        event2.timestamp = 200.0
        reloader._queue_reload(event2)

        assert len(reloader._pending_reloads) == 1
        assert reloader._pending_reloads[0].timestamp == 200.0

    def test_on_reload_callback(self) -> None:
        """Test registering reload callback."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)

        callback = MagicMock()
        reloader.on_reload(callback)

        assert callback in reloader._reload_callbacks

    def test_is_hot_reload_available(self) -> None:
        """Test checking if hot reload is available."""
        # This depends on whether watchdog is installed
        result = is_hot_reload_available()
        assert isinstance(result, bool)


class TestHotReloaderWithoutWatchdog:
    """Tests for HotReloader when watchdog is not available."""

    @patch("bevel.development.hot_reload.WATCHDOG_AVAILABLE", False)
    def test_start_without_watchdog(self) -> None:
        """Test start fails gracefully without watchdog."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)

        result = reloader.start()

        assert result is False
        assert reloader.enabled is False


class TestStatePreserverEdgeCases:
    """Tests for edge cases in state preservation."""

    def test_preserve_empty_scene(self, mock_scene: MagicMock) -> None:
        """Test preserving scene with no root."""
        preserver = StatePreserver()
        mock_scene.root = None

        preserved = preserver.preserve_scene(mock_scene)

        assert preserved.node_states == {}

    def test_restore_empty_scene(self, mock_scene: MagicMock) -> None:
        """Test restoring to scene with no root."""
        preserver = StatePreserver()
        mock_scene.root = None

        preserved = PreservedState()
        preserved.node_states["Test"] = {"position": (0, 0)}

        # Should not raise
        preserver.restore_scene(mock_scene, preserved)

    def test_restore_missing_node(self, mock_scene: MagicMock) -> None:
        """Test restoring state for node that doesn't exist."""
        preserver = StatePreserver()

        node = Node(name="Existing")
        node._scene = mock_scene
        mock_scene.root = node

        preserved = PreservedState()
        preserved.node_states["NonExistent"] = {"position": (0, 0)}

        # Should not raise
        preserver.restore_scene(mock_scene, preserved)

    def test_preserve_callback_error_caught(self, mock_scene: MagicMock) -> None:
        """Test errors in preserve callbacks are caught."""
        preserver = StatePreserver()

        def bad_callback(node: Node) -> dict:
            raise ValueError("Error")

        preserver.register_preserve_callback(bad_callback)

        node = Node(name="Test")
        node._scene = mock_scene
        mock_scene.root = node

        # Should not raise
        preserved = preserver.preserve_scene(mock_scene)
        assert "Test" in preserved.node_states

    def test_restore_callback_error_caught(self, mock_scene: MagicMock) -> None:
        """Test errors in restore callbacks are caught."""
        preserver = StatePreserver()

        def bad_callback(node: Node, state: dict) -> None:
            raise ValueError("Error")

        preserver.register_restore_callback(bad_callback)

        node = Node(name="Test")
        node._scene = mock_scene
        mock_scene.root = node

        preserved = PreservedState()
        preserved.node_states["Test"] = {"position": (0, 0)}

        # Should not raise
        preserver.restore_scene(mock_scene, preserved)


class TestReloadProcessing:
    """Tests for reload processing logic."""

    def test_update_without_pending(self) -> None:
        """Test update does nothing without pending reloads."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)
        reloader._enabled = True

        reloader.update()

        # No errors, nothing processed

    def test_update_respects_debounce(self) -> None:
        """Test update respects debounce time."""
        mock_scene_manager = MagicMock()
        reloader = HotReloader(mock_scene_manager)
        reloader._enabled = True
        reloader._debounce_time = 1.0  # Long debounce

        import time

        event = ReloadEvent(path=Path("test.py"), event_type="modified")
        event.timestamp = time.time()  # Just now
        reloader._pending_reloads.append(event)

        reloader.update()

        # Should still be pending (not enough time passed)
        assert len(reloader._pending_reloads) == 1
