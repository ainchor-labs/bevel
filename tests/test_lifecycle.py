"""Tests for the Lifecycle System.

Tests cover:
- LifecycleEvent enum
- LifecycleDispatcher methods
- Lifecycle event order (awake -> on_enable -> start -> update -> ...)
- Error handling in lifecycle callbacks
- Tree-wide dispatching
"""

import pytest
from unittest.mock import MagicMock

from bevel.core.node import Node
from bevel.core.lifecycle import LifecycleEvent, LifecycleDispatcher
from tests.conftest import MockScript


class TestLifecycleEvent:
    """Tests for LifecycleEvent enum."""

    def test_events_exist(self) -> None:
        """Test all lifecycle events exist."""
        assert LifecycleEvent.AWAKE
        assert LifecycleEvent.ON_ENABLE
        assert LifecycleEvent.START
        assert LifecycleEvent.UPDATE
        assert LifecycleEvent.FIXED_UPDATE
        assert LifecycleEvent.LATE_UPDATE
        assert LifecycleEvent.ON_DISABLE
        assert LifecycleEvent.DESTROY


class TestDispatchAwake:
    """Tests for awake lifecycle event."""

    def test_dispatch_awake(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test awake is dispatched to script."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_awake(node)

        assert mock_script.awake_called is True

    def test_awake_triggers_on_enable(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test awake triggers on_enable for active nodes."""
        node.attach_script(mock_script)
        node._active = True

        lifecycle_dispatcher.dispatch_awake(node)

        assert mock_script.awake_called is True
        assert mock_script.on_enable_called is True

    def test_awake_no_on_enable_if_inactive(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test awake doesn't trigger on_enable for inactive nodes."""
        node.attach_script(mock_script)
        node._active = False

        lifecycle_dispatcher.dispatch_awake(node)

        assert mock_script.awake_called is True
        assert mock_script.on_enable_called is False

    def test_awake_queues_start(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test awake queues node for start."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_awake(node)

        assert node in lifecycle_dispatcher._pending_start


class TestDispatchStart:
    """Tests for start lifecycle event."""

    def test_dispatch_start(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test start is dispatched to script."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_start(node)

        assert mock_script.start_called is True
        assert node._started is True

    def test_start_only_once(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test start is only called once per node."""
        call_count = 0

        class CountingScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0

            def start(self) -> None:
                nonlocal call_count
                call_count += 1

        script = CountingScript()
        node.attach_script(script)

        lifecycle_dispatcher.dispatch_start(node)
        lifecycle_dispatcher.dispatch_start(node)

        assert call_count == 1


class TestDispatchUpdate:
    """Tests for update lifecycle events."""

    def test_dispatch_update(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test update is dispatched with delta."""
        node.attach_script(mock_script)
        node._started = True

        lifecycle_dispatcher.dispatch_update(node, 0.016)

        assert mock_script.update_called is True
        assert mock_script.update_delta == 0.016

    def test_update_skipped_if_inactive(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test update is skipped for inactive nodes."""
        node.attach_script(mock_script)
        node._active = False

        lifecycle_dispatcher.dispatch_update(node, 0.016)

        assert mock_script.update_called is False

    def test_update_triggers_pending_start(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test update triggers start for pending nodes."""
        node.attach_script(mock_script)
        lifecycle_dispatcher._pending_start.append(node)

        lifecycle_dispatcher.dispatch_update(node, 0.016)

        assert mock_script.start_called is True
        assert node not in lifecycle_dispatcher._pending_start

    def test_dispatch_fixed_update(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test fixed_update is dispatched."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_fixed_update(node, 0.02)

        assert mock_script.fixed_update_called is True
        assert mock_script.fixed_update_delta == 0.02

    def test_dispatch_late_update(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test late_update is dispatched."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_late_update(node, 0.016)

        assert mock_script.late_update_called is True
        assert mock_script.late_update_delta == 0.016


class TestDispatchEnableDisable:
    """Tests for enable/disable lifecycle events."""

    def test_dispatch_enable(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test on_enable is dispatched."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_enable(node)

        assert mock_script.on_enable_called is True

    def test_dispatch_disable(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test on_disable is dispatched."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_disable(node)

        assert mock_script.on_disable_called is True


class TestDispatchDestroy:
    """Tests for destroy lifecycle event."""

    def test_dispatch_destroy(
        self,
        node: Node,
        mock_script: MockScript,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test on_destroy is dispatched."""
        node.attach_script(mock_script)

        lifecycle_dispatcher.dispatch_destroy(node)

        assert mock_script.on_destroy_called is True

    def test_destroy_disconnects_signals(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test destroy disconnects all signals."""
        node.define_signal("test_signal")
        target = Node(name="Target")
        target.callback = MagicMock()
        node.connect("test_signal", target, "callback")

        lifecycle_dispatcher.dispatch_destroy(node)

        # Signal should be disconnected
        node.emit_signal("test_signal")
        target.callback.assert_not_called()


class TestTreeDispatching:
    """Tests for tree-wide lifecycle dispatching."""

    def test_dispatch_tree_awake(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test awake is dispatched to entire tree."""
        root = node_with_children
        scripts = []

        # Attach scripts to all nodes
        for node in [root] + list(root.get_descendants()):
            script = MockScript()
            node.attach_script(script)
            scripts.append(script)

        lifecycle_dispatcher.dispatch_tree_awake(root)

        # All scripts should have awake called
        for script in scripts:
            assert script.awake_called is True

    def test_dispatch_tree_update(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test update is dispatched to entire tree."""
        root = node_with_children
        scripts = []

        for node in [root] + list(root.get_descendants()):
            script = MockScript()
            node.attach_script(script)
            node._started = True
            scripts.append(script)

        lifecycle_dispatcher.dispatch_tree_update(root, 0.016)

        for script in scripts:
            assert script.update_called is True

    def test_dispatch_tree_fixed_update(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test fixed_update is dispatched to entire tree."""
        root = node_with_children
        scripts = []

        for node in [root] + list(root.get_descendants()):
            script = MockScript()
            node.attach_script(script)
            scripts.append(script)

        lifecycle_dispatcher.dispatch_tree_fixed_update(root, 0.02)

        for script in scripts:
            assert script.fixed_update_called is True

    def test_dispatch_tree_late_update(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test late_update is dispatched to entire tree."""
        root = node_with_children
        scripts = []

        for node in [root] + list(root.get_descendants()):
            script = MockScript()
            node.attach_script(script)
            scripts.append(script)

        lifecycle_dispatcher.dispatch_tree_late_update(root, 0.016)

        for script in scripts:
            assert script.late_update_called is True

    def test_dispatch_tree_destroy(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test destroy is dispatched to entire tree (children first)."""
        root = node_with_children
        destroy_order: list[str] = []

        class OrderTrackingScript:
            def __init__(self, name: str) -> None:
                self.name = name
                self.node = None
                self._priority = 0

            def on_destroy(self) -> None:
                destroy_order.append(self.name)

        # Attach scripts
        root.attach_script(OrderTrackingScript("Root"))
        child1 = root.get_child("Child1")
        if child1:
            child1.attach_script(OrderTrackingScript("Child1"))
            grandchild = child1.get_child("Grandchild")
            if grandchild:
                grandchild.attach_script(OrderTrackingScript("Grandchild"))

        lifecycle_dispatcher.dispatch_tree_destroy(root)

        # Children should be destroyed before parents
        assert destroy_order.index("Grandchild") < destroy_order.index("Child1")
        assert destroy_order.index("Child1") < destroy_order.index("Root")

    def test_dispatch_tree_enable(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test on_enable is dispatched to tree."""
        root = node_with_children
        scripts = []

        for node in [root] + list(root.get_descendants()):
            script = MockScript()
            node.attach_script(script)
            scripts.append(script)

        lifecycle_dispatcher.dispatch_tree_enable(root)

        for script in scripts:
            assert script.on_enable_called is True

    def test_dispatch_tree_disable(
        self,
        node_with_children: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test on_disable is dispatched to tree of inactive nodes."""
        root = node_with_children
        root._active = False

        scripts = []
        for node in [root] + list(root.get_descendants()):
            script = MockScript()
            node.attach_script(script)
            node._active = False
            scripts.append(script)

        lifecycle_dispatcher.dispatch_tree_disable(root)

        for script in scripts:
            assert script.on_disable_called is True


class TestErrorHandling:
    """Tests for error handling in lifecycle callbacks."""

    def test_awake_error_caught(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test errors in awake don't crash."""

        class ErrorScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0

            def awake(self) -> None:
                raise ValueError("Test error")

        script = ErrorScript()
        node.attach_script(script)

        # Should not raise
        lifecycle_dispatcher.dispatch_awake(node)

    def test_update_error_caught(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test errors in update don't crash."""

        class ErrorScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0

            def update(self, delta: float) -> None:
                raise ValueError("Test error")

        script = ErrorScript()
        node.attach_script(script)
        node._started = True

        # Should not raise
        lifecycle_dispatcher.dispatch_update(node, 0.016)

    def test_error_in_one_script_others_still_run(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test error in one script doesn't stop others."""

        class ErrorScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0

            def update(self, delta: float) -> None:
                raise ValueError("Error")

        error_script = ErrorScript()
        good_script = MockScript()

        node.attach_script(error_script, priority=0)
        node.attach_script(good_script, priority=1)
        node._started = True

        lifecycle_dispatcher.dispatch_update(node, 0.016)

        # Good script should still run
        assert good_script.update_called is True


class TestLifecycleOrder:
    """Tests for correct lifecycle event ordering."""

    def test_lifecycle_order(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test lifecycle events are called in correct order."""
        order: list[str] = []

        class OrderTrackingScript:
            def __init__(self) -> None:
                self.node = None
                self._priority = 0

            def awake(self) -> None:
                order.append("awake")

            def on_enable(self) -> None:
                order.append("on_enable")

            def start(self) -> None:
                order.append("start")

            def update(self, delta: float) -> None:
                order.append("update")

            def fixed_update(self, delta: float) -> None:
                order.append("fixed_update")

            def late_update(self, delta: float) -> None:
                order.append("late_update")

        script = OrderTrackingScript()
        node.attach_script(script)

        # Simulate full lifecycle
        lifecycle_dispatcher.dispatch_awake(node)  # awake, on_enable
        lifecycle_dispatcher.dispatch_update(node, 0.016)  # start, update
        lifecycle_dispatcher.dispatch_fixed_update(node, 0.02)  # fixed_update
        lifecycle_dispatcher.dispatch_late_update(node, 0.016)  # late_update

        expected = ["awake", "on_enable", "start", "update", "fixed_update", "late_update"]
        assert order == expected


class TestMultipleScriptsLifecycle:
    """Tests for lifecycle with multiple scripts."""

    def test_all_scripts_receive_events(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test all attached scripts receive lifecycle events."""
        scripts = [MockScript() for _ in range(5)]

        for i, script in enumerate(scripts):
            node.attach_script(script, priority=i)

        lifecycle_dispatcher.dispatch_awake(node)

        for script in scripts:
            assert script.awake_called is True
            assert script.on_enable_called is True

    def test_scripts_called_in_priority_order(
        self,
        node: Node,
        lifecycle_dispatcher: LifecycleDispatcher,
    ) -> None:
        """Test scripts are called in priority order."""
        call_order: list[int] = []

        class PriorityScript:
            def __init__(self, priority: int) -> None:
                self.node = None
                self._priority = priority
                self.id = priority

            def update(self, delta: float) -> None:
                call_order.append(self.id)

        scripts = [PriorityScript(i) for i in [5, 1, 3, 2, 4]]
        for script in scripts:
            node.attach_script(script, priority=script._priority)

        node._started = True
        lifecycle_dispatcher.dispatch_update(node, 0.016)

        # Should be called in priority order
        assert call_order == [1, 2, 3, 4, 5]
