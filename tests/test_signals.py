"""Tests for the Signal System.

Tests cover:
- Signal definition and emission
- Signal connections (connect/disconnect)
- One-shot connections
- Deferred connections
- Weak references (target cleanup)
- Signal parameters
- Multiple connections to same signal
"""

import pytest
from unittest.mock import MagicMock
from weakref import ref

from bevel.core.node import Node
from bevel.core.signals import Signal, SignalDefinition, SignalManager, SignalConnection


class TestSignalDefinition:
    """Tests for SignalDefinition dataclass."""

    def test_create_signal_definition(self) -> None:
        """Test creating a signal definition."""
        definition = SignalDefinition(
            name="test_signal",
            parameters=["arg1", "arg2"],
            description="A test signal",
        )
        assert definition.name == "test_signal"
        assert definition.parameters == ["arg1", "arg2"]
        assert definition.description == "A test signal"

    def test_default_parameters(self) -> None:
        """Test default empty parameters."""
        definition = SignalDefinition(name="simple")
        assert definition.parameters == []
        assert definition.description == ""


class TestSignal:
    """Tests for Signal class."""

    def test_create_signal(self) -> None:
        """Test creating a signal."""
        owner = Node(name="Owner")
        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)

        assert signal.name == "test_signal"
        assert signal.owner is owner
        assert signal.connection_count == 0

    def test_connect_to_node_method(self) -> None:
        """Test connecting a signal to a node method."""
        owner = Node(name="Owner")
        target = Node(name="Target")

        # Add a method to target
        target.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)

        result = signal.connect(target, "on_signal")
        assert result is True
        assert signal.connection_count == 1

    def test_connect_to_script_method(self, mock_script: MagicMock) -> None:
        """Test connecting a signal to a script method."""
        owner = Node(name="Owner")
        target = Node(name="Target")
        target.attach_script(mock_script)

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)

        result = signal.connect(target, "on_signal")
        assert result is True
        assert signal.connection_count == 1

    def test_connect_fails_for_missing_method(self) -> None:
        """Test that connecting to missing method fails."""
        owner = Node(name="Owner")
        target = Node(name="Target")

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)

        result = signal.connect(target, "nonexistent_method")
        assert result is False
        assert signal.connection_count == 0

    def test_emit_calls_connected_method(self) -> None:
        """Test that emitting a signal calls connected methods."""
        owner = Node(name="Owner")
        target = Node(name="Target")
        target.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target, "on_signal")

        signal.emit("arg1", "arg2")
        target.on_signal.assert_called_once_with("arg1", "arg2")

    def test_emit_calls_script_method(self, mock_script: MagicMock) -> None:
        """Test that emitting calls script methods."""
        owner = Node(name="Owner")
        target = Node(name="Target")
        target.attach_script(mock_script)

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target, "on_signal")

        signal.emit(42, "test")
        assert mock_script.signal_received is True
        assert mock_script.signal_args == (42, "test")

    def test_disconnect(self) -> None:
        """Test disconnecting a signal."""
        owner = Node(name="Owner")
        target = Node(name="Target")
        target.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target, "on_signal")
        assert signal.connection_count == 1

        result = signal.disconnect(target, "on_signal")
        assert result is True
        assert signal.connection_count == 0

        # Emit should not call method after disconnect
        signal.emit()
        target.on_signal.assert_not_called()

    def test_disconnect_nonexistent(self) -> None:
        """Test disconnecting non-existent connection returns False."""
        owner = Node(name="Owner")
        target = Node(name="Target")

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)

        result = signal.disconnect(target, "method")
        assert result is False

    def test_one_shot_connection(self) -> None:
        """Test one-shot connections auto-disconnect after first emit."""
        owner = Node(name="Owner")
        target = Node(name="Target")
        target.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target, "on_signal", one_shot=True)

        # First emit should call and disconnect
        signal.emit()
        assert target.on_signal.call_count == 1
        assert signal.connection_count == 0

        # Second emit should not call
        signal.emit()
        assert target.on_signal.call_count == 1

    def test_multiple_connections(self) -> None:
        """Test multiple connections to same signal."""
        owner = Node(name="Owner")
        target1 = Node(name="Target1")
        target2 = Node(name="Target2")
        target1.on_signal = MagicMock()
        target2.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target1, "on_signal")
        signal.connect(target2, "on_signal")

        assert signal.connection_count == 2

        signal.emit("test")
        target1.on_signal.assert_called_once_with("test")
        target2.on_signal.assert_called_once_with("test")

    def test_duplicate_connection_ignored(self) -> None:
        """Test that duplicate connections are ignored."""
        owner = Node(name="Owner")
        target = Node(name="Target")
        target.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target, "on_signal")
        signal.connect(target, "on_signal")  # Duplicate

        assert signal.connection_count == 1

    def test_disconnect_all(self) -> None:
        """Test disconnecting all connections."""
        owner = Node(name="Owner")
        target1 = Node(name="Target1")
        target2 = Node(name="Target2")
        target1.on_signal = MagicMock()
        target2.on_signal = MagicMock()

        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)
        signal.connect(target1, "on_signal")
        signal.connect(target2, "on_signal")

        signal.disconnect_all()
        assert signal.connection_count == 0

    def test_weak_reference_cleanup(self) -> None:
        """Test that dead weak references are cleaned up on emit."""
        import gc

        owner = Node(name="Owner")
        definition = SignalDefinition(name="test_signal")
        signal = Signal(definition, owner)

        # Create a target and connect
        target = Node(name="Target")
        target.on_signal = MagicMock()
        signal.connect(target, "on_signal")
        assert signal.connection_count == 1

        # Delete target reference (weak ref should become invalid)
        del target
        gc.collect()

        # Emit triggers cleanup of dead weak references
        signal.emit()

        # Connection count should be 0 after cleanup via emit
        assert signal.connection_count == 0


class TestSignalManager:
    """Tests for SignalManager class."""

    def test_define_signal(self, signal_manager: SignalManager) -> None:
        """Test defining a signal."""
        signal = signal_manager.define_signal(
            "test_signal",
            parameters=["value"],
            description="Test signal",
        )
        assert signal is not None
        assert signal.name == "test_signal"
        assert signal_manager.has_signal("test_signal") is True

    def test_get_signal(self, signal_manager: SignalManager) -> None:
        """Test getting a defined signal."""
        signal_manager.define_signal("my_signal")
        signal = signal_manager.get_signal("my_signal")
        assert signal is not None
        assert signal.name == "my_signal"

    def test_get_undefined_signal(self, signal_manager: SignalManager) -> None:
        """Test getting an undefined signal returns None."""
        signal = signal_manager.get_signal("nonexistent")
        assert signal is None

    def test_emit_signal(self, signal_manager: SignalManager) -> None:
        """Test emitting a signal by name."""
        signal_manager.define_signal("test_signal")

        target = Node(name="Target")
        target.callback = MagicMock()
        signal_manager.connect_signal("test_signal", target, "callback")

        result = signal_manager.emit_signal("test_signal", 42)
        assert result is True
        target.callback.assert_called_once_with(42)

    def test_emit_undefined_signal(self, signal_manager: SignalManager) -> None:
        """Test emitting undefined signal returns False."""
        result = signal_manager.emit_signal("nonexistent")
        assert result is False

    def test_connect_signal(self, signal_manager: SignalManager) -> None:
        """Test connecting to a signal through manager."""
        signal_manager.define_signal("test_signal")

        target = Node(name="Target")
        target.callback = MagicMock()

        result = signal_manager.connect_signal("test_signal", target, "callback")
        assert result is True

    def test_connect_undefined_signal(self, signal_manager: SignalManager) -> None:
        """Test connecting to undefined signal returns False."""
        target = Node(name="Target")
        target.callback = MagicMock()

        result = signal_manager.connect_signal("nonexistent", target, "callback")
        assert result is False

    def test_disconnect_signal(self, signal_manager: SignalManager) -> None:
        """Test disconnecting from a signal."""
        signal_manager.define_signal("test_signal")

        target = Node(name="Target")
        target.callback = MagicMock()
        signal_manager.connect_signal("test_signal", target, "callback")

        result = signal_manager.disconnect_signal("test_signal", target, "callback")
        assert result is True

    def test_get_signal_list(self, signal_manager: SignalManager) -> None:
        """Test getting list of defined signals."""
        signal_manager.define_signal("signal1")
        signal_manager.define_signal("signal2")
        signal_manager.define_signal("signal3")

        signals = signal_manager.get_signal_list()
        assert len(signals) == 3
        assert "signal1" in signals
        assert "signal2" in signals
        assert "signal3" in signals

    def test_disconnect_all_signals(self, signal_manager: SignalManager) -> None:
        """Test disconnecting all signal connections."""
        signal_manager.define_signal("signal1")
        signal_manager.define_signal("signal2")

        target = Node(name="Target")
        target.callback = MagicMock()
        signal_manager.connect_signal("signal1", target, "callback")
        signal_manager.connect_signal("signal2", target, "callback")

        signal_manager.disconnect_all_signals()

        # Emit should not call callback
        signal_manager.emit_signal("signal1")
        signal_manager.emit_signal("signal2")
        target.callback.assert_not_called()


class TestNodeSignalIntegration:
    """Tests for signal integration with Node class."""

    def test_node_define_signal(self, node: Node) -> None:
        """Test defining a signal on a node."""
        signal = node.define_signal("health_changed", ["new_health", "max_health"])
        assert signal is not None
        assert node.has_signal("health_changed") is True

    def test_node_emit_signal(self, node: Node) -> None:
        """Test emitting a signal from a node."""
        node.define_signal("health_changed")

        target = Node(name="Target")
        target.on_health_changed = MagicMock()
        node.connect("health_changed", target, "on_health_changed")

        result = node.emit_signal("health_changed", 50, 100)
        assert result is True
        target.on_health_changed.assert_called_once_with(50, 100)

    def test_node_connect_disconnect(self, node: Node) -> None:
        """Test connecting and disconnecting signals on node."""
        node.define_signal("test_signal")

        target = Node(name="Target")
        target.callback = MagicMock()

        # Connect
        result = node.connect("test_signal", target, "callback")
        assert result is True

        # Disconnect
        result = node.disconnect("test_signal", target, "callback")
        assert result is True

    def test_node_get_signal(self, node: Node) -> None:
        """Test getting a signal from a node."""
        node.define_signal("my_signal")
        signal = node.get_signal("my_signal")
        assert signal is not None
        assert signal.name == "my_signal"

    def test_node_get_signal_list(self, node: Node) -> None:
        """Test getting signal list from a node."""
        node.define_signal("signal_a")
        node.define_signal("signal_b")

        signals = node.get_signal_list()
        assert len(signals) == 2
        assert "signal_a" in signals
        assert "signal_b" in signals

    def test_node_disconnect_all_signals(self, node: Node) -> None:
        """Test disconnecting all signals from a node."""
        node.define_signal("signal1")
        node.define_signal("signal2")

        target = Node(name="Target")
        target.callback = MagicMock()
        node.connect("signal1", target, "callback")
        node.connect("signal2", target, "callback")

        node.disconnect_all_signals()

        # Signals should not call callbacks
        node.emit_signal("signal1")
        node.emit_signal("signal2")
        target.callback.assert_not_called()
