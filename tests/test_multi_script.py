"""Tests for Multi-Script Support.

Tests cover:
- Attaching multiple scripts to a node
- Script priority ordering
- Script detachment
- Script lookup by name and type
- Script-to-script communication via node
- Script communication through signals
"""

import pytest
from typing import Any

from bevel.core.node import Node


class ScriptA:
    """Test script A."""

    def __init__(self) -> None:
        self.node: Node | None = None
        self._priority: int = 0
        self.value: int = 0
        self.update_order: list[str] = []

    def update(self, delta: float) -> None:
        self.value += 1
        if self.node is not None:
            shared = self.node.get_property("update_order", [])
            shared.append("A")
            self.node.set_property("update_order", shared)


class ScriptB:
    """Test script B."""

    def __init__(self) -> None:
        self.node: Node | None = None
        self._priority: int = 0
        self.value: int = 0

    def update(self, delta: float) -> None:
        self.value += 10
        if self.node is not None:
            shared = self.node.get_property("update_order", [])
            shared.append("B")
            self.node.set_property("update_order", shared)


class HealthScript:
    """Script that manages health."""

    def __init__(self) -> None:
        self.node: Node | None = None
        self._priority: int = 0
        self.health: int = 100
        self.max_health: int = 100

    def take_damage(self, amount: int) -> None:
        self.health = max(0, self.health - amount)
        if self.node is not None:
            self.node.emit_signal("health_changed", self.health, self.max_health)


class UIScript:
    """Script that handles UI updates."""

    def __init__(self) -> None:
        self.node: Node | None = None
        self._priority: int = 0
        self.displayed_health: int = 100
        self.signal_received: bool = False

    def on_health_changed(self, health: int, max_health: int) -> None:
        self.displayed_health = health
        self.signal_received = True


class TestAttachScript:
    """Tests for attaching scripts to nodes."""

    def test_attach_single_script(self, node: Node) -> None:
        """Test attaching a single script."""
        script = ScriptA()
        node.attach_script(script)

        assert len(node.scripts) == 1
        assert script.node is node

    def test_attach_multiple_scripts(self, node: Node) -> None:
        """Test attaching multiple scripts."""
        script_a = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a)
        node.attach_script(script_b)

        assert len(node.scripts) == 2
        assert script_a in node.scripts
        assert script_b in node.scripts

    def test_script_has_node_reference(self, node: Node) -> None:
        """Test that attached script has reference to node."""
        script = ScriptA()
        node.attach_script(script)

        assert script.node is node


class TestScriptPriority:
    """Tests for script priority ordering."""

    def test_scripts_sorted_by_priority(self, node: Node) -> None:
        """Test scripts are sorted by priority (lower first)."""
        script_high = ScriptA()
        script_low = ScriptB()

        node.attach_script(script_high, priority=10)
        node.attach_script(script_low, priority=1)

        # Lower priority should be first
        assert node.scripts[0] is script_low
        assert node.scripts[1] is script_high

    def test_equal_priority_preserves_order(self, node: Node) -> None:
        """Test equal priority preserves attachment order."""
        script_a = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a, priority=5)
        node.attach_script(script_b, priority=5)

        # First attached should still be first
        assert node.scripts[0] is script_a
        assert node.scripts[1] is script_b

    def test_negative_priority(self, node: Node) -> None:
        """Test scripts with negative priority run first."""
        script_normal = ScriptA()
        script_early = ScriptB()

        node.attach_script(script_normal, priority=0)
        node.attach_script(script_early, priority=-10)

        assert node.scripts[0] is script_early


class TestDetachScript:
    """Tests for detaching scripts from nodes."""

    def test_detach_script(self, node: Node) -> None:
        """Test detaching a script."""
        script = ScriptA()
        node.attach_script(script)

        result = node.detach_script(script)

        assert result is True
        assert len(node.scripts) == 0
        assert script.node is None

    def test_detach_nonexistent_script(self, node: Node) -> None:
        """Test detaching script that isn't attached."""
        script = ScriptA()

        result = node.detach_script(script)

        assert result is False

    def test_detach_one_of_multiple(self, node: Node) -> None:
        """Test detaching one script while others remain."""
        script_a = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a)
        node.attach_script(script_b)

        node.detach_script(script_a)

        assert len(node.scripts) == 1
        assert script_b in node.scripts
        assert script_a not in node.scripts


class TestScriptLookup:
    """Tests for looking up scripts by name or type."""

    def test_get_script_by_name(self, node: Node) -> None:
        """Test getting a script by class name."""
        script_a = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a)
        node.attach_script(script_b)

        result = node.get_script("ScriptA")
        assert result is script_a

        result = node.get_script("ScriptB")
        assert result is script_b

    def test_get_script_not_found(self, node: Node) -> None:
        """Test getting script that doesn't exist."""
        result = node.get_script("NonExistent")
        assert result is None

    def test_get_script_by_type(self, node: Node) -> None:
        """Test getting a script by type."""
        script_a = ScriptA()
        node.attach_script(script_a)

        result = node.get_script_by_type(ScriptA)
        assert result is script_a

    def test_get_script_by_type_not_found(self, node: Node) -> None:
        """Test getting script by type that doesn't exist."""
        result = node.get_script_by_type(ScriptA)
        assert result is None

    def test_get_scripts_by_type(self, node: Node) -> None:
        """Test getting all scripts of a type."""
        script_a1 = ScriptA()
        script_a2 = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a1)
        node.attach_script(script_b)
        node.attach_script(script_a2)

        results = node.get_scripts_by_type(ScriptA)
        assert len(results) == 2
        assert script_a1 in results
        assert script_a2 in results
        assert script_b not in results

    def test_has_script(self, node: Node) -> None:
        """Test checking if script is attached."""
        script = ScriptA()
        node.attach_script(script)

        assert node.has_script("ScriptA") is True
        assert node.has_script("ScriptB") is False


class TestScriptCommunication:
    """Tests for script-to-script communication."""

    def test_scripts_share_node_properties(self, node: Node) -> None:
        """Test scripts can communicate via node properties."""
        script_a = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a)
        node.attach_script(script_b)

        # Script A sets a property
        script_a.node.set_property("shared_value", 42)  # type: ignore

        # Script B reads it
        value = script_b.node.get_property("shared_value")  # type: ignore
        assert value == 42

    def test_get_script_from_other_script(self, node: Node) -> None:
        """Test script can get reference to another script."""
        health_script = HealthScript()
        ui_script = UIScript()

        node.attach_script(health_script)
        node.attach_script(ui_script)

        # UI script gets health script reference
        health = ui_script.node.get_script("HealthScript")  # type: ignore
        assert health is health_script
        assert health.health == 100


class TestScriptSignalCommunication:
    """Tests for script communication via signals."""

    def test_script_emits_signal(self, node: Node) -> None:
        """Test script can emit signals."""
        health_script = HealthScript()
        node.attach_script(health_script)

        # Define signal
        node.define_signal("health_changed", ["health", "max_health"])

        # Take damage triggers signal
        health_script.take_damage(25)
        assert health_script.health == 75

    def test_script_to_script_signal(self, node: Node) -> None:
        """Test one script's signal reaches another script."""
        player_node = Node(name="Player")
        ui_node = Node(name="UI")

        health_script = HealthScript()
        ui_script = UIScript()

        player_node.attach_script(health_script)
        ui_node.attach_script(ui_script)

        # Define signal on player
        player_node.define_signal("health_changed", ["health", "max_health"])

        # Connect UI to player's signal
        player_node.connect("health_changed", ui_node, "on_health_changed")

        # Damage triggers signal
        health_script.take_damage(30)

        assert ui_script.signal_received is True
        assert ui_script.displayed_health == 70

    def test_scripts_on_same_node_signal(self, node: Node) -> None:
        """Test scripts on same node can communicate via signals."""
        health_script = HealthScript()
        ui_script = UIScript()

        node.attach_script(health_script)
        node.attach_script(ui_script)

        node.define_signal("health_changed")
        node.connect("health_changed", node, "on_health_changed")

        health_script.take_damage(20)

        assert ui_script.signal_received is True
        assert ui_script.displayed_health == 80


class TestMultipleNodesWithScripts:
    """Tests for scripts across multiple nodes."""

    def test_scripts_in_hierarchy(self, node_with_children: Node) -> None:
        """Test scripts on nodes in hierarchy."""
        root = node_with_children
        child = root.get_child("Child1")

        root_script = ScriptA()
        child_script = ScriptB()

        root.attach_script(root_script)
        if child is not None:
            child.attach_script(child_script)

        assert len(root.scripts) == 1
        assert child is not None
        assert len(child.scripts) == 1

    def test_find_scripts_in_descendants(self, node_with_children: Node) -> None:
        """Test finding scripts in descendant nodes."""
        root = node_with_children
        child = root.get_child("Child1")

        if child is not None:
            child.attach_script(ScriptA())

        # Find all nodes that have ScriptA
        nodes_with_script = []
        for node in [root] + list(root.get_descendants()):
            if node.has_script("ScriptA"):
                nodes_with_script.append(node)

        assert len(nodes_with_script) == 1
        assert nodes_with_script[0].name == "Child1"


class TestEdgeCases:
    """Tests for edge cases."""

    def test_reattach_same_script(self, node: Node) -> None:
        """Test reattaching same script instance."""
        script = ScriptA()

        node.attach_script(script)
        node.attach_script(script)  # Attach again

        # Should have duplicate entries (matching Godot behavior)
        assert len(node.scripts) == 2

    def test_script_priority_reorder(self, node: Node) -> None:
        """Test scripts reorder when new one with different priority added."""
        script_a = ScriptA()
        script_b = ScriptB()

        node.attach_script(script_a, priority=10)
        node.attach_script(script_b, priority=5)

        # B should be first due to lower priority
        assert node.scripts[0] is script_b
        assert node.scripts[1] is script_a

    def test_detach_updates_node_reference(self, node: Node) -> None:
        """Test detaching clears script's node reference."""
        script = ScriptA()
        node.attach_script(script)

        assert script.node is node

        node.detach_script(script)

        assert script.node is None

    def test_many_scripts(self, node: Node) -> None:
        """Test node with many scripts."""
        scripts = [ScriptA() for _ in range(100)]

        for i, script in enumerate(scripts):
            node.attach_script(script, priority=i)

        assert len(node.scripts) == 100
        # Should be in priority order
        assert node.scripts[0]._priority == 0
        assert node.scripts[99]._priority == 99
