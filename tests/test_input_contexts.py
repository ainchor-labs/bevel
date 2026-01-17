"""Tests for the Input Context System.

Tests cover:
- InputAction creation
- InputContext creation and configuration
- Context stack operations (push, pop, enable, disable)
- Priority-based context handling
- Context blocking
- Action lookup with priorities
- Configuration loading from YAML-like dicts
"""

import pytest

from bevel.input.input_context import (
    InputAction,
    InputContext,
    InputContextStack,
)


class TestInputAction:
    """Tests for InputAction dataclass."""

    def test_create_action(self) -> None:
        """Test creating an input action."""
        action = InputAction(
            name="jump",
            keys=[32],  # Space
            mouse_buttons=[],
        )
        assert action.name == "jump"
        assert action.keys == [32]
        assert action.mouse_buttons == []

    def test_default_values(self) -> None:
        """Test default empty lists."""
        action = InputAction(name="test")
        assert action.keys == []
        assert action.mouse_buttons == []

    def test_multiple_keys(self) -> None:
        """Test action with multiple key bindings."""
        action = InputAction(
            name="move_left",
            keys=[65, 263],  # A, LEFT
        )
        assert len(action.keys) == 2


class TestInputContext:
    """Tests for InputContext dataclass."""

    def test_create_context(self) -> None:
        """Test creating an input context."""
        context = InputContext(
            name="gameplay",
            priority=10,
            actions={"jump": InputAction(name="jump", keys=[32])},
            blocks=["menu"],
            enabled=True,
        )
        assert context.name == "gameplay"
        assert context.priority == 10
        assert "jump" in context.actions
        assert context.blocks == ["menu"]
        assert context.enabled is True

    def test_default_values(self) -> None:
        """Test default context values."""
        context = InputContext(name="test")
        assert context.priority == 0
        assert context.actions == {}
        assert context.blocks == []
        assert context.enabled is True


class TestInputContextStack:
    """Tests for InputContextStack class."""

    def test_register_context(self, input_context_stack: InputContextStack) -> None:
        """Test registering a context."""
        context = InputContext(name="gameplay", priority=10)
        input_context_stack.register_context(context)

        # Context should be registered but not active until pushed/enabled
        assert input_context_stack.is_context_active("gameplay") is False

    def test_push_context(self, input_context_stack: InputContextStack) -> None:
        """Test pushing a context onto the stack."""
        context = InputContext(name="gameplay", priority=10)
        input_context_stack.register_context(context)

        result = input_context_stack.push_context("gameplay")
        assert result is True
        assert input_context_stack.is_context_active("gameplay") is True

    def test_push_unknown_context(self, input_context_stack: InputContextStack) -> None:
        """Test pushing unknown context returns False."""
        result = input_context_stack.push_context("unknown")
        assert result is False

    def test_push_context_once(self, input_context_stack: InputContextStack) -> None:
        """Test pushing same context multiple times only adds once."""
        context = InputContext(name="gameplay")
        input_context_stack.register_context(context)

        input_context_stack.push_context("gameplay")
        input_context_stack.push_context("gameplay")  # Duplicate

        active = input_context_stack.get_active_contexts()
        assert active.count("gameplay") == 1

    def test_pop_context(self, input_context_stack: InputContextStack) -> None:
        """Test popping a context from the stack."""
        context = InputContext(name="gameplay")
        input_context_stack.register_context(context)
        input_context_stack.push_context("gameplay")

        popped = input_context_stack.pop_context()
        assert popped == "gameplay"
        assert input_context_stack.is_context_active("gameplay") is False

    def test_pop_specific_context(self, input_context_stack: InputContextStack) -> None:
        """Test popping a specific context."""
        ctx1 = InputContext(name="gameplay")
        ctx2 = InputContext(name="menu")
        input_context_stack.register_context(ctx1)
        input_context_stack.register_context(ctx2)
        input_context_stack.push_context("gameplay")
        input_context_stack.push_context("menu")

        popped = input_context_stack.pop_context("gameplay")
        assert popped == "gameplay"
        assert input_context_stack.is_context_active("gameplay") is False
        assert input_context_stack.is_context_active("menu") is True

    def test_pop_empty_stack(self, input_context_stack: InputContextStack) -> None:
        """Test popping from empty stack returns None."""
        result = input_context_stack.pop_context()
        assert result is None

    def test_enable_context(self, input_context_stack: InputContextStack) -> None:
        """Test enabling a context without pushing."""
        context = InputContext(name="gameplay")
        input_context_stack.register_context(context)

        input_context_stack.enable_context("gameplay")
        assert input_context_stack.is_context_active("gameplay") is True

    def test_disable_context(self, input_context_stack: InputContextStack) -> None:
        """Test disabling a context."""
        context = InputContext(name="gameplay")
        input_context_stack.register_context(context)
        input_context_stack.enable_context("gameplay")

        input_context_stack.disable_context("gameplay")
        assert input_context_stack.is_context_active("gameplay") is False

    def test_enable_unknown_context(self, input_context_stack: InputContextStack) -> None:
        """Test enabling unknown context does nothing."""
        input_context_stack.enable_context("unknown")
        assert input_context_stack.is_context_active("unknown") is False


class TestContextPriority:
    """Tests for priority-based context handling."""

    def test_get_active_contexts_sorted_by_priority(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test active contexts are sorted by priority (highest first)."""
        ctx_low = InputContext(name="low", priority=1)
        ctx_high = InputContext(name="high", priority=100)
        ctx_mid = InputContext(name="mid", priority=50)

        input_context_stack.register_context(ctx_low)
        input_context_stack.register_context(ctx_high)
        input_context_stack.register_context(ctx_mid)

        input_context_stack.push_context("low")
        input_context_stack.push_context("high")
        input_context_stack.push_context("mid")

        active = input_context_stack.get_active_contexts()
        assert active == ["high", "mid", "low"]

    def test_get_effective_action_uses_priority(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test action lookup returns from highest priority context."""
        low_action = InputAction(name="jump", keys=[32])  # Space
        high_action = InputAction(name="jump", keys=[87])  # W

        ctx_low = InputContext(
            name="low",
            priority=1,
            actions={"jump": low_action},
        )
        ctx_high = InputContext(
            name="high",
            priority=100,
            actions={"jump": high_action},
        )

        input_context_stack.register_context(ctx_low)
        input_context_stack.register_context(ctx_high)
        input_context_stack.push_context("low")
        input_context_stack.push_context("high")

        action = input_context_stack.get_effective_action("jump")
        assert action is not None
        assert action.keys == [87]  # From high priority context

    def test_get_effective_action_not_found(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test action lookup returns None if not found."""
        ctx = InputContext(name="gameplay", priority=10)
        input_context_stack.register_context(ctx)
        input_context_stack.push_context("gameplay")

        action = input_context_stack.get_effective_action("nonexistent")
        assert action is None


class TestContextBlocking:
    """Tests for context blocking functionality."""

    def test_blocking_context(self, input_context_stack: InputContextStack) -> None:
        """Test context blocking lower priority contexts."""
        gameplay = InputContext(name="gameplay", priority=10)
        menu = InputContext(name="menu", priority=100, blocks=["gameplay"])

        input_context_stack.register_context(gameplay)
        input_context_stack.register_context(menu)

        input_context_stack.push_context("gameplay")
        input_context_stack.push_context("menu")

        # Gameplay should be blocked
        assert input_context_stack.is_context_active("gameplay") is False
        assert input_context_stack.is_context_active("menu") is True

    def test_unblocking_on_pop(self, input_context_stack: InputContextStack) -> None:
        """Test context is unblocked when blocker is popped."""
        gameplay = InputContext(name="gameplay", priority=10)
        menu = InputContext(name="menu", priority=100, blocks=["gameplay"])

        input_context_stack.register_context(gameplay)
        input_context_stack.register_context(menu)

        input_context_stack.push_context("gameplay")
        input_context_stack.push_context("menu")

        # Pop menu - gameplay should be unblocked
        input_context_stack.pop_context("menu")
        assert input_context_stack.is_context_active("gameplay") is True

    def test_blocked_context_not_in_active_list(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test blocked contexts are not in active list."""
        gameplay = InputContext(name="gameplay", priority=10)
        menu = InputContext(name="menu", priority=100, blocks=["gameplay"])

        input_context_stack.register_context(gameplay)
        input_context_stack.register_context(menu)

        input_context_stack.push_context("gameplay")
        input_context_stack.push_context("menu")

        active = input_context_stack.get_active_contexts()
        assert "gameplay" not in active
        assert "menu" in active

    def test_multiple_blocking_contexts(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test multiple contexts can block different targets."""
        gameplay = InputContext(name="gameplay", priority=10)
        inventory = InputContext(name="inventory", priority=20)
        menu = InputContext(name="menu", priority=100, blocks=["gameplay", "inventory"])

        input_context_stack.register_context(gameplay)
        input_context_stack.register_context(inventory)
        input_context_stack.register_context(menu)

        input_context_stack.push_context("gameplay")
        input_context_stack.push_context("inventory")
        input_context_stack.push_context("menu")

        assert input_context_stack.is_context_active("gameplay") is False
        assert input_context_stack.is_context_active("inventory") is False
        assert input_context_stack.is_context_active("menu") is True


class TestActionKeyLookup:
    """Tests for action key/button lookup methods."""

    def test_get_action_keys(self, input_context_stack: InputContextStack) -> None:
        """Test getting keys for an action."""
        action = InputAction(name="jump", keys=[32, 87])
        ctx = InputContext(name="gameplay", actions={"jump": action})

        input_context_stack.register_context(ctx)
        input_context_stack.push_context("gameplay")

        keys = input_context_stack.get_action_keys("jump")
        assert keys == [32, 87]

    def test_get_action_keys_not_found(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test getting keys for nonexistent action."""
        keys = input_context_stack.get_action_keys("nonexistent")
        assert keys == []

    def test_get_action_mouse_buttons(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test getting mouse buttons for an action."""
        action = InputAction(name="shoot", mouse_buttons=[0, 1])
        ctx = InputContext(name="gameplay", actions={"shoot": action})

        input_context_stack.register_context(ctx)
        input_context_stack.push_context("gameplay")

        buttons = input_context_stack.get_action_mouse_buttons("shoot")
        assert buttons == [0, 1]


class TestConfigLoading:
    """Tests for loading contexts from configuration."""

    def test_load_from_config(self, input_context_stack: InputContextStack) -> None:
        """Test loading contexts from dict config."""
        config = {
            "gameplay": {
                "priority": 10,
                "enabled": True,
                "actions": {
                    "jump": [32, 87],  # Simple key list
                    "shoot": {  # Dict with keys and mouse
                        "keys": [32],
                        "mouse_buttons": [0],
                    },
                },
            },
            "menu": {
                "priority": 100,
                "blocks": ["gameplay"],
                "actions": {
                    "confirm": [257],  # Enter
                },
            },
        }

        input_context_stack.load_from_config(config)

        # Contexts should be registered and enabled
        # Note: gameplay is blocked by menu (higher priority with blocks)
        # So we need to check menu's action or disable menu first
        input_context_stack.disable_context("menu")

        action = input_context_stack.get_effective_action("jump")
        assert action is not None
        assert action.keys == [32, 87]

    def test_load_auto_enables(self, input_context_stack: InputContextStack) -> None:
        """Test loading auto-enables contexts marked as enabled."""
        config = {
            "gameplay": {
                "priority": 10,
                "enabled": True,
                "actions": {},
            },
            "debug": {
                "priority": 1,
                "enabled": False,
                "actions": {},
            },
        }

        input_context_stack.load_from_config(config)

        assert input_context_stack.is_context_active("gameplay") is True
        assert input_context_stack.is_context_active("debug") is False

    def test_load_with_blocks(self, input_context_stack: InputContextStack) -> None:
        """Test loading contexts with blocking."""
        config = {
            "gameplay": {
                "priority": 10,
                "enabled": True,
                "actions": {},
            },
            "pause": {
                "priority": 100,
                "enabled": True,
                "blocks": ["gameplay"],
                "actions": {},
            },
        }

        input_context_stack.load_from_config(config)

        # Gameplay should be blocked by pause
        assert input_context_stack.is_context_active("gameplay") is False
        assert input_context_stack.is_context_active("pause") is True


class TestContextStackClear:
    """Tests for clearing the context stack."""

    def test_clear(self, input_context_stack: InputContextStack) -> None:
        """Test clearing all contexts."""
        ctx1 = InputContext(name="ctx1")
        ctx2 = InputContext(name="ctx2")

        input_context_stack.register_context(ctx1)
        input_context_stack.register_context(ctx2)
        input_context_stack.push_context("ctx1")
        input_context_stack.enable_context("ctx2")

        input_context_stack.clear()

        assert input_context_stack.get_active_contexts() == []
        assert input_context_stack.is_context_active("ctx1") is False
        assert input_context_stack.is_context_active("ctx2") is False


class TestEdgeCases:
    """Tests for edge cases and unusual scenarios."""

    def test_context_with_no_actions(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test context with no actions still works."""
        ctx = InputContext(name="empty", priority=10)
        input_context_stack.register_context(ctx)
        input_context_stack.push_context("empty")

        assert input_context_stack.is_context_active("empty") is True
        assert input_context_stack.get_effective_action("anything") is None

    def test_action_with_no_bindings(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test action with no key or mouse bindings."""
        action = InputAction(name="unbound")
        ctx = InputContext(name="test", actions={"unbound": action})

        input_context_stack.register_context(ctx)
        input_context_stack.push_context("test")

        keys = input_context_stack.get_action_keys("unbound")
        buttons = input_context_stack.get_action_mouse_buttons("unbound")

        assert keys == []
        assert buttons == []

    def test_same_action_in_multiple_contexts(
        self, input_context_stack: InputContextStack
    ) -> None:
        """Test same action name in multiple contexts."""
        low_jump = InputAction(name="jump", keys=[32])
        high_jump = InputAction(name="jump", keys=[87])

        ctx_low = InputContext(
            name="low",
            priority=1,
            actions={"jump": low_jump},
        )
        ctx_high = InputContext(
            name="high",
            priority=100,
            actions={"jump": high_jump},
        )

        input_context_stack.register_context(ctx_low)
        input_context_stack.register_context(ctx_high)

        # Only low enabled
        input_context_stack.enable_context("low")
        assert input_context_stack.get_action_keys("jump") == [32]

        # Both enabled - high priority wins
        input_context_stack.enable_context("high")
        assert input_context_stack.get_action_keys("jump") == [87]

        # High disabled - low is used again
        input_context_stack.disable_context("high")
        assert input_context_stack.get_action_keys("jump") == [32]
