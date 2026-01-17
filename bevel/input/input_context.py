"""Input context system for Bevel engine.

Contexts allow grouping input actions and enabling/disabling them
based on game state (e.g., gameplay vs menu).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from bevel.utils.logger import get_logger

logger = get_logger("input_context")


@dataclass
class InputAction:
    """An input action mapping."""

    name: str
    keys: list[int] = field(default_factory=list)
    mouse_buttons: list[int] = field(default_factory=list)


@dataclass
class InputContext:
    """A context that groups related input actions."""

    name: str
    priority: int = 0  # Higher priority contexts are checked first
    actions: dict[str, InputAction] = field(default_factory=dict)
    blocks: list[str] = field(default_factory=list)  # Context names to block
    enabled: bool = True


class InputContextStack:
    """Manages a stack of input contexts.

    Contexts with higher priority are checked first. A context can
    block lower-priority contexts from receiving input.
    """

    def __init__(self) -> None:
        """Initialize the context stack."""
        self._contexts: dict[str, InputContext] = {}
        self._active_stack: list[str] = []  # Stack of pushed context names
        self._enabled_contexts: set[str] = set()  # Explicitly enabled contexts
        self._blocked_contexts: set[str] = set()  # Currently blocked contexts

    def register_context(self, context: InputContext) -> None:
        """Register a context definition.

        Args:
            context: The context to register
        """
        self._contexts[context.name] = context
        logger.debug(
            f"Registered input context '{context.name}' (priority={context.priority})"
        )

    def push_context(self, name: str) -> bool:
        """Push a context onto the active stack.

        Args:
            name: The context name to push

        Returns:
            True if pushed successfully
        """
        if name not in self._contexts:
            logger.warning(f"Cannot push unknown context: {name}")
            return False

        if name not in self._active_stack:
            self._active_stack.append(name)
            self._update_blocked_contexts()
            logger.debug(f"Pushed context: {name}")
        return True

    def pop_context(self, name: str | None = None) -> str | None:
        """Pop a context from the stack.

        Args:
            name: Specific context to pop, or None to pop the top

        Returns:
            The popped context name, or None if stack is empty
        """
        if not self._active_stack:
            return None

        if name is not None:
            if name in self._active_stack:
                self._active_stack.remove(name)
                self._update_blocked_contexts()
                logger.debug(f"Popped context: {name}")
                return name
            return None
        else:
            popped = self._active_stack.pop()
            self._update_blocked_contexts()
            logger.debug(f"Popped context: {popped}")
            return popped

    def enable_context(self, name: str) -> None:
        """Enable a context (without pushing to stack).

        Args:
            name: The context name to enable
        """
        if name not in self._contexts:
            logger.warning(f"Cannot enable unknown context: {name}")
            return

        self._enabled_contexts.add(name)
        self._update_blocked_contexts()
        logger.debug(f"Enabled context: {name}")

    def disable_context(self, name: str) -> None:
        """Disable a context.

        Args:
            name: The context name to disable
        """
        self._enabled_contexts.discard(name)
        self._update_blocked_contexts()
        logger.debug(f"Disabled context: {name}")

    def is_context_active(self, name: str) -> bool:
        """Check if a context is currently active.

        A context is active if it's in the stack, enabled, and not blocked.

        Args:
            name: The context name

        Returns:
            True if active
        """
        if name in self._blocked_contexts:
            return False
        return name in self._active_stack or name in self._enabled_contexts

    def get_active_contexts(self) -> list[str]:
        """Get list of active context names, sorted by priority.

        Returns:
            List of active context names (highest priority first)
        """
        active = set(self._active_stack) | self._enabled_contexts
        active -= self._blocked_contexts

        # Sort by priority (highest first)
        return sorted(
            active,
            key=lambda n: -self._contexts.get(n, InputContext(name=n)).priority,
        )

    def get_effective_action(self, action_name: str) -> InputAction | None:
        """Get an action considering context priorities and blocking.

        Args:
            action_name: The action name to look up

        Returns:
            The InputAction from the highest priority active context, or None
        """
        for context_name in self.get_active_contexts():
            context = self._contexts.get(context_name)
            if context is not None and action_name in context.actions:
                return context.actions[action_name]
        return None

    def get_action_keys(self, action_name: str) -> list[int]:
        """Get the key bindings for an action.

        Args:
            action_name: The action name

        Returns:
            List of key codes
        """
        action = self.get_effective_action(action_name)
        return action.keys if action is not None else []

    def get_action_mouse_buttons(self, action_name: str) -> list[int]:
        """Get the mouse button bindings for an action.

        Args:
            action_name: The action name

        Returns:
            List of mouse button codes
        """
        action = self.get_effective_action(action_name)
        return action.mouse_buttons if action is not None else []

    def _update_blocked_contexts(self) -> None:
        """Update the set of blocked contexts based on active contexts."""
        self._blocked_contexts.clear()

        # Collect all blocked contexts from active ones
        for context_name in self._active_stack:
            context = self._contexts.get(context_name)
            if context is not None:
                for blocked in context.blocks:
                    self._blocked_contexts.add(blocked)

        for context_name in self._enabled_contexts:
            context = self._contexts.get(context_name)
            if context is not None:
                for blocked in context.blocks:
                    self._blocked_contexts.add(blocked)

    def load_from_config(self, config: dict[str, Any]) -> None:
        """Load contexts from configuration.

        Args:
            config: Dictionary with context definitions
        """
        for context_name, context_data in config.items():
            actions: dict[str, InputAction] = {}

            # Parse actions
            actions_data = context_data.get("actions", {})
            for action_name, bindings in actions_data.items():
                action = InputAction(name=action_name)

                if isinstance(bindings, list):
                    # Simple list of keys
                    action.keys = bindings
                elif isinstance(bindings, dict):
                    # Dict with keys and mouse_buttons
                    action.keys = bindings.get("keys", [])
                    action.mouse_buttons = bindings.get("mouse_buttons", [])

                actions[action_name] = action

            context = InputContext(
                name=context_name,
                priority=context_data.get("priority", 0),
                actions=actions,
                blocks=context_data.get("blocks", []),
                enabled=context_data.get("enabled", True),
            )
            self.register_context(context)

            # Auto-enable if specified
            if context.enabled:
                self.enable_context(context_name)

    def clear(self) -> None:
        """Clear all contexts."""
        self._contexts.clear()
        self._active_stack.clear()
        self._enabled_contexts.clear()
        self._blocked_contexts.clear()
