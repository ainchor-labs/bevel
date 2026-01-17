"""Input management system for Bevel engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from bevel.utils.math import Vector2
from bevel.utils.logger import get_logger
from bevel.input.input_context import InputContextStack, InputContext, InputAction as ContextInputAction

logger = get_logger("input")

# Raylib key constants (will be populated when raylib is available)
# These are the common key codes from raylib
KEY_CODES = {
    # Letters
    "KEY_A": 65, "KEY_B": 66, "KEY_C": 67, "KEY_D": 68, "KEY_E": 69,
    "KEY_F": 70, "KEY_G": 71, "KEY_H": 72, "KEY_I": 73, "KEY_J": 74,
    "KEY_K": 75, "KEY_L": 76, "KEY_M": 77, "KEY_N": 78, "KEY_O": 79,
    "KEY_P": 80, "KEY_Q": 81, "KEY_R": 82, "KEY_S": 83, "KEY_T": 84,
    "KEY_U": 85, "KEY_V": 86, "KEY_W": 87, "KEY_X": 88, "KEY_Y": 89,
    "KEY_Z": 90,
    # Numbers
    "KEY_0": 48, "KEY_1": 49, "KEY_2": 50, "KEY_3": 51, "KEY_4": 52,
    "KEY_5": 53, "KEY_6": 54, "KEY_7": 55, "KEY_8": 56, "KEY_9": 57,
    # Function keys
    "KEY_F1": 290, "KEY_F2": 291, "KEY_F3": 292, "KEY_F4": 293,
    "KEY_F5": 294, "KEY_F6": 295, "KEY_F7": 296, "KEY_F8": 297,
    "KEY_F9": 298, "KEY_F10": 299, "KEY_F11": 300, "KEY_F12": 301,
    # Arrow keys
    "KEY_LEFT": 263, "KEY_RIGHT": 262, "KEY_UP": 265, "KEY_DOWN": 264,
    # Special keys
    "KEY_SPACE": 32, "KEY_ENTER": 257, "KEY_ESCAPE": 256,
    "KEY_BACKSPACE": 259, "KEY_TAB": 258, "KEY_DELETE": 261,
    "KEY_INSERT": 260, "KEY_HOME": 268, "KEY_END": 269,
    "KEY_PAGE_UP": 266, "KEY_PAGE_DOWN": 267,
    # Modifiers
    "KEY_LEFT_SHIFT": 340, "KEY_LEFT_CONTROL": 341, "KEY_LEFT_ALT": 342,
    "KEY_RIGHT_SHIFT": 344, "KEY_RIGHT_CONTROL": 345, "KEY_RIGHT_ALT": 346,
}

# Mouse button constants
MOUSE_BUTTONS = {
    "MOUSE_LEFT": 0,
    "MOUSE_RIGHT": 1,
    "MOUSE_MIDDLE": 2,
}


@dataclass
class InputAction:
    """Represents an input action that can be triggered by multiple keys."""

    name: str
    keys: list[int] = field(default_factory=list)
    mouse_buttons: list[int] = field(default_factory=list)


class InputManager:
    """Manages input state and actions."""

    def __init__(self) -> None:
        self._actions: dict[str, InputAction] = {}
        self._key_states: dict[int, bool] = {}
        self._prev_key_states: dict[int, bool] = {}
        self._mouse_states: dict[int, bool] = {}
        self._prev_mouse_states: dict[int, bool] = {}
        self._mouse_position: Vector2 = Vector2.zero()
        self._mouse_delta: Vector2 = Vector2.zero()
        self._mouse_wheel: float = 0.0
        self._raylib: Any = None
        self._context_stack: InputContextStack = InputContextStack()
        self._use_contexts: bool = False  # Toggle between legacy and context-based mode

    def set_raylib(self, raylib: Any) -> None:
        """Set the raylib module reference.

        Args:
            raylib: The raylib module
        """
        self._raylib = raylib

    def load_actions(self, path: str | Path) -> bool:
        """Load input actions from a YAML file.

        Supports two formats:
        1. Legacy format with top-level 'actions' key
        2. Context-based format with 'contexts' key

        Args:
            path: Path to the input.yaml file

        Returns:
            True if successful, False otherwise
        """
        path = Path(path)
        if not path.exists():
            logger.warning(f"Input config file not found: {path}")
            return False

        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            logger.error(f"Failed to load input config: {e}")
            return False

        # Check for context-based format
        if "contexts" in data:
            return self._load_contexts(data)

        # Legacy format with top-level actions
        actions_data = data.get("actions", {})
        for name, action_data in actions_data.items():
            keys = []
            mouse_buttons = []

            # Parse keys
            for key_name in action_data.get("keys", []):
                if key_name in KEY_CODES:
                    keys.append(KEY_CODES[key_name])
                else:
                    logger.warning(f"Unknown key: {key_name}")

            # Parse mouse buttons
            for button_name in action_data.get("mouse_buttons", []):
                if button_name in MOUSE_BUTTONS:
                    mouse_buttons.append(MOUSE_BUTTONS[button_name])
                else:
                    logger.warning(f"Unknown mouse button: {button_name}")

            self._actions[name] = InputAction(
                name=name,
                keys=keys,
                mouse_buttons=mouse_buttons,
            )
            logger.debug(f"Loaded action: {name}")

        logger.info(f"Loaded {len(self._actions)} input actions")
        return True

    def _load_contexts(self, data: dict[str, Any]) -> bool:
        """Load context-based input configuration.

        Args:
            data: The parsed YAML data with 'contexts' key

        Returns:
            True if successful
        """
        self._use_contexts = True
        contexts_data = data.get("contexts", {})

        for context_name, context_data in contexts_data.items():
            actions: dict[str, ContextInputAction] = {}

            # Parse actions in this context
            actions_data = context_data.get("actions", {})
            for action_name, bindings in actions_data.items():
                keys: list[int] = []
                mouse_buttons: list[int] = []

                # Handle list of key names (simple format)
                if isinstance(bindings, list):
                    for key_name in bindings:
                        if key_name in KEY_CODES:
                            keys.append(KEY_CODES[key_name])
                        elif key_name in MOUSE_BUTTONS:
                            mouse_buttons.append(MOUSE_BUTTONS[key_name])
                        else:
                            logger.warning(f"Unknown binding: {key_name}")
                # Handle dict with keys and mouse_buttons
                elif isinstance(bindings, dict):
                    for key_name in bindings.get("keys", []):
                        if key_name in KEY_CODES:
                            keys.append(KEY_CODES[key_name])
                        else:
                            logger.warning(f"Unknown key: {key_name}")
                    for button_name in bindings.get("mouse_buttons", []):
                        if button_name in MOUSE_BUTTONS:
                            mouse_buttons.append(MOUSE_BUTTONS[button_name])
                        else:
                            logger.warning(f"Unknown mouse button: {button_name}")

                actions[action_name] = ContextInputAction(
                    name=action_name,
                    keys=keys,
                    mouse_buttons=mouse_buttons,
                )

            context = InputContext(
                name=context_name,
                priority=context_data.get("priority", 0),
                actions=actions,
                blocks=context_data.get("blocks", []),
                enabled=context_data.get("enabled", True),
            )
            self._context_stack.register_context(context)

            # Auto-enable if specified (default True)
            if context.enabled:
                self._context_stack.enable_context(context_name)

            logger.debug(
                f"Loaded context '{context_name}' with {len(actions)} actions "
                f"(priority={context.priority})"
            )

        logger.info(f"Loaded {len(contexts_data)} input contexts")
        return True

    def register_action(self, name: str, keys: list[str] | None = None,
                       mouse_buttons: list[str] | None = None) -> None:
        """Register an input action programmatically.

        Args:
            name: Action name
            keys: List of key names (e.g., ["KEY_A", "KEY_LEFT"])
            mouse_buttons: List of mouse button names
        """
        key_codes = []
        button_codes = []

        if keys:
            for key_name in keys:
                if key_name in KEY_CODES:
                    key_codes.append(KEY_CODES[key_name])

        if mouse_buttons:
            for button_name in mouse_buttons:
                if button_name in MOUSE_BUTTONS:
                    button_codes.append(MOUSE_BUTTONS[button_name])

        self._actions[name] = InputAction(
            name=name,
            keys=key_codes,
            mouse_buttons=button_codes,
        )

    def update(self) -> None:
        """Update input state. Called once per frame."""
        if self._raylib is None:
            return

        rl = self._raylib

        # Store previous states
        self._prev_key_states = dict(self._key_states)
        self._prev_mouse_states = dict(self._mouse_states)

        # Update key states
        for key_code in KEY_CODES.values():
            self._key_states[key_code] = rl.IsKeyDown(key_code)

        # Update mouse states
        for button_code in MOUSE_BUTTONS.values():
            self._mouse_states[button_code] = rl.IsMouseButtonDown(button_code)

        # Update mouse position
        pos = rl.GetMousePosition()
        new_pos = Vector2(pos.x, pos.y)
        self._mouse_delta = new_pos - self._mouse_position
        self._mouse_position = new_pos

        # Update mouse wheel
        self._mouse_wheel = rl.GetMouseWheelMove()

    def is_key_pressed(self, key: int | str) -> bool:
        """Check if a key is currently pressed.

        Args:
            key: Key code or name (e.g., KEY_A or 65)

        Returns:
            True if the key is pressed
        """
        if isinstance(key, str):
            key = KEY_CODES.get(key, -1)
        return self._key_states.get(key, False)

    def is_key_just_pressed(self, key: int | str) -> bool:
        """Check if a key was just pressed this frame.

        Args:
            key: Key code or name

        Returns:
            True if the key was just pressed
        """
        if isinstance(key, str):
            key = KEY_CODES.get(key, -1)
        return self._key_states.get(key, False) and not self._prev_key_states.get(key, False)

    def is_key_just_released(self, key: int | str) -> bool:
        """Check if a key was just released this frame.

        Args:
            key: Key code or name

        Returns:
            True if the key was just released
        """
        if isinstance(key, str):
            key = KEY_CODES.get(key, -1)
        return not self._key_states.get(key, False) and self._prev_key_states.get(key, False)

    def is_mouse_button_pressed(self, button: int | str) -> bool:
        """Check if a mouse button is currently pressed.

        Args:
            button: Button code or name

        Returns:
            True if the button is pressed
        """
        if isinstance(button, str):
            button = MOUSE_BUTTONS.get(button, -1)
        return self._mouse_states.get(button, False)

    def is_mouse_button_just_pressed(self, button: int | str) -> bool:
        """Check if a mouse button was just pressed.

        Args:
            button: Button code or name

        Returns:
            True if the button was just pressed
        """
        if isinstance(button, str):
            button = MOUSE_BUTTONS.get(button, -1)
        return (
            self._mouse_states.get(button, False)
            and not self._prev_mouse_states.get(button, False)
        )

    def is_mouse_button_just_released(self, button: int | str) -> bool:
        """Check if a mouse button was just released.

        Args:
            button: Button code or name

        Returns:
            True if the button was just released
        """
        if isinstance(button, str):
            button = MOUSE_BUTTONS.get(button, -1)
        return (
            not self._mouse_states.get(button, False)
            and self._prev_mouse_states.get(button, False)
        )

    def get_mouse_position(self) -> Vector2:
        """Get the current mouse position.

        Returns:
            The mouse position as a Vector2
        """
        return self._mouse_position.copy()

    def get_mouse_delta(self) -> Vector2:
        """Get the mouse movement since last frame.

        Returns:
            The mouse delta as a Vector2
        """
        return self._mouse_delta.copy()

    def get_mouse_wheel(self) -> float:
        """Get the mouse wheel movement.

        Returns:
            The wheel movement (positive = up, negative = down)
        """
        return self._mouse_wheel

    def _get_action_bindings(self, action: str) -> tuple[list[int], list[int]]:
        """Get key and mouse button bindings for an action.

        Checks context-based actions first, then falls back to legacy actions.

        Args:
            action: The action name

        Returns:
            Tuple of (key codes, mouse button codes)
        """
        if self._use_contexts:
            action_def = self._context_stack.get_effective_action(action)
            if action_def is not None:
                return action_def.keys, action_def.mouse_buttons
            return [], []

        # Legacy mode
        if action not in self._actions:
            return [], []
        action_def = self._actions[action]
        return action_def.keys, action_def.mouse_buttons

    def is_action_pressed(self, action: str) -> bool:
        """Check if an action is currently active.

        Args:
            action: The action name

        Returns:
            True if any key/button for the action is pressed
        """
        keys, buttons = self._get_action_bindings(action)

        for key in keys:
            if self._key_states.get(key, False):
                return True

        for button in buttons:
            if self._mouse_states.get(button, False):
                return True

        return False

    def is_action_just_pressed(self, action: str) -> bool:
        """Check if an action was just activated this frame.

        Args:
            action: The action name

        Returns:
            True if the action was just activated
        """
        keys, buttons = self._get_action_bindings(action)

        for key in keys:
            if self.is_key_just_pressed(key):
                return True

        for button in buttons:
            if self.is_mouse_button_just_pressed(button):
                return True

        return False

    def is_action_just_released(self, action: str) -> bool:
        """Check if an action was just deactivated this frame.

        Args:
            action: The action name

        Returns:
            True if the action was just deactivated
        """
        keys, buttons = self._get_action_bindings(action)

        if not keys and not buttons:
            return False

        # Check if all keys/buttons for action are now released
        # but at least one was pressed last frame
        all_released = True
        any_was_pressed = False

        for key in keys:
            if self._key_states.get(key, False):
                all_released = False
            if self._prev_key_states.get(key, False):
                any_was_pressed = True

        for button in buttons:
            if self._mouse_states.get(button, False):
                all_released = False
            if self._prev_mouse_states.get(button, False):
                any_was_pressed = True

        return all_released and any_was_pressed

    def get_action_strength(self, action: str) -> float:
        """Get the strength of an action (0.0 or 1.0 for digital inputs).

        Args:
            action: The action name

        Returns:
            The action strength (0.0-1.0)
        """
        return 1.0 if self.is_action_pressed(action) else 0.0

    def get_axis(self, negative_action: str, positive_action: str) -> float:
        """Get an axis value from two actions.

        Args:
            negative_action: Action for negative direction
            positive_action: Action for positive direction

        Returns:
            -1.0, 0.0, or 1.0
        """
        return self.get_action_strength(positive_action) - self.get_action_strength(negative_action)

    def get_vector(self, left: str, right: str, up: str, down: str) -> Vector2:
        """Get a normalized movement vector from four actions.

        Args:
            left: Action for left
            right: Action for right
            up: Action for up
            down: Action for down

        Returns:
            A normalized Vector2
        """
        x = self.get_axis(left, right)
        y = self.get_axis(up, down)
        vec = Vector2(x, y)
        if vec.length > 0:
            return vec.normalized()
        return vec

    # Context management methods

    def push_context(self, name: str) -> bool:
        """Push an input context onto the stack.

        Args:
            name: The context name to push

        Returns:
            True if pushed successfully
        """
        return self._context_stack.push_context(name)

    def pop_context(self, name: str | None = None) -> str | None:
        """Pop an input context from the stack.

        Args:
            name: Specific context to pop, or None to pop the top

        Returns:
            The popped context name, or None
        """
        return self._context_stack.pop_context(name)

    def enable_context(self, name: str) -> None:
        """Enable an input context.

        Args:
            name: The context name to enable
        """
        self._context_stack.enable_context(name)

    def disable_context(self, name: str) -> None:
        """Disable an input context.

        Args:
            name: The context name to disable
        """
        self._context_stack.disable_context(name)

    def is_context_active(self, name: str) -> bool:
        """Check if an input context is currently active.

        Args:
            name: The context name

        Returns:
            True if the context is active
        """
        return self._context_stack.is_context_active(name)

    def get_active_contexts(self) -> list[str]:
        """Get list of active context names, sorted by priority.

        Returns:
            List of active context names (highest priority first)
        """
        return self._context_stack.get_active_contexts()

    @property
    def context_stack(self) -> InputContextStack:
        """Get the context stack for direct manipulation.

        Returns:
            The InputContextStack instance
        """
        return self._context_stack


# Global input singleton
_input_manager: InputManager | None = None


class InputSingleton:
    """Singleton proxy for the InputManager."""

    @staticmethod
    def _get_manager() -> InputManager:
        global _input_manager
        if _input_manager is None:
            _input_manager = InputManager()
        return _input_manager

    @staticmethod
    def is_key_pressed(key: int | str) -> bool:
        return InputSingleton._get_manager().is_key_pressed(key)

    @staticmethod
    def is_key_just_pressed(key: int | str) -> bool:
        return InputSingleton._get_manager().is_key_just_pressed(key)

    @staticmethod
    def is_key_just_released(key: int | str) -> bool:
        return InputSingleton._get_manager().is_key_just_released(key)

    @staticmethod
    def is_mouse_button_pressed(button: int | str) -> bool:
        return InputSingleton._get_manager().is_mouse_button_pressed(button)

    @staticmethod
    def is_mouse_button_just_pressed(button: int | str) -> bool:
        return InputSingleton._get_manager().is_mouse_button_just_pressed(button)

    @staticmethod
    def is_mouse_button_just_released(button: int | str) -> bool:
        return InputSingleton._get_manager().is_mouse_button_just_released(button)

    @staticmethod
    def get_mouse_position() -> Vector2:
        return InputSingleton._get_manager().get_mouse_position()

    @staticmethod
    def get_mouse_delta() -> Vector2:
        return InputSingleton._get_manager().get_mouse_delta()

    @staticmethod
    def get_mouse_wheel() -> float:
        return InputSingleton._get_manager().get_mouse_wheel()

    @staticmethod
    def is_action_pressed(action: str) -> bool:
        return InputSingleton._get_manager().is_action_pressed(action)

    @staticmethod
    def is_action_just_pressed(action: str) -> bool:
        return InputSingleton._get_manager().is_action_just_pressed(action)

    @staticmethod
    def is_action_just_released(action: str) -> bool:
        return InputSingleton._get_manager().is_action_just_released(action)

    @staticmethod
    def get_action_strength(action: str) -> float:
        return InputSingleton._get_manager().get_action_strength(action)

    @staticmethod
    def get_axis(negative_action: str, positive_action: str) -> float:
        return InputSingleton._get_manager().get_axis(negative_action, positive_action)

    @staticmethod
    def get_vector(left: str, right: str, up: str, down: str) -> Vector2:
        return InputSingleton._get_manager().get_vector(left, right, up, down)

    @staticmethod
    def push_context(name: str) -> bool:
        """Push an input context onto the stack."""
        return InputSingleton._get_manager().push_context(name)

    @staticmethod
    def pop_context(name: str | None = None) -> str | None:
        """Pop an input context from the stack."""
        return InputSingleton._get_manager().pop_context(name)

    @staticmethod
    def enable_context(name: str) -> None:
        """Enable an input context."""
        InputSingleton._get_manager().enable_context(name)

    @staticmethod
    def disable_context(name: str) -> None:
        """Disable an input context."""
        InputSingleton._get_manager().disable_context(name)

    @staticmethod
    def is_context_active(name: str) -> bool:
        """Check if an input context is currently active."""
        return InputSingleton._get_manager().is_context_active(name)

    @staticmethod
    def get_active_contexts() -> list[str]:
        """Get list of active context names."""
        return InputSingleton._get_manager().get_active_contexts()


# Export Input as the singleton interface
Input = InputSingleton


def get_input_manager() -> InputManager:
    """Get the global input manager instance."""
    global _input_manager
    if _input_manager is None:
        _input_manager = InputManager()
    return _input_manager


def set_input_manager(manager: InputManager) -> None:
    """Set the global input manager instance."""
    global _input_manager
    _input_manager = manager
