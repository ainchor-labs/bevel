"""Scene transition effects for Bevel engine.

Provides visual transitions when changing between scenes.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Callable

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.scene import Scene

logger = get_logger("transitions")


class TransitionState(Enum):
    """Current state of a transition."""

    IDLE = auto()
    OUT = auto()  # Transitioning out of current scene
    SWITCH = auto()  # Scene switch point
    IN = auto()  # Transitioning into new scene
    COMPLETE = auto()


@dataclass
class TransitionConfig:
    """Configuration for a transition."""

    duration: float = 0.5  # Total duration in seconds
    out_duration: float | None = None  # Duration of out phase (defaults to half)
    in_duration: float | None = None  # Duration of in phase (defaults to half)
    ease_out: str = "linear"  # Easing function for out phase
    ease_in: str = "linear"  # Easing function for in phase
    color: tuple[int, int, int, int] = (0, 0, 0, 255)  # Transition color


class Transition(ABC):
    """Base class for scene transitions."""

    def __init__(self, config: TransitionConfig | None = None) -> None:
        """Initialize the transition.

        Args:
            config: Transition configuration
        """
        self.config = config or TransitionConfig()
        self._state = TransitionState.IDLE
        self._progress: float = 0.0
        self._elapsed: float = 0.0
        self._on_switch: Callable[[], None] | None = None
        self._on_complete: Callable[[], None] | None = None

        # Calculate phase durations
        if self.config.out_duration is None:
            self._out_duration = self.config.duration / 2
        else:
            self._out_duration = self.config.out_duration

        if self.config.in_duration is None:
            self._in_duration = self.config.duration / 2
        else:
            self._in_duration = self.config.in_duration

    @property
    def state(self) -> TransitionState:
        """Get the current transition state."""
        return self._state

    @property
    def progress(self) -> float:
        """Get the transition progress (0.0 to 1.0)."""
        return self._progress

    @property
    def is_active(self) -> bool:
        """Check if the transition is active."""
        return self._state not in (TransitionState.IDLE, TransitionState.COMPLETE)

    def start(
        self,
        on_switch: Callable[[], None] | None = None,
        on_complete: Callable[[], None] | None = None,
    ) -> None:
        """Start the transition.

        Args:
            on_switch: Callback when scene should switch
            on_complete: Callback when transition completes
        """
        self._state = TransitionState.OUT
        self._progress = 0.0
        self._elapsed = 0.0
        self._on_switch = on_switch
        self._on_complete = on_complete
        logger.debug(f"Started {self.__class__.__name__} transition")

    def update(self, delta: float) -> None:
        """Update the transition.

        Args:
            delta: Time since last frame in seconds
        """
        if self._state == TransitionState.IDLE or self._state == TransitionState.COMPLETE:
            return

        self._elapsed += delta

        if self._state == TransitionState.OUT:
            if self._out_duration > 0:
                self._progress = min(self._elapsed / self._out_duration, 1.0)
            else:
                self._progress = 1.0

            if self._progress >= 1.0:
                # Switch scene
                self._state = TransitionState.SWITCH
                self._elapsed = 0.0
                if self._on_switch is not None:
                    self._on_switch()
                # Immediately transition to IN phase
                self._state = TransitionState.IN

        elif self._state == TransitionState.IN:
            if self._in_duration > 0:
                self._progress = 1.0 - min(self._elapsed / self._in_duration, 1.0)
            else:
                self._progress = 0.0

            if self._progress <= 0.0:
                self._state = TransitionState.COMPLETE
                if self._on_complete is not None:
                    self._on_complete()
                logger.debug(f"Completed {self.__class__.__name__} transition")

    @abstractmethod
    def render(self, renderer: Any) -> None:
        """Render the transition overlay.

        Args:
            renderer: The renderer to draw with
        """
        pass

    def reset(self) -> None:
        """Reset the transition to idle state."""
        self._state = TransitionState.IDLE
        self._progress = 0.0
        self._elapsed = 0.0
        self._on_switch = None
        self._on_complete = None


class ImmediateTransition(Transition):
    """Instant scene switch with no visual effect."""

    def __init__(self) -> None:
        super().__init__(TransitionConfig(duration=0.0))

    def start(
        self,
        on_switch: Callable[[], None] | None = None,
        on_complete: Callable[[], None] | None = None,
    ) -> None:
        self._state = TransitionState.COMPLETE
        if on_switch is not None:
            on_switch()
        if on_complete is not None:
            on_complete()

    def render(self, renderer: Any) -> None:
        pass  # No visual effect


class FadeTransition(Transition):
    """Fade to color, switch scene, fade from color."""

    def render(self, renderer: Any) -> None:
        """Render the fade overlay.

        Args:
            renderer: The renderer with raylib reference
        """
        if not self.is_active:
            return

        # Calculate alpha based on progress
        alpha = int(self._progress * 255)
        color = (
            self.config.color[0],
            self.config.color[1],
            self.config.color[2],
            alpha,
        )

        # Draw fullscreen overlay
        if hasattr(renderer, "_raylib") and renderer._raylib is not None:
            rl = renderer._raylib
            width = rl.GetScreenWidth()
            height = rl.GetScreenHeight()
            rl.DrawRectangle(0, 0, width, height, color)


class SlideTransition(Transition):
    """Slide the scenes in a direction."""

    def __init__(
        self,
        config: TransitionConfig | None = None,
        direction: str = "left",
    ) -> None:
        """Initialize slide transition.

        Args:
            config: Transition configuration
            direction: Slide direction ("left", "right", "up", "down")
        """
        super().__init__(config)
        self.direction = direction

    def render(self, renderer: Any) -> None:
        """Render the slide effect.

        For slide transitions, the actual sliding is done by the renderer
        based on the offset values. This just renders a cover.
        """
        if not self.is_active:
            return

        if hasattr(renderer, "_raylib") and renderer._raylib is not None:
            rl = renderer._raylib
            width = rl.GetScreenWidth()
            height = rl.GetScreenHeight()

            # Calculate offset based on direction and progress
            if self._state == TransitionState.OUT:
                progress = self._progress
            else:
                progress = 1.0 - self._progress

            offset_x = 0
            offset_y = 0

            if self.direction == "left":
                offset_x = int(width * progress)
            elif self.direction == "right":
                offset_x = -int(width * progress)
            elif self.direction == "up":
                offset_y = int(height * progress)
            elif self.direction == "down":
                offset_y = -int(height * progress)

            # Draw cover rectangle sliding in
            color = self.config.color
            if self.direction in ("left", "right"):
                x = width - offset_x if self.direction == "left" else -width + abs(offset_x)
                rl.DrawRectangle(x, 0, width, height, color)
            else:
                y = height - offset_y if self.direction == "up" else -height + abs(offset_y)
                rl.DrawRectangle(0, y, width, height, color)


class WipeTransition(Transition):
    """Wipe effect revealing the new scene."""

    def __init__(
        self,
        config: TransitionConfig | None = None,
        direction: str = "left",
    ) -> None:
        """Initialize wipe transition.

        Args:
            config: Transition configuration
            direction: Wipe direction ("left", "right", "up", "down")
        """
        super().__init__(config)
        self.direction = direction

    def render(self, renderer: Any) -> None:
        """Render the wipe effect."""
        if not self.is_active:
            return

        if hasattr(renderer, "_raylib") and renderer._raylib is not None:
            rl = renderer._raylib
            width = rl.GetScreenWidth()
            height = rl.GetScreenHeight()
            color = self.config.color

            # During OUT phase, wipe covers screen
            # During IN phase, wipe reveals screen
            if self._state == TransitionState.OUT:
                progress = self._progress
            else:
                progress = self._progress  # Still covers based on progress

            if self.direction == "left":
                w = int(width * progress)
                rl.DrawRectangle(0, 0, w, height, color)
            elif self.direction == "right":
                w = int(width * progress)
                rl.DrawRectangle(width - w, 0, w, height, color)
            elif self.direction == "up":
                h = int(height * progress)
                rl.DrawRectangle(0, 0, width, h, color)
            elif self.direction == "down":
                h = int(height * progress)
                rl.DrawRectangle(0, height - h, width, h, color)


class CircleTransition(Transition):
    """Circle wipe effect (closing/opening circle)."""

    def __init__(
        self,
        config: TransitionConfig | None = None,
        center: tuple[float, float] | None = None,
    ) -> None:
        """Initialize circle transition.

        Args:
            config: Transition configuration
            center: Center point as (x, y) normalized (0-1). Defaults to center.
        """
        super().__init__(config)
        self.center = center or (0.5, 0.5)

    def render(self, renderer: Any) -> None:
        """Render the circle effect.

        Note: This is a simplified version that draws a filled circle.
        A full implementation would use a shader or stencil buffer.
        """
        if not self.is_active:
            return

        if hasattr(renderer, "_raylib") and renderer._raylib is not None:
            rl = renderer._raylib
            width = rl.GetScreenWidth()
            height = rl.GetScreenHeight()

            # Calculate center position
            cx = int(width * self.center[0])
            cy = int(height * self.center[1])

            # Max radius is corner distance
            max_radius = ((width ** 2 + height ** 2) ** 0.5) / 2 + 100

            # Calculate current radius (inverse for circle opening)
            if self._state == TransitionState.OUT:
                # Circle closing
                radius = int(max_radius * (1.0 - self._progress))
            else:
                # Circle opening
                radius = int(max_radius * (1.0 - self._progress))

            color = self.config.color

            # Draw border around the circle to create the effect
            # This is a simplified approach - draw rectangle with circle cut
            # In practice, this would use shaders or multiple draw calls
            border_width = int(max_radius * 2)
            segments = 64

            # Draw the outer rectangle minus the circle
            # Simplified: just draw the circle outline very thick
            if radius > 0:
                # Draw four rectangles around the circle area
                # Top
                rl.DrawRectangle(0, 0, width, max(0, cy - radius), color)
                # Bottom
                rl.DrawRectangle(0, min(height, cy + radius), width, max(0, height - (cy + radius)), color)
                # Left
                rl.DrawRectangle(0, max(0, cy - radius), max(0, cx - radius), min(height, radius * 2), color)
                # Right
                rl.DrawRectangle(min(width, cx + radius), max(0, cy - radius), max(0, width - (cx + radius)), min(height, radius * 2), color)


# Transition registry
_transitions: dict[str, type[Transition]] = {
    "immediate": ImmediateTransition,
    "fade": FadeTransition,
    "slide": SlideTransition,
    "slide_left": lambda: SlideTransition(direction="left"),
    "slide_right": lambda: SlideTransition(direction="right"),
    "slide_up": lambda: SlideTransition(direction="up"),
    "slide_down": lambda: SlideTransition(direction="down"),
    "wipe": WipeTransition,
    "wipe_left": lambda: WipeTransition(direction="left"),
    "wipe_right": lambda: WipeTransition(direction="right"),
    "wipe_up": lambda: WipeTransition(direction="up"),
    "wipe_down": lambda: WipeTransition(direction="down"),
    "circle": CircleTransition,
}


def register_transition(name: str, transition_class: type[Transition]) -> None:
    """Register a custom transition type.

    Args:
        name: The transition name
        transition_class: The Transition class
    """
    _transitions[name] = transition_class


def create_transition(
    name: str,
    duration: float = 0.5,
    color: tuple[int, int, int, int] | None = None,
    **kwargs: Any,
) -> Transition:
    """Create a transition by name.

    Args:
        name: The transition type name
        duration: Transition duration in seconds
        color: Transition color (RGBA)
        **kwargs: Additional arguments for specific transitions

    Returns:
        A Transition instance
    """
    if name not in _transitions:
        logger.warning(f"Unknown transition '{name}', using fade")
        name = "fade"

    transition_class = _transitions[name]

    # Handle factory functions
    if callable(transition_class) and not isinstance(transition_class, type):
        return transition_class()

    config = TransitionConfig(
        duration=duration,
        color=color or (0, 0, 0, 255),
    )

    return transition_class(config, **kwargs)


def get_transition_names() -> list[str]:
    """Get list of available transition names.

    Returns:
        List of registered transition names
    """
    return list(_transitions.keys())
