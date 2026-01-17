"""Time management system for Bevel engine.

This module provides a Unity-like Time class with static access to time-related
information such as delta time, elapsed time, frame count, and time scale.

Usage:
    from bevel.utils import Time

    # In your script's update method:
    def update(self, delta: float):
        # You can use the passed delta or access via Time
        velocity = self.speed * Time.delta_time
        print(f"Game running for {Time.time} seconds")
        print(f"Frame {Time.frame_count}")
"""

from __future__ import annotations


class _TimeState:
    """Internal state holder for time values.

    This class stores the actual time values that are updated each frame
    by the engine. The TimeSingleton class provides static access to these.
    """

    def __init__(self) -> None:
        # Frame timing
        self._delta_time: float = 0.0
        self._unscaled_delta_time: float = 0.0
        self._smooth_delta_time: float = 0.0
        self._smooth_delta_samples: list[float] = []
        self._smooth_sample_count: int = 10

        # Elapsed time
        self._time: float = 0.0
        self._unscaled_time: float = 0.0
        self._realtime_since_startup: float = 0.0
        self._time_since_level_load: float = 0.0

        # Fixed timestep
        self._fixed_delta_time: float = 1.0 / 60.0  # 60 Hz default
        self._fixed_time: float = 0.0
        self._fixed_unscaled_time: float = 0.0
        self._fixed_unscaled_delta_time: float = 1.0 / 60.0
        self._in_fixed_time_step: bool = False

        # Frame count
        self._frame_count: int = 0

        # Time scale
        self._time_scale: float = 1.0

        # Maximum delta time (to prevent spiral of death)
        self._maximum_delta_time: float = 1.0 / 3.0  # ~333ms max
        self._maximum_particle_delta_time: float = 0.05  # 50ms

        # Capture mode (for recording)
        self._capture_delta_time: float = 0.0
        self._capture_framerate: int = 0

    def reset(self) -> None:
        """Reset all time values to initial state."""
        self._delta_time = 0.0
        self._unscaled_delta_time = 0.0
        self._smooth_delta_time = 0.0
        self._smooth_delta_samples.clear()
        self._time = 0.0
        self._unscaled_time = 0.0
        self._realtime_since_startup = 0.0
        self._time_since_level_load = 0.0
        self._fixed_time = 0.0
        self._fixed_unscaled_time = 0.0
        self._in_fixed_time_step = False
        self._frame_count = 0
        self._time_scale = 1.0
        self._capture_delta_time = 0.0
        self._capture_framerate = 0


# Global time state instance
_time_state: _TimeState | None = None


def _get_time_state() -> _TimeState:
    """Get or create the global time state."""
    global _time_state
    if _time_state is None:
        _time_state = _TimeState()
    return _time_state


class TimeSingleton:
    """Singleton proxy for time information.

    Provides Unity-like static access to time-related values.
    All properties are read-only except for time_scale, fixed_delta_time,
    maximum_delta_time, maximum_particle_delta_time, and capture settings.
    """

    # =========================================================================
    # Delta Time Properties
    # =========================================================================

    @staticmethod
    def _get_state() -> _TimeState:
        return _get_time_state()

    @property
    def delta_time(self) -> float:
        """The interval in seconds from the last frame to the current one.

        This value is scaled by time_scale and clamped by maximum_delta_time.
        When called from within a fixed update, returns fixed_delta_time.

        Returns:
            Time in seconds since last frame (scaled).
        """
        state = self._get_state()
        if state._in_fixed_time_step:
            return state._fixed_delta_time
        return state._delta_time

    @property
    def unscaled_delta_time(self) -> float:
        """The timeScale-independent interval in seconds from last frame.

        Use this for UI animations or anything that should not be affected
        by time scale (like pause menus).

        Returns:
            Time in seconds since last frame (unscaled).
        """
        return self._get_state()._unscaled_delta_time

    @property
    def smooth_delta_time(self) -> float:
        """A smoothed out delta_time.

        This value is averaged over recent frames to reduce jitter.
        Useful for camera movement or other smooth operations.

        Returns:
            Smoothed time in seconds since last frame.
        """
        return self._get_state()._smooth_delta_time

    # =========================================================================
    # Elapsed Time Properties
    # =========================================================================

    @property
    def time(self) -> float:
        """The time at the beginning of this frame in seconds since game start.

        This value is affected by time_scale. When time_scale is 0,
        this value stops increasing.

        Returns:
            Scaled elapsed time in seconds.
        """
        return self._get_state()._time

    @property
    def unscaled_time(self) -> float:
        """The timeScale-independent time for this frame in seconds.

        This continues to increase even when time_scale is 0.

        Returns:
            Unscaled elapsed time in seconds.
        """
        return self._get_state()._unscaled_time

    @property
    def realtime_since_startup(self) -> float:
        """The real time in seconds since the game started.

        This is the actual wall-clock time and is never affected by
        time_scale or pausing.

        Returns:
            Real elapsed time in seconds since startup.
        """
        return self._get_state()._realtime_since_startup

    @property
    def time_since_level_load(self) -> float:
        """The time in seconds since the current scene was loaded.

        Resets when a new scene is loaded (non-additive).

        Returns:
            Time in seconds since current scene started.
        """
        return self._get_state()._time_since_level_load

    # =========================================================================
    # Fixed Timestep Properties
    # =========================================================================

    @property
    def fixed_delta_time(self) -> float:
        """The interval in seconds at which physics and fixed updates occur.

        Default is 1/60 (60 Hz). Can be modified to change physics precision.

        Returns:
            Fixed timestep interval in seconds.
        """
        return self._get_state()._fixed_delta_time

    @fixed_delta_time.setter
    def fixed_delta_time(self, value: float) -> None:
        """Set the fixed delta time.

        Args:
            value: The new fixed timestep in seconds.
        """
        state = self._get_state()
        state._fixed_delta_time = max(0.0001, value)  # Minimum 0.1ms
        state._fixed_unscaled_delta_time = state._fixed_delta_time

    @property
    def fixed_time(self) -> float:
        """The time the latest fixed update started, in seconds since game start.

        Returns:
            Time at start of current/last fixed update.
        """
        return self._get_state()._fixed_time

    @property
    def fixed_unscaled_time(self) -> float:
        """The timeScale-independent time at the start of the last fixed update.

        Returns:
            Unscaled time at start of current/last fixed update.
        """
        return self._get_state()._fixed_unscaled_time

    @property
    def fixed_unscaled_delta_time(self) -> float:
        """The timeScale-independent interval for fixed updates.

        Returns:
            Unscaled fixed timestep interval in seconds.
        """
        return self._get_state()._fixed_unscaled_delta_time

    @property
    def in_fixed_time_step(self) -> bool:
        """Returns True if called from within a fixed update callback.

        Returns:
            Whether currently executing a fixed update.
        """
        return self._get_state()._in_fixed_time_step

    # =========================================================================
    # Frame Count
    # =========================================================================

    @property
    def frame_count(self) -> int:
        """The total number of frames since the game started.

        This increments by 1 every frame, regardless of time_scale.

        Returns:
            Total frame count.
        """
        return self._get_state()._frame_count

    # =========================================================================
    # Time Scale
    # =========================================================================

    @property
    def time_scale(self) -> float:
        """The scale at which time passes.

        - 1.0 = normal speed
        - 0.5 = half speed (slow motion)
        - 2.0 = double speed
        - 0.0 = paused (time and delta_time stop, but unscaled versions continue)

        Returns:
            Current time scale factor.
        """
        return self._get_state()._time_scale

    @time_scale.setter
    def time_scale(self, value: float) -> None:
        """Set the time scale.

        Args:
            value: The new time scale (>= 0).
        """
        self._get_state()._time_scale = max(0.0, value)

    # =========================================================================
    # Maximum Delta Time
    # =========================================================================

    @property
    def maximum_delta_time(self) -> float:
        """The maximum value of delta_time in any given frame.

        This prevents the "spiral of death" where a slow frame causes
        even slower subsequent frames. Default is 1/3 second.

        Returns:
            Maximum delta time in seconds.
        """
        return self._get_state()._maximum_delta_time

    @maximum_delta_time.setter
    def maximum_delta_time(self, value: float) -> None:
        """Set the maximum delta time.

        Args:
            value: Maximum delta time in seconds (minimum 0.0001).
        """
        self._get_state()._maximum_delta_time = max(0.0001, value)

    @property
    def maximum_particle_delta_time(self) -> float:
        """The maximum time a frame can spend on particle updates.

        Returns:
            Maximum particle delta time in seconds.
        """
        return self._get_state()._maximum_particle_delta_time

    @maximum_particle_delta_time.setter
    def maximum_particle_delta_time(self, value: float) -> None:
        """Set the maximum particle delta time.

        Args:
            value: Maximum particle delta time in seconds.
        """
        self._get_state()._maximum_particle_delta_time = max(0.0001, value)

    # =========================================================================
    # Capture Mode (for recording/screenshots)
    # =========================================================================

    @property
    def capture_delta_time(self) -> float:
        """If set to a value > 0, time advances by this amount each frame.

        This is useful for capturing screenshots at a fixed rate.
        Set to 0 to disable capture mode.

        Returns:
            Capture delta time in seconds (0 = disabled).
        """
        return self._get_state()._capture_delta_time

    @capture_delta_time.setter
    def capture_delta_time(self, value: float) -> None:
        """Set capture delta time.

        Args:
            value: Delta time per frame for capture mode (0 = disabled).
        """
        state = self._get_state()
        state._capture_delta_time = max(0.0, value)
        if value > 0:
            state._capture_framerate = int(1.0 / value)
        else:
            state._capture_framerate = 0

    @property
    def capture_framerate(self) -> int:
        """Reciprocal of capture_delta_time.

        Returns:
            Target framerate for capture mode (0 = disabled).
        """
        return self._get_state()._capture_framerate

    @capture_framerate.setter
    def capture_framerate(self, value: int) -> None:
        """Set capture framerate.

        Args:
            value: Target framerate for capture (0 = disabled).
        """
        state = self._get_state()
        if value > 0:
            state._capture_framerate = value
            state._capture_delta_time = 1.0 / value
        else:
            state._capture_framerate = 0
            state._capture_delta_time = 0.0


# Create singleton instance
Time = TimeSingleton()


# =========================================================================
# Internal functions for engine use
# =========================================================================

def _update_time(raw_delta: float, realtime_delta: float) -> None:
    """Update time values for a new frame.

    This should be called once per frame by the engine, before any
    game logic updates.

    Args:
        raw_delta: Raw delta time from renderer (unscaled).
        realtime_delta: Actual wall-clock time since last frame.
    """
    state = _get_time_state()

    # Update frame count
    state._frame_count += 1

    # Store unscaled delta (clamped by maximum)
    state._unscaled_delta_time = min(raw_delta, state._maximum_delta_time)

    # Use capture delta time if in capture mode
    if state._capture_delta_time > 0:
        state._unscaled_delta_time = state._capture_delta_time

    # Calculate scaled delta time
    state._delta_time = state._unscaled_delta_time * state._time_scale

    # Update smooth delta time (rolling average)
    state._smooth_delta_samples.append(state._delta_time)
    if len(state._smooth_delta_samples) > state._smooth_sample_count:
        state._smooth_delta_samples.pop(0)
    state._smooth_delta_time = sum(state._smooth_delta_samples) / len(state._smooth_delta_samples)

    # Update elapsed times
    state._time += state._delta_time
    state._unscaled_time += state._unscaled_delta_time
    state._realtime_since_startup += realtime_delta
    state._time_since_level_load += state._delta_time


def _begin_fixed_update() -> None:
    """Mark the beginning of a fixed update step.

    Called by the engine before dispatching fixed_update to scripts.
    """
    state = _get_time_state()
    state._in_fixed_time_step = True
    state._fixed_time = state._time
    state._fixed_unscaled_time = state._unscaled_time


def _end_fixed_update() -> None:
    """Mark the end of a fixed update step.

    Called by the engine after all fixed_update calls complete.
    """
    _get_time_state()._in_fixed_time_step = False


def _on_scene_loaded() -> None:
    """Reset time_since_level_load when a new scene is loaded.

    Called by the scene manager when loading a new scene.
    """
    _get_time_state()._time_since_level_load = 0.0


def _reset_time() -> None:
    """Reset all time values.

    Called when the engine is reset or restarted.
    """
    _get_time_state().reset()


def get_time_state() -> _TimeState:
    """Get the time state for direct access (for engine internals).

    Returns:
        The global _TimeState instance.
    """
    return _get_time_state()