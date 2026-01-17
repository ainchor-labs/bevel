"""Core module for Bevel engine."""

from .node import Node
from .transform import Transform2D
from .scene import Scene, SceneManager
from .scene_loader import SceneLoader
from .lifecycle import LifecycleDispatcher
from .transitions import (
    Transition,
    TransitionConfig,
    TransitionState,
    ImmediateTransition,
    FadeTransition,
    SlideTransition,
    WipeTransition,
    CircleTransition,
    create_transition,
    register_transition,
    get_transition_names,
)

__all__ = [
    "Node",
    "Transform2D",
    "Scene",
    "SceneManager",
    "SceneLoader",
    "LifecycleDispatcher",
    "Transition",
    "TransitionConfig",
    "TransitionState",
    "ImmediateTransition",
    "FadeTransition",
    "SlideTransition",
    "WipeTransition",
    "CircleTransition",
    "create_transition",
    "register_transition",
    "get_transition_names",
]
