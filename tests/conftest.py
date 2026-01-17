"""Pytest configuration and fixtures for Bevel tests."""

import pytest
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

from bevel.core.node import Node
from bevel.core.signals import SignalManager, SignalDefinition, Signal
from bevel.core.lifecycle import LifecycleDispatcher
from bevel.core.transitions import (
    Transition,
    TransitionConfig,
    TransitionState,
    FadeTransition,
    ImmediateTransition,
)
from bevel.input.input_context import InputContext, InputContextStack, InputAction
from bevel.rendering.layers import RenderLayer, LayerManager
from bevel.resources.resource_groups import ResourceGroup, ResourceGroupManager


@pytest.fixture
def node() -> Node:
    """Create a basic Node for testing."""
    return Node(name="TestNode")


@pytest.fixture
def node_with_children() -> Node:
    """Create a Node with a hierarchy of children."""
    root = Node(name="Root")
    child1 = Node(name="Child1")
    child2 = Node(name="Child2")
    grandchild = Node(name="Grandchild")

    root.add_child(child1)
    root.add_child(child2)
    child1.add_child(grandchild)

    return root


@pytest.fixture
def mock_scene() -> MagicMock:
    """Create a mock Scene object."""
    scene = MagicMock()
    scene._lifecycle = LifecycleDispatcher()
    scene.queue_deferred_call = MagicMock()
    return scene


@pytest.fixture
def lifecycle_dispatcher() -> LifecycleDispatcher:
    """Create a LifecycleDispatcher for testing."""
    return LifecycleDispatcher()


@pytest.fixture
def signal_manager() -> SignalManager:
    """Create a SignalManager with an owner node."""
    manager = SignalManager()
    owner = Node(name="Owner")
    manager.set_owner(owner)
    return manager


@pytest.fixture
def input_context_stack() -> InputContextStack:
    """Create an InputContextStack for testing."""
    return InputContextStack()


@pytest.fixture
def layer_manager() -> LayerManager:
    """Create a fresh LayerManager for testing."""
    manager = LayerManager()
    return manager


@pytest.fixture
def mock_resource_manager() -> MagicMock:
    """Create a mock ResourceManager."""
    manager = MagicMock()
    manager.get_texture = MagicMock(return_value=MagicMock())
    manager.get_sound = MagicMock(return_value=MagicMock())
    manager.get_font = MagicMock(return_value=MagicMock())
    manager.unload_texture = MagicMock()
    manager.unload_sound = MagicMock()
    manager.unload_font = MagicMock()
    return manager


@pytest.fixture
def resource_group_manager(mock_resource_manager: MagicMock) -> ResourceGroupManager:
    """Create a ResourceGroupManager for testing."""
    return ResourceGroupManager(mock_resource_manager)


@pytest.fixture
def transition_config() -> TransitionConfig:
    """Create a TransitionConfig for testing."""
    return TransitionConfig(
        duration=1.0,
        color=(0, 0, 0, 255),
    )


@pytest.fixture
def fade_transition(transition_config: TransitionConfig) -> FadeTransition:
    """Create a FadeTransition for testing."""
    return FadeTransition(transition_config)


@pytest.fixture
def mock_renderer() -> MagicMock:
    """Create a mock renderer with raylib."""
    renderer = MagicMock()
    rl = MagicMock()
    rl.GetScreenWidth = MagicMock(return_value=800)
    rl.GetScreenHeight = MagicMock(return_value=600)
    rl.DrawRectangle = MagicMock()
    renderer._raylib = rl
    return renderer


class MockScript:
    """A mock script class for testing lifecycle and multi-script support."""

    def __init__(self) -> None:
        self.node: Node | None = None
        self._priority: int = 0
        self.awake_called = False
        self.start_called = False
        self.on_enable_called = False
        self.on_disable_called = False
        self.update_called = False
        self.update_delta: float = 0.0
        self.fixed_update_called = False
        self.fixed_update_delta: float = 0.0
        self.late_update_called = False
        self.late_update_delta: float = 0.0
        self.on_destroy_called = False
        self.signal_received = False
        self.signal_args: tuple[Any, ...] = ()

    def awake(self) -> None:
        self.awake_called = True

    def start(self) -> None:
        self.start_called = True

    def on_enable(self) -> None:
        self.on_enable_called = True

    def on_disable(self) -> None:
        self.on_disable_called = True

    def update(self, delta: float) -> None:
        self.update_called = True
        self.update_delta = delta

    def fixed_update(self, delta: float) -> None:
        self.fixed_update_called = True
        self.fixed_update_delta = delta

    def late_update(self, delta: float) -> None:
        self.late_update_called = True
        self.late_update_delta = delta

    def on_destroy(self) -> None:
        self.on_destroy_called = True

    def on_signal(self, *args: Any) -> None:
        self.signal_received = True
        self.signal_args = args


@pytest.fixture
def mock_script() -> MockScript:
    """Create a MockScript for testing."""
    return MockScript()


@pytest.fixture
def temp_project_dir(tmp_path: Path) -> Path:
    """Create a temporary project directory structure."""
    project_dir = tmp_path / "test_project"
    project_dir.mkdir()

    # Create directories
    (project_dir / "scenes").mkdir()
    (project_dir / "scripts").mkdir()
    (project_dir / "assets").mkdir()

    # Create a simple scene file
    scene_content = """
scene_config:
  name: "Test Scene"
  type: "2D"
root:
  type: Node2D
  name: "Root"
  children:
    - type: Node2D
      name: "Child"
"""
    (project_dir / "scenes" / "main.yaml").write_text(scene_content)

    # Create a simple script
    script_content = """
class TestScript:
    def __init__(self):
        self.node = None
        self.value = 0

    def update(self, delta):
        self.value += 1
"""
    (project_dir / "scripts" / "test_script.py").write_text(script_content)

    return project_dir