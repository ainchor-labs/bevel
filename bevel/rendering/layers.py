"""Rendering layer system for Bevel engine.

Layers allow organizing nodes into distinct rendering groups with
different z-ordering and camera handling (e.g., UI layers that don't
move with the camera).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.node import Node

logger = get_logger("layers")


@dataclass
class RenderLayer:
    """A rendering layer with a name and z-index."""

    name: str
    z_index: int = 0
    independent_transform: bool = False  # If True, ignores camera transform (for UI)
    visible: bool = True


class LayerManager:
    """Manages render layers for the engine."""

    def __init__(self) -> None:
        """Initialize the layer manager with default layers."""
        self._layers: dict[str, RenderLayer] = {}
        self._sorted_layers: list[RenderLayer] | None = None

        # Register default layers
        self.register_layer(RenderLayer("background", z_index=-1000))
        self.register_layer(RenderLayer("world", z_index=0))
        self.register_layer(RenderLayer("foreground", z_index=500))
        self.register_layer(
            RenderLayer("ui", z_index=1000, independent_transform=True)
        )

    def register_layer(self, layer: RenderLayer) -> None:
        """Register a render layer.

        Args:
            layer: The layer to register
        """
        self._layers[layer.name] = layer
        self._sorted_layers = None  # Invalidate cache
        logger.debug(f"Registered layer '{layer.name}' (z_index={layer.z_index})")

    def get_layer(self, name: str) -> RenderLayer | None:
        """Get a layer by name.

        Args:
            name: The layer name

        Returns:
            The layer or None if not found
        """
        return self._layers.get(name)

    def get_sorted_layers(self) -> list[RenderLayer]:
        """Get layers sorted by z_index.

        Returns:
            List of layers sorted by z_index (lowest first)
        """
        if self._sorted_layers is None:
            self._sorted_layers = sorted(
                self._layers.values(), key=lambda l: l.z_index
            )
        return self._sorted_layers

    def set_layer_visible(self, name: str, visible: bool) -> None:
        """Set layer visibility.

        Args:
            name: The layer name
            visible: Whether the layer should be visible
        """
        layer = self._layers.get(name)
        if layer is not None:
            layer.visible = visible

    def is_layer_visible(self, name: str) -> bool:
        """Check if a layer is visible.

        Args:
            name: The layer name

        Returns:
            True if visible (defaults to True if layer not found)
        """
        layer = self._layers.get(name)
        return layer.visible if layer is not None else True

    def is_camera_independent(self, name: str) -> bool:
        """Check if a layer has independent transform (ignores camera).

        Args:
            name: The layer name

        Returns:
            True if independent_transform is set
        """
        layer = self._layers.get(name)
        return layer.independent_transform if layer is not None else False

    def get_layer_z_index(self, name: str) -> int:
        """Get the z_index of a layer.

        Args:
            name: The layer name

        Returns:
            The layer's z_index (defaults to 0 if layer not found)
        """
        layer = self._layers.get(name)
        return layer.z_index if layer is not None else 0

    def group_nodes_by_layer(
        self, nodes: list["Node"]
    ) -> dict[str, list["Node"]]:
        """Group nodes by their layer.

        Args:
            nodes: List of nodes to group

        Returns:
            Dictionary mapping layer names to lists of nodes
        """
        groups: dict[str, list["Node"]] = {}

        for node in nodes:
            layer_name = getattr(node, "layer", "world")
            if layer_name not in groups:
                groups[layer_name] = []
            groups[layer_name].append(node)

        return groups

    def clear_layers(self) -> None:
        """Remove all layers."""
        self._layers.clear()
        self._sorted_layers = None

    def load_from_config(self, config: list[dict]) -> None:
        """Load layers from configuration.

        Args:
            config: List of layer definitions from YAML
        """
        for layer_data in config:
            name = layer_data.get("name")
            if name is None:
                continue

            layer = RenderLayer(
                name=name,
                z_index=layer_data.get("z_index", 0),
                independent_transform=layer_data.get("independent_transform", False),
                visible=layer_data.get("visible", True),
            )
            self.register_layer(layer)


# Global layer manager instance
_layer_manager: LayerManager | None = None


def get_layer_manager() -> LayerManager:
    """Get the global layer manager instance.

    Returns:
        The global LayerManager
    """
    global _layer_manager
    if _layer_manager is None:
        _layer_manager = LayerManager()
    return _layer_manager


def set_layer_manager(manager: LayerManager) -> None:
    """Set the global layer manager instance.

    Args:
        manager: The LayerManager to use globally
    """
    global _layer_manager
    _layer_manager = manager
