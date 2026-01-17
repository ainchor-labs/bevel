"""Tests for the Scene Layers System.

Tests cover:
- RenderLayer creation and configuration
- LayerManager registration and lookup
- Layer z-ordering and sorting
- Layer visibility
- Camera-independent layers (for UI)
- Node grouping by layer
- Configuration loading from YAML-like dicts
"""

import pytest

from bevel.core.node import Node
from bevel.rendering.layers import (
    RenderLayer,
    LayerManager,
    get_layer_manager,
    set_layer_manager,
)


class TestRenderLayer:
    """Tests for RenderLayer dataclass."""

    def test_create_layer(self) -> None:
        """Test creating a render layer."""
        layer = RenderLayer(
            name="world",
            z_index=0,
            independent_transform=False,
            visible=True,
        )
        assert layer.name == "world"
        assert layer.z_index == 0
        assert layer.independent_transform is False
        assert layer.visible is True

    def test_default_values(self) -> None:
        """Test default layer values."""
        layer = RenderLayer(name="test")
        assert layer.z_index == 0
        assert layer.independent_transform is False
        assert layer.visible is True

    def test_ui_layer(self) -> None:
        """Test creating a UI layer with independent transform."""
        layer = RenderLayer(
            name="ui",
            z_index=1000,
            independent_transform=True,
        )
        assert layer.independent_transform is True


class TestLayerManager:
    """Tests for LayerManager class."""

    def test_default_layers(self, layer_manager: LayerManager) -> None:
        """Test default layers are created."""
        # LayerManager creates default layers in __init__
        assert layer_manager.get_layer("background") is not None
        assert layer_manager.get_layer("world") is not None
        assert layer_manager.get_layer("foreground") is not None
        assert layer_manager.get_layer("ui") is not None

    def test_default_layer_z_indices(self, layer_manager: LayerManager) -> None:
        """Test default layer z-indices are correct."""
        assert layer_manager.get_layer_z_index("background") == -1000
        assert layer_manager.get_layer_z_index("world") == 0
        assert layer_manager.get_layer_z_index("foreground") == 500
        assert layer_manager.get_layer_z_index("ui") == 1000

    def test_ui_is_camera_independent(self, layer_manager: LayerManager) -> None:
        """Test UI layer has independent transform by default."""
        assert layer_manager.is_camera_independent("ui") is True
        assert layer_manager.is_camera_independent("world") is False

    def test_register_layer(self, layer_manager: LayerManager) -> None:
        """Test registering a new layer."""
        custom = RenderLayer(name="custom", z_index=100)
        layer_manager.register_layer(custom)

        layer = layer_manager.get_layer("custom")
        assert layer is not None
        assert layer.z_index == 100

    def test_get_layer_not_found(self, layer_manager: LayerManager) -> None:
        """Test getting non-existent layer returns None."""
        layer = layer_manager.get_layer("nonexistent")
        assert layer is None

    def test_get_layer_z_index_not_found(self, layer_manager: LayerManager) -> None:
        """Test getting z_index of non-existent layer returns 0."""
        z = layer_manager.get_layer_z_index("nonexistent")
        assert z == 0

    def test_is_camera_independent_not_found(
        self, layer_manager: LayerManager
    ) -> None:
        """Test checking independent_transform of non-existent layer returns False."""
        result = layer_manager.is_camera_independent("nonexistent")
        assert result is False


class TestLayerSorting:
    """Tests for layer sorting by z-index."""

    def test_get_sorted_layers(self, layer_manager: LayerManager) -> None:
        """Test layers are sorted by z-index (lowest first)."""
        sorted_layers = layer_manager.get_sorted_layers()

        z_indices = [layer.z_index for layer in sorted_layers]
        assert z_indices == sorted(z_indices)

    def test_sorted_layers_order(self, layer_manager: LayerManager) -> None:
        """Test default layers appear in correct order."""
        sorted_layers = layer_manager.get_sorted_layers()
        names = [layer.name for layer in sorted_layers]

        # Background should be first (lowest z), UI last (highest z)
        assert names.index("background") < names.index("world")
        assert names.index("world") < names.index("foreground")
        assert names.index("foreground") < names.index("ui")

    def test_custom_layer_sorting(self, layer_manager: LayerManager) -> None:
        """Test custom layers are sorted correctly."""
        layer_manager.register_layer(RenderLayer(name="effects", z_index=250))

        sorted_layers = layer_manager.get_sorted_layers()
        names = [layer.name for layer in sorted_layers]

        # Effects should be between world (0) and foreground (500)
        assert names.index("world") < names.index("effects")
        assert names.index("effects") < names.index("foreground")

    def test_sorted_layers_cached(self, layer_manager: LayerManager) -> None:
        """Test sorted layers are cached and invalidated correctly."""
        sorted1 = layer_manager.get_sorted_layers()
        sorted2 = layer_manager.get_sorted_layers()

        # Same object (cached)
        assert sorted1 is sorted2

        # Register new layer - cache should be invalidated
        layer_manager.register_layer(RenderLayer(name="new", z_index=50))
        sorted3 = layer_manager.get_sorted_layers()

        assert sorted3 is not sorted1


class TestLayerVisibility:
    """Tests for layer visibility control."""

    def test_layer_visible_by_default(self, layer_manager: LayerManager) -> None:
        """Test layers are visible by default."""
        assert layer_manager.is_layer_visible("world") is True

    def test_set_layer_visible(self, layer_manager: LayerManager) -> None:
        """Test setting layer visibility."""
        layer_manager.set_layer_visible("world", False)
        assert layer_manager.is_layer_visible("world") is False

        layer_manager.set_layer_visible("world", True)
        assert layer_manager.is_layer_visible("world") is True

    def test_is_layer_visible_not_found(self, layer_manager: LayerManager) -> None:
        """Test checking visibility of non-existent layer returns True."""
        result = layer_manager.is_layer_visible("nonexistent")
        assert result is True

    def test_set_visibility_unknown_layer(self, layer_manager: LayerManager) -> None:
        """Test setting visibility of unknown layer does nothing."""
        layer_manager.set_layer_visible("unknown", False)
        # Should not raise, just do nothing


class TestNodeGroupingByLayer:
    """Tests for grouping nodes by their layer."""

    def test_group_nodes_by_layer(self, layer_manager: LayerManager) -> None:
        """Test grouping nodes by layer."""
        node1 = Node(name="Node1")
        node1.layer = "world"
        node2 = Node(name="Node2")
        node2.layer = "world"
        node3 = Node(name="Node3")
        node3.layer = "ui"

        groups = layer_manager.group_nodes_by_layer([node1, node2, node3])

        assert "world" in groups
        assert "ui" in groups
        assert len(groups["world"]) == 2
        assert len(groups["ui"]) == 1
        assert node1 in groups["world"]
        assert node3 in groups["ui"]

    def test_group_nodes_default_layer(self, layer_manager: LayerManager) -> None:
        """Test nodes default to 'world' layer."""
        node = Node(name="Test")
        # Default layer is "world"
        assert node.layer == "world"

        groups = layer_manager.group_nodes_by_layer([node])
        assert "world" in groups
        assert node in groups["world"]

    def test_group_nodes_empty_list(self, layer_manager: LayerManager) -> None:
        """Test grouping empty node list."""
        groups = layer_manager.group_nodes_by_layer([])
        assert groups == {}

    def test_group_nodes_custom_layer(self, layer_manager: LayerManager) -> None:
        """Test grouping nodes with custom layer."""
        layer_manager.register_layer(RenderLayer(name="custom", z_index=100))

        node = Node(name="Test")
        node.layer = "custom"

        groups = layer_manager.group_nodes_by_layer([node])
        assert "custom" in groups
        assert node in groups["custom"]


class TestConfigLoading:
    """Tests for loading layers from configuration."""

    def test_load_from_config(self, layer_manager: LayerManager) -> None:
        """Test loading layers from config list."""
        config = [
            {"name": "particles", "z_index": 750, "visible": True},
            {"name": "debug", "z_index": 2000, "independent_transform": True},
        ]

        layer_manager.load_from_config(config)

        particles = layer_manager.get_layer("particles")
        assert particles is not None
        assert particles.z_index == 750

        debug = layer_manager.get_layer("debug")
        assert debug is not None
        assert debug.independent_transform is True

    def test_load_config_missing_name(self, layer_manager: LayerManager) -> None:
        """Test loading config without name is skipped."""
        config = [
            {"z_index": 100},  # Missing name
            {"name": "valid", "z_index": 200},
        ]

        layer_manager.load_from_config(config)

        assert layer_manager.get_layer("valid") is not None

    def test_load_config_defaults(self, layer_manager: LayerManager) -> None:
        """Test loading config with minimal values uses defaults."""
        config = [
            {"name": "minimal"},
        ]

        layer_manager.load_from_config(config)

        layer = layer_manager.get_layer("minimal")
        assert layer is not None
        assert layer.z_index == 0
        assert layer.independent_transform is False
        assert layer.visible is True


class TestClearLayers:
    """Tests for clearing layers."""

    def test_clear_layers(self, layer_manager: LayerManager) -> None:
        """Test clearing all layers."""
        # Verify layers exist
        assert layer_manager.get_layer("world") is not None

        layer_manager.clear_layers()

        # All layers should be gone
        assert layer_manager.get_layer("world") is None
        assert layer_manager.get_layer("ui") is None
        assert layer_manager.get_sorted_layers() == []


class TestGlobalLayerManager:
    """Tests for global layer manager functions."""

    def test_get_layer_manager(self) -> None:
        """Test getting global layer manager."""
        manager = get_layer_manager()
        assert isinstance(manager, LayerManager)

    def test_get_layer_manager_singleton(self) -> None:
        """Test global manager is a singleton."""
        manager1 = get_layer_manager()
        manager2 = get_layer_manager()
        assert manager1 is manager2

    def test_set_layer_manager(self) -> None:
        """Test setting custom global layer manager."""
        custom = LayerManager()
        custom.register_layer(RenderLayer(name="custom_layer", z_index=999))

        set_layer_manager(custom)

        manager = get_layer_manager()
        assert manager is custom
        assert manager.get_layer("custom_layer") is not None


class TestNodeLayerProperty:
    """Tests for layer property on Node class."""

    def test_node_default_layer(self) -> None:
        """Test node has default layer 'world'."""
        node = Node(name="Test")
        assert node.layer == "world"

    def test_node_set_layer(self) -> None:
        """Test setting node layer."""
        node = Node(name="Test")
        node.layer = "ui"
        assert node.layer == "ui"

    def test_node_layer_in_hierarchy(self) -> None:
        """Test different layers in node hierarchy."""
        root = Node(name="Root")
        root.layer = "world"

        child = Node(name="Child")
        child.layer = "ui"
        root.add_child(child)

        # Layers are independent
        assert root.layer == "world"
        assert child.layer == "ui"


class TestEdgeCases:
    """Tests for edge cases."""

    def test_overwrite_existing_layer(self, layer_manager: LayerManager) -> None:
        """Test registering layer with existing name overwrites it."""
        original = layer_manager.get_layer("world")
        assert original is not None
        original_z = original.z_index

        new_layer = RenderLayer(name="world", z_index=9999)
        layer_manager.register_layer(new_layer)

        updated = layer_manager.get_layer("world")
        assert updated is not None
        assert updated.z_index == 9999
        assert updated.z_index != original_z

    def test_negative_z_index(self, layer_manager: LayerManager) -> None:
        """Test layers can have negative z-index."""
        layer = RenderLayer(name="far_back", z_index=-5000)
        layer_manager.register_layer(layer)

        sorted_layers = layer_manager.get_sorted_layers()
        assert sorted_layers[0].name == "far_back"

    def test_very_large_z_index(self, layer_manager: LayerManager) -> None:
        """Test layers can have very large z-index."""
        layer = RenderLayer(name="front", z_index=1000000)
        layer_manager.register_layer(layer)

        sorted_layers = layer_manager.get_sorted_layers()
        assert sorted_layers[-1].name == "front"
