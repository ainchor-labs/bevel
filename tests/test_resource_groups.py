"""Tests for the Resource Groups System.

Tests cover:
- ResourceGroup creation
- ResourceGroupManager operations
- Group loading and unloading
- Smart unloading (shared resources)
- Group queries
- Dynamic group modification
"""

import pytest
from unittest.mock import MagicMock

from bevel.resources.resource_groups import ResourceGroup, ResourceGroupManager


class TestResourceGroup:
    """Tests for ResourceGroup dataclass."""

    def test_create_group(self) -> None:
        """Test creating a resource group."""
        group = ResourceGroup(
            name="level_1",
            textures=["player", "tileset"],
            sounds=["jump", "collect"],
            fonts=["main_font"],
            loaded=False,
        )
        assert group.name == "level_1"
        assert group.textures == ["player", "tileset"]
        assert group.sounds == ["jump", "collect"]
        assert group.fonts == ["main_font"]
        assert group.loaded is False

    def test_default_values(self) -> None:
        """Test default empty lists."""
        group = ResourceGroup(name="empty")
        assert group.textures == []
        assert group.sounds == []
        assert group.fonts == []
        assert group.loaded is False


class TestResourceGroupManager:
    """Tests for ResourceGroupManager class."""

    def test_register_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test registering a resource group."""
        group = ResourceGroup(
            name="test",
            textures=["tex1"],
        )
        resource_group_manager.register_group(group)

        result = resource_group_manager.get_group("test")
        assert result is not None
        assert result.name == "test"

    def test_create_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test creating and registering a group in one step."""
        group = resource_group_manager.create_group(
            name="level_1",
            textures=["player", "enemy"],
            sounds=["bgm"],
        )

        assert group.name == "level_1"
        assert "player" in group.textures
        assert "bgm" in group.sounds

        # Should also be registered
        assert resource_group_manager.get_group("level_1") is not None

    def test_get_group_not_found(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test getting non-existent group returns None."""
        result = resource_group_manager.get_group("nonexistent")
        assert result is None

    def test_get_all_groups(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test getting all registered group names."""
        resource_group_manager.create_group("group1")
        resource_group_manager.create_group("group2")
        resource_group_manager.create_group("group3")

        all_groups = resource_group_manager.get_all_groups()
        assert "group1" in all_groups
        assert "group2" in all_groups
        assert "group3" in all_groups


class TestGroupLoading:
    """Tests for loading resource groups."""

    def test_load_group(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test loading a group loads all its resources."""
        resource_group_manager.create_group(
            name="test",
            textures=["tex1", "tex2"],
            sounds=["sound1"],
            fonts=["font1"],
        )

        result = resource_group_manager.load_group("test")
        assert result is True

        # Verify resources were requested
        mock_resource_manager.get_texture.assert_any_call("tex1")
        mock_resource_manager.get_texture.assert_any_call("tex2")
        mock_resource_manager.get_sound.assert_called_with("sound1")
        mock_resource_manager.get_font.assert_called_with("font1")

    def test_load_group_marks_loaded(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test loading marks group as loaded."""
        resource_group_manager.create_group(name="test", textures=["tex1"])

        resource_group_manager.load_group("test")

        assert resource_group_manager.is_group_loaded("test") is True

    def test_load_already_loaded(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test loading already loaded group does nothing."""
        resource_group_manager.create_group(name="test", textures=["tex1"])
        resource_group_manager.load_group("test")

        # Reset mock to track new calls
        mock_resource_manager.get_texture.reset_mock()

        # Load again
        result = resource_group_manager.load_group("test")
        assert result is True
        mock_resource_manager.get_texture.assert_not_called()

    def test_load_unknown_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test loading unknown group returns False."""
        result = resource_group_manager.load_group("unknown")
        assert result is False

    def test_load_group_with_failed_resource(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test loading group with failed resource returns False but continues."""
        mock_resource_manager.get_texture.return_value = None

        resource_group_manager.create_group(
            name="test",
            textures=["missing_tex"],
        )

        result = resource_group_manager.load_group("test")
        assert result is False
        # Group should still be marked loaded
        assert resource_group_manager.is_group_loaded("test") is True


class TestGroupUnloading:
    """Tests for unloading resource groups."""

    def test_unload_group(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test unloading a group unloads its resources."""
        resource_group_manager.create_group(
            name="test",
            textures=["tex1"],
            sounds=["sound1"],
            fonts=["font1"],
        )
        resource_group_manager.load_group("test")

        result = resource_group_manager.unload_group("test")
        assert result is True

        mock_resource_manager.unload_texture.assert_called_with("tex1")
        mock_resource_manager.unload_sound.assert_called_with("sound1")
        mock_resource_manager.unload_font.assert_called_with("font1")

    def test_unload_marks_unloaded(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test unloading marks group as not loaded."""
        resource_group_manager.create_group(name="test", textures=["tex1"])
        resource_group_manager.load_group("test")

        resource_group_manager.unload_group("test")

        assert resource_group_manager.is_group_loaded("test") is False

    def test_unload_not_loaded(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test unloading non-loaded group does nothing."""
        resource_group_manager.create_group(name="test", textures=["tex1"])

        result = resource_group_manager.unload_group("test")
        assert result is True
        mock_resource_manager.unload_texture.assert_not_called()

    def test_unload_unknown_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test unloading unknown group returns False."""
        result = resource_group_manager.unload_group("unknown")
        assert result is False


class TestSharedResourceUnloading:
    """Tests for smart unloading with shared resources."""

    def test_shared_resource_not_unloaded(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test shared resources are not unloaded when one group is unloaded."""
        # Two groups share "shared_tex"
        resource_group_manager.create_group(
            name="group1",
            textures=["shared_tex", "unique1"],
        )
        resource_group_manager.create_group(
            name="group2",
            textures=["shared_tex", "unique2"],
        )

        resource_group_manager.load_group("group1")
        resource_group_manager.load_group("group2")

        # Unload group1
        resource_group_manager.unload_group("group1")

        # shared_tex should NOT be unloaded (still used by group2)
        unload_calls = [
            call[0][0] for call in mock_resource_manager.unload_texture.call_args_list
        ]
        assert "shared_tex" not in unload_calls
        assert "unique1" in unload_calls

    def test_shared_resource_unloaded_when_all_groups_unloaded(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test shared resources are unloaded when all using groups are unloaded."""
        resource_group_manager.create_group(
            name="group1",
            textures=["shared_tex"],
        )
        resource_group_manager.create_group(
            name="group2",
            textures=["shared_tex"],
        )

        resource_group_manager.load_group("group1")
        resource_group_manager.load_group("group2")

        resource_group_manager.unload_group("group1")
        resource_group_manager.unload_group("group2")

        # Now shared_tex should be unloaded
        unload_calls = [
            call[0][0] for call in mock_resource_manager.unload_texture.call_args_list
        ]
        assert "shared_tex" in unload_calls


class TestGroupQueries:
    """Tests for querying group states."""

    def test_is_group_loaded(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test checking if group is loaded."""
        resource_group_manager.create_group(name="test")

        assert resource_group_manager.is_group_loaded("test") is False

        resource_group_manager.load_group("test")
        assert resource_group_manager.is_group_loaded("test") is True

    def test_is_group_loaded_unknown(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test checking loaded state of unknown group."""
        assert resource_group_manager.is_group_loaded("unknown") is False

    def test_get_loaded_groups(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test getting list of loaded groups."""
        resource_group_manager.create_group(name="group1")
        resource_group_manager.create_group(name="group2")
        resource_group_manager.create_group(name="group3")

        resource_group_manager.load_group("group1")
        resource_group_manager.load_group("group3")

        loaded = resource_group_manager.get_loaded_groups()
        assert "group1" in loaded
        assert "group3" in loaded
        assert "group2" not in loaded


class TestDynamicGroupModification:
    """Tests for dynamically modifying groups."""

    def test_add_to_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test adding a resource to a group."""
        resource_group_manager.create_group(name="test")

        result = resource_group_manager.add_to_group("test", "new_tex", "texture")
        assert result is True

        group = resource_group_manager.get_group("test")
        assert group is not None
        assert "new_tex" in group.textures

    def test_add_to_group_sound(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test adding a sound to a group."""
        resource_group_manager.create_group(name="test")

        resource_group_manager.add_to_group("test", "new_sound", "sound")

        group = resource_group_manager.get_group("test")
        assert group is not None
        assert "new_sound" in group.sounds

    def test_add_to_group_font(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test adding a font to a group."""
        resource_group_manager.create_group(name="test")

        resource_group_manager.add_to_group("test", "new_font", "font")

        group = resource_group_manager.get_group("test")
        assert group is not None
        assert "new_font" in group.fonts

    def test_add_to_group_unknown_type(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test adding unknown resource type returns False."""
        resource_group_manager.create_group(name="test")

        result = resource_group_manager.add_to_group("test", "res", "unknown_type")
        assert result is False

    def test_add_to_unknown_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test adding to unknown group returns False."""
        result = resource_group_manager.add_to_group("unknown", "res", "texture")
        assert result is False

    def test_add_duplicate_ignored(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test adding duplicate resource doesn't create duplicate."""
        resource_group_manager.create_group(name="test", textures=["tex1"])

        resource_group_manager.add_to_group("test", "tex1", "texture")

        group = resource_group_manager.get_group("test")
        assert group is not None
        assert group.textures.count("tex1") == 1

    def test_remove_from_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test removing a resource from a group."""
        resource_group_manager.create_group(
            name="test",
            textures=["tex1", "tex2"],
        )

        result = resource_group_manager.remove_from_group("test", "tex1", "texture")
        assert result is True

        group = resource_group_manager.get_group("test")
        assert group is not None
        assert "tex1" not in group.textures
        assert "tex2" in group.textures

    def test_remove_from_group_unknown_type(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test removing unknown resource type returns False."""
        resource_group_manager.create_group(name="test")

        result = resource_group_manager.remove_from_group("test", "res", "unknown")
        assert result is False

    def test_remove_from_unknown_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test removing from unknown group returns False."""
        result = resource_group_manager.remove_from_group("unknown", "res", "texture")
        assert result is False


class TestGroupClear:
    """Tests for clearing groups."""

    def test_clear(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test clearing all groups."""
        resource_group_manager.create_group(name="group1")
        resource_group_manager.create_group(name="group2")
        resource_group_manager.load_group("group1")

        resource_group_manager.clear()

        assert resource_group_manager.get_all_groups() == []
        assert resource_group_manager.get_loaded_groups() == []


class TestEdgeCases:
    """Tests for edge cases."""

    def test_empty_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test loading/unloading empty group."""
        resource_group_manager.create_group(name="empty")

        result = resource_group_manager.load_group("empty")
        assert result is True
        assert resource_group_manager.is_group_loaded("empty") is True

        result = resource_group_manager.unload_group("empty")
        assert result is True
        assert resource_group_manager.is_group_loaded("empty") is False

    def test_many_resources_in_group(
        self,
        resource_group_manager: ResourceGroupManager,
        mock_resource_manager: MagicMock,
    ) -> None:
        """Test group with many resources."""
        textures = [f"tex_{i}" for i in range(100)]
        resource_group_manager.create_group(name="large", textures=textures)

        result = resource_group_manager.load_group("large")
        assert result is True
        assert mock_resource_manager.get_texture.call_count == 100

    def test_overwrite_group(
        self,
        resource_group_manager: ResourceGroupManager,
    ) -> None:
        """Test registering group with same name overwrites."""
        resource_group_manager.create_group(name="test", textures=["old"])

        new_group = ResourceGroup(name="test", textures=["new"])
        resource_group_manager.register_group(new_group)

        group = resource_group_manager.get_group("test")
        assert group is not None
        assert group.textures == ["new"]
