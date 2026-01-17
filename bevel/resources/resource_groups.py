"""Resource group management for Bevel engine.

Groups allow batch loading/unloading of related resources,
useful for level streaming and memory management.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.resources.resource_manager import ResourceManager

logger = get_logger("resource_groups")


@dataclass
class ResourceGroup:
    """A named group of resources that can be loaded/unloaded together."""

    name: str
    textures: list[str] = field(default_factory=list)
    sounds: list[str] = field(default_factory=list)
    fonts: list[str] = field(default_factory=list)
    loaded: bool = False


class ResourceGroupManager:
    """Manages groups of resources for batch loading/unloading."""

    def __init__(self, resource_manager: "ResourceManager") -> None:
        """Initialize the group manager.

        Args:
            resource_manager: The ResourceManager to use for loading resources
        """
        self._resource_manager = resource_manager
        self._groups: dict[str, ResourceGroup] = {}

    def register_group(self, group: ResourceGroup) -> None:
        """Register a resource group.

        Args:
            group: The group to register
        """
        self._groups[group.name] = group
        logger.debug(
            f"Registered group '{group.name}' with "
            f"{len(group.textures)} textures, "
            f"{len(group.sounds)} sounds, "
            f"{len(group.fonts)} fonts"
        )

    def create_group(
        self,
        name: str,
        textures: list[str] | None = None,
        sounds: list[str] | None = None,
        fonts: list[str] | None = None,
    ) -> ResourceGroup:
        """Create and register a new resource group.

        Args:
            name: The group name
            textures: List of texture resource names
            sounds: List of sound resource names
            fonts: List of font resource names

        Returns:
            The created ResourceGroup
        """
        group = ResourceGroup(
            name=name,
            textures=textures or [],
            sounds=sounds or [],
            fonts=fonts or [],
        )
        self.register_group(group)
        return group

    def load_group(self, name: str) -> bool:
        """Load all resources in a group.

        Args:
            name: The group name to load

        Returns:
            True if all resources loaded successfully
        """
        if name not in self._groups:
            logger.warning(f"Cannot load unknown group: {name}")
            return False

        group = self._groups[name]

        if group.loaded:
            logger.debug(f"Group '{name}' already loaded")
            return True

        logger.info(f"Loading resource group: {name}")
        success = True

        # Load textures
        for tex_name in group.textures:
            texture = self._resource_manager.get_texture(tex_name)
            if texture is None:
                logger.warning(f"Failed to load texture '{tex_name}' in group '{name}'")
                success = False

        # Load sounds
        for sound_name in group.sounds:
            sound = self._resource_manager.get_sound(sound_name)
            if sound is None:
                logger.warning(f"Failed to load sound '{sound_name}' in group '{name}'")
                success = False

        # Load fonts
        for font_name in group.fonts:
            font = self._resource_manager.get_font(font_name)
            if font is None:
                logger.warning(f"Failed to load font '{font_name}' in group '{name}'")
                success = False

        group.loaded = True
        logger.info(f"Resource group '{name}' loaded (success={success})")
        return success

    def unload_group(self, name: str) -> bool:
        """Unload all resources in a group.

        Only unloads resources that aren't used by other loaded groups.

        Args:
            name: The group name to unload

        Returns:
            True if successful
        """
        if name not in self._groups:
            logger.warning(f"Cannot unload unknown group: {name}")
            return False

        group = self._groups[name]

        if not group.loaded:
            logger.debug(f"Group '{name}' not loaded")
            return True

        logger.info(f"Unloading resource group: {name}")

        # Get resources used by other loaded groups
        other_textures: set[str] = set()
        other_sounds: set[str] = set()
        other_fonts: set[str] = set()

        for other_name, other_group in self._groups.items():
            if other_name != name and other_group.loaded:
                other_textures.update(other_group.textures)
                other_sounds.update(other_group.sounds)
                other_fonts.update(other_group.fonts)

        # Unload textures not used by other groups
        for tex_name in group.textures:
            if tex_name not in other_textures:
                self._resource_manager.unload_texture(tex_name)

        # Unload sounds not used by other groups
        for sound_name in group.sounds:
            if sound_name not in other_sounds:
                self._resource_manager.unload_sound(sound_name)

        # Unload fonts not used by other groups
        for font_name in group.fonts:
            if font_name not in other_fonts:
                self._resource_manager.unload_font(font_name)

        group.loaded = False
        logger.info(f"Resource group '{name}' unloaded")
        return True

    def is_group_loaded(self, name: str) -> bool:
        """Check if a group is loaded.

        Args:
            name: The group name

        Returns:
            True if the group exists and is loaded
        """
        return name in self._groups and self._groups[name].loaded

    def get_group(self, name: str) -> ResourceGroup | None:
        """Get a group by name.

        Args:
            name: The group name

        Returns:
            The ResourceGroup or None if not found
        """
        return self._groups.get(name)

    def get_loaded_groups(self) -> list[str]:
        """Get names of all loaded groups.

        Returns:
            List of loaded group names
        """
        return [name for name, group in self._groups.items() if group.loaded]

    def get_all_groups(self) -> list[str]:
        """Get names of all registered groups.

        Returns:
            List of all group names
        """
        return list(self._groups.keys())

    def add_to_group(self, group_name: str, resource_name: str, resource_type: str) -> bool:
        """Add a resource to a group.

        Args:
            group_name: The group name
            resource_name: The resource name to add
            resource_type: Type of resource ("texture", "sound", or "font")

        Returns:
            True if added successfully
        """
        if group_name not in self._groups:
            logger.warning(f"Cannot add to unknown group: {group_name}")
            return False

        group = self._groups[group_name]

        if resource_type == "texture":
            if resource_name not in group.textures:
                group.textures.append(resource_name)
        elif resource_type == "sound":
            if resource_name not in group.sounds:
                group.sounds.append(resource_name)
        elif resource_type == "font":
            if resource_name not in group.fonts:
                group.fonts.append(resource_name)
        else:
            logger.warning(f"Unknown resource type: {resource_type}")
            return False

        return True

    def remove_from_group(self, group_name: str, resource_name: str, resource_type: str) -> bool:
        """Remove a resource from a group.

        Args:
            group_name: The group name
            resource_name: The resource name to remove
            resource_type: Type of resource ("texture", "sound", or "font")

        Returns:
            True if removed successfully
        """
        if group_name not in self._groups:
            logger.warning(f"Cannot remove from unknown group: {group_name}")
            return False

        group = self._groups[group_name]

        if resource_type == "texture":
            if resource_name in group.textures:
                group.textures.remove(resource_name)
        elif resource_type == "sound":
            if resource_name in group.sounds:
                group.sounds.remove(resource_name)
        elif resource_type == "font":
            if resource_name in group.fonts:
                group.fonts.remove(resource_name)
        else:
            logger.warning(f"Unknown resource type: {resource_type}")
            return False

        return True

    def clear(self) -> None:
        """Clear all groups (does not unload resources)."""
        self._groups.clear()
