"""Resource management for Bevel engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TYPE_CHECKING

import yaml

from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.resources.resource_groups import ResourceGroupManager

logger = get_logger("resources")


@dataclass
class TextureResource:
    """A texture resource."""

    name: str
    path: str
    data: Any = None
    loaded: bool = False


@dataclass
class SoundResource:
    """A sound resource."""

    name: str
    path: str
    data: Any = None
    loaded: bool = False


@dataclass
class FontResource:
    """A font resource."""

    name: str
    path: str
    size: int = 20
    data: Any = None
    loaded: bool = False


class ResourceManager:
    """Manages game resources with lazy loading and caching."""

    def __init__(self, project_root: Path | None = None) -> None:
        """Initialize the resource manager.

        Args:
            project_root: The project root directory
        """
        self.project_root = project_root or Path.cwd()
        self._textures: dict[str, TextureResource] = {}
        self._sounds: dict[str, SoundResource] = {}
        self._fonts: dict[str, FontResource] = {}
        self._raylib: Any = None
        self._group_manager: "ResourceGroupManager | None" = None
        self._preload_list: list[str] = []  # Resources to preload

    def set_raylib(self, raylib: Any) -> None:
        """Set the raylib module reference.

        Args:
            raylib: The raylib module
        """
        self._raylib = raylib

    def load_resources(self, path: str | Path) -> bool:
        """Load resource definitions from a YAML file.

        Args:
            path: Path to resources.yaml

        Returns:
            True if successful
        """
        file_path = self.project_root / path
        if not file_path.exists():
            logger.warning(f"Resources file not found: {file_path}")
            return False

        try:
            with open(file_path, "r") as f:
                data = yaml.safe_load(f)
                
            # Handle empty YAML files
            if data is None:
                logger.debug(f"Empty or null YAML file: {file_path}")
                data = {}
                
        except yaml.YAMLError as e:
            logger.error(f"YAML parsing error in {file_path}: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to load resources file: {e}")
            return False

        # Parse textures
        textures_data = data.get("textures", {}) or {}
        for name, tex_path in textures_data.items():
            self._textures[name] = TextureResource(name=name, path=tex_path)
            logger.debug(f"Registered texture: {name}")

        # Parse sounds
        sounds_data = data.get("sounds", {}) or {}
        for name, sound_path in sounds_data.items():
            self._sounds[name] = SoundResource(name=name, path=sound_path)
            logger.debug(f"Registered sound: {name}")

        # Parse fonts
        fonts_data = data.get("fonts", {}) or {}
        for name, font_info in fonts_data.items():
            if isinstance(font_info, str):
                self._fonts[name] = FontResource(name=name, path=font_info)
            else:
                self._fonts[name] = FontResource(
                    name=name,
                    path=font_info.get("path", ""),
                    size=font_info.get("size", 20)
                )
            logger.debug(f"Registered font: {name}")

        # Parse groups
        groups_data = data.get("groups", {})
        if groups_data and self._group_manager is not None:
            self._load_groups(groups_data)

        # Store preload list for later
        self._preload_list = data.get("preload", [])

        logger.info(
            f"Registered {len(self._textures)} textures, "
            f"{len(self._sounds)} sounds, {len(self._fonts)} fonts"
        )
        return True


    def _load_groups(self, groups_data: dict[str, Any]) -> None:
        """Load resource groups from configuration.

        Args:
            groups_data: Dictionary of group definitions
        """
        if self._group_manager is None:
            return

        from bevel.resources.resource_groups import ResourceGroup

        for group_name, resources in groups_data.items():
            textures: list[str] = []
            sounds: list[str] = []
            fonts: list[str] = []

            # resources can be a list of names or a dict with types
            if isinstance(resources, list):
                # Simple list - classify by what's registered
                for res_name in resources:
                    if res_name in self._textures:
                        textures.append(res_name)
                    elif res_name in self._sounds:
                        sounds.append(res_name)
                    elif res_name in self._fonts:
                        fonts.append(res_name)
                    else:
                        logger.warning(
                            f"Unknown resource '{res_name}' in group '{group_name}'"
                        )
            elif isinstance(resources, dict):
                # Dict with explicit types
                textures = resources.get("textures", [])
                sounds = resources.get("sounds", [])
                fonts = resources.get("fonts", [])

            group = ResourceGroup(
                name=group_name,
                textures=textures,
                sounds=sounds,
                fonts=fonts,
            )
            self._group_manager.register_group(group)
            logger.debug(f"Loaded group '{group_name}'")

    def set_group_manager(self, manager: "ResourceGroupManager") -> None:
        """Set the resource group manager.

        Args:
            manager: The ResourceGroupManager instance
        """
        self._group_manager = manager

    @property
    def group_manager(self) -> "ResourceGroupManager | None":
        """Get the resource group manager.

        Returns:
            The ResourceGroupManager or None
        """
        return self._group_manager

    def get_texture(self, name: str) -> Any:
        """Get a texture by name (lazy loading).

        Args:
            name: The texture name

        Returns:
            The loaded texture or None
        """
        if name not in self._textures:
            logger.warning(f"Texture not found: {name}")
            return None

        resource = self._textures[name]

        # Lazy load if not loaded
        if not resource.loaded:
            self._load_texture(resource)

        return resource.data

    def _load_texture(self, resource: TextureResource) -> bool:
        """Actually load a texture.

        Args:
            resource: The texture resource to load

        Returns:
            True if successful
        """
        if self._raylib is None:
            logger.error("Raylib not set, cannot load textures")
            return False

        full_path = self.project_root / resource.path
        if not full_path.exists():
            logger.error(f"Texture file not found: {full_path}")
            return False

        try:
            texture = self._raylib.LoadTexture(str(full_path).encode())
            if texture.id == 0:
                logger.error(f"Failed to load texture: {resource.path}")
                return False

            resource.data = texture
            resource.loaded = True
            logger.debug(f"Loaded texture: {resource.name} ({texture.width}x{texture.height})")
            return True

        except Exception as e:
            logger.error(f"Error loading texture '{resource.name}': {e}")
            return False

    def get_sound(self, name: str) -> Any:
        """Get a sound by name (lazy loading).

        Args:
            name: The sound name

        Returns:
            The loaded sound or None
        """
        if name not in self._sounds:
            logger.warning(f"Sound not found: {name}")
            return None

        resource = self._sounds[name]

        if not resource.loaded:
            self._load_sound(resource)

        return resource.data

    def _load_sound(self, resource: SoundResource) -> bool:
        """Actually load a sound.

        Args:
            resource: The sound resource to load

        Returns:
            True if successful
        """
        if self._raylib is None:
            logger.error("Raylib not set, cannot load sounds")
            return False

        full_path = self.project_root / resource.path
        if not full_path.exists():
            logger.error(f"Sound file not found: {full_path}")
            return False

        try:
            sound = self._raylib.LoadSound(str(full_path).encode())
            resource.data = sound
            resource.loaded = True
            logger.debug(f"Loaded sound: {resource.name}")
            return True

        except Exception as e:
            logger.error(f"Error loading sound '{resource.name}': {e}")
            return False

    def get_font(self, name: str) -> Any:
        """Get a font by name (lazy loading).

        Args:
            name: The font name

        Returns:
            The loaded font or None
        """
        if name not in self._fonts:
            logger.warning(f"Font not found: {name}")
            return None

        resource = self._fonts[name]

        if not resource.loaded:
            self._load_font(resource)

        return resource.data

    def _load_font(self, resource: FontResource) -> bool:
        """Actually load a font.

        Args:
            resource: The font resource to load

        Returns:
            True if successful
        """
        if self._raylib is None:
            logger.error("Raylib not set, cannot load fonts")
            return False

        full_path = self.project_root / resource.path
        if not full_path.exists():
            logger.error(f"Font file not found: {full_path}")
            return False

        try:
            font = self._raylib.LoadFontEx(
                str(full_path).encode(),
                resource.size,
                None,
                0
            )
            resource.data = font
            resource.loaded = True
            logger.debug(f"Loaded font: {resource.name}")
            return True

        except Exception as e:
            logger.error(f"Error loading font '{resource.name}': {e}")
            return False

    def preload_all(self) -> None:
        """Preload all registered resources."""
        logger.info("Preloading all resources...")

        for resource in self._textures.values():
            if not resource.loaded:
                self._load_texture(resource)

        for resource in self._sounds.values():
            if not resource.loaded:
                self._load_sound(resource)

        for resource in self._fonts.values():
            if not resource.loaded:
                self._load_font(resource)

        logger.info("Preloading complete")

    def unload_all(self) -> None:
        """Unload all loaded resources."""
        if self._raylib is None:
            return

        for resource in self._textures.values():
            if resource.loaded and resource.data is not None:
                self._raylib.UnloadTexture(resource.data)
                resource.data = None
                resource.loaded = False

        for resource in self._sounds.values():
            if resource.loaded and resource.data is not None:
                self._raylib.UnloadSound(resource.data)
                resource.data = None
                resource.loaded = False

        for resource in self._fonts.values():
            if resource.loaded and resource.data is not None:
                self._raylib.UnloadFont(resource.data)
                resource.data = None
                resource.loaded = False

        logger.info("Unloaded all resources")

    def register_texture(self, name: str, path: str) -> None:
        """Register a texture resource programmatically.

        Args:
            name: The texture name
            path: Path to the texture file
        """
        self._textures[name] = TextureResource(name=name, path=path)

    def register_sound(self, name: str, path: str) -> None:
        """Register a sound resource programmatically.

        Args:
            name: The sound name
            path: Path to the sound file
        """
        self._sounds[name] = SoundResource(name=name, path=path)

    def register_font(self, name: str, path: str, size: int = 20) -> None:
        """Register a font resource programmatically.

        Args:
            name: The font name
            path: Path to the font file
            size: Font size in pixels
        """
        self._fonts[name] = FontResource(name=name, path=path, size=size)

    def is_texture_loaded(self, name: str) -> bool:
        """Check if a texture is loaded.

        Args:
            name: The texture name

        Returns:
            True if loaded
        """
        return name in self._textures and self._textures[name].loaded

    def is_sound_loaded(self, name: str) -> bool:
        """Check if a sound is loaded.

        Args:
            name: The sound name

        Returns:
            True if loaded
        """
        return name in self._sounds and self._sounds[name].loaded

    def is_font_loaded(self, name: str) -> bool:
        """Check if a font is loaded.

        Args:
            name: The font name

        Returns:
            True if loaded
        """
        return name in self._fonts and self._fonts[name].loaded

    def unload_texture(self, name: str) -> bool:
        """Unload a specific texture.

        Args:
            name: The texture name

        Returns:
            True if unloaded
        """
        if name not in self._textures:
            return False

        resource = self._textures[name]
        if resource.loaded and resource.data is not None:
            if self._raylib is not None:
                self._raylib.UnloadTexture(resource.data)
            resource.data = None
            resource.loaded = False
            logger.debug(f"Unloaded texture: {name}")
            return True
        return False

    def unload_sound(self, name: str) -> bool:
        """Unload a specific sound.

        Args:
            name: The sound name

        Returns:
            True if unloaded
        """
        if name not in self._sounds:
            return False

        resource = self._sounds[name]
        if resource.loaded and resource.data is not None:
            if self._raylib is not None:
                self._raylib.UnloadSound(resource.data)
            resource.data = None
            resource.loaded = False
            logger.debug(f"Unloaded sound: {name}")
            return True
        return False

    def unload_font(self, name: str) -> bool:
        """Unload a specific font.

        Args:
            name: The font name

        Returns:
            True if unloaded
        """
        if name not in self._fonts:
            return False

        resource = self._fonts[name]
        if resource.loaded and resource.data is not None:
            if self._raylib is not None:
                self._raylib.UnloadFont(resource.data)
            resource.data = None
            resource.loaded = False
            logger.debug(f"Unloaded font: {name}")
            return True
        return False

    # Group convenience methods

    def load_group(self, name: str) -> bool:
        """Load a resource group by name.

        Args:
            name: The group name

        Returns:
            True if loaded successfully
        """
        if self._group_manager is None:
            logger.warning("No group manager set")
            return False
        return self._group_manager.load_group(name)

    def unload_group(self, name: str) -> bool:
        """Unload a resource group by name.

        Args:
            name: The group name

        Returns:
            True if unloaded successfully
        """
        if self._group_manager is None:
            logger.warning("No group manager set")
            return False
        return self._group_manager.unload_group(name)

    def is_group_loaded(self, name: str) -> bool:
        """Check if a resource group is loaded.

        Args:
            name: The group name

        Returns:
            True if the group is loaded
        """
        if self._group_manager is None:
            return False
        return self._group_manager.is_group_loaded(name)

    def preload_resources(self) -> None:
        """Preload resources marked for preloading in config."""
        if not self._preload_list:
            return

        logger.info(f"Preloading {len(self._preload_list)} resources...")

        for name in self._preload_list:
            # Try each resource type
            if name in self._textures:
                self.get_texture(name)
            elif name in self._sounds:
                self.get_sound(name)
            elif name in self._fonts:
                self.get_font(name)
            elif self._group_manager is not None and name in self._group_manager.get_all_groups():
                # It's a group name
                self._group_manager.load_group(name)
            else:
                logger.warning(f"Unknown preload resource: {name}")

        logger.info("Preloading complete")
