"""Raylib-based renderer for Bevel engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, TYPE_CHECKING

from bevel.utils.math import Vector2
from bevel.utils.logger import get_logger

if TYPE_CHECKING:
    from bevel.core.scene import Scene
    from bevel.nodes.node_2d import Sprite, Camera2D

logger = get_logger("renderer")


@dataclass
class WindowConfig:
    """Window configuration."""

    width: int = 1280
    height: int = 720
    title: str = "Bevel Game"
    resizable: bool = True
    fullscreen: bool = False
    vsync: bool = True
    fps: int = 60


@dataclass
class RenderStats:
    """Rendering statistics."""

    draw_calls: int = 0
    sprites_rendered: int = 0
    frame_time: float = 0.0
    fps: float = 0.0


class Renderer:
    """Handles all rendering using raylib."""

    def __init__(self, config: WindowConfig | None = None) -> None:
        """Initialize the renderer.

        Args:
            config: Window configuration
        """
        self.config = config or WindowConfig()
        self._rl: Any = None
        self._initialized = False
        self._textures: dict[str, Any] = {}
        self._current_camera: Camera2D | None = None
        self._stats = RenderStats()
        self._resource_manager: Any = None

    def set_resource_manager(self, manager: Any) -> None:
        """Set the resource manager for texture loading.

        Args:
            manager: The ResourceManager instance
        """
        self._resource_manager = manager

    def initialize(self) -> bool:
        """Initialize the rendering system.

        Returns:
            True if successful, False otherwise
        """
        try:
            import raylib as rl
            self._rl = rl

            # Set config flags (must be before InitWindow)
            flags = 0
            if self.config.resizable:
                flags |= rl.FLAG_WINDOW_RESIZABLE
            if self.config.vsync:
                flags |= rl.FLAG_VSYNC_HINT
            if flags:
                rl.SetConfigFlags(flags)

            # Initialize window
            rl.InitWindow(
                self.config.width,
                self.config.height,
                self.config.title.encode()
            )

            if self.config.fullscreen:
                rl.ToggleFullscreen()

            rl.SetTargetFPS(self.config.fps)

            self._initialized = True
            logger.info(
                f"Renderer initialized: {self.config.width}x{self.config.height} "
                f"@ {self.config.fps} FPS"
            )
            return True

        except ImportError as e:
            logger.error(f"Failed to import raylib: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to initialize renderer: {e}")
            return False

    def shutdown(self) -> None:
        """Shut down the rendering system."""
        if not self._initialized:
            return

        # Unload textures
        for texture in self._textures.values():
            self._rl.UnloadTexture(texture)
        self._textures.clear()

        self._rl.CloseWindow()
        self._initialized = False
        logger.info("Renderer shut down")

    def should_close(self) -> bool:
        """Check if the window should close.

        Returns:
            True if the window should close
        """
        if not self._initialized:
            return True
        return self._rl.WindowShouldClose()

    def begin_frame(self, background_color: tuple[int, int, int, int] = (0, 0, 0, 255)) -> None:
        """Begin a new frame.

        Args:
            background_color: RGBA background color
        """
        if not self._initialized:
            return

        self._stats = RenderStats()
        self._rl.BeginDrawing()
        self._rl.ClearBackground(background_color)

    def end_frame(self) -> None:
        """End the current frame."""
        if not self._initialized:
            return

        self._stats.frame_time = self._rl.GetFrameTime()
        self._stats.fps = self._rl.GetFPS()
        self._rl.EndDrawing()

    def get_delta_time(self) -> float:
        """Get the time since last frame.

        Returns:
            Delta time in seconds
        """
        if not self._initialized:
            return 0.0
        return self._rl.GetFrameTime()

    def get_screen_size(self) -> Vector2:
        """Get the current screen size.

        Returns:
            Screen dimensions as Vector2
        """
        if not self._initialized:
            return Vector2(self.config.width, self.config.height)
        return Vector2(
            self._rl.GetScreenWidth(),
            self._rl.GetScreenHeight()
        )

    def load_texture(self, name: str, path: str) -> bool:
        """Load a texture.

        Args:
            name: Texture identifier
            path: Path to the texture file

        Returns:
            True if successful
        """
        if not self._initialized:
            return False

        if name in self._textures:
            return True

        try:
            texture = self._rl.LoadTexture(path.encode())
            if texture.id == 0:
                logger.error(f"Failed to load texture: {path}")
                return False

            self._textures[name] = texture
            logger.debug(f"Loaded texture: {name} ({texture.width}x{texture.height})")
            return True

        except Exception as e:
            logger.error(f"Error loading texture '{name}': {e}")
            return False

    def get_texture(self, name: str) -> Any:
        """Get a loaded texture.

        Args:
            name: Texture identifier

        Returns:
            The texture or None if not found
        """
        # Try resource manager first
        if self._resource_manager is not None:
            texture = self._resource_manager.get_texture(name)
            if texture is not None:
                return texture

        return self._textures.get(name)

    def render_scene(self, scene: "Scene") -> None:
        """Render a scene.

        Args:
            scene: The scene to render
        """
        if not self._initialized or scene.root is None:
            return

        # Check if this is a 2D or 3D scene
        from bevel.nodes.node_2d import Camera2D, Sprite, CollisionShape2D
        from bevel.nodes.node_3d import Camera3D, Node3D, MeshInstance

        # Try to find a 3D camera first
        camera_3d = None
        for cam in scene.root.find_nodes_by_type(Camera3D):
            if cam.current:
                camera_3d = cam
                break

        # If no 3D camera, try 2D
        if camera_3d is None:
            # 2D rendering path
            self._current_camera = None
            for cam in scene.root.find_nodes_by_type(Camera2D):
                if cam.current:
                    self._current_camera = cam
                    break

            # Update camera smoothing
            if self._current_camera is not None:
                self._current_camera.update_smoothing(self.get_delta_time())

            # Collect all renderable nodes and sort by z_index
            renderables: list[tuple[int, Any]] = []
            self._collect_renderables(scene.root, renderables)
            renderables.sort(key=lambda r: r[0])

            # Set up camera transform
            if self._current_camera is not None:
                self._begin_camera_2d()

            # Render all nodes
            for z_index, node in renderables:
                if isinstance(node, Sprite):
                    self._render_sprite(node)
                elif isinstance(node, CollisionShape2D):
                    self._render_collision_shape(node)

            # End camera transform
            if self._current_camera is not None:
                self._end_camera_2d()
        else:
            # 3D rendering path
            self._render_scene_3d(scene, camera_3d)

    def _render_scene_3d(self, scene: Any, camera: Any) -> None:
        """Render a 3D scene.

        Args:
            scene: The scene to render
            camera: The Camera3D node to use
        """
        from bevel.nodes.node_3d import MeshInstance, Light3D, Node3D

        # Setup 3D camera
        self._begin_camera_3d(camera)

        # Collect all 3D renderable nodes
        mesh_instances: list[MeshInstance] = []
        lights: list[Light3D] = []
        self._collect_3d_renderables(scene.root, mesh_instances, lights)

        # Render all mesh instances
        for mesh_instance in mesh_instances:
            if mesh_instance.visible:
                self._render_mesh_instance(mesh_instance)

        # End 3D camera mode
        self._end_camera_3d()

        # Draw UI/debug info (2D overlay)
        self._draw_3d_debug_info()

    def _collect_3d_renderables(self, node: Any, meshes: list[Any], lights: list[Any]) -> None:
        """Recursively collect 3D renderable nodes.

        Args:
            node: Current node
            meshes: List to collect MeshInstance nodes into
            lights: List to collect Light3D nodes into
        """
        from bevel.nodes.node_3d import MeshInstance, Light3D

        if not node.visible:
            return

        if isinstance(node, MeshInstance):
            meshes.append(node)
        elif isinstance(node, Light3D):
            lights.append(node)

        for child in node.children:
            self._collect_3d_renderables(child, meshes, lights)

    def _begin_camera_3d(self, camera: Any) -> None:
        """Begin 3D camera mode.

        Args:
            camera: The Camera3D node
        """
        from bevel.nodes.node_3d import ProjectionType
        from bevel.utils.math import Vector3

        # Create raylib Camera3D using FFI
        cam_pos = camera.global_position
        
        # Calculate forward direction from rotation
        # Default forward is -Z (0, 0, -1), rotate it
        forward = camera.rotation.rotate_vector(Vector3(0, 0, -1))
        target_pos = cam_pos + forward

        # Build camera struct using FFI
        rl_camera = self._rl.ffi.new("Camera3D *", {
            "position": {"x": cam_pos.x, "y": cam_pos.y, "z": cam_pos.z},
            "target": {"x": target_pos.x, "y": target_pos.y, "z": target_pos.z},
            "up": {"x": 0, "y": 1, "z": 0},
            "fovy": camera.fov,
            "projection": 0  # CAMERA_PERSPECTIVE
        })

        self._rl.BeginMode3D(rl_camera[0])

    def _end_camera_3d(self) -> None:
        """End 3D camera mode."""
        self._rl.EndMode3D()

    def _render_mesh_instance(self, mesh_instance: Any) -> None:
        """Render a mesh instance.

        Args:
            mesh_instance: The MeshInstance node to render
        """
        if not mesh_instance.visible or not mesh_instance.mesh:
            return

        pos = mesh_instance.global_position
        scale_2d = mesh_instance.transform.global_scale if hasattr(mesh_instance.transform, 'global_scale') else None
        
        # Handle both 2D and 3D transforms
        if scale_2d is not None and hasattr(scale_2d, 'z'):
            scale_x, scale_y, scale_z = scale_2d.x, scale_2d.y, scale_2d.z
        elif scale_2d is not None:
            # 2D transform - use same scale for x and z, 1.0 for y
            scale_x, scale_y, scale_z = scale_2d.x, 1.0, scale_2d.x
        else:
            scale_x, scale_y, scale_z = 1.0, 1.0, 1.0

        # Draw a colored cube representing the mesh
        self._rl.DrawCube(
            (pos.x, pos.y, pos.z),
            scale_x * 2,
            scale_y * 2,
            scale_z * 2,
            (100, 150, 200, 255)
        )

        # Draw wireframe
        self._rl.DrawCubeWires(
            (pos.x, pos.y, pos.z),
            scale_x * 2,
            scale_y * 2,
            scale_z * 2,
            (200, 200, 200, 255)
        )

        self._stats.draw_calls += 1

    def _draw_3d_debug_info(self) -> None:
        """Draw debug information overlay for 3D scene."""
        # Draw grid for reference
        self._rl.DrawGrid(10, 1.0)

    def _collect_renderables(self, node: Any, renderables: list[tuple[int, Any]]) -> None:
        """Recursively collect all visible renderable nodes.

        Args:
            node: Current node
            renderables: List to collect (z_index, node) tuples into
        """
        from bevel.nodes.node_2d import Sprite, CollisionShape2D

        if not node.visible:
            return

        if isinstance(node, (Sprite, CollisionShape2D)):
            renderables.append((node.z_index, node))

        for child in node.children:
            self._collect_renderables(child, renderables)

    def _collect_sprites(self, node: Any, sprites: list["Sprite"]) -> None:
        """Recursively collect all visible sprites.

        Args:
            node: Current node
            sprites: List to collect sprites into
        """
        from bevel.nodes.node_2d import Sprite

        if not node.visible:
            return

        if isinstance(node, Sprite):
            sprites.append(node)

        for child in node.children:
            self._collect_sprites(child, sprites)

    def _begin_camera_2d(self) -> None:
        """Begin 2D camera mode."""
        if self._current_camera is None:
            return

        screen_size = self.get_screen_size()
        cam_pos = self._current_camera.get_camera_position()
        offset = self._current_camera.offset
        zoom = self._current_camera.zoom

        # Create raylib Camera2D using ffi
        camera = self._rl.ffi.new("Camera2D *", {
            "offset": {"x": screen_size.x / 2 + offset.x, "y": screen_size.y / 2 + offset.y},
            "target": {"x": cam_pos.x, "y": cam_pos.y},
            "rotation": 0.0,
            "zoom": zoom
        })
        self._rl.BeginMode2D(camera[0])

    def _end_camera_2d(self) -> None:
        """End 2D camera mode."""
        self._rl.EndMode2D()

    def _render_sprite(self, sprite: "Sprite") -> None:
        """Render a single sprite.

        Args:
            sprite: The sprite to render
        """
        if not sprite.visible or not sprite.texture:
            return

        texture = self.get_texture(sprite.texture)
        if texture is None:
            return

        # Store texture data on sprite for size calculations
        sprite._texture_data = texture

        pos = sprite.global_position
        rotation = sprite.rotation_degrees
        scale = sprite.transform.global_scale

        # Calculate source and destination rectangles
        src_width = texture.width
        src_height = texture.height

        if sprite.region_enabled:
            src_rect = (
                sprite.region_rect[0],
                sprite.region_rect[1],
                sprite.region_rect[2],
                sprite.region_rect[3]
            )
            src_width = sprite.region_rect[2]
            src_height = sprite.region_rect[3]
        else:
            # Handle flip
            src_w = -texture.width if sprite.flip_h else texture.width
            src_h = -texture.height if sprite.flip_v else texture.height
            src_rect = (0, 0, src_w, src_h)

        dest_width = abs(src_width) * scale.x
        dest_height = abs(src_height) * scale.y

        dest_rect = (pos.x, pos.y, dest_width, dest_height)

        # Origin (center if centered)
        if sprite.centered:
            origin = (dest_width / 2, dest_height / 2)
        else:
            origin = (0, 0)

        # Draw
        self._rl.DrawTexturePro(
            texture,
            src_rect,
            dest_rect,
            origin,
            rotation,
            sprite.modulate
        )

        self._stats.sprites_rendered += 1
        self._stats.draw_calls += 1

    def _render_collision_shape(self, shape: Any) -> None:
        """Render a collision shape as a colored primitive.

        Args:
            shape: The CollisionShape2D to render
        """
        from bevel.nodes.node_2d import ShapeType, BodyType

        if not shape.visible:
            return

        pos = shape.global_position

        # Determine color based on body type or script property
        color = (100, 100, 100, 255)  # Default gray

        # Check if script has a color property
        for script in shape.scripts:
            if hasattr(script, "color"):
                color = script.color
                break
        else:
            # Default colors based on body type
            if shape.physics.body_type == BodyType.STATIC:
                color = (80, 160, 80, 255)  # Green for static
            elif shape.physics.body_type == BodyType.KINEMATIC:
                color = (80, 80, 160, 255)  # Blue for kinematic
            else:  # Dynamic
                color = (200, 100, 100, 255)  # Red for dynamic

        if shape.shape == ShapeType.RECTANGLE:
            # Draw centered rectangle
            x = pos.x - shape.size.x / 2
            y = pos.y - shape.size.y / 2
            self._rl.DrawRectangle(
                int(x), int(y),
                int(shape.size.x), int(shape.size.y),
                color
            )
        else:  # Circle
            self._rl.DrawCircle(
                int(pos.x), int(pos.y),
                shape.radius,
                color
            )

        self._stats.draw_calls += 1

    def draw_rectangle(self, x: float, y: float, width: float, height: float,
                       color: tuple[int, int, int, int] = (255, 255, 255, 255)) -> None:
        """Draw a filled rectangle.

        Args:
            x: X position
            y: Y position
            width: Rectangle width
            height: Rectangle height
            color: RGBA color
        """
        if not self._initialized:
            return
        self._rl.DrawRectangle(int(x), int(y), int(width), int(height), color)
        self._stats.draw_calls += 1

    def draw_rectangle_lines(self, x: float, y: float, width: float, height: float,
                             color: tuple[int, int, int, int] = (255, 255, 255, 255)) -> None:
        """Draw a rectangle outline.

        Args:
            x: X position
            y: Y position
            width: Rectangle width
            height: Rectangle height
            color: RGBA color
        """
        if not self._initialized:
            return
        self._rl.DrawRectangleLines(int(x), int(y), int(width), int(height), color)
        self._stats.draw_calls += 1

    def draw_circle(self, x: float, y: float, radius: float,
                    color: tuple[int, int, int, int] = (255, 255, 255, 255)) -> None:
        """Draw a filled circle.

        Args:
            x: Center X
            y: Center Y
            radius: Circle radius
            color: RGBA color
        """
        if not self._initialized:
            return
        self._rl.DrawCircle(int(x), int(y), radius, color)
        self._stats.draw_calls += 1

    def draw_line(self, x1: float, y1: float, x2: float, y2: float,
                  color: tuple[int, int, int, int] = (255, 255, 255, 255)) -> None:
        """Draw a line.

        Args:
            x1: Start X
            y1: Start Y
            x2: End X
            y2: End Y
            color: RGBA color
        """
        if not self._initialized:
            return
        self._rl.DrawLine(int(x1), int(y1), int(x2), int(y2), color)
        self._stats.draw_calls += 1

    def draw_text(self, text: str, x: float, y: float, font_size: int = 20,
                  color: tuple[int, int, int, int] = (255, 255, 255, 255)) -> None:
        """Draw text.

        Args:
            text: Text to draw
            x: X position
            y: Y position
            font_size: Font size in pixels
            color: RGBA color
        """
        if not self._initialized:
            return
        self._rl.DrawText(text.encode(), int(x), int(y), font_size, color)
        self._stats.draw_calls += 1

    @property
    def stats(self) -> RenderStats:
        """Get rendering statistics."""
        return self._stats

    @property
    def raylib(self) -> Any:
        """Get the raylib module."""
        return self._rl
