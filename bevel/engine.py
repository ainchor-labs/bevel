"""Main engine class for Bevel."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from bevel.core.scene import SceneManager
from bevel.core.scene_loader import SceneLoader
from bevel.rendering.renderer import Renderer, WindowConfig
from bevel.physics.box2d import PhysicsWorld, PhysicsConfig
from bevel.input.input_manager import get_input_manager, set_input_manager, InputManager
from bevel.resources.resource_manager import ResourceManager
from bevel.utils.logger import get_logger, setup_logging
from bevel.utils.time import _update_time, _reset_time, _on_scene_loaded

logger = get_logger("engine")


@dataclass
class ProjectConfig:
    """Project configuration loaded from project.yaml."""

    project_name: str = "Bevel Game"
    version: str = "0.1.0"
    project_type: str = "2D"  # "2D" or "3D"
    window_width: int = 1280
    window_height: int = 720
    window_title: str = "Bevel Game"
    window_resizable: bool = True
    window_fullscreen: bool = False
    window_vsync: bool = True
    fps: int = 60
    entry_scene: str = "scenes/main.yaml"
    # 2D Physics
    physics_enabled: bool = True
    gravity_x: float = 0.0
    gravity_y: float = 314.0  # ~9.8 * 32 pixels/meter
    # 3D Physics
    physics_3d_enabled: bool = True
    gravity_3d_x: float = 0.0
    gravity_3d_y: float = -9.81
    gravity_3d_z: float = 0.0
    # Hot reload
    hot_reload_watch_paths: list[str] = field(default_factory=lambda: ["scenes", "scripts", "prefabs"])


def load_project_config(path: Path) -> ProjectConfig:
    """Load project configuration from a YAML file.

    Args:
        path: Path to project.yaml

    Returns:
        The loaded ProjectConfig
    """
    config = ProjectConfig()

    if not path.exists():
        logger.warning(f"Project config not found: {path}")
        return config

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except Exception as e:
        logger.error(f"Failed to load project config: {e}")
        return config

    config.project_name = data.get("project_name", config.project_name)
    config.version = data.get("version", config.version)
    config.project_type = data.get("project_type", config.project_type)

    window = data.get("window", {})
    config.window_width = window.get("width", config.window_width)
    config.window_height = window.get("height", config.window_height)
    config.window_title = window.get("title", config.window_title)
    config.window_resizable = window.get("resizable", config.window_resizable)
    config.window_fullscreen = window.get("fullscreen", config.window_fullscreen)
    config.window_vsync = window.get("vsync", config.window_vsync)
    config.fps = window.get("fps", config.fps)

    config.entry_scene = data.get("entry_scene", config.entry_scene)

    physics = data.get("physics", {})
    config.physics_enabled = physics.get("enabled", config.physics_enabled)
    gravity = physics.get("gravity", [config.gravity_x, config.gravity_y])
    if isinstance(gravity, list) and len(gravity) >= 2:
        config.gravity_x = gravity[0]
        config.gravity_y = gravity[1]

    # 3D physics config
    physics_3d = data.get("physics_3d", {})
    config.physics_3d_enabled = physics_3d.get("enabled", config.physics_3d_enabled)
    gravity_3d = physics_3d.get("gravity", [config.gravity_3d_x, config.gravity_3d_y, config.gravity_3d_z])
    if isinstance(gravity_3d, list) and len(gravity_3d) >= 3:
        config.gravity_3d_x = gravity_3d[0]
        config.gravity_3d_y = gravity_3d[1]
        config.gravity_3d_z = gravity_3d[2]

    # Hot reload config
    hot_reload_config = data.get("hot_reload", {})
    watch_paths = hot_reload_config.get("watch_paths")
    if watch_paths and isinstance(watch_paths, list):
        config.hot_reload_watch_paths = watch_paths

    return config


class Engine:
    """The main Bevel engine class."""

    def __init__(self, project_root: Path | None = None, hot_reload: bool = False) -> None:
        """Initialize the engine.

        Args:
            project_root: The project root directory
            hot_reload: Enable hot reload (file watching)
        """
        self.project_root = project_root or Path.cwd()
        self.config: ProjectConfig | None = None
        self.renderer: Renderer | None = None
        self.physics: PhysicsWorld | None = None  # 2D physics (Box2D)
        self.physics_3d: Any = None  # 3D physics (Jolt)
        self.scene_manager: SceneManager | None = None
        self.scene_loader: SceneLoader | None = None
        self.resource_manager: ResourceManager | None = None
        self.input_manager: InputManager | None = None
        self._running = False
        self._hot_reload_enabled = hot_reload
        self._hot_reloader: Any = None  # HotReloader instance

    def initialize(self) -> bool:
        """Initialize all engine systems.

        Returns:
            True if successful
        """
        setup_logging()
        logger.info(f"Initializing Bevel engine from: {self.project_root}")

        # Load project config
        config_path = self.project_root / "project.yaml"
        self.config = load_project_config(config_path)
        logger.info(f"Project: {self.config.project_name} v{self.config.version}")

        # Initialize renderer
        window_config = WindowConfig(
            width=self.config.window_width,
            height=self.config.window_height,
            title=self.config.window_title,
            resizable=self.config.window_resizable,
            fullscreen=self.config.window_fullscreen,
            vsync=self.config.window_vsync,
            fps=self.config.fps,
        )
        self.renderer = Renderer(window_config)
        if not self.renderer.initialize():
            logger.error("Failed to initialize renderer")
            return False

        # Initialize resource manager
        self.resource_manager = ResourceManager(self.project_root)
        self.resource_manager.set_raylib(self.renderer.raylib)
        self.resource_manager.load_resources("resources.yaml")
        self.renderer.set_resource_manager(self.resource_manager)

        # Initialize input manager
        self.input_manager = InputManager()
        self.input_manager.set_raylib(self.renderer.raylib)
        self.input_manager.load_actions(self.project_root / "input.yaml")
        set_input_manager(self.input_manager)

        # Register 3D node types if this is a 3D project
        if self.config.project_type == "3D":
            from bevel.nodes.node_3d import register_3d_types
            register_3d_types()
            logger.info("Registered 3D node types")

        # Initialize physics based on project type
        if self.config.project_type == "2D" and self.config.physics_enabled:
            self._init_physics_2d()
        elif self.config.project_type == "3D" and self.config.physics_3d_enabled:
            self._init_physics_3d()

        # Initialize scene system
        self.scene_loader = SceneLoader(self.project_root)
        self.scene_manager = SceneManager()
        self.scene_manager.set_loader(self.scene_loader)
        self.scene_manager.set_renderer(self.renderer)

        # Register time reset on scene change
        self.scene_manager.on_scene_changed(lambda _: _on_scene_loaded())

        # Initialize hot reload if enabled
        if self._hot_reload_enabled:
            self._init_hot_reload()

        logger.info("Engine initialized successfully")
        return True

    def _init_hot_reload(self) -> None:
        """Initialize hot reload system."""
        try:
            from bevel.development.hot_reload import HotReloader, is_hot_reload_available

            if not is_hot_reload_available():
                logger.warning("Hot reload not available (watchdog not installed)")
                self._hot_reload_enabled = False
                return

            self._hot_reloader = HotReloader(
                scene_manager=self.scene_manager,
                project_root=self.project_root,
            )

            # Get watch paths from project config
            watch_paths = self.config.hot_reload_watch_paths

            if self._hot_reloader.start(watch_paths):
                logger.info("Hot reload initialized successfully")
            else:
                logger.warning("Failed to start hot reload")
                self._hot_reload_enabled = False
        except Exception as e:
            logger.warning(f"Failed to initialize hot reload: {e}")
            self._hot_reload_enabled = False

    def _init_physics_2d(self) -> None:
        """Initialize 2D physics (Box2D)."""
        from bevel.utils.math import Vector2

        physics_config = PhysicsConfig(
            gravity=Vector2(self.config.gravity_x, self.config.gravity_y)
        )
        self.physics = PhysicsWorld(physics_config)
        if not self.physics.initialize():
            logger.warning("2D Physics initialization failed, continuing without physics")
            self.physics = None
        else:
            logger.info("2D Physics (Box2D) initialized")

    def _init_physics_3d(self) -> None:
        """Initialize 3D physics (Jolt)."""
        try:
            from bevel.physics.jolt import PhysicsWorld3D, PhysicsConfig3D
            from bevel.utils.math import Vector3

            physics_config = PhysicsConfig3D(
                gravity=Vector3(
                    self.config.gravity_3d_x,
                    self.config.gravity_3d_y,
                    self.config.gravity_3d_z,
                )
            )
            self.physics_3d = PhysicsWorld3D(physics_config)
            if not self.physics_3d.initialize():
                logger.warning("3D Physics initialization failed, continuing without physics")
                self.physics_3d = None
            else:
                logger.info("3D Physics (Jolt) initialized")
        except ImportError as e:
            logger.warning(f"3D Physics not available: {e}")
            self.physics_3d = None

    def run(self, scene_path: str | None = None) -> int:
        """Run the game loop.

        Args:
            scene_path: Optional path to a scene file (overrides entry_scene)

        Returns:
            Exit code (0 = success)
        """
        if self.renderer is None or self.scene_manager is None:
            logger.error("Engine not initialized")
            return 1

        # Load the entry scene
        entry_scene = scene_path or self.config.entry_scene
        if not self.scene_manager.change_scene(entry_scene):
            logger.error(f"Failed to load entry scene: {entry_scene}")
            return 1

        # Set up physics for the scene
        if self.physics is not None and self.scene_manager.current_scene is not None:
            self.physics.setup_scene(self.scene_manager.current_scene)
        if self.physics_3d is not None and self.scene_manager.current_scene is not None:
            self.physics_3d.setup_scene(self.scene_manager.current_scene)

        # Reset time for a fresh start
        _reset_time()

        self._running = True
        logger.info("Starting game loop")

        try:
            while self._running and not self.renderer.should_close():
                self._update_frame()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Error in game loop: {e}")
            import traceback
            traceback.print_exc()
            return 1
        finally:
            self.shutdown()

        return 0

    def _update_frame(self) -> None:
        """Update a single frame."""
        raw_delta = self.renderer.get_delta_time()

        # Update Time singleton (this applies time scale and updates all time values)
        _update_time(raw_delta, raw_delta)

        # Import Time to get the scaled delta
        from bevel.utils.time import Time
        delta = Time.delta_time

        # Update hot reload
        if self._hot_reloader is not None:
            self._hot_reloader.update()

        # Update input
        self.input_manager.update()

        # Update physics (uses scaled delta time)
        if self.physics is not None:
            self.physics.step(delta)
        if self.physics_3d is not None:
            self.physics_3d.step(delta)

        # Update scene
        self.scene_manager.update(delta)
        self.scene_manager.late_update(delta)

        # Render
        scene = self.scene_manager.current_scene
        background = (0, 0, 0, 255)
        if scene is not None:
            background = scene.config.background_color

        self.renderer.begin_frame(background)
        if scene is not None:
            self.renderer.render_scene(scene)

        # Render transition overlay if active
        self.scene_manager.render_transition()

        self.renderer.end_frame()

    def stop(self) -> None:
        """Stop the game loop."""
        self._running = False

    def shutdown(self) -> None:
        """Shut down all engine systems."""
        logger.info("Shutting down engine")

        # Stop hot reload
        if self._hot_reloader is not None:
            self._hot_reloader.stop()

        if self.physics is not None:
            self.physics.shutdown()

        if self.physics_3d is not None:
            self.physics_3d.shutdown()

        if self.resource_manager is not None:
            self.resource_manager.unload_all()

        if self.renderer is not None:
            self.renderer.shutdown()

        logger.info("Engine shut down complete")


# Global engine instance
_engine: Engine | None = None


def get_engine() -> Engine | None:
    """Get the global engine instance."""
    return _engine


def create_engine(project_root: Path | None = None, hot_reload: bool = False) -> Engine:
    """Create and set the global engine instance.

    Args:
        project_root: The project root directory
        hot_reload: Enable hot reload (file watching)

    Returns:
        The created Engine instance
    """
    global _engine
    _engine = Engine(project_root, hot_reload=hot_reload)
    return _engine
