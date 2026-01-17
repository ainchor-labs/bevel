"""YAML scene loader for Bevel engine."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any

import yaml

from bevel.core.node import Node
from bevel.core.scene import Scene, SceneConfig
from typing import TYPE_CHECKING
from bevel.nodes.node_2d import (
    NODE_TYPES,
    get_node_type,
    BodyType,
    ShapeType,
    PhysicsConfig,
)
from bevel.utils.math import Vector2, Vector3, Quaternion
from bevel.utils.logger import get_logger

logger = get_logger("scene_loader")


class SceneLoader:
    """Loads scenes from YAML files."""

    def __init__(self, project_root: Path | None = None) -> None:
        """Initialize the scene loader.

        Args:
            project_root: The root directory of the project
        """
        self.project_root = project_root or Path.cwd()
        self._script_cache: dict[str, Any] = {}
        self._prefab_cache: dict[str, dict[str, Any]] = {}

    def load(self, path: str) -> Scene | None:
        """Load a scene from a YAML file.

        Args:
            path: Path to the scene file (relative to project root)

        Returns:
            The loaded Scene or None if loading failed
        """
        file_path = self.project_root / path
        if not file_path.exists():
            logger.error(f"Scene file not found: {file_path}")
            return None

        try:
            with open(file_path, "r") as f:
                data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            logger.error(f"Failed to parse scene file: {e}")
            return None
        except Exception as e:
            logger.error(f"Failed to read scene file: {e}")
            return None

        return self._parse_scene(data, str(file_path))

    def _parse_scene(self, data: dict[str, Any], file_path: str) -> Scene | None:
        """Parse scene data from a dictionary.

        Args:
            data: The parsed YAML data
            file_path: Path to the scene file

        Returns:
            The parsed Scene or None if parsing failed
        """
        try:
            # Parse scene config
            config_data = data.get("scene_config", {})
            background = config_data.get("background_color", [0, 0, 0, 255])
            if isinstance(background, list):
                background = tuple(background)

            config = SceneConfig(
                name=config_data.get("name", "Untitled Scene"),
                type=config_data.get("type", "2D"),
                background_color=background,
            )

            # Parse root node
            root_data = data.get("root")
            if root_data is None:
                logger.error("Scene has no root node")
                return None

            root = self._parse_node(root_data)
            if root is None:
                return None

            scene = Scene(config=config, root=root, file_path=file_path)

            # Setup signal connections after all nodes exist
            connections_data = data.get("connections", [])
            self._setup_scene_connections(scene, connections_data)

            logger.info(f"Loaded scene: {config.name}")
            return scene

        except Exception as e:
            logger.error(f"Failed to parse scene: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _parse_node(self, data: dict[str, Any]) -> Node | None:
        """Parse a node from YAML data.

        Args:
            data: The node data dictionary

        Returns:
            The parsed Node or None if parsing failed
        """
        # Handle prefab references
        if "prefab" in data:
            prefab_path = data["prefab"]
            prefab_data = self._load_prefab(prefab_path)
            if prefab_data is None:
                return None
            data = self._merge_node_data(prefab_data, data)

        node_type_name = data.get("type", "Node")
        node_class = get_node_type(node_type_name)

        if node_class is None:
            logger.error(f"Unknown node type: {node_type_name}")
            return None

        # Build node kwargs
        kwargs: dict[str, Any] = {}

        # Name
        if "name" in data:
            kwargs["name"] = data["name"]

        # Position will be set after node creation

        # Active/visible state
        if "active" in data:
            kwargs["_active"] = data["active"]
        if "visible" in data:
            kwargs["_visible"] = data["visible"]

        # Z-index
        if "z_index" in data:
            kwargs["z_index"] = data["z_index"]

        # Render layer
        if "layer" in data:
            kwargs["layer"] = data["layer"]

        # Groups
        if "groups" in data:
            kwargs["_groups"] = set(data["groups"])

        # Type-specific properties
        if node_type_name == "Sprite":
            if "texture" in data:
                kwargs["texture"] = data["texture"]
            if "centered" in data:
                kwargs["centered"] = data["centered"]
            if "flip_h" in data:
                kwargs["flip_h"] = data["flip_h"]
            if "flip_v" in data:
                kwargs["flip_v"] = data["flip_v"]
            if "modulate" in data:
                kwargs["modulate"] = tuple(data["modulate"])

        elif node_type_name == "Camera2D":
            if "current" in data:
                kwargs["current"] = data["current"]
            if "zoom" in data:
                kwargs["zoom"] = data["zoom"]
            if "offset" in data:
                kwargs["offset"] = Vector2(data["offset"][0], data["offset"][1])
            if "smoothing_enabled" in data:
                kwargs["smoothing_enabled"] = data["smoothing_enabled"]
            if "smoothing_speed" in data:
                kwargs["smoothing_speed"] = data["smoothing_speed"]
            for limit in ["limit_left", "limit_right", "limit_top", "limit_bottom"]:
                if limit in data:
                    kwargs[limit] = data[limit]

        elif node_type_name == "CollisionShape2D":
            if "shape" in data:
                shape_name = data["shape"].upper()
                kwargs["shape"] = ShapeType[shape_name]
            if "size" in data:
                kwargs["size"] = Vector2(data["size"][0], data["size"][1])
            if "radius" in data:
                kwargs["radius"] = data["radius"]
            if "collision_layer" in data:
                kwargs["collision_layer"] = data["collision_layer"]
            if "collision_mask" in data:
                kwargs["collision_mask"] = data["collision_mask"]

            # Physics config
            if "physics" in data:
                physics_data = data["physics"]
                physics_kwargs: dict[str, Any] = {}

                if "body_type" in physics_data:
                    body_type_name = physics_data["body_type"].upper()
                    physics_kwargs["body_type"] = BodyType[body_type_name]
                if "mass" in physics_data:
                    physics_kwargs["mass"] = physics_data["mass"]
                if "friction" in physics_data:
                    physics_kwargs["friction"] = physics_data["friction"]
                if "restitution" in physics_data:
                    physics_kwargs["restitution"] = physics_data["restitution"]
                if "linear_damping" in physics_data:
                    physics_kwargs["linear_damping"] = physics_data["linear_damping"]
                if "angular_damping" in physics_data:
                    physics_kwargs["angular_damping"] = physics_data["angular_damping"]
                if "fixed_rotation" in physics_data:
                    physics_kwargs["fixed_rotation"] = physics_data["fixed_rotation"]
                if "is_sensor" in physics_data:
                    physics_kwargs["is_sensor"] = physics_data["is_sensor"]
                if "gravity_scale" in physics_data:
                    physics_kwargs["gravity_scale"] = physics_data["gravity_scale"]

                kwargs["physics"] = PhysicsConfig(**physics_kwargs)

        # 3D node types
        elif node_type_name == "Node3D":
            pass  # Node3D uses Transform3D, handled in transform section

        elif node_type_name == "MeshInstance":
            if "mesh" in data:
                kwargs["mesh"] = data["mesh"]
            if "material" in data:
                kwargs["material"] = data["material"]
            if "cast_shadows" in data:
                kwargs["cast_shadows"] = data["cast_shadows"]
            if "receive_shadows" in data:
                kwargs["receive_shadows"] = data["receive_shadows"]

        elif node_type_name == "Camera3D":
            if "current" in data:
                kwargs["current"] = data["current"]
            if "projection" in data:
                from bevel.nodes.node_3d import ProjectionType
                kwargs["projection"] = ProjectionType[data["projection"].upper()]
            if "fov" in data:
                kwargs["fov"] = data["fov"]
            if "near" in data:
                kwargs["near"] = data["near"]
            if "far" in data:
                kwargs["far"] = data["far"]
            if "ortho_size" in data:
                kwargs["ortho_size"] = data["ortho_size"]

        elif node_type_name == "Light3D":
            if "light_type" in data:
                from bevel.nodes.node_3d import LightType
                kwargs["light_type"] = LightType[data["light_type"].upper()]
            if "color" in data:
                kwargs["color"] = tuple(data["color"])
            if "intensity" in data:
                kwargs["intensity"] = data["intensity"]
            if "range" in data:
                kwargs["range"] = data["range"]
            if "spot_angle" in data:
                kwargs["spot_angle"] = data["spot_angle"]
            if "cast_shadows" in data:
                kwargs["cast_shadows"] = data["cast_shadows"]

        elif node_type_name == "CollisionShape3D":
            if "shape" in data:
                from bevel.nodes.node_3d import ShapeType3D
                kwargs["shape"] = ShapeType3D[data["shape"].upper()]
            if "size" in data:
                kwargs["size"] = Vector3(data["size"][0], data["size"][1], data["size"][2])
            if "radius" in data:
                kwargs["radius"] = data["radius"]
            if "height" in data:
                kwargs["height"] = data["height"]
            if "mesh" in data:
                kwargs["mesh"] = data["mesh"]
            if "collision_layer" in data:
                kwargs["collision_layer"] = data["collision_layer"]
            if "collision_mask" in data:
                kwargs["collision_mask"] = data["collision_mask"]

            # 3D Physics config
            if "physics" in data:
                from bevel.nodes.node_3d import PhysicsConfig3D, BodyType3D
                physics_data = data["physics"]
                physics_kwargs_3d: dict[str, Any] = {}

                if "body_type" in physics_data:
                    physics_kwargs_3d["body_type"] = BodyType3D[physics_data["body_type"].upper()]
                if "mass" in physics_data:
                    physics_kwargs_3d["mass"] = physics_data["mass"]
                if "friction" in physics_data:
                    physics_kwargs_3d["friction"] = physics_data["friction"]
                if "restitution" in physics_data:
                    physics_kwargs_3d["restitution"] = physics_data["restitution"]
                if "linear_damping" in physics_data:
                    physics_kwargs_3d["linear_damping"] = physics_data["linear_damping"]
                if "angular_damping" in physics_data:
                    physics_kwargs_3d["angular_damping"] = physics_data["angular_damping"]
                if "gravity_scale" in physics_data:
                    physics_kwargs_3d["gravity_scale"] = physics_data["gravity_scale"]
                if "is_sensor" in physics_data:
                    physics_kwargs_3d["is_sensor"] = physics_data["is_sensor"]

                kwargs["physics"] = PhysicsConfig3D(**physics_kwargs_3d)

        # Custom properties
        if "properties" in data:
            kwargs["_properties"] = dict(data["properties"])

        # Create the node
        try:
            node = node_class(**kwargs)
        except Exception as e:
            logger.error(f"Failed to create node '{data.get('name', 'unnamed')}': {e}")
            import traceback
            traceback.print_exc()
            return None

        # Set position/rotation/scale after creation
        # Check if this is a 3D node type
        is_3d_node = node_type_name in ("Node3D", "MeshInstance", "Camera3D", "Light3D", "CollisionShape3D")

        if "position" in data:
            pos = data["position"]
            if is_3d_node and len(pos) >= 3:
                node.position = Vector3(pos[0], pos[1], pos[2])
            else:
                node.position = Vector2(pos[0], pos[1])

        if "rotation" in data:
            node.rotation = data["rotation"]

        if "rotation_degrees" in data:
            rot = data["rotation_degrees"]
            if is_3d_node and isinstance(rot, list) and len(rot) >= 3:
                # Euler angles for 3D nodes
                node.rotation_degrees = Vector3(rot[0], rot[1], rot[2])
            else:
                node.rotation_degrees = rot

        if "rotation_euler" in data and is_3d_node:
            # Explicit euler rotation for 3D (in degrees)
            rot = data["rotation_euler"]
            node.rotation_degrees = Vector3(rot[0], rot[1], rot[2])

        if "scale" in data:
            scale = data["scale"]
            if is_3d_node and len(scale) >= 3:
                node.scale = Vector3(scale[0], scale[1], scale[2])
            else:
                node.scale = Vector2(scale[0], scale[1])

        # Load and attach scripts
        if "scripts" in data:
            script_entries = self._parse_script_entries(data["scripts"])
            for script, priority in script_entries:
                node.attach_script(script, priority)

        # Define signals
        if "signals" in data:
            self._setup_node_signals(node, data["signals"])

        # Store pending connections for later setup (after scene is fully loaded)
        if "connections" in data:
            # Store on node temporarily for later processing
            node._pending_connections = data["connections"]

        # Parse children
        if "children" in data:
            for child_data in data["children"]:
                child = self._parse_node(child_data)
                if child is not None:
                    node.add_child(child)

        return node

    def _load_prefab(self, prefab_path: str) -> dict[str, Any] | None:
        """Load a prefab from a YAML file.

        Prefab files use CloudFormation-style format:
            resource_id:
              name: "DisplayName"
              type: NodeType
              properties:
                shape: "rectangle"
                physics:
                  body_type: "dynamic"
              scripts:
                - "scripts/foo.py"

        Args:
            prefab_path: Path to the prefab file (relative to project root)

        Returns:
            The prefab data dictionary (flattened) or None if loading failed
        """
        if prefab_path in self._prefab_cache:
            return self._prefab_cache[prefab_path]

        full_path = self.project_root / prefab_path
        if not full_path.exists():
            logger.error(f"Prefab file not found: {full_path}")
            return None

        try:
            with open(full_path, "r") as f:
                data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            logger.error(f"Failed to parse prefab file: {e}")
            return None
        except Exception as e:
            logger.error(f"Failed to read prefab file: {e}")
            return None

        if not isinstance(data, dict):
            logger.error(f"Prefab file must contain a dictionary: {prefab_path}")
            return None

        if "scene_config" in data or "root" in data:
            logger.error(
                f"Prefab file should not contain 'scene_config' or 'root'. "
                f"It should be a single node definition: {prefab_path}"
            )
            return None

        # CloudFormation-style: single top-level key is the resource ID
        # The value contains name, type, properties, scripts, children
        if len(data) == 1:
            resource_id = next(iter(data.keys()))
            resource_data = data[resource_id]

            if isinstance(resource_data, dict) and "type" in resource_data:
                # Flatten the CF-style format to internal format
                flattened = self._flatten_prefab_data(resource_data)
                self._prefab_cache[prefab_path] = flattened
                logger.debug(f"Loaded prefab: {prefab_path} (resource: {resource_id})")
                return flattened

        # Fallback: old format with type at top level
        if "type" not in data:
            logger.error(f"Prefab file must contain a 'type' field: {prefab_path}")
            return None

        self._prefab_cache[prefab_path] = data
        logger.debug(f"Loaded prefab: {prefab_path}")
        return data

    def _flatten_prefab_data(self, data: dict[str, Any]) -> dict[str, Any]:
        """Flatten CloudFormation-style prefab data to internal format.

        Converts:
            name: "Player"
            type: CollisionShape2D
            properties:
              shape: "rectangle"
              physics:
                body_type: "dynamic"
            scripts:
              - "scripts/player.py"

        To:
            name: "Player"
            type: CollisionShape2D
            shape: "rectangle"
            physics:
              body_type: "dynamic"
            scripts:
              - "scripts/player.py"

        Args:
            data: The CF-style resource data

        Returns:
            Flattened data dictionary
        """
        result: dict[str, Any] = {}

        for key, value in data.items():
            if key == "properties" and isinstance(value, dict):
                # Flatten properties to top level
                for prop_key, prop_value in value.items():
                    result[prop_key] = prop_value
            else:
                result[key] = value

        return result

    def _merge_node_data(
        self, base: dict[str, Any], overrides: dict[str, Any]
    ) -> dict[str, Any]:
        """Deep merge prefab base data with overrides.

        Args:
            base: The base prefab data
            overrides: The override data from the scene file

        Returns:
            Merged data dictionary
        """
        import copy

        result = copy.deepcopy(base)

        for key, value in overrides.items():
            if key == "prefab":
                continue

            if key in ("physics", "properties") and key in result:
                if isinstance(result[key], dict) and isinstance(value, dict):
                    result[key] = {**result[key], **value}
                else:
                    result[key] = value
            else:
                result[key] = value

        return result

    def _parse_script_entries(self, scripts_data: list) -> list[tuple[Any, int]]:
        """Parse script entries and load scripts with priorities.

        Scripts can be specified as:
        - Simple string path: "scripts/player.py"
        - Dict with path and priority: {path: "scripts/player.py", priority: 10}

        Args:
            scripts_data: List of script entries from YAML

        Returns:
            List of (script_instance, priority) tuples, sorted by priority
        """
        entries: list[tuple[Any, int]] = []

        for entry in scripts_data:
            if isinstance(entry, str):
                # Simple string path
                script = self._load_script(entry)
                priority = 0
            elif isinstance(entry, dict):
                # Dict with path and optional priority
                path = entry.get("path")
                if path is None:
                    logger.warning("Script entry missing 'path' field")
                    continue
                priority = entry.get("priority", 0)
                script = self._load_script(path)
            else:
                logger.warning(f"Invalid script entry type: {type(entry)}")
                continue

            if script is not None:
                entries.append((script, priority))

        # Sort by priority (lower runs first)
        entries.sort(key=lambda x: x[1])
        return entries

    def _load_script(self, script_path: str) -> Any:
        """Load a script from a Python file.

        Args:
            script_path: Path to the script file (relative to project root)

        Returns:
            A script instance or None if loading failed
        """
        # Check cache
        if script_path in self._script_cache:
            # Return a new instance from cached class
            return self._script_cache[script_path]()

        full_path = self.project_root / script_path
        if not full_path.exists():
            logger.error(f"Script file not found: {full_path}")
            return None

        try:
            # Load the module dynamically
            module_name = full_path.stem
            spec = importlib.util.spec_from_file_location(module_name, full_path)
            if spec is None or spec.loader is None:
                logger.error(f"Failed to load script spec: {script_path}")
                return None

            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)

            # Find the script class (first class that's not imported)
            script_class = None
            for name in dir(module):
                obj = getattr(module, name)
                if (
                    isinstance(obj, type)
                    and obj.__module__ == module_name
                    and hasattr(obj, "update")  # Must have update method
                ):
                    script_class = obj
                    break

            if script_class is None:
                # Try to find any class with lifecycle methods
                for name in dir(module):
                    obj = getattr(module, name)
                    if (
                        isinstance(obj, type)
                        and obj.__module__ == module_name
                        and (hasattr(obj, "awake") or hasattr(obj, "start") or hasattr(obj, "update"))
                    ):
                        script_class = obj
                        break

            if script_class is None:
                logger.warning(f"No script class found in: {script_path}")
                return None

            # Cache and instantiate
            self._script_cache[script_path] = script_class
            return script_class()

        except Exception as e:
            logger.error(f"Failed to load script '{script_path}': {e}")
            import traceback
            traceback.print_exc()
            return None

    def clear_script_cache(self) -> None:
        """Clear the script cache."""
        self._script_cache.clear()

    def _setup_node_signals(self, node: Node, signals_data: list) -> None:
        """Define signals on a node from YAML data.

        Args:
            node: The node to define signals on
            signals_data: List of signal definitions from YAML
        """
        for signal_data in signals_data:
            name = signal_data.get("name")
            if name is None:
                logger.warning("Signal definition missing 'name' field")
                continue

            parameters = signal_data.get("parameters", [])
            description = signal_data.get("description", "")
            node.define_signal(name, parameters, description)
            logger.debug(f"Defined signal '{name}' on '{node.name}'")

    def _setup_scene_connections(self, scene: "Scene", connections_data: list) -> None:
        """Setup signal connections after scene is fully loaded.

        Handles both scene-level connections and node-level pending connections.

        Args:
            scene: The loaded scene
            connections_data: List of connection definitions from YAML
        """
        if scene.root is None:
            return

        # Process scene-level connections
        for conn_data in connections_data:
            self._create_connection(scene.root, conn_data)

        # Process node-level pending connections
        self._process_pending_connections(scene.root, scene.root)

    def _process_pending_connections(self, node: Node, root: Node) -> None:
        """Process pending connections stored on nodes during parsing.

        Args:
            node: The current node to process
            root: The root node for path resolution
        """
        # Process this node's pending connections
        if hasattr(node, "_pending_connections"):
            for conn_data in node._pending_connections:
                # For node-level connections, source is the current node
                conn_data_with_source = dict(conn_data)
                conn_data_with_source["_source_node"] = node
                self._create_connection(root, conn_data_with_source)
            delattr(node, "_pending_connections")

        # Process children
        for child in node.children:
            self._process_pending_connections(child, root)

    def _create_connection(self, root: Node, conn_data: dict) -> None:
        """Create a single signal connection.

        Args:
            root: The root node for path resolution
            conn_data: Connection data dictionary
        """
        # Get source node
        if "_source_node" in conn_data:
            source_node = conn_data["_source_node"]
        else:
            source_path = conn_data.get("source")
            if source_path is None:
                logger.warning("Connection missing 'source' field")
                return
            source_node = self._resolve_node_path(root, source_path)

        if source_node is None:
            logger.warning(f"Connection source not found: {conn_data.get('source', 'unknown')}")
            return

        # Get target node
        target_path = conn_data.get("target")
        if target_path is None:
            logger.warning("Connection missing 'target' field")
            return

        target_node = self._resolve_node_path(root, target_path)
        if target_node is None:
            logger.warning(f"Connection target not found: {target_path}")
            return

        # Get signal and method
        signal_name = conn_data.get("signal")
        if signal_name is None:
            logger.warning("Connection missing 'signal' field")
            return

        method_name = conn_data.get("method")
        if method_name is None:
            logger.warning("Connection missing 'method' field")
            return

        # Optional flags
        one_shot = conn_data.get("one_shot", False)
        deferred = conn_data.get("deferred", False)

        # Check if signal exists
        if not source_node.has_signal(signal_name):
            logger.warning(
                f"Signal '{signal_name}' not defined on '{source_node.name}'"
            )
            return

        # Create the connection
        if source_node.connect(signal_name, target_node, method_name, one_shot, deferred):
            logger.debug(
                f"Connected {source_node.name}.{signal_name} -> "
                f"{target_node.name}.{method_name}"
            )

    def _resolve_node_path(self, root: Node, path: str) -> Node | None:
        """Resolve a node path to a node.

        Args:
            root: The root node for path resolution
            path: The node path (e.g., "Player" or "UI/HealthBar")

        Returns:
            The resolved node or None if not found
        """
        # Try direct path first
        if "/" in path:
            return root.get_node(path)
        else:
            # Try finding by name in the tree
            return root.find_node(path)


def load_scene_from_string(yaml_string: str, project_root: Path | None = None) -> Scene | None:
    """Load a scene from a YAML string.

    Args:
        yaml_string: The YAML content
        project_root: The project root directory

    Returns:
        The loaded Scene or None if loading failed
    """
    loader = SceneLoader(project_root)
    try:
        data = yaml.safe_load(yaml_string)
        return loader._parse_scene(data, "<string>")
    except Exception as e:
        logger.error(f"Failed to parse scene from string: {e}")
        return None
