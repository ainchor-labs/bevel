# Bevel Hot Reload Guide

## What is Hot Reload?

Hot reload allows you to modify your game's code and scenes while the game is running, and see changes immediately without restarting. This significantly speeds up development and iteration.

## How to Enable Hot Reload

Hot reload is **enabled by default** when you run your game with the `bevel run` command.

```bash
# Hot reload is enabled by default
bevel run

# Explicitly enable hot reload
bevel run --watch

# Disable hot reload if needed
bevel run --no-watch
```

## Requirements

Hot reload requires the `watchdog` library. Install it with:

```bash
pip install watchdog
# or
pip install bevel[hotreload]
```

## What Gets Reloaded?

By default, Bevel watches these directories:
- `scenes/` - Scene YAML files
- `scripts/` - Python script files  
- `prefabs/` - Prefab YAML files

When you modify a file in these directories:

### Python Scripts (.py)
- The script module is reloaded
- The current scene is reloaded with the new script code
- **Your game state is preserved** (positions, velocities, etc.)

### Scene/Prefab Files (.yaml, .yml)
- The scene is reloaded with the new structure
- Node positions and properties are preserved where possible

## Configuring Watch Paths

You can customize which directories are watched by adding a `hot_reload` section to your `project.yaml`:

```yaml
hot_reload:
  watch_paths:
    - "scenes"
    - "scripts"
    - "prefabs"
    - "custom_ai"      # Watch additional directories
    - "behaviors"
```

## Example Workflow

1. Start your game:
   ```bash
   cd my_game
   bevel run
   ```

2. You'll see:
   ```
   Hot reload active - edit .py or .yaml files to see changes
   Hot reload watching: scenes
   Hot reload watching: scripts
   Hot reload watching: prefabs
   ```

3. Edit a script (e.g., `scripts/player.py`) in your editor

4. Save the file

5. The console shows:
   ```
   Queued reload for: player.py
   Hot reloading: /path/to/scripts/player.py
   Reloaded module: scripts.player
   ```

6. Your changes are immediately active in the running game!

## State Preservation

Hot reload intelligently preserves state during reloads:

### Automatically Preserved:
- Node positions, rotation, scale
- Active/visible states
- Layer and z-index
- Custom node properties

### Script State Preservation:
Your scripts can define custom state preservation:

```python
class MyScript:
    def __init__(self):
        self.health = 100
        self.score = 0
    
    # Optional: custom state preservation
    def preserve_state(self):
        """Return state to preserve during reload"""
        return {
            "health": self.health,
            "score": self.score,
            "custom_data": self.custom_data
        }
    
    def restore_state(self, state):
        """Restore state after reload"""
        self.health = state.get("health", 100)
        self.score = state.get("score", 0)
        self.custom_data = state.get("custom_data")
```

Without custom methods, hot reload will automatically preserve simple types (int, float, str, bool, list, dict).

## Limitations

### What is NOT watched:
- **Asset files** (textures, sounds, fonts) are loaded once and cached
- Files outside the configured watch paths
- Files in the root project directory

### To reload assets:
Restart the game or implement custom asset reloading in your code.

## Troubleshooting

### "Hot reload not available (watchdog not installed)"
Install watchdog:
```bash
pip install watchdog
```

### "No watch paths found"
Check that your `project.yaml` watch paths exist. The default paths are:
- `scenes/`
- `scripts/`
- `prefabs/`

Create these directories if they don't exist.

### Changes not appearing
1. Check the console for reload messages
2. Ensure you're editing files in watched directories
3. Only `.py`, `.yaml`, and `.yml` files trigger reloads
4. Check for syntax errors in your code (reload may fail silently)

### State is lost after reload
Implement `preserve_state()` and `restore_state()` methods in your scripts.

## Best Practices

1. **Save frequently** - Each save triggers a reload
2. **Test small changes** - Make incremental changes to catch errors early
3. **Watch the console** - Look for reload messages and errors
4. **Use state preservation** - For complex scripts, implement preserve/restore methods
5. **Disable when not needed** - Use `--no-watch` for production builds

## Performance

Hot reload has minimal performance impact:
- File watching runs in a background thread
- Reloads only happen when files change
- Changes are debounced (0.1 second delay)
- No impact when files aren't being edited
