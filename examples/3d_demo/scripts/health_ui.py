"""Health UI script for 3D demo.

Demonstrates:
- Receiving signals from other nodes
- UI layer (camera-independent)
- Getting scripts from other nodes
"""


class HealthUI:
    """Displays player health in the UI layer."""

    def __init__(self) -> None:
        self.node = None  # Set by engine
        self.max_width = 200
        self.height = 20
        self.health_percent = 1.0
        self._player_health = None

    def awake(self) -> None:
        """Called when node enters the scene."""
        print("HealthUI initialized")

    def start(self) -> None:
        """Called before first update."""
        # Find the player and get their health script
        if self.node.scene is not None:
            player = self.node.scene.find_node("Player")
            if player is not None:
                # Use multi-script API to get the health script
                self._player_health = player.get_script("PlayerHealth")
                if self._player_health is not None:
                    print("HealthUI connected to PlayerHealth")
                    self.health_percent = self._player_health.health_percent

    def update(self, delta: float) -> None:
        """Called every frame."""
        # Continuously update from player health
        if self._player_health is not None:
            self.health_percent = self._player_health.health_percent

    def on_player_damaged(self, damage: int, source: str) -> None:
        """Signal callback: Player took damage.

        This method is called when the player emits 'player_damaged' signal
        because we connected to it in the scene YAML.
        """
        print(f"UI: Player damaged by {damage} from {source}")

        # Update health display
        if self._player_health is not None:
            self.health_percent = self._player_health.health_percent

        # Could trigger visual effects here:
        # - Flash red
        # - Shake effect
        # - Damage numbers

    def render(self) -> None:
        """Called during rendering (if the node is in a renderable layer).

        Note: This is a placeholder. Actual UI rendering would use
        raylib drawing functions or a UI system.
        """
        # Calculate bar width based on health
        bar_width = int(self.max_width * self.health_percent)

        # In a real implementation:
        # DrawRectangle(x, y, max_width, height, GRAY)  # Background
        # DrawRectangle(x, y, bar_width, height, RED)   # Health bar

    # Hot reload support
    def preserve_state(self) -> dict:
        """Preserve state for hot reload."""
        return {
            "health_percent": self.health_percent,
            "max_width": self.max_width,
            "height": self.height,
        }

    def restore_state(self, state: dict) -> None:
        """Restore state after hot reload."""
        self.health_percent = state.get("health_percent", 1.0)
        self.max_width = state.get("max_width", 200)
        self.height = state.get("height", 20)

        # Re-find player health script
        if self.node.scene is not None:
            player = self.node.scene.find_node("Player")
            if player is not None:
                self._player_health = player.get_script("PlayerHealth")
