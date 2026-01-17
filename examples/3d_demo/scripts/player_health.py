"""Player health system for 3D demo.

Demonstrates:
- Signal system (emitting signals)
- Multi-script support (coexists with PlayerMovement)
- Script communication
"""


class PlayerHealth:
    """Manages player health and damage."""

    def __init__(self) -> None:
        self.node = None  # Set by engine
        self.max_health = 100
        self.current_health = 100
        self.invincibility_time = 1.0  # Seconds after damage
        self._invincibility_timer = 0.0

    def awake(self) -> None:
        """Called when node enters the scene."""
        # Define signals on this node
        self.node.define_signal(
            "player_damaged",
            parameters=["damage", "source"],
            description="Emitted when player takes damage",
        )
        self.node.define_signal(
            "player_died",
            parameters=[],
            description="Emitted when player health reaches 0",
        )
        print(f"PlayerHealth initialized with {self.max_health} HP")

    def start(self) -> None:
        """Called before first update."""
        self.current_health = self.max_health

    def update(self, delta: float) -> None:
        """Called every frame."""
        # Update invincibility timer
        if self._invincibility_timer > 0:
            self._invincibility_timer -= delta

    def take_damage(self, amount: int, source: str = "unknown") -> None:
        """Apply damage to the player.

        Args:
            amount: Damage amount
            source: What caused the damage
        """
        # Check invincibility
        if self._invincibility_timer > 0:
            return

        # Apply damage
        old_health = self.current_health
        self.current_health = max(0, self.current_health - amount)

        print(f"Player took {amount} damage from {source}! Health: {self.current_health}/{self.max_health}")

        # Start invincibility
        self._invincibility_timer = self.invincibility_time

        # Emit damage signal
        self.node.emit_signal("player_damaged", amount, source)

        # Check for death
        if self.current_health <= 0 and old_health > 0:
            self._die()

    def heal(self, amount: int) -> None:
        """Heal the player.

        Args:
            amount: Amount to heal
        """
        old_health = self.current_health
        self.current_health = min(self.max_health, self.current_health + amount)

        if self.current_health != old_health:
            print(f"Player healed {self.current_health - old_health} HP. Health: {self.current_health}/{self.max_health}")

    def _die(self) -> None:
        """Handle player death."""
        print("Player died!")
        self.node.emit_signal("player_died")

    @property
    def health_percent(self) -> float:
        """Get health as a percentage (0.0 to 1.0)."""
        return self.current_health / self.max_health if self.max_health > 0 else 0

    @property
    def is_invincible(self) -> bool:
        """Check if player is currently invincible."""
        return self._invincibility_timer > 0

    # Hot reload support
    def preserve_state(self) -> dict:
        """Preserve state for hot reload."""
        return {
            "current_health": self.current_health,
            "max_health": self.max_health,
            "invincibility_timer": self._invincibility_timer,
        }

    def restore_state(self, state: dict) -> None:
        """Restore state after hot reload."""
        self.current_health = state.get("current_health", self.max_health)
        self.max_health = state.get("max_health", 100)
        self._invincibility_timer = state.get("invincibility_timer", 0)
