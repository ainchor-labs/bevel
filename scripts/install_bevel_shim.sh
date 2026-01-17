#!/usr/bin/env bash
# Install the local `bevel` shim into the user's local bin directory
# Usage: run from repo root or anywhere; it will resolve the repo root

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SHIM_SRC="$REPO_ROOT/bin/bevel"
DEST_DIR="$HOME/.local/bin"
DEST_PATH="$DEST_DIR/bevel"

if [ ! -f "$SHIM_SRC" ]; then
  echo "Error: shim not found at $SHIM_SRC" >&2
  exit 2
fi

mkdir -p "$DEST_DIR"
chmod +x "$SHIM_SRC"
ln -sf "$SHIM_SRC" "$DEST_PATH"

echo "Installed shim to $DEST_PATH"
echo "Make sure $DEST_DIR is on your PATH (add 'export PATH=\"$HOME/.local/bin:\$PATH\"' to ~/.bashrc)"
