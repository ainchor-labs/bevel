"""Jolt Physics C++ bindings for Bevel engine.

This package contains the pybind11 bindings for Jolt Physics.
The _jolt module is built from C++ during package installation.
"""

try:
    from ._jolt import JoltPhysicsWorld
except ImportError:
    JoltPhysicsWorld = None  # type: ignore
