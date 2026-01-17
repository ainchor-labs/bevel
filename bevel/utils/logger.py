"""Logging utilities for Bevel."""

import logging
import sys
from typing import Optional


def setup_logging(level: int = logging.INFO, log_file: Optional[str] = None) -> None:
    """Set up logging for Bevel.

    Args:
        level: The logging level (default: INFO)
        log_file: Optional file path to write logs to
    """
    formatter = logging.Formatter(
        "[%(asctime)s] %(levelname)s [%(name)s] %(message)s",
        datefmt="%H:%M:%S"
    )

    root_logger = logging.getLogger("bevel")
    root_logger.setLevel(level)

    # Clear any existing handlers
    root_logger.handlers.clear()

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # File handler (optional)
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)


def get_logger(name: str) -> logging.Logger:
    """Get a logger with the given name under the bevel namespace.

    Args:
        name: The name of the logger (will be prefixed with 'bevel.')

    Returns:
        A configured logger instance
    """
    return logging.getLogger(f"bevel.{name}")
