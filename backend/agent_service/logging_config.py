"""
Logging configuration for the Agent-Based RAG Backend service.

This module sets up structured logging for the agent service with appropriate
levels, formatters, and handlers for different environments.
"""

import logging
import sys
import os
from typing import Optional
from logging.handlers import RotatingFileHandler


def setup_logging(log_level: Optional[str] = None) -> logging.Logger:
    """
    Set up logging configuration for the agent service.

    Args:
        log_level: Optional log level to use (DEBUG, INFO, WARNING, ERROR, CRITICAL)

    Returns:
        logging.Logger: Configured logger instance
    """
    # Determine log level
    level = log_level or os.getenv("LOG_LEVEL", "INFO").upper()
    numeric_level = getattr(logging, level, logging.INFO)

    # Create logger
    logger = logging.getLogger("agent_service")
    logger.setLevel(numeric_level)

    # Prevent duplicate handlers
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)

    # Create file handler with rotation
    log_dir = os.getenv("LOG_DIR", "logs")
    os.makedirs(log_dir, exist_ok=True)

    file_handler = RotatingFileHandler(
        os.path.join(log_dir, "agent_service.log"),
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(numeric_level)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    return logger


# Initialize logger
logger = setup_logging()


def get_logger() -> logging.Logger:
    """
    Get the configured logger instance.

    Returns:
        logging.Logger: The configured logger instance
    """
    return logger