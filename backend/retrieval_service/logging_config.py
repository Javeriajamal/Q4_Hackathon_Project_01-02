"""
Logging configuration for the RAG retrieval service.

This module sets up logging with appropriate formatting and handlers
for the retrieval service.
"""

import logging
import sys
from typing import Optional


def setup_logging(level: Optional[str] = None) -> logging.Logger:
    """
    Set up logging configuration for the retrieval service.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
               If None, defaults to INFO

    Returns:
        logging.Logger: Configured logger instance
    """
    # Determine logging level
    log_level = getattr(logging, level or "INFO")

    # Create logger
    logger = logging.getLogger("retrieval_service")
    logger.setLevel(log_level)

    # Prevent duplicate handlers if logger already configured
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console_handler)

    return logger


# Global logger instance
logger = setup_logging()


def get_logger() -> logging.Logger:
    """
    Get the configured logger instance.

    Returns:
        logging.Logger: The configured logger instance
    """
    return logger