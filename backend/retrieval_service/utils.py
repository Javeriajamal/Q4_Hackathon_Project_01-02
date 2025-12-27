"""
Utility functions for the RAG retrieval service.

This module provides common utility functions for validation,
formatting, and other helper operations.
"""

from typing import Any, Dict, List, Optional
from .models import RetrievalRequest, QueryType
from .logging_config import get_logger


def validate_query_text(query_text: str) -> bool:
    """
    Validate the query text length and content.

    Args:
        query_text: The query text to validate

    Returns:
        bool: True if query text is valid, False otherwise
    """
    if not query_text or not isinstance(query_text, str):
        return False

    if not (1 <= len(query_text) <= 500):
        return False

    return True


def validate_top_k(top_k: int) -> bool:
    """
    Validate the top_k parameter.

    Args:
        top_k: The top_k value to validate

    Returns:
        bool: True if top_k is valid, False otherwise
    """
    if not isinstance(top_k, int):
        return False

    if not (1 <= top_k <= 20):
        return False

    return True


def validate_min_relevance(min_relevance: float) -> bool:
    """
    Validate the min_relevance parameter.

    Args:
        min_relevance: The min_relevance value to validate

    Returns:
        bool: True if min_relevance is valid, False otherwise
    """
    if not isinstance(min_relevance, (int, float)):
        return False

    if not (0.0 <= min_relevance <= 1.0):
        return False

    return True


def validate_query_types(query_types: Optional[List[str]]) -> bool:
    """
    Validate the query_types parameter.

    Args:
        query_types: List of query types to validate

    Returns:
        bool: True if query_types is valid, False otherwise
    """
    if query_types is None:
        return True  # None is valid

    if not isinstance(query_types, list):
        return False

    valid_types = ["conceptual", "factual", "section-based"]
    for qt in query_types:
        if not isinstance(qt, str) or qt not in valid_types:
            return False

    return True


def validate_retrieval_request(request: RetrievalRequest) -> List[str]:
    """
    Validate a retrieval request and return a list of validation errors.

    Args:
        request: The retrieval request to validate

    Returns:
        List[str]: List of validation error messages, empty if valid
    """
    errors = []

    if not validate_query_text(request.query):
        errors.append("Query must be between 1 and 500 characters")

    if not validate_top_k(request.top_k):
        errors.append("top_k must be between 1 and 20")

    if not validate_min_relevance(request.min_relevance):
        errors.append("min_relevance must be between 0.0 and 1.0")

    if not validate_query_types(request.query_types):
        errors.append("All query types must be one of: conceptual, factual, section-based")

    return errors


def clean_text(text: str) -> str:
    """
    Clean and normalize text content.

    Args:
        text: The text to clean

    Returns:
        str: Cleaned text
    """
    if not text:
        return ""

    # Remove extra whitespace and normalize line breaks
    import re
    cleaned = re.sub(r'\s+', ' ', text)
    cleaned = cleaned.strip()

    return cleaned


def validate_text_content(text: str) -> bool:
    """
    Validate that text content is meaningful.

    Args:
        text: The text to validate

    Returns:
        bool: True if text is meaningful, False otherwise
    """
    if not text or len(text.strip()) < 10:  # At least 10 characters
        return False

    # Check if text has enough content (not just special characters)
    content_chars = sum(1 for c in text if c.isalnum() or c.isspace())
    if content_chars / len(text) < 0.5:  # At least 50% should be alphanumeric or spaces
        return False

    return True


def calculate_relevance_score(text1: str, text2: str) -> float:
    """
    Calculate a basic relevance score between two texts (placeholder implementation).
    In a real implementation, this would use more sophisticated methods.

    Args:
        text1: First text for comparison
        text2: Second text for comparison

    Returns:
        float: Relevance score between 0.0 and 1.0
    """
    # This is a placeholder - in a real implementation, this would use
    # more sophisticated text similarity algorithms
    if not text1 or not text2:
        return 0.0

    # Simple word overlap as a placeholder
    words1 = set(text1.lower().split())
    words2 = set(text2.lower().split())

    if not words1 and not words2:
        return 1.0  # Both empty texts are identical
    if not words1 or not words2:
        return 0.0  # One empty, one not

    intersection = words1.intersection(words2)
    union = words1.union(words2)

    return len(intersection) / len(union) if union else 0.0


def format_query_type(query_type: str) -> Optional[QueryType]:
    """
    Convert a string query type to the QueryType enum.

    Args:
        query_type: String representation of query type

    Returns:
        Optional[QueryType]: The corresponding QueryType enum value, or None if invalid
    """
    try:
        return QueryType(query_type.lower())
    except ValueError:
        return None


def get_logger_with_context(context: str):
    """
    Get a logger instance with additional context.

    Args:
        context: Context string to add to the logger

    Returns:
        Logger: Configured logger with context
    """
    from .logging_config import get_logger
    logger = get_logger()
    # We return the same logger since we're using a simple logging setup
    # In a more complex system, we might add context to the logger name
    return logger