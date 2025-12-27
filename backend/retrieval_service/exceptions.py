"""
Custom exception classes for the RAG retrieval service.

This module defines application-specific exceptions that can be raised
throughout the service to provide more meaningful error information.
"""


class RetrievalServiceError(Exception):
    """Base exception class for the retrieval service."""
    pass


class ConfigurationError(RetrievalServiceError):
    """Raised when there are configuration issues."""
    pass


class EmbeddingError(RetrievalServiceError):
    """Raised when there are issues with embedding generation."""
    pass


class VectorDBError(RetrievalServiceError):
    """Raised when there are issues with vector database operations."""
    pass


class ValidationError(RetrievalServiceError):
    """Raised when validation fails."""
    pass


class QueryProcessingError(RetrievalServiceError):
    """Raised when there are issues processing a query."""
    pass


class SearchResultError(RetrievalServiceError):
    """Raised when there are issues with search results."""
    pass


class APIError(RetrievalServiceError):
    """Raised when there are API-related errors."""
    pass


class CohereAPIError(APIError):
    """Raised when there are Cohere API errors."""
    pass


class QdrantAPIError(APIError):
    """Raised when there are Qdrant API errors."""
    pass


class RateLimitError(RetrievalServiceError):
    """Raised when API rate limits are exceeded."""
    pass


class NoResultsError(RetrievalServiceError):
    """Raised when no results are found for a query."""
    pass


class InvalidQueryTypeError(RetrievalServiceError):
    """Raised when an invalid query type is provided."""
    pass


class InvalidParameterError(ValidationError):
    """Raised when an invalid parameter is provided."""
    pass


def handle_retrieval_error(error: Exception, context: str = "") -> RetrievalServiceError:
    """
    Standardize error handling by wrapping exceptions in appropriate service errors.

    Args:
        error: The original exception to handle
        context: Additional context about where the error occurred

    Returns:
        RetrievalServiceError: A wrapped service-specific error
    """
    error_msg = f"{context}: {str(error)}" if context else str(error)

    # Map common errors to specific service errors
    if "rate limit" in str(error).lower():
        return RateLimitError(error_msg)
    elif "validation" in str(error).lower() or "invalid" in str(error).lower():
        return ValidationError(error_msg)
    elif "cohere" in str(error).lower():
        return CohereAPIError(error_msg)
    elif "qdrant" in str(error).lower() or "vector" in str(error).lower():
        return QdrantAPIError(error_msg)
    elif "configuration" in str(error).lower() or "config" in str(error).lower():
        return ConfigurationError(error_msg)
    elif "embedding" in str(error).lower():
        return EmbeddingError(error_msg)
    elif "query" in str(error).lower():
        return QueryProcessingError(error_msg)

    # Default to a generic retrieval service error
    return RetrievalServiceError(error_msg)