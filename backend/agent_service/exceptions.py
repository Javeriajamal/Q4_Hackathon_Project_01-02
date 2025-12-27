"""
Custom exception classes for the Agent-Based RAG Backend service.

This module defines application-specific exceptions that can be raised
throughout the service to provide more meaningful error information.
"""


class AgentServiceError(Exception):
    """Base exception class for the agent service."""
    pass


class ConfigurationError(AgentServiceError):
    """Raised when there are configuration issues."""
    pass


class EmbeddingError(AgentServiceError):
    """Raised when there are issues with embedding generation."""
    pass


class VectorDBError(AgentServiceError):
    """Raised when there are issues with vector database operations."""
    pass


class ValidationError(AgentServiceError):
    """Raised when validation fails."""
    pass


class QueryProcessingError(AgentServiceError):
    """Raised when there are issues processing a query."""
    pass


class SearchResultError(AgentServiceError):
    """Raised when there are issues with search results."""
    pass


class APIError(AgentServiceError):
    """Raised when there are API-related errors."""
    pass


class OpenAIAPIError(APIError):
    """Raised when there are OpenAI API errors."""
    pass


class CohereAPIError(APIError):
    """Raised when there are Cohere API errors."""
    pass


class QdrantAPIError(APIError):
    """Raised when there are Qdrant API errors."""
    pass


class RateLimitError(AgentServiceError):
    """Raised when API rate limits are exceeded."""
    pass


class NoResultsError(AgentServiceError):
    """Raised when no results are found for a query."""
    pass


class InvalidQueryTypeError(AgentServiceError):
    """Raised when an invalid query type is provided."""
    pass


class InvalidParameterError(ValidationError):
    """Raised when an invalid parameter is provided."""
    pass


class SessionNotFoundError(AgentServiceError):
    """Raised when a requested session is not found."""
    pass


class ServiceUnavailableError(AgentServiceError):
    """Raised when a required service is unavailable."""
    pass


class RetrievalTimeoutError(AgentServiceError):
    """Raised when a retrieval operation times out."""
    pass


class ContextInjectionError(AgentServiceError):
    """Raised when there are issues injecting context into the agent."""
    pass


class ResponseFormattingError(AgentServiceError):
    """Raised when there are issues formatting the response."""
    pass


def handle_retrieval_error(error: Exception, context: str = "") -> AgentServiceError:
    """
    Standardize error handling by wrapping exceptions in appropriate service errors.

    Args:
        error: The original exception to handle
        context: Additional context about where the error occurred

    Returns:
        AgentServiceError: A wrapped service-specific error
    """
    error_msg = f"{context}: {str(error)}" if context else str(error)

    # Map common errors to specific service errors
    if "rate limit" in str(error).lower():
        return RateLimitError(error_msg)
    elif "validation" in str(error).lower() or "invalid" in str(error).lower():
        return ValidationError(error_msg)
    elif "openai" in str(error).lower():
        return OpenAIAPIError(error_msg)
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
    elif "session" in str(error).lower():
        return SessionNotFoundError(error_msg)
    elif "timeout" in str(error).lower():
        return RetrievalTimeoutError(error_msg)

    # Default to a generic agent service error
    return AgentServiceError(error_msg)