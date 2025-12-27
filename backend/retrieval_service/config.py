"""
Configuration module for the RAG retrieval service.

This module handles loading settings from environment variables
and provides a centralized configuration interface.
"""

import os
from typing import Optional


class Config:
    """Configuration class to manage application settings."""

    # Cohere settings
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-multilingual-v3.0")

    # Qdrant settings
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    COLLECTION_NAME: str = os.getenv("COLLECTION_NAME", "rag_embedding")

    # Service settings
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_MIN_RELEVANCE: float = float(os.getenv("DEFAULT_MIN_RELEVANCE", "0.3"))
    HOST: str = os.getenv("HOST", "0.0.0.0")
    PORT: int = int(os.getenv("PORT", "8000"))

    # Validation settings
    MAX_QUERY_LENGTH: int = int(os.getenv("MAX_QUERY_LENGTH", "500"))
    MAX_TOP_K: int = int(os.getenv("MAX_TOP_K", "20"))
    MIN_RELEVANCE_THRESHOLD: float = float(os.getenv("MIN_RELEVANCE_THRESHOLD", "0.0"))
    MAX_RELEVANCE_THRESHOLD: float = float(os.getenv("MAX_RELEVANCE_THRESHOLD", "1.0"))

    @classmethod
    def validate_config(cls) -> bool:
        """
        Validate that required configuration values are present.

        Returns:
            bool: True if all required configuration is valid, False otherwise
        """
        required_vars = [
            cls.COHERE_API_KEY,
            cls.QDRANT_URL,
            cls.QDRANT_API_KEY
        ]

        return all(var for var in required_vars)

    @classmethod
    def get_embedding_input_type(cls) -> str:
        """
        Get the appropriate input type for Cohere embeddings based on usage.

        Returns:
            str: Input type for Cohere API (search_query for queries, search_document for stored content)
        """
        return "search_query"


# Initialize configuration
config = Config()


def get_config() -> Config:
    """
    Get the application configuration instance.

    Returns:
        Config: The configuration instance
    """
    return config