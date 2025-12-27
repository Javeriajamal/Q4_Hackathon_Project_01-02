"""
Configuration module for the Agent-Based RAG Backend service.

This module handles loading settings from environment variables and provides
a centralized configuration interface for the agent service.
"""

import os
from typing import Optional
from dataclasses import dataclass


@dataclass
class Config:
    """Configuration class to manage application settings."""

    # OpenAI settings
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    OPENAI_MODEL_NAME: str = os.getenv("OPENAI_MODEL_NAME", "gpt-4-turbo")

    # Cohere settings (for compatibility with existing retrieval pipeline)
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-multilingual-v3.0")

    # Qdrant settings (for compatibility with existing retrieval pipeline)
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

    # Service settings
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_MIN_RELEVANCE: float = float(os.getenv("DEFAULT_MIN_RELEVANCE", "0.3"))
    DEFAULT_MAX_QUERY_LENGTH: int = int(os.getenv("DEFAULT_MAX_QUERY_LENGTH", "500"))
    HOST: str = os.getenv("HOST", "0.0.0.0")
    PORT: int = int(os.getenv("PORT", "8001"))
    SESSION_TIMEOUT_MINUTES: int = int(os.getenv("SESSION_TIMEOUT_MINUTES", "30"))

    # Performance settings
    MAX_CONCURRENT_QUERIES: int = int(os.getenv("MAX_CONCURRENT_QUERIES", "50"))
    RESPONSE_TIME_THRESHOLD: float = float(os.getenv("RESPONSE_TIME_THRESHOLD", "10.0"))  # seconds

    # Validation settings
    MIN_PRECISION_THRESHOLD: float = float(os.getenv("MIN_PRECISION_THRESHOLD", "0.7"))
    MIN_RECALL_THRESHOLD: float = float(os.getenv("MIN_RECALL_THRESHOLD", "0.7"))
    MIN_F1_THRESHOLD: float = float(os.getenv("MIN_F1_THRESHOLD", "0.7"))

    def validate_config(self) -> bool:
        """
        Validate that required configuration values are present.

        Returns:
            bool: True if all required configuration is valid, False otherwise
        """
        required_vars = [
            self.OPENAI_API_KEY,
            self.COHERE_API_KEY,
            self.QDRANT_URL,
            self.QDRANT_API_KEY
        ]

        return all(var for var in required_vars)

    def get_embedding_input_type(self) -> str:
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