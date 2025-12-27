"""
Cohere client initialization and embedding generation for the RAG retrieval service.

This module handles connecting to Cohere, managing the client instance,
and providing methods for generating embeddings from text.
"""

from typing import List, Union
import cohere
from .config import config, get_config
from .logging_config import get_logger


class EmbeddingGenerator:
    """Manages Cohere client and provides embedding generation functionality."""

    def __init__(self):
        """Initialize the EmbeddingGenerator with configuration."""
        self._client: Optional[cohere.Client] = None
        self._config = get_config()
        self._logger = get_logger()

    def get_client(self) -> cohere.Client:
        """
        Get or create the Cohere client instance.

        Returns:
            cohere.Client: The Cohere client instance
        """
        if self._client is None:
            if not self._config.COHERE_API_KEY:
                raise ValueError("COHERE_API_KEY must be configured")

            self._client = cohere.Client(self._config.COHERE_API_KEY)
            self._logger.info("Cohere client initialized")

        return self._client

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere API.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List[List[float]]: List of embeddings, one for each input text
        """
        client = self.get_client()

        try:
            # Generate embeddings
            response = client.embed(
                texts=texts,
                model=self._config.COHERE_EMBED_MODEL,
                input_type=self._config.get_embedding_input_type()  # Using search_query for queries
            )

            # Extract embeddings from response
            embeddings = [embedding for embedding in response.embeddings]
            self._logger.info(f"Generated embeddings for {len(texts)} texts")
            return embeddings

        except cohere.CohereError as e:
            self._logger.error(f"Cohere API error: {str(e)}")
            if "rate limit" in str(e).lower():
                raise Exception(f"Rate limit exceeded: {str(e)}")
            else:
                raise Exception(f"Cohere API error: {str(e)}")
        except Exception as e:
            self._logger.error(f"Embedding generation error: {str(e)}")
            raise Exception(f"Failed to generate embeddings: {str(e)}")

    def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            List[float]: Embedding vector for the input text
        """
        embeddings = self.generate_embeddings([text])
        return embeddings[0]  # Return the first (and only) embedding


# Global instance of EmbeddingGenerator
embedding_generator = EmbeddingGenerator()


def get_embedding_generator() -> EmbeddingGenerator:
    """
    Get the global EmbeddingGenerator instance.

    Returns:
        EmbeddingGenerator: The global EmbeddingGenerator instance
    """
    return embedding_generator


def validate_embedding_quality(embedding: List[float]) -> bool:
    """
    Validate that the embedding has proper dimensions and values.

    Args:
        embedding: The embedding vector to validate

    Returns:
        bool: True if embedding is valid, False otherwise
    """
    if not embedding:
        return False

    # Check that all values are finite numbers (not NaN or Infinity)
    for value in embedding:
        if not (isinstance(value, (int, float)) and abs(value) < float('inf')):
            return False

    # Check dimensions - should be 1024 for Cohere multilingual-v3.0 model
    if len(embedding) != 1024:
        return False

    return True