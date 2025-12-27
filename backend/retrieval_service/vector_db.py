"""
Qdrant client initialization and connection management for the RAG retrieval service.

This module handles connecting to Qdrant, managing the client instance,
and providing methods for vector search operations.
"""

from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
from .config import config, get_config
from .models import RetrievalResult, ChunkMetadata
from .logging_config import get_logger


class VectorDBManager:
    """Manages Qdrant client connection and vector database operations."""

    def __init__(self):
        """Initialize the VectorDBManager with configuration."""
        self._client: Optional[QdrantClient] = None
        self._config = get_config()
        self._logger = get_logger()

    def get_client(self) -> QdrantClient:
        """
        Get or create the Qdrant client instance.

        Returns:
            QdrantClient: The Qdrant client instance
        """
        if self._client is None:
            if not self._config.QDRANT_URL or not self._config.QDRANT_API_KEY:
                raise ValueError("QDRANT_URL and QDRANT_API_KEY must be configured")

            self._client = QdrantClient(
                url=self._config.QDRANT_URL,
                api_key=self._config.QDRANT_API_KEY,
                timeout=30
            )
            self._logger.info(f"Qdrant client initialized for URL: {self._config.QDRANT_URL}")

        return self._client

    def collection_exists(self, collection_name: Optional[str] = None) -> bool:
        """
        Check if the specified collection exists.

        Args:
            collection_name: Name of the collection to check. If None, uses default collection.

        Returns:
            bool: True if collection exists, False otherwise
        """
        client = self.get_client()
        collection_name = collection_name or self._config.COLLECTION_NAME

        try:
            collections = client.get_collections()
            existing_collection_names = [collection.name for collection in collections.collections]
            return collection_name in existing_collection_names
        except Exception as e:
            self._logger.error(f"Error checking if collection exists: {str(e)}")
            return False

    def search_similar_vectors(
        self,
        query_vector: List[float],
        top_k: int = 5,
        min_relevance: float = 0.3,
        collection_name: Optional[str] = None
    ) -> List[RetrievalResult]:
        """
        Search for similar vectors in the Qdrant collection.

        Args:
            query_vector: The query vector to search for
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold
            collection_name: Name of the collection to search. If None, uses default collection.

        Returns:
            List[RetrievalResult]: List of similar vectors with metadata
        """
        client = self.get_client()
        collection_name = collection_name or self._config.COLLECTION_NAME

        # Perform the search
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=top_k,
            score_threshold=min_relevance,
            with_payload=True  # Include payload (metadata) in results
        )

        # Format results
        formatted_results = []
        for i, result in enumerate(search_results):
            # Extract metadata from payload
            payload = result.payload or {}
            metadata = ChunkMetadata(
                source_url=payload.get('url', ''),
                section_title=payload.get('section_title', ''),
                chunk_index=payload.get('chunk_index', 0)
            )

            formatted_result = RetrievalResult(
                chunk_id=str(result.id),
                content=payload.get('text', ''),
                relevance_score=result.score,
                metadata=metadata,
                rank=i + 1
            )
            formatted_results.append(formatted_result)

        self._logger.info(f"Found {len(formatted_results)} similar vectors in collection '{collection_name}'")
        return formatted_results

    def get_collection_info(self, collection_name: Optional[str] = None) -> Optional[models.CollectionInfo]:
        """
        Get information about a collection.

        Args:
            collection_name: Name of the collection. If None, uses default collection.

        Returns:
            CollectionInfo: Information about the collection, or None if not found
        """
        client = self.get_client()
        collection_name = collection_name or self._config.COLLECTION_NAME

        try:
            return client.get_collection(collection_name)
        except Exception as e:
            self._logger.error(f"Error getting collection info: {str(e)}")
            return None


# Global instance of VectorDBManager
vector_db_manager = VectorDBManager()


def get_vector_db_manager() -> VectorDBManager:
    """
    Get the global VectorDBManager instance.

    Returns:
        VectorDBManager: The global VectorDBManager instance
    """
    return vector_db_manager