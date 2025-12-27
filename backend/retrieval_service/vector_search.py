"""
Vector search functionality for the RAG retrieval service.

This module handles performing similarity search against Qdrant vectors
and returning relevant content chunks.
"""

from typing import List, Optional
from .models import RetrievalResult
from .vector_db import get_vector_db_manager
from .logging_config import get_logger
from .exceptions import VectorDBError, NoResultsError


class VectorSearchEngine:
    """Performs vector similarity search against the Qdrant database."""

    def __init__(self):
        """Initialize the VectorSearchEngine."""
        self._vector_db_manager = get_vector_db_manager()
        self._logger = get_logger()

    def search_similar_chunks(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        min_relevance: float = 0.3,
        collection_name: Optional[str] = None
    ) -> List[RetrievalResult]:
        """
        Search for similar content chunks in the vector database.

        Args:
            query_embedding: The embedding vector for the query
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold
            collection_name: Name of the collection to search (optional)

        Returns:
            List[RetrievalResult]: List of similar content chunks with metadata
        """
        try:
            # Perform the similarity search
            results = self._vector_db_manager.search_similar_vectors(
                query_vector=query_embedding,
                top_k=top_k,
                min_relevance=min_relevance,
                collection_name=collection_name
            )

            # Log the search results
            self._logger.info(f"Found {len(results)} similar chunks in collection")

            # If no results found, raise an exception
            if not results:
                self._logger.warning(f"No results found for query with min_relevance {min_relevance}")
                raise NoResultsError(f"No relevant content found for the query with relevance threshold {min_relevance}")

            return results

        except Exception as e:
            self._logger.error(f"Error performing vector search: {str(e)}")
            raise VectorDBError(f"Vector search failed: {str(e)}")

    def search_similar_chunks_by_text(
        self,
        query_text: str,
        query_embedding: List[float],
        top_k: int = 5,
        min_relevance: float = 0.3,
        query_types: Optional[List[str]] = None,
        collection_name: Optional[str] = None
    ) -> List[RetrievalResult]:
        """
        Search for similar content chunks by text query and embedding.

        Args:
            query_text: The original query text
            query_embedding: The embedding vector for the query
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold
            query_types: List of query types to filter by (optional)
            collection_name: Name of the collection to search (optional)

        Returns:
            List[RetrievalResult]: List of similar content chunks with metadata
        """
        try:
            # Perform the similarity search
            results = self.search_similar_chunks(
                query_embedding=query_embedding,
                top_k=top_k,
                min_relevance=min_relevance,
                collection_name=collection_name
            )

            # If query types are specified, filter results by type
            if query_types:
                self._logger.info(f"Applying query type filter: {query_types}")
                results = self.filter_results_by_query_types(results, query_types)

            # Log successful search
            self._logger.info(f"Successfully retrieved {len(results)} results for query: '{query_text[:50]}...'")

            return results

        except Exception as e:
            self._logger.error(f"Error searching similar chunks by text: {str(e)}")
            raise VectorDBError(f"Search by text failed: {str(e)}")

    def filter_results_by_query_types(
        self,
        results: List[RetrievalResult],
        query_types: List[str]
    ) -> List[RetrievalResult]:
        """
        Filter results based on query types by analyzing metadata.
        This is a basic implementation that uses heuristics based on content and metadata.

        Args:
            results: List of retrieval results to filter
            query_types: List of query types to filter by

        Returns:
            List[RetrievalResult]: Filtered list of results
        """
        if not results or not query_types:
            return results

        filtered_results = []
        for result in results:
            # Basic heuristic: match result to query type based on content characteristics
            include_result = False

            for query_type in query_types:
                if query_type.lower() == "conceptual":
                    # Conceptual results tend to have more explanatory content
                    content_lower = result.content.lower()
                    if any(keyword in content_lower for keyword in [
                        "definition", "means", "represents", "implies", "describes",
                        "explains", "consists of", "includes", "refers to", "is"
                    ]):
                        include_result = True
                        break

                elif query_type.lower() == "factual":
                    # Factual results tend to have numbers, dates, statistics
                    if any(char.isdigit() for char in result.content) or any(
                        keyword in result.content.lower() for keyword in [
                            "number", "amount", "quantity", "size", "duration",
                            "frequency", "percentage", "ratio", "date", "time"
                        ]
                    ):
                        include_result = True
                        break

                elif query_type.lower() == "section-based":
                    # Section-based results match the section title or URL
                    # This would be enhanced in a real implementation
                    include_result = True  # For now, include all for section-based
                    break

            if include_result:
                filtered_results.append(result)

        self._logger.info(f"Filtered {len(results)} results to {len(filtered_results)} based on query types: {query_types}")
        return filtered_results

    def validate_collection_exists(self, collection_name: Optional[str] = None) -> bool:
        """
        Validate that the specified collection exists in Qdrant.

        Args:
            collection_name: Name of the collection to check (optional)

        Returns:
            bool: True if collection exists, False otherwise
        """
        return self._vector_db_manager.collection_exists(collection_name)

    def get_collection_info(self, collection_name: Optional[str] = None):
        """
        Get information about the collection.

        Args:
            collection_name: Name of the collection to get info for (optional)

        Returns:
            CollectionInfo: Information about the collection
        """
        return self._vector_db_manager.get_collection_info(collection_name)


# Global instance of VectorSearchEngine
vector_search_engine = VectorSearchEngine()


def get_vector_search_engine() -> VectorSearchEngine:
    """
    Get the global VectorSearchEngine instance.

    Returns:
        VectorSearchEngine: The global VectorSearchEngine instance
    """
    return vector_search_engine