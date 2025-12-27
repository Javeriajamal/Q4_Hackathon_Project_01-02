"""
Retrieval Tool for the Agent-Based RAG Backend service.

This module wraps the existing retrieval pipeline as a tool that can be used
by the OpenAI agent to fetch relevant context from the vector database.
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime

from .models import RetrievalResult, ChunkMetadata
from .logging_config import get_logger
from .exceptions import (
    VectorDBError,
    CohereAPIError,
    ValidationError
)
from backend.retrieval_service.vector_search import get_vector_search_engine
from backend.retrieval_service.embeddings import get_embedding_generator


class RetrievalTool:
    """Wrapper around the existing retrieval pipeline for agent integration."""

    def __init__(self):
        """Initialize the RetrievalTool."""
        self._logger = get_logger()
        self._vector_search_engine = get_vector_search_engine()
        self._embedding_generator = get_embedding_generator()

    def retrieve_context(
        self,
        query: str,
        top_k: int = 5,
        min_relevance: float = 0.3
    ) -> List[Dict[str, Any]]:
        """
        Retrieve context for the agent based on the query.

        Args:
            query: The query text to search for
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold

        Returns:
            List[Dict[str, Any]]: List of context chunks with metadata
        """
        try:
            # Validate input parameters
            if not query or not query.strip():
                raise ValidationError("Query cannot be empty or whitespace only")

            if not isinstance(top_k, int) or not (1 <= top_k <= 20):
                raise ValidationError("top_k must be an integer between 1 and 20")

            if not isinstance(min_relevance, (int, float)) or not (0.0 <= min_relevance <= 1.0):
                raise ValidationError("min_relevance must be a number between 0.0 and 1.0")

            # Generate embedding for the query
            query_embedding = self._embedding_generator.generate_single_embedding(query)

            # Perform vector search
            results = self._vector_search_engine.search_similar_chunks_by_text(
                query_text=query,
                query_embedding=query_embedding,
                top_k=top_k,
                min_relevance=min_relevance
            )

            # Format results for agent consumption
            formatted_results = []
            for result in results:
                formatted_result = {
                    "id": result.chunk_id,
                    "content": result.content,
                    "relevance_score": result.relevance_score,
                    "metadata": {
                        "source_url": result.metadata.source_url,
                        "section_title": result.metadata.section_title,
                        "chunk_index": result.metadata.chunk_index
                    }
                }
                formatted_results.append(formatted_result)

            self._logger.info(f"Retrieved {len(formatted_results)} context chunks for query: '{query[:50]}...'")
            return formatted_results

        except ValidationError as e:
            self._logger.error(f"Validation error in retrieval tool: {str(e)}")
            raise
        except Exception as e:
            self._logger.error(f"Error in retrieval tool: {str(e)}")
            raise VectorDBError(f"Context retrieval failed: {str(e)}")

    def get_tool_specification(self) -> Dict[str, Any]:
        """
        Get the tool specification for OpenAI agent integration.

        Returns:
            Dict[str, Any]: Tool specification in OpenAI-compatible format
        """
        return {
            "type": "function",
            "function": {
                "name": "retrieve_context",
                "description": "Retrieve relevant context from the Physical AI & Humanoid Robotics textbook knowledge base",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Natural language query to search for relevant content"
                        },
                        "top_k": {
                            "type": "integer",
                            "description": "Number of results to return (default: 5, max: 20)",
                            "default": 5,
                            "minimum": 1,
                            "maximum": 20
                        },
                        "min_relevance": {
                            "type": "number",
                            "description": "Minimum relevance threshold (default: 0.3, range: 0.0-1.0)",
                            "default": 0.3,
                            "minimum": 0.0,
                            "maximum": 1.0
                        }
                    },
                    "required": ["query"]
                }
            }
        }

    def validate_retrieval_quality(
        self,
        query: str,
        results: List[Dict[str, Any]],
        expected_sources: Optional[List[str]] = None
    ) -> Dict[str, float]:
        """
        Validate the quality of retrieval results.

        Args:
            query: The original query
            results: List of retrieved results
            expected_sources: Optional list of expected source URLs

        Returns:
            Dict[str, float]: Quality metrics including precision, recall, and F1 score
        """
        if not results:
            return {"precision": 0.0, "recall": 0.0, "f1": 0.0}

        # For now, we'll implement a basic quality check
        # In a full implementation, we would have more sophisticated validation

        # Count how many results have valid content
        valid_results = [r for r in results if r.get('content') and len(r['content'].strip()) > 10]

        # If we have expected sources, calculate precision/recall against them
        if expected_sources:
            retrieved_sources = {r['metadata']['source_url'] for r in results}
            expected_set = set(expected_sources)

            relevant_retrieved = retrieved_sources.intersection(expected_set)
            precision = len(relevant_retrieved) / len(retrieved_sources) if retrieved_sources else 0.0
            recall = len(relevant_retrieved) / len(expected_set) if expected_set else 0.0

            f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
        else:
            # Without expected sources, we'll use a simpler metric
            # Consider results with good relevance scores as "relevant"
            relevant_results = [r for r in results if r.get('relevance_score', 0) >= 0.5]
            precision = len(relevant_results) / len(results) if results else 0.0
            recall = len(relevant_results) / len(valid_results) if valid_results else 0.0
            f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

        quality_metrics = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "valid_content_ratio": len(valid_results) / len(results) if results else 0.0,
            "average_relevance": sum(r.get('relevance_score', 0) for r in results) / len(results) if results else 0.0
        }

        self._logger.info(f"Retrieval quality metrics - Precision: {precision:.3f}, Recall: {recall:.3f}, F1: {f1:.3f}")
        return quality_metrics

    def get_relevant_chunks_for_query(
        self,
        query: str,
        top_k: int = 5,
        min_relevance: float = 0.3,
        filter_by_source: Optional[str] = None
    ) -> List[RetrievalResult]:
        """
        Get relevant chunks for a query with additional filtering options.

        Args:
            query: The query text to search for
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold
            filter_by_source: Optional source URL to filter results

        Returns:
            List[RetrievalResult]: List of relevant retrieval results
        """
        try:
            # Retrieve context using the main method
            raw_results = self.retrieve_context(query, top_k, min_relevance)

            # Convert to RetrievalResult objects
            results = []
            for raw_result in raw_results:
                # Apply source filtering if specified
                if filter_by_source:
                    if raw_result['metadata']['source_url'] != filter_by_source:
                        continue

                metadata = ChunkMetadata(
                    source_url=raw_result['metadata']['source_url'],
                    section_title=raw_result['metadata']['section_title'],
                    chunk_index=raw_result['metadata']['chunk_index']
                )

                result = RetrievalResult(
                    chunk_id=raw_result['id'],
                    content=raw_result['content'],
                    relevance_score=raw_result['relevance_score'],
                    metadata=metadata,
                    rank=len(results) + 1  # Assign rank based on order
                )
                results.append(result)

            self._logger.info(f"Retrieved {len(results)} relevant chunks for query with filtering")
            return results

        except Exception as e:
            self._logger.error(f"Error getting relevant chunks: {str(e)}")
            raise VectorDBError(f"Failed to retrieve relevant chunks: {str(e)}")

    def get_collection_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the vector collection.

        Returns:
            Dict[str, Any]: Collection statistics
        """
        try:
            stats = self._vector_search_engine.get_collection_info()
            return stats
        except Exception as e:
            self._logger.error(f"Error getting collection statistics: {str(e)}")
            return {"error": str(e)}


# Global instance of RetrievalTool
retrieval_tool = RetrievalTool()


def get_retrieval_tool() -> RetrievalTool:
    """
    Get the global RetrievalTool instance.

    Returns:
        RetrievalTool: The global RetrievalTool instance
    """
    return retrieval_tool