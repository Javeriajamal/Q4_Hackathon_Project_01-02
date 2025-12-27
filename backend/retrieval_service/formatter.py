"""
Response formatting functionality for the RAG retrieval service.

This module handles formatting retrieval responses with proper metadata
and structure for API output.
"""

from typing import List
from .models import RetrievalResult, RetrievalResponse
from .logging_config import get_logger


class ResponseFormatter:
    """Formats retrieval responses for API output."""

    def __init__(self):
        """Initialize the ResponseFormatter."""
        self._logger = get_logger()

    def format_retrieval_response(
        self,
        results: List[RetrievalResult],
        query_embedding_time: float,
        search_time: float,
        total_time: float,
        query_text: str
    ) -> RetrievalResponse:
        """
        Format retrieval results into a structured response.

        Args:
            results: List of retrieval results to format
            query_embedding_time: Time taken to generate query embedding
            search_time: Time taken for vector search
            total_time: Total processing time
            query_text: Original query text

        Returns:
            RetrievalResponse: Formatted response object
        """
        retrieval_count = len(results)

        response = RetrievalResponse(
            results=results,
            query_embedding_time=query_embedding_time,
            search_time=search_time,
            total_time=total_time,
            retrieval_count=retrieval_count,
            query_text=query_text
        )

        self._logger.info(f"Formatted response with {retrieval_count} results")
        return response

    def format_result_for_api(self, result: RetrievalResult) -> dict:
        """
        Format a single retrieval result for API output.

        Args:
            result: The retrieval result to format

        Returns:
            dict: Formatted result as dictionary
        """
        return {
            "chunk_id": result.chunk_id,
            "content": result.content,
            "relevance_score": result.relevance_score,
            "metadata": {
                "source_url": result.metadata.source_url,
                "section_title": result.metadata.section_title,
                "chunk_index": result.metadata.chunk_index
            },
            "rank": result.rank
        }

    def format_results_for_api(self, results: List[RetrievalResult]) -> List[dict]:
        """
        Format multiple retrieval results for API output.

        Args:
            results: List of retrieval results to format

        Returns:
            List[dict]: Formatted results as list of dictionaries
        """
        return [self.format_result_for_api(result) for result in results]

    def format_api_response(
        self,
        results: List[RetrievalResult],
        query_embedding_time: float,
        search_time: float,
        total_time: float,
        retrieval_count: int,
        query_text: str,
        query_type: Optional[str] = None
    ) -> dict:
        """
        Format a complete API response.

        Args:
            results: List of retrieval results
            query_embedding_time: Time taken to generate query embedding
            search_time: Time taken for vector search
            total_time: Total processing time
            retrieval_count: Number of results returned
            query_text: Original query text
            query_type: Optional query type classification

        Returns:
            dict: Complete API response as dictionary
        """
        formatted_results = self.format_results_for_api(results)

        response = {
            "results": formatted_results,
            "query_embedding_time": query_embedding_time,
            "search_time": search_time,
            "total_time": total_time,
            "retrieval_count": retrieval_count,
            "query_text": query_text
        }

        # Include query type if available
        if query_type:
            response["query_type"] = query_type

        return response


# Global instance of ResponseFormatter
response_formatter = ResponseFormatter()


def get_response_formatter() -> ResponseFormatter:
    """
    Get the global ResponseFormatter instance.

    Returns:
        ResponseFormatter: The global ResponseFormatter instance
    """
    return response_formatter