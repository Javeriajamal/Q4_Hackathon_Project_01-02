"""
Response formatting module for the Agent-Based RAG Backend service.

This module handles formatting of agent responses with proper metadata
and structure for API output.
"""

from typing import List, Dict, Any, Optional
from datetime import datetime

from .models import RetrievalResult, AgentResponse
from .logging_config import get_logger


class ResponseFormatter:
    """Formats agent responses for API output."""

    def __init__(self):
        """Initialize the ResponseFormatter."""
        self._logger = get_logger()

    def format_api_response(
        self,
        results: List[RetrievalResult],
        query_embedding_time: float,
        search_time: float,
        total_time: float,
        retrieval_count: int,
        query_text: str,
        query_type: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Format a complete API response with all necessary metadata.

        Args:
            results: List of retrieval results to format
            query_embedding_time: Time taken to generate query embedding
            search_time: Time taken for vector search
            total_time: Total processing time
            retrieval_count: Number of results returned
            query_text: Original query text
            query_type: Optional query type classification

        Returns:
            Dict[str, Any]: Formatted API response
        """
        formatted_results = self.format_results_for_api(results)

        response = {
            "results": formatted_results,
            "query_embedding_time": query_embedding_time,
            "search_time": search_time,
            "total_time": total_time,
            "retrieval_count": retrieval_count,
            "query_text": query_text,
            "timestamp": datetime.now().isoformat()
        }

        # Include query type if available
        if query_type:
            response["query_type"] = query_type

        self._logger.info(f"Formatted response with {retrieval_count} results for query: '{query_text[:50]}...'")
        return response

    def format_results_for_api(self, results: List[RetrievalResult]) -> List[Dict[str, Any]]:
        """
        Format multiple retrieval results for API output.

        Args:
            results: List of retrieval results to format

        Returns:
            List[Dict[str, Any]]: Formatted results as list of dictionaries
        """
        return [self.format_result_for_api(result) for result in results]

    def format_result_for_api(self, result: RetrievalResult) -> Dict[str, Any]:
        """
        Format a single retrieval result for API output.

        Args:
            result: The retrieval result to format

        Returns:
            Dict[str, Any]: Formatted result as dictionary
        """
        formatted_result = {
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

        return formatted_result

    def format_agent_response(
        self,
        response_text: str,
        source_attributions: List[Dict[str, Any]],
        confidence_score: float,
        processing_time: float,
        session_id: str,
        query_text: str
    ) -> AgentResponse:
        """
        Format a complete agent response object.

        Args:
            response_text: The agent's response text
            source_attributions: List of source attributions
            confidence_score: Agent's confidence in the response
            processing_time: Total processing time
            session_id: Session identifier
            query_text: Original query text

        Returns:
            AgentResponse: Formatted agent response object
        """
        agent_response = AgentResponse(
            response_text=response_text,
            source_attributions=source_attributions,
            confidence_score=confidence_score,
            processing_time=processing_time,
            session_id=session_id,
            query_text=query_text
        )

        return agent_response

    def format_validation_response(
        self,
        is_properly_grounded: bool,
        grounding_percentage: float,
        validation_notes: Optional[str] = None,
        validation_time: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Format a validation response.

        Args:
            is_properly_grounded: Whether the response is properly grounded
            grounding_percentage: Percentage of response grounded in context
            validation_notes: Optional validation notes
            validation_time: Optional validation processing time

        Returns:
            Dict[str, Any]: Formatted validation response
        """
        response = {
            "is_properly_grounded": is_properly_grounded,
            "grounding_percentage": grounding_percentage,
            "validation_time": validation_time or 0.0
        }

        if validation_notes:
            response["validation_notes"] = validation_notes

        return response

    def format_retrieval_statistics(
        self,
        total_queries: int,
        avg_response_time: float,
        success_rate: float,
        avg_relevance: float
    ) -> Dict[str, Any]:
        """
        Format retrieval statistics for reporting.

        Args:
            total_queries: Total number of queries processed
            avg_response_time: Average response time in seconds
            success_rate: Success rate (0.0-1.0)
            avg_relevance: Average relevance score (0.0-1.0)

        Returns:
            Dict[str, Any]: Formatted statistics
        """
        return {
            "total_queries_processed": total_queries,
            "average_response_time": avg_response_time,
            "success_rate": success_rate,
            "average_relevance_score": avg_relevance,
            "timestamp": datetime.now().isoformat()
        }

    def format_error_response(
        self,
        error_type: str,
        error_message: str,
        details: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Format an error response.

        Args:
            error_type: Type of error that occurred
            error_message: Human-readable error message
            details: Optional additional error details

        Returns:
            Dict[str, Any]: Formatted error response
        """
        response = {
            "error": error_type,
            "message": error_message,
            "timestamp": datetime.now().isoformat()
        }

        if details:
            response["details"] = details

        return response

    def format_quality_metrics(
        self,
        precision: float,
        recall: float,
        f1_score: float,
        mrr: float,
        ndcg: float,
        map_score: float
    ) -> Dict[str, float]:
        """
        Format quality metrics for response.

        Args:
            precision: Precision score (0.0-1.0)
            recall: Recall score (0.0-1.0)
            f1_score: F1 score (0.0-1.0)
            mrr: Mean Reciprocal Rank (0.0-1.0)
            ndcg: Normalized Discounted Cumulative Gain (0.0-1.0)
            map_score: Mean Average Precision (0.0-1.0)

        Returns:
            Dict[str, float]: Formatted quality metrics
        """
        return {
            "precision": precision,
            "recall": recall,
            "f1": f1_score,
            "mrr": mrr,
            "ndcg": ndcg,
            "map": map_score
        }

    def format_query_log_for_output(
        self,
        log_id: str,
        query: str,
        results_count: int,
        avg_relevance: float,
        processing_time: float,
        timestamp: datetime
    ) -> Dict[str, Any]:
        """
        Format a query log for API output or reporting.

        Args:
            log_id: Unique log identifier
            query: Original query text
            results_count: Number of results returned
            avg_relevance: Average relevance of results
            processing_time: Time taken to process
            timestamp: When the query was processed

        Returns:
            Dict[str, Any]: Formatted query log
        """
        return {
            "log_id": log_id,
            "query": query,
            "results_count": results_count,
            "avg_relevance": avg_relevance,
            "processing_time": processing_time,
            "timestamp": timestamp.isoformat()
        }

    def format_session_details(
        self,
        session_id: str,
        created_at: datetime,
        last_interaction: datetime,
        conversation_history: List[Dict[str, Any]],
        active: bool
    ) -> Dict[str, Any]:
        """
        Format session details for API output.

        Args:
            session_id: Unique session identifier
            created_at: When the session was created
            last_interaction: When the last interaction occurred
            conversation_history: List of conversation turns
            active: Whether the session is still active

        Returns:
            Dict[str, Any]: Formatted session details
        """
        return {
            "session_id": session_id,
            "created_at": created_at.isoformat(),
            "last_interaction": last_interaction.isoformat(),
            "conversation_history": conversation_history,
            "active": active
        }

    def sanitize_content_for_output(self, content: str) -> str:
        """
        Sanitize content before including in API response.

        Args:
            content: Content to sanitize

        Returns:
            str: Sanitized content
        """
        if not content:
            return ""

        # Remove any potentially sensitive information
        # This is a basic implementation - in production, use more comprehensive sanitization
        sanitized = content.replace("\x00", "")  # Remove null bytes
        sanitized = sanitized.replace("\ufffd", "")  # Remove replacement characters

        return sanitized


# Global instance of ResponseFormatter
response_formatter = ResponseFormatter()


def get_response_formatter() -> ResponseFormatter:
    """
    Get the global ResponseFormatter instance.

    Returns:
        ResponseFormatter: The global ResponseFormatter instance
    """
    return response_formatter