"""
Logging functionality for the RAG retrieval service.

This module provides logging for retrieval requests and system performance.
"""

import logging
from datetime import datetime
from typing import Dict, Any, Optional
from .models import QueryLog
from .logging_config import get_logger


class RetrievalLogger:
    """Handles logging for retrieval requests and performance monitoring."""

    def __init__(self):
        """Initialize the RetrievalLogger."""
        self._logger = get_logger()

    def log_retrieval_request(
        self,
        query: str,
        query_type: str,
        results_count: int,
        avg_relevance: float,
        processing_time: float,
        user_agent: Optional[str] = None
    ) -> QueryLog:
        """
        Log a retrieval request with performance metrics.

        Args:
            query: The query text
            query_type: The type of query (conceptual, factual, section-based)
            results_count: Number of results returned
            avg_relevance: Average relevance score of results
            processing_time: Total processing time in seconds
            user_agent: Optional user agent string

        Returns:
            QueryLog: The created log entry
        """
        log_id = f"log_{int(datetime.now().timestamp())}_{hash(query) % 10000}"

        query_log = QueryLog(
            log_id=log_id,
            query=query,
            query_type=query_type,
            results_count=results_count,
            avg_relevance=avg_relevance,
            processing_time=processing_time,
            user_agent=user_agent
        )

        # Log the event
        self._logger.info(
            f"RETRIEVAL_LOG - ID: {log_id}, Query: '{query[:50]}...', "
            f"Type: {query_type}, Results: {results_count}, Avg_Relevance: {avg_relevance:.3f}, "
            f"Time: {processing_time:.3f}s"
        )

        return query_log

    def log_validation_request(
        self,
        query: str,
        expected_chunks: list,
        retrieved_chunks: list,
        precision: float,
        recall: float,
        f1_score: float,
        validation_time: float
    ) -> None:
        """
        Log a validation request with quality metrics.

        Args:
            query: The query text
            expected_chunks: List of expected chunk IDs
            retrieved_chunks: List of retrieved chunk IDs
            precision: Precision score
            recall: Recall score
            f1_score: F1 score
            validation_time: Time taken for validation
        """
        relevant_retrieved = len(set(expected_chunks).intersection(set(retrieved_chunks)))

        self._logger.info(
            f"VALIDATION_LOG - Query: '{query[:50]}...', Expected: {len(expected_chunks)}, "
            f"Retrieved: {len(retrieved_chunks)}, Relevant_Retrieved: {relevant_retrieved}, "
            f"P: {precision:.3f}, R: {recall:.3f}, F1: {f1_score:.3f}, "
            f"Time: {validation_time:.3f}s"
        )

    def log_error(
        self,
        query: str,
        error_message: str,
        error_type: str = "retrieval_error"
    ) -> None:
        """
        Log an error during retrieval.

        Args:
            query: The query text that caused the error
            error_message: The error message
            error_type: Type of error (e.g., retrieval_error, api_error, validation_error)
        """
        self._logger.error(
            f"ERROR_LOG - Type: {error_type}, Query: '{query[:50]}...', Message: {error_message}"
        )

    def log_performance_metric(
        self,
        metric_name: str,
        metric_value: float,
        additional_info: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Log a performance metric.

        Args:
            metric_name: Name of the metric
            metric_value: Value of the metric
            additional_info: Additional information to log
        """
        log_msg = f"PERFORMANCE_LOG - Metric: {metric_name}, Value: {metric_value}"
        if additional_info:
            for key, value in additional_info.items():
                log_msg += f", {key}: {value}"

        self._logger.info(log_msg)

    def log_system_status(
        self,
        status: str,
        collection_exists: bool,
        collection_name: str,
        additional_info: Optional[Dict[str, Any]] = None
    ) -> None:
        """
        Log system status information.

        Args:
            status: Current system status
            collection_exists: Whether the expected collection exists
            collection_name: Name of the collection
            additional_info: Additional status information
        """
        log_msg = f"SYSTEM_STATUS - Status: {status}, Collection: {collection_name}, Exists: {collection_exists}"
        if additional_info:
            for key, value in additional_info.items():
                log_msg += f", {key}: {value}"

        self._logger.info(log_msg)

    def calculate_and_log_query_statistics(
        self,
        query_logs: list,
        time_period: str = "overall"
    ) -> Dict[str, float]:
        """
        Calculate and log statistics for a set of query logs.

        Args:
            query_logs: List of query logs to analyze
            time_period: Time period for the statistics

        Returns:
            Dict[str, float]: Calculated statistics
        """
        if not query_logs:
            return {}

        total_requests = len(query_logs)
        total_processing_time = sum(getattr(log, 'processing_time', 0) for log in query_logs)
        avg_processing_time = total_processing_time / total_requests if total_requests > 0 else 0
        total_results = sum(getattr(log, 'results_count', 0) for log in query_logs)
        avg_results_per_query = total_results / total_requests if total_requests > 0 else 0
        avg_relevance = sum(getattr(log, 'avg_relevance', 0) for log in query_logs) / total_requests if total_requests > 0 else 0

        stats = {
            'time_period': time_period,
            'total_requests': total_requests,
            'avg_processing_time': avg_processing_time,
            'avg_results_per_query': avg_results_per_query,
            'avg_relevance': avg_relevance,
            'total_results_returned': total_results
        }

        self._logger.info(
            f"STATS_LOG - Period: {time_period}, Requests: {total_requests}, "
            f"Avg_Time: {avg_processing_time:.3f}s, Avg_Results: {avg_results_per_query:.2f}, "
            f"Avg_Relevance: {avg_relevance:.3f}"
        )

        return stats


# Global instance of RetrievalLogger
retrieval_logger = RetrievalLogger()


def get_retrieval_logger() -> RetrievalLogger:
    """
    Get the global RetrievalLogger instance.

    Returns:
        RetrievalLogger: The global RetrievalLogger instance
    """
    return retrieval_logger