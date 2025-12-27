"""
Debugging and comprehensive logging functionality for the RAG retrieval service.

This module provides enhanced logging, debugging tools, and diagnostic capabilities
for the retrieval system.
"""

import logging
import sys
import traceback
from datetime import datetime
from typing import Dict, Any, Optional, List
from functools import wraps
import time
import json
from pathlib import Path

from .logging_config import get_logger
from .models import RetrievalResult, QueryType


class DebuggingService:
    """Provides debugging and diagnostic capabilities for the retrieval service."""

    def __init__(self):
        """Initialize the DebuggingService."""
        self._logger = get_logger()
        self._debug_enabled = True
        self._request_log_file = "logs/request_debug.log"
        self._error_log_file = "logs/error_debug.log"

        # Create logs directory if it doesn't exist
        Path("logs").mkdir(exist_ok=True)

        # Set up file handlers for debug logs
        self._setup_debug_handlers()

    def _setup_debug_handlers(self):
        """Set up additional file handlers for debug information."""
        # Request debug handler
        request_handler = logging.FileHandler(self._request_log_file)
        request_handler.setLevel(logging.DEBUG)
        request_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - REQUEST_DEBUG - %(message)s'
        )
        request_handler.setFormatter(request_formatter)

        # Error debug handler
        error_handler = logging.FileHandler(self._error_log_file)
        error_handler.setLevel(logging.ERROR)
        error_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - ERROR_DEBUG - %(message)s\n%(exc_info)s'
        )
        error_handler.setFormatter(error_formatter)

        # Add handlers to the main logger
        self._logger.addHandler(request_handler)
        self._logger.addHandler(error_handler)

    def enable_debug_mode(self):
        """Enable debug mode with verbose logging."""
        self._debug_enabled = True
        self._logger.setLevel(logging.DEBUG)
        self._logger.info("Debug mode enabled")

    def disable_debug_mode(self):
        """Disable debug mode."""
        self._debug_enabled = False
        self._logger.setLevel(logging.INFO)
        self._logger.info("Debug mode disabled")

    def log_request_details(
        self,
        query: str,
        top_k: int,
        min_relevance: float,
        query_types: Optional[List[str]] = None,
        processing_start_time: Optional[float] = None
    ):
        """
        Log detailed information about a retrieval request.

        Args:
            query: The query text
            top_k: Number of results requested
            min_relevance: Minimum relevance threshold
            query_types: List of query types
            processing_start_time: Start time for processing (optional)
        """
        if not self._debug_enabled:
            return

        details = {
            "query_length": len(query),
            "query_word_count": len(query.split()),
            "top_k": top_k,
            "min_relevance": min_relevance,
            "query_types": query_types or [],
            "timestamp": datetime.now().isoformat(),
            "processing_start_time": processing_start_time
        }

        self._logger.debug(f"REQUEST_DETAILS - Query hash: {hash(query) % 10000}, Details: {json.dumps(details)}")

    def log_response_details(
        self,
        results: List[RetrievalResult],
        query_embedding_time: float,
        search_time: float,
        total_time: float,
        retrieval_count: int,
        query_text: str
    ):
        """
        Log detailed information about a retrieval response.

        Args:
            results: List of retrieval results
            query_embedding_time: Time taken to generate query embedding
            search_time: Time taken for vector search
            total_time: Total processing time
            retrieval_count: Number of results returned
            query_text: Original query text
        """
        if not self._debug_enabled:
            return

        # Calculate result statistics
        relevance_scores = [r.relevance_score for r in results]
        avg_relevance = sum(relevance_scores) / len(relevance_scores) if relevance_scores else 0.0
        min_relevance = min(relevance_scores) if relevance_scores else 0.0
        max_relevance = max(relevance_scores) if relevance_scores else 0.0

        response_stats = {
            "result_count": retrieval_count,
            "average_relevance": avg_relevance,
            "min_relevance": min_relevance,
            "max_relevance": max_relevance,
            "query_embedding_time": query_embedding_time,
            "search_time": search_time,
            "total_time": total_time,
            "query_length": len(query_text),
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"RESPONSE_DETAILS - Query hash: {hash(query_text) % 10000}, Stats: {json.dumps(response_stats)}")

    def log_error_with_context(
        self,
        error: Exception,
        context: str,
        query: Optional[str] = None,
        additional_info: Optional[Dict[str, Any]] = None
    ):
        """
        Log an error with rich contextual information.

        Args:
            error: The exception that occurred
            context: Context where the error occurred
            query: Original query that triggered the error (optional)
            additional_info: Additional information to log (optional)
        """
        error_details = {
            "error_type": type(error).__name__,
            "error_message": str(error),
            "context": context,
            "query_sample": query[:100] if query else None,
            "timestamp": datetime.now().isoformat(),
            "traceback": traceback.format_exc()
        }

        if additional_info:
            error_details.update(additional_info)

        self._logger.error(f"ERROR_WITH_CONTEXT - Details: {json.dumps(error_details)}")

    def log_performance_breakdown(
        self,
        query_processing_time: float,
        embedding_generation_time: float,
        vector_search_time: float,
        result_ranking_time: float,
        response_formatting_time: float,
        query: str
    ):
        """
        Log detailed performance breakdown for request processing.

        Args:
            query_processing_time: Time for query processing
            embedding_generation_time: Time for embedding generation
            vector_search_time: Time for vector search
            result_ranking_time: Time for result ranking
            response_formatting_time: Time for response formatting
            query: The query being processed
        """
        if not self._debug_enabled:
            return

        breakdown = {
            "query_processing_time": query_processing_time,
            "embedding_generation_time": embedding_generation_time,
            "vector_search_time": vector_search_time,
            "result_ranking_time": result_ranking_time,
            "response_formatting_time": response_formatting_time,
            "total_time": (query_processing_time + embedding_generation_time +
                          vector_search_time + result_ranking_time + response_formatting_time),
            "query_hash": hash(query) % 10000,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"PERFORMANCE_BREAKDOWN - {json.dumps(breakdown)}")

    def log_chunk_detailed_info(self, chunk: RetrievalResult):
        """
        Log detailed information about a content chunk.

        Args:
            chunk: The content chunk to log information for
        """
        if not self._debug_enabled:
            return

        chunk_info = {
            "chunk_id": chunk.chunk_id,
            "content_length": len(chunk.content),
            "content_preview": chunk.content[:100],
            "relevance_score": chunk.relevance_score,
            "rank": chunk.rank,
            "source_url": chunk.metadata.source_url if chunk.metadata else "N/A",
            "section_title": chunk.metadata.section_title if chunk.metadata else "N/A",
            "chunk_index": chunk.metadata.chunk_index if chunk.metadata else "N/A",
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"CHUNK_INFO - {json.dumps(chunk_info)}")

    def log_system_state(self, additional_info: Optional[Dict[str, Any]] = None):
        """
        Log current system state for debugging purposes.

        Args:
            additional_info: Additional system information to log
        """
        import psutil
        import os

        system_info = {
            "timestamp": datetime.now().isoformat(),
            "memory_usage_percent": psutil.virtual_memory().percent,
            "cpu_percent": psutil.cpu_percent(interval=1),
            "disk_usage_percent": psutil.disk_usage('/').percent,
            "process_id": os.getpid(),
            "python_version": sys.version,
            "working_directory": os.getcwd()
        }

        if additional_info:
            system_info.update(additional_info)

        self._logger.debug(f"SYSTEM_STATE - {json.dumps(system_info)}")

    def log_query_type_analysis(
        self,
        query: str,
        detected_type: Optional[QueryType],
        confidence: float,
        processing_strategy: str
    ):
        """
        Log analysis of query type classification.

        Args:
            query: The original query text
            detected_type: The detected query type
            confidence: Confidence in the classification
            processing_strategy: Strategy used for processing
        """
        if not self._debug_enabled:
            return

        analysis = {
            "query_hash": hash(query) % 10000,
            "query_length": len(query),
            "detected_type": detected_type.value if detected_type else "None",
            "classification_confidence": confidence,
            "processing_strategy": processing_strategy,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"QUERY_TYPE_ANALYSIS - {json.dumps(analysis)}")

    def log_embedding_generation_details(
        self,
        query_text: str,
        embedding_length: int,
        embedding_generation_time: float,
        embedding_model_used: str
    ):
        """
        Log details about embedding generation process.

        Args:
            query_text: The text that was embedded
            embedding_length: Length of the generated embedding vector
            embedding_generation_time: Time taken to generate the embedding
            embedding_model_used: Model used for embedding generation
        """
        if not self._debug_enabled:
            return

        embedding_details = {
            "query_hash": hash(query_text) % 10000,
            "embedding_length": embedding_length,
            "generation_time": embedding_generation_time,
            "model_used": embedding_model_used,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"EMBEDDING_GENERATION - {json.dumps(embedding_details)}")

    def log_similarity_search_details(
        self,
        query_embedding: List[float],
        search_results_count: int,
        search_time: float,
        top_k_param: int,
        min_relevance_param: float
    ):
        """
        Log details about similarity search process.

        Args:
            query_embedding: The query embedding vector
            search_results_count: Number of results returned from search
            search_time: Time taken for the search operation
            top_k_param: Top-k parameter used for search
            min_relevance_param: Minimum relevance parameter used
        """
        if not self._debug_enabled:
            return

        search_details = {
            "query_embedding_length": len(query_embedding),
            "results_count": search_results_count,
            "search_time": search_time,
            "top_k": top_k_param,
            "min_relevance": min_relevance_param,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"SIMILARITY_SEARCH - {json.dumps(search_details)}")

    def log_result_ranking_details(
        self,
        original_results_count: int,
        ranked_results_count: int,
        ranking_algorithm: str,
        ranking_time: float
    ):
        """
        Log details about result ranking process.

        Args:
            original_results_count: Number of results before ranking
            ranked_results_count: Number of results after ranking/filtering
            ranking_algorithm: Algorithm used for ranking
            ranking_time: Time taken for ranking operation
        """
        if not self._debug_enabled:
            return

        ranking_details = {
            "original_results_count": original_results_count,
            "final_results_count": ranked_results_count,
            "ranking_algorithm": ranking_algorithm,
            "ranking_time": ranking_time,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"RESULT_RANKING - {json.dumps(ranking_details)}")

    def debug_function_call(self, func_name: str, args: tuple, kwargs: dict, result=None):
        """
        Log function calls for debugging purposes.

        Args:
            func_name: Name of the function being called
            args: Arguments passed to the function
            kwargs: Keyword arguments passed to the function
            result: Result of the function call (optional)
        """
        if not self._debug_enabled:
            return

        call_details = {
            "function_name": func_name,
            "args_count": len(args),
            "kwargs_count": len(kwargs),
            "has_result": result is not None,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"FUNCTION_CALL - {func_name}, Details: {json.dumps(call_details)}")

    def log_validation_details(
        self,
        validator_name: str,
        input_data: Any,
        validation_passed: bool,
        validation_time: float,
        validation_errors: Optional[List[str]] = None
    ):
        """
        Log details about validation processes.

        Args:
            validator_name: Name of the validator
            input_data: Data being validated (will be represented as hash for privacy)
            validation_passed: Whether validation passed
            validation_time: Time taken for validation
            validation_errors: List of validation errors (if any)
        """
        if not self._debug_enabled:
            return

        validation_details = {
            "validator": validator_name,
            "input_hash": hash(str(input_data)) % 10000,
            "passed": validation_passed,
            "validation_time": validation_time,
            "error_count": len(validation_errors) if validation_errors else 0,
            "timestamp": datetime.now().isoformat()
        }

        self._logger.debug(f"VALIDATION - {json.dumps(validation_details)}")


def debug_timer(func):
    """
    Decorator to time function execution and log performance details.

    Args:
        func: The function to decorate

    Returns:
        The decorated function
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        dbg_service = get_debugging_service()

        try:
            result = func(*args, **kwargs)
            end_time = time.time()

            execution_time = end_time - start_time

            # Log the timing information
            dbg_service.log_performance_breakdown(
                query_processing_time=0,  # Placeholder
                embedding_generation_time=0,  # Placeholder
                vector_search_time=execution_time,  # Using this for the function time
                result_ranking_time=0,  # Placeholder
                response_formatting_time=0,  # Placeholder
                query=str(args[0])[:50] if args else "unknown"  # Use first arg as query if available
            )

            return result
        except Exception as e:
            end_time = time.time()
            execution_time = end_time - start_time

            # Log error with timing information
            dbg_service.log_error_with_context(
                error=e,
                context=f"{func.__name__} execution",
                additional_info={"execution_time": execution_time}
            )
            raise

    return wrapper


def debug_validation(validation_func):
    """
    Decorator to add debugging to validation functions.

    Args:
        validation_func: The validation function to decorate

    Returns:
        The decorated validation function
    """
    @wraps(validation_func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        dbg_service = get_debugging_service()

        # Log validation start
        dbg_service.log_validation_details(
            validator_name=validation_func.__name__,
            input_data=args[0] if args else kwargs,
            validation_passed=False,  # Will update after execution
            validation_time=0,  # Will update after execution
            validation_errors=None
        )

        try:
            result = validation_func(*args, **kwargs)
            end_time = time.time()

            # Determine if validation passed
            validation_passed = not isinstance(result, list) or len(result) == 0  # Assuming list of errors

            # Log validation completion
            dbg_service.log_validation_details(
                validator_name=validation_func.__name__,
                input_data=args[0] if args else kwargs,
                validation_passed=validation_passed,
                validation_time=end_time - start_time,
                validation_errors=result if isinstance(result, list) else None
            )

            return result
        except Exception as e:
            end_time = time.time()

            # Log validation error
            dbg_service.log_error_with_context(
                error=e,
                context=f"{validation_func.__name__} validation",
                additional_info={"validation_time": end_time - start_time}
            )
            raise

    return wrapper


def log_function_calls(cls):
    """
    Class decorator to add logging to all public methods.

    Args:
        cls: The class to decorate

    Returns:
        The decorated class
    """
    for attr_name in dir(cls):
        attr = getattr(cls, attr_name)

        # Only wrap callable public methods that aren't special methods
        if (callable(attr) and
            not attr_name.startswith('_') and
            not attr_name.startswith('__')):

            wrapped_method = debug_timer(attr)
            setattr(cls, attr_name, wrapped_method)

    return cls


# Global instance of DebuggingService
debugging_service = DebuggingService()


def get_debugging_service() -> DebuggingService:
    """
    Get the global DebuggingService instance.

    Returns:
        DebuggingService: The global DebuggingService instance
    """
    return debugging_service


def enable_debug_mode():
    """Enable debug mode globally."""
    dbg_service = get_debugging_service()
    dbg_service.enable_debug_mode()


def disable_debug_mode():
    """Disable debug mode globally."""
    dbg_service = get_debugging_service()
    dbg_service.disable_debug_mode()


def log_request_debug_info(
    query: str,
    top_k: int,
    min_relevance: float,
    query_types: Optional[List[str]] = None
):
    """
    Convenience function to log request details.

    Args:
        query: The query text
        top_k: Number of results requested
        min_relevance: Minimum relevance threshold
        query_types: List of query types
    """
    dbg_service = get_debugging_service()
    dbg_service.log_request_details(query, top_k, min_relevance, query_types)


def log_response_debug_info(
    results: List[RetrievalResult],
    query_embedding_time: float,
    search_time: float,
    total_time: float,
    retrieval_count: int,
    query_text: str
):
    """
    Convenience function to log response details.

    Args:
        results: List of retrieval results
        query_embedding_time: Time taken to generate query embedding
        search_time: Time taken for vector search
        total_time: Total processing time
        retrieval_count: Number of results returned
        query_text: Original query text
    """
    dbg_service = get_debugging_service()
    dbg_service.log_response_details(
        results, query_embedding_time, search_time,
        total_time, retrieval_count, query_text
    )


def log_error_debug_info(
    error: Exception,
    context: str,
    query: Optional[str] = None,
    additional_info: Optional[Dict[str, Any]] = None
):
    """
    Convenience function to log error with context.

    Args:
        error: The exception that occurred
        context: Context where the error occurred
        query: Original query that triggered the error (optional)
        additional_info: Additional information to log (optional)
    """
    dbg_service = get_debugging_service()
    dbg_service.log_error_with_context(error, context, query, additional_info)


# Example usage of decorators
@debug_timer
def example_timed_function(query: str) -> List[str]:
    """Example function with timing decorator."""
    # Simulate some processing
    time.sleep(0.1)
    return [f"result for {query}"]


@debug_validation
def example_validation_function(query: str) -> List[str]:
    """Example validation function with debugging."""
    errors = []
    if not query or len(query.strip()) == 0:
        errors.append("Query cannot be empty")
    if len(query) > 500:
        errors.append("Query too long")

    return errors