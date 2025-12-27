"""
API endpoints for the RAG retrieval service.

This module defines the FastAPI application and retrieval endpoints.
"""

import time
from typing import List, Optional, Dict, Any
from datetime import datetime
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
import asyncio

from .models import RetrievalRequest
from .query_processor import get_query_processor
from .vector_search import get_vector_search_engine
from .ranker import get_result_ranker
from .formatter import get_response_formatter
from .utils import validate_retrieval_request
from .config import get_config
from .logging_config import get_logger
from .exceptions import (
    RetrievalServiceError,
    ValidationError,
    NoResultsError,
    handle_retrieval_error
)


# Initialize FastAPI app
app = FastAPI(
    title="RAG Content Retrieval API",
    description="API for retrieving relevant content chunks from the RAG system using semantic similarity search",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize service components
query_processor = get_query_processor()
vector_search_engine = get_vector_search_engine()
result_ranker = get_result_ranker()
response_formatter = get_response_formatter()
config = get_config()
logger = get_logger()


@app.get("/")
async def root():
    """Root endpoint for health check."""
    return {"message": "RAG Content Retrieval Service", "status": "healthy"}


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    try:
        # Check if the configured collection exists
        collection_exists = vector_search_engine.validate_collection_exists()
        if not collection_exists:
            logger.warning(f"Configured collection '{config.COLLECTION_NAME}' does not exist")

        return {
            "status": "healthy",
            "collection_exists": collection_exists,
            "collection_name": config.COLLECTION_NAME
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=503, detail=f"Service unhealthy: {str(e)}")


@app.post("/api/retrieve")
async def retrieve_content(request_data: Dict[str, Any]):
    """
    Retrieve relevant content chunks for a query.

    This endpoint accepts a natural language query and returns relevant content chunks
    from the knowledge base with metadata and relevance scores.
    """
    start_time = time.time()

    try:
        # Parse and validate the request
        retrieval_request = RetrievalRequest(
            query=request_data.get("query", ""),
            top_k=request_data.get("top_k", config.DEFAULT_TOP_K),
            min_relevance=request_data.get("min_relevance", config.DEFAULT_MIN_RELEVANCE),
            query_types=request_data.get("query_types")
        )

        # Validate the request parameters
        validation_errors = validate_retrieval_request(retrieval_request)
        if validation_errors:
            logger.warning(f"Request validation failed: {validation_errors}")
            raise ValidationError(f"Invalid request parameters: {', '.join(validation_errors)}")

        logger.info(f"Processing retrieval request for query: '{retrieval_request.query[:50]}...'")

        # Measure query embedding time
        embedding_start = time.time()
        query_obj = query_processor.process_query(
            retrieval_request.query,
            query_type=retrieval_request.query_types[0] if retrieval_request.query_types else None
        )
        query_embedding_time = time.time() - embedding_start

        # Measure search time
        search_start = time.time()
        raw_results = vector_search_engine.search_similar_chunks_by_text(
            query_text=retrieval_request.query,
            query_embedding=query_obj.embedding,
            top_k=retrieval_request.top_k,
            min_relevance=retrieval_request.min_relevance,
            query_types=retrieval_request.query_types
        )
        search_time = time.time() - search_start

        # Rank and filter results
        ranked_results = result_ranker.rank_and_filter_results(
            raw_results,
            min_relevance=retrieval_request.min_relevance,
            top_k=retrieval_request.top_k
        )

        # Determine the query type for response inclusion
        query_type_str = None
        if hasattr(query_obj, 'query_type') and query_obj.query_type:
            query_type_str = query_obj.query_type.value

        # Calculate total processing time
        total_time = time.time() - start_time

        # Calculate quality metrics for the response
        from .metrics import get_metrics_calculator
        metrics_calc = get_metrics_calculator()

        # For now, we'll calculate basic quality metrics based on the results
        # In a real implementation, we might have expected results to compare against
        quality_metrics = {
            "average_relevance_score": sum(r.relevance_score for r in ranked_results) / len(ranked_results) if ranked_results else 0.0,
            "result_diversity": len(set(r.metadata.section_title for r in ranked_results)) / len(ranked_results) if ranked_results else 0.0,
            "response_time": total_time
        }

        # Format the response
        response = response_formatter.format_api_response(
            results=ranked_results,
            query_embedding_time=query_embedding_time,
            search_time=search_time,
            total_time=total_time,
            retrieval_count=len(ranked_results),
            query_text=retrieval_request.query,
            query_type=query_type_str
        )

        # Add quality metrics to the response
        response["quality_metrics"] = quality_metrics
        response["processing_timestamp"] = datetime.now().isoformat()

        logger.info(f"Retrieval completed in {total_time:.3f}s, returned {len(ranked_results)} results")

        return response

    except ValidationError as e:
        logger.warning(f"Validation error: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))

    except NoResultsError as e:
        logger.info(f"No results found: {str(e)}")
        # Return empty results rather than error for this case
        total_time = time.time() - start_time
        return {
            "results": [],
            "query_embedding_time": 0.0,
            "search_time": 0.0,
            "total_time": total_time,
            "retrieval_count": 0,
            "query_text": retrieval_request.query if 'retrieval_request' in locals() else ""
        }

    except RetrievalServiceError as e:
        logger.error(f"Retrieval service error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Retrieval service error: {str(e)}")

    except Exception as e:
        logger.error(f"Unexpected error during retrieval: {str(e)}")
        error = handle_retrieval_error(e, "API retrieval endpoint")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(error)}")


@app.post("/api/validate-retrieval")
async def validate_retrieval_quality(request_data: Dict[str, Any]):
    """
    Validate retrieval quality with expected results.

    This endpoint allows testing retrieval quality by providing expected results
    and calculating metrics like precision, recall, and F1 score.
    """
    try:
        query = request_data.get("query", "")
        expected_chunks = request_data.get("expected_chunks", [])
        top_k = request_data.get("top_k", 5)

        if not query:
            raise ValidationError("Query is required for validation")

        # Perform retrieval
        retrieval_request = RetrievalRequest(
            query=query,
            top_k=top_k,
            min_relevance=0.0  # Don't filter by relevance for validation
        )

        query_obj = query_processor.process_query(query)
        raw_results = vector_search_engine.search_similar_chunks_by_text(
            query_text=query,
            query_embedding=query_obj.embedding,
            top_k=top_k,
            min_relevance=0.0
        )

        # Calculate validation metrics
        retrieved_chunk_ids = [result.chunk_id for result in raw_results]

        # Calculate precision, recall, F1
        expected_set = set(expected_chunks)
        retrieved_set = set(retrieved_chunk_ids)

        relevant_retrieved = len(expected_set.intersection(retrieved_set))
        total_retrieved = len(retrieved_set)
        total_expected = len(expected_set)

        precision = relevant_retrieved / total_retrieved if total_retrieved > 0 else 0
        recall = relevant_retrieved / total_expected if total_expected > 0 else 0
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

        validation_time = time.time()

        return {
            "precision": precision,
            "recall": recall,
            "f1_score": f1_score,
            "retrieved_chunks": retrieved_chunk_ids,
            "relevant_retrieved": relevant_retrieved,
            "validation_time": validation_time
        }

    except ValidationError as e:
        logger.warning(f"Validation error: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))

    except Exception as e:
        logger.error(f"Error during validation: {str(e)}")
        error = handle_retrieval_error(e, "API validation endpoint")
        raise HTTPException(status_code=500, detail=f"Validation error: {str(error)}")


# Request validation function (T024)
def validate_request_params(query: str, top_k: int, min_relevance: float) -> List[str]:
    """
    Validate request parameters for the retrieval endpoint.

    Args:
        query: The query string
        top_k: Number of results to return
        min_relevance: Minimum relevance threshold

    Returns:
        List of validation error messages, empty if valid
    """
    errors = []

    if not query or not isinstance(query, str) or len(query.strip()) == 0:
        errors.append("Query is required and must be a non-empty string")
    elif len(query) > 500:
        errors.append("Query must be 500 characters or less")

    if not isinstance(top_k, int) or not (1 <= top_k <= 20):
        errors.append("top_k must be an integer between 1 and 20")

    if not isinstance(min_relevance, (int, float)) or not (0.0 <= min_relevance <= 1.0):
        errors.append("min_relevance must be a number between 0.0 and 1.0")

    return errors


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "retrieval_service.api:app",
        host=config.HOST,
        port=config.PORT,
        reload=True
    )