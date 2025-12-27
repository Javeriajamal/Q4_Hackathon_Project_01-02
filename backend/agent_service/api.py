"""
API endpoints for the Agent-Based RAG Backend service.

This module defines the FastAPI application and endpoints for interacting
with the AI agent that performs retrieval-augmented generation.
"""

import time
from typing import Dict, Any, Optional
from datetime import datetime
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware

from .models import RetrievalRequest, AgentResponse
from .config import config
from .logging_config import get_logger
from .agent_manager import get_agent_manager
from .retrieval_tool import get_retrieval_tool
from .context_injector import get_context_injector
from .formatter import get_response_formatter
from .validation import get_validation_service
from .chatbot import create_chatbot_service
from .chat_models import ChatRequest, ChatResponse, ErrorResponse
from .exceptions import (
    ValidationError,
    QueryProcessingError,
    VectorDBError,
    OpenAIAPIError
)


# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for interacting with the AI agent that performs retrieval-augmented generation",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=config.ALLOWED_ORIGINS.split(',') if hasattr(config, 'ALLOWED_ORIGINS') and config.ALLOWED_ORIGINS else ["*"],  # Allow specific origins for local development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize service components
agent_manager = get_agent_manager()
retrieval_tool = get_retrieval_tool()
context_injector = get_context_injector()
response_formatter = get_response_formatter()
validation_service = get_validation_service()
chatbot_service = create_chatbot_service(config)
logger = get_logger()


@app.on_event("startup")
async def startup_event():
    """Initialize the agent when the application starts."""
    try:
        agent_id = agent_manager.initialize_agent()
        logger.info(f"Agent initialized successfully with ID: {agent_id}")
    except Exception as e:
        logger.error(f"Failed to initialize agent: {str(e)}")
        raise


@app.get("/")
async def root():
    """Root endpoint for basic service information."""
    return {
        "message": "RAG Agent Service",
        "status": "running",
        "timestamp": datetime.now().isoformat()
    }


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    try:
        # Check if we can access the agent
        # In a real implementation, we might check more detailed health indicators
        is_healthy = True

        # Check if retrieval tool is accessible
        try:
            # Attempt a simple operation to verify the retrieval service is working
            dummy_results = retrieval_tool.retrieve_context("health check", top_k=1, min_relevance=0.1)
            retrieval_healthy = len(dummy_results) >= 0  # Should not raise an exception
        except:
            retrieval_healthy = False
            is_healthy = False

        health_status = {
            "status": "healthy" if is_healthy else "unhealthy",
            "timestamp": datetime.now().isoformat(),
            "services": {
                "agent": True,  # If we reached this point, agent is accessible
                "retrieval": retrieval_healthy,
                "config_loaded": config.validate_config()
            },
            "metrics": {
                "current_time": datetime.now().isoformat()
            }
        }

        status_code = 200 if is_healthy else 503
        return health_status

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "status": "unhealthy",
            "error": str(e),
            "timestamp": datetime.now().isoformat()
        }, 503


@app.post("/api/agent/query")
async def query_agent_endpoint(request_data: Dict[str, Any]):
    """
    Process a query through the AI agent with retrieval augmentation.

    This endpoint accepts a natural language query, retrieves relevant context
    from the knowledge base, and generates a response using the AI agent.
    """
    start_time = time.time()

    try:
        # Parse and validate the request
        retrieval_request = RetrievalRequest(
            query=request_data.get("query", ""),
            top_k=request_data.get("top_k", config.DEFAULT_TOP_K),
            min_relevance=request_data.get("min_relevance", config.DEFAULT_MIN_RELEVANCE),
            query_types=request_data.get("query_types"),
            session_id=request_data.get("session_id")
        )

        # Validate the request parameters
        validation_errors = []
        if not retrieval_request.query or len(retrieval_request.query.strip()) == 0:
            validation_errors.append("Query cannot be empty")

        if not (1 <= retrieval_request.top_k <= 20):
            validation_errors.append("top_k must be between 1 and 20")

        if not (0.0 <= retrieval_request.min_relevance <= 1.0):
            validation_errors.append("min_relevance must be between 0.0 and 1.0")

        if validation_errors:
            logger.warning(f"Request validation failed: {validation_errors}")
            raise HTTPException(status_code=400, detail=f"Invalid request parameters: {', '.join(validation_errors)}")

        logger.info(f"Processing agent query: '{retrieval_request.query[:50]}...'")

        # Retrieve relevant context
        retrieval_start_time = time.time()
        retrieved_context = retrieval_tool.retrieve_context(
            query=retrieval_request.query,
            top_k=retrieval_request.top_k,
            min_relevance=retrieval_request.min_relevance
        )
        retrieval_time = time.time() - retrieval_start_time

        logger.info(f"Retrieved {len(retrieved_context)} context chunks in {retrieval_time:.3f}s")

        # Process with agent
        agent_processing_start = time.time()

        # Convert retrieved context to our internal model
        from .models import RetrievalResult, ChunkMetadata
        retrieval_results = []
        for ctx in retrieved_context:
            metadata = ChunkMetadata(
                source_url=ctx['metadata']['source_url'],
                section_title=ctx['metadata']['section_title'],
                chunk_index=ctx['metadata']['chunk_index']
            )

            result = RetrievalResult(
                chunk_id=ctx['id'],
                content=ctx['content'],
                relevance_score=ctx['relevance_score'],
                metadata=metadata,
                rank=len(retrieval_results) + 1
            )
            retrieval_results.append(result)

        # Create query object
        from .models import AgentQuery, QueryType
        query_obj = AgentQuery(
            query_text=retrieval_request.query,
            session_id=retrieval_request.session_id or f"session_{int(time.time())}",
            query_type=None,  # Will be determined by the query processor
            retrieved_context=retrieval_results
        )

        # Process the query with the agent
        agent_response = agent_manager.process_query_with_context(
            query=query_obj,
            retrieved_results=retrieval_results
        )

        agent_processing_time = time.time() - agent_processing_start
        total_time = time.time() - start_time

        logger.info(f"Agent processed query in {agent_processing_time:.3f}s, total time: {total_time:.3f}s")

        # Validate the response quality
        is_validated = validation_service.validate_agent_response(
            query=retrieval_request.query,
            response=agent_response.response_text,
            retrieved_context=retrieved_context
        )

        if not is_validated.get('is_properly_grounded', True):
            logger.warning("Agent response may not be properly grounded in retrieved context")

        # Format and return response
        response = {
            "response": agent_response.response_text,
            "source_attributions": agent_response.source_attributions,
            "confidence_score": agent_response.confidence_score,
            "processing_time": total_time,
            "retrieval_time": retrieval_time,
            "agent_processing_time": agent_processing_time,
            "retrieval_count": len(retrieved_context),
            "session_id": agent_response.session_id,
            "timestamp": agent_response.timestamp.isoformat(),
            "query_text": retrieval_request.query,
            "quality_metrics": {
                "is_properly_grounded": is_validated.get('is_properly_grounded', True),
                "grounding_percentage": is_validated.get('grounding_percentage', 0.0),
                "validation_notes": is_validated.get('validation_notes', '')
            }
        }

        return response

    except ValidationError as e:
        logger.warning(f"Validation error: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))

    except (QueryProcessingError, VectorDBError, OpenAIAPIError) as e:
        logger.error(f"Service error during query processing: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Service error: {str(e)}")

    except Exception as e:
        logger.error(f"Unexpected error during query processing: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@app.post("/api/agent/validate-response")
async def validate_agent_response_endpoint(request_data: Dict[str, Any]):
    """
    Validate that an agent response is properly grounded in retrieved context.
    """
    try:
        query = request_data.get("query", "")
        response = request_data.get("response", "")
        retrieved_context = request_data.get("retrieved_context", [])

        if not query or not response:
            raise HTTPException(status_code=400, detail="Query and response are required")

        validation_result = validation_service.validate_agent_response(
            query=query,
            response=response,
            retrieved_context=retrieved_context
        )

        return validation_result

    except ValidationError as e:
        logger.warning(f"Validation error: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))

    except Exception as e:
        logger.error(f"Error validating agent response: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Validation error: {str(e)}")


@app.get("/api/agent/stats")
async def get_agent_statistics():
    """
    Get statistics about agent performance and usage.
    """
    try:
        # In a real implementation, this would gather statistics from a database or cache
        # For now, we'll return basic placeholder stats
        stats = {
            "total_queries_processed": 0,
            "average_response_time": 0.0,
            "success_rate": 1.0,
            "top_query_types": [],
            "relevance_metrics": {
                "average_relevance": 0.0
            },
            "timestamp": datetime.now().isoformat()
        }

        return stats

    except Exception as e:
        logger.error(f"Error retrieving agent statistics: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Statistics retrieval error: {str(e)}")


@app.post("/api/chat")
async def chat_endpoint(request_data: Dict[str, Any]):
    """
    Process a chat query and return a response based on RAG functionality.

    This endpoint is designed for the embedded chatbot UI and supports both
    general queries and context-specific queries (e.g., based on selected text).
    """
    try:
        # Extract parameters from request
        query = request_data.get("query", "")
        context = request_data.get("context")
        session_id = request_data.get("session_id")

        # Process the query using the chatbot service
        result = chatbot_service.process_query(
            query=query,
            context=context,
            session_id=session_id
        )

        return result

    except ValidationError as e:
        logger.warning(f"Validation error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))

    except (QueryProcessingError, VectorDBError, OpenAIAPIError) as e:
        logger.error(f"Service error during chat processing: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Service error: {str(e)}")

    except Exception as e:
        logger.error(f"Unexpected error during chat processing: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


# Error handlers
@app.exception_handler(ValidationError)
async def validation_exception_handler(request: Request, exc: ValidationError):
    """Handle validation errors."""
    logger.warning(f"Validation error: {str(exc)}")
    return {"detail": str(exc), "error_type": "validation_error"}


@app.exception_handler(QueryProcessingError)
async def query_processing_exception_handler(request: Request, exc: QueryProcessingError):
    """Handle query processing errors."""
    logger.error(f"Query processing error: {str(exc)}")
    return {"detail": str(exc), "error_type": "query_processing_error"}


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle general errors."""
    logger.error(f"General error: {str(exc)}")
    return {"detail": "Internal server error", "error_type": "internal_error"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host=config.HOST,
        port=config.PORT,
        reload=True  # Set to False in production
    )