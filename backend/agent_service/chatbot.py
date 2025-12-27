"""
Chatbot logic and integration module for RAG Chatbot Integration feature.
"""
from typing import Dict, Any, Optional
from pydantic import BaseModel, Field
from datetime import datetime
import logging
from .retrieval_tool import RetrievalTool
from .context_injector import ContextInjector
from .formatter import ResponseFormatter
from .validation import ResponseValidator
from .config import Config
from .exceptions import (
    QueryProcessingError,
    RetrievalError,
    ValidationError,
    ContextInjectionError
)

logger = logging.getLogger(__name__)


class ChatbotService:
    """
    Service class for handling chatbot logic and integration with RAG functionality.
    """

    def __init__(self, config: Config):
        """
        Initialize the ChatbotService with required dependencies.

        Args:
            config: Configuration object containing service settings
        """
        self.config = config
        self.retrieval_tool = RetrievalTool(config)
        self.context_injector = ContextInjector()
        self.formatter = ResponseFormatter()
        self.validator = ResponseValidator()

    def process_query(
        self,
        query: str,
        context: Optional[Dict[str, Any]] = None,
        session_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Process a chat query and return a response based on RAG functionality.

        Args:
            query: The user's query string
            context: Optional context information (e.g., selected text)
            session_id: Optional session identifier for continuity

        Returns:
            Dictionary containing the response and metadata

        Raises:
            QueryProcessingError: If there's an error during query processing
        """
        if not query or not query.strip():
            raise QueryProcessingError("Query cannot be empty")

        start_time = datetime.now()

        try:
            # Determine query context type
            context_type = "full-book"
            selected_text = None

            if context and isinstance(context, dict):
                context_type = context.get("type", "full-book")
                selected_text = context.get("content")

            # Validate context type
            if context_type not in ["full-book", "selected-text"]:
                raise QueryProcessingError(f"Invalid context type: {context_type}")

            # If context is selected text, modify the query to focus on that text
            search_query = query
            if context_type == "selected-text" and selected_text:
                if not selected_text.strip():
                    raise QueryProcessingError("Selected text cannot be empty when context type is 'selected-text'")
                search_query = f"Based on this text: '{selected_text}', {query}"

            # Retrieve relevant context using the existing retrieval tool
            try:
                retrieval_results = self.retrieval_tool.retrieve_context(
                    query=search_query,
                    top_k=self.config.top_k,
                    min_relevance=self.config.min_relevance
                )
            except Exception as e:
                logger.error(f"Retrieval tool failed: {str(e)}", exc_info=True)
                raise RetrievalError(f"Failed to retrieve context: {str(e)}")

            # Inject retrieved context into the response
            try:
                context_injected = self.context_injector.inject_context(
                    query=query,
                    retrieval_results=retrieval_results,
                    context_type=context_type
                )
            except Exception as e:
                logger.error(f"Context injection failed: {str(e)}", exc_info=True)
                raise ContextInjectionError(f"Failed to inject context: {str(e)}")

            # Format the response with proper attribution
            try:
                formatted_response = self.formatter.format_response(
                    query=query,
                    context_injected=context_injected,
                    retrieval_results=retrieval_results
                )
            except Exception as e:
                logger.error(f"Response formatting failed: {str(e)}", exc_info=True)
                raise QueryProcessingError(f"Failed to format response: {str(e)}")

            # Validate the response quality
            try:
                validation_result = self.validator.validate_response(
                    query=query,
                    response=formatted_response.response,
                    retrieved_context=retrieval_results
                )
            except Exception as e:
                logger.error(f"Response validation failed: {str(e)}", exc_info=True)
                raise ValidationError(f"Failed to validate response: {str(e)}")

            # Calculate processing time
            processing_time = (datetime.now() - start_time).total_seconds()

            # Prepare the final response
            result = {
                "response": formatted_response.response,
                "source_attributions": formatted_response.source_attributions,
                "confidence_score": validation_result.confidence_score,
                "processing_time": processing_time,
                "session_id": session_id or f"session_{int(datetime.now().timestamp())}",
                "timestamp": datetime.now().isoformat(),
                "query_type": context_type
            }

            logger.info(f"Successfully processed query for session {result['session_id']}, processing time: {processing_time}s")

            return result

        except (RetrievalError, ContextInjectionError, ValidationError) as e:
            # Re-raise specific errors that we want to handle at a higher level
            logger.error(f"Specific error during query processing: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error processing query: {str(e)}", exc_info=True)
            raise QueryProcessingError(f"Unexpected error during query processing: {str(e)}")


# Example usage function
def create_chatbot_service(config: Config) -> ChatbotService:
    """
    Factory function to create a ChatbotService instance.

    Args:
        config: Configuration object

    Returns:
        ChatbotService instance
    """
    return ChatbotService(config)