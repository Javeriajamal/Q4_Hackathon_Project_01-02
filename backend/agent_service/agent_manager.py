"""
Agent Manager for the Agent-Based RAG Backend service.

This module handles the initialization and management of the OpenAI agent,
including configuration, session management, and interaction with the
retrieval pipeline.
"""

import os
import logging
from typing import Optional, Dict, Any, List
from datetime import datetime

import openai
from openai import OpenAI

from .config import config
from .models import AgentQuery, RetrievalResult, AgentResponse
from .logging_config import get_logger
from .exceptions import (
    ConfigurationError,
    OpenAIAPIError,
    QueryProcessingError
)


class AgentManager:
    """Manages the OpenAI agent instance and its interactions."""

    def __init__(self):
        """Initialize the AgentManager."""
        self._logger = get_logger()
        self._client: Optional[OpenAI] = None
        self._agent_id: Optional[str] = None
        self._initialized = False

        # Validate configuration
        if not config.OPENAI_API_KEY:
            raise ConfigurationError("OPENAI_API_KEY environment variable must be set")

        # Initialize the OpenAI client
        self._initialize_client()

    def _initialize_client(self):
        """Initialize the OpenAI client with configuration."""
        try:
            self._client = OpenAI(api_key=config.OPENAI_API_KEY)

            # Verify the API key works by making a simple call
            # For now we'll just store the client and initialize the agent when needed
            self._logger.info("OpenAI client initialized successfully")
        except Exception as e:
            self._logger.error(f"Failed to initialize OpenAI client: {str(e)}")
            raise OpenAIAPIError(f"OpenAI client initialization failed: {str(e)}")

    def initialize_agent(self, model_name: Optional[str] = None) -> str:
        """
        Initialize the OpenAI agent with appropriate configuration.

        Args:
            model_name: Optional model name to use (defaults to configured model)

        Returns:
            str: The agent ID
        """
        try:
            model = model_name or config.OPENAI_MODEL_NAME

            # Create an assistant (using the newer Assistants API)
            self._assistant = self._client.beta.assistants.create(
                name="Physical AI & Robotics RAG Assistant",
                description="An AI assistant that answers questions based on the Physical AI & Humanoid Robotics textbook content",
                model=model,
                instructions="""You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
                Your purpose is to answer questions based only on the content provided in the knowledge base.
                Do not generate responses that are not grounded in the retrieved content.
                Always provide source attribution when citing information.
                If you cannot find relevant information in the provided context, clearly state that you don't have enough information to answer the question."""
            )

            self._agent_id = self._assistant.id
            self._initialized = True

            self._logger.info(f"Agent initialized successfully with ID: {self._agent_id}")
            return self._agent_id

        except Exception as e:
            self._logger.error(f"Failed to initialize agent: {str(e)}")
            raise OpenAIAPIError(f"Agent initialization failed: {str(e)}")

    def process_query_with_context(
        self,
        query: AgentQuery,
        retrieved_results: List[RetrievalResult]
    ) -> AgentResponse:
        """
        Process a query with retrieved context using the agent.

        Args:
            query: The query object to process
            retrieved_results: List of retrieval results to provide as context

        Returns:
            AgentResponse: The agent's response with metadata
        """
        if not self._initialized:
            agent_id = self.initialize_agent()
            self._logger.info(f"Agent auto-initialized with ID: {agent_id}")

        try:
            start_time = datetime.now()

            # Format the context from retrieved results
            formatted_context = self._format_context_for_agent(retrieved_results)

            # Create a thread for the conversation
            thread = self._client.beta.threads.create(
                messages=[
                    {
                        "role": "user",
                        "content": f"Context:\n{formatted_context}\n\nQuestion: {query.query_text}"
                    }
                ]
            )

            # Run the assistant on the thread
            run = self._client.beta.threads.runs.create(
                thread_id=thread.id,
                assistant_id=self._agent_id
            )

            # Wait for the run to complete (with timeout)
            import time
            timeout = 30  # 30 second timeout
            start_wait = time.time()

            while run.status in ["queued", "in_progress"]:
                if time.time() - start_wait > timeout:
                    raise QueryProcessingError("Agent processing timed out")

                time.sleep(0.5)
                run = self._client.beta.threads.runs.retrieve(
                    thread_id=thread.id,
                    run_id=run.id
                )

            if run.status == "failed":
                raise QueryProcessingError(f"Agent processing failed: {run.last_error}")

            # Get the messages from the thread
            messages = self._client.beta.threads.messages.list(thread_id=thread.id)

            # Extract the agent's response (assuming it's the last message)
            agent_response_text = ""
            for message in reversed(messages.data):
                if message.role == "assistant":
                    agent_response_text = message.content[0].text.value
                    break

            processing_time = (datetime.now() - start_time).total_seconds()

            # Create source attributions from the retrieved results
            source_attributions = []
            for result in retrieved_results:
                attribution = {
                    "source_url": result.metadata.source_url,
                    "section_title": result.metadata.section_title,
                    "chunk_index": result.metadata.chunk_index,
                    "relevance_score": result.relevance_score,
                    "extracted_text": result.content[:200] + "..." if len(result.content) > 200 else result.content
                }
                source_attributions.append(attribution)

            # Calculate a basic confidence score based on the highest relevance score
            confidence_score = max([r.relevance_score for r in retrieved_results]) if retrieved_results else 0.0

            response = AgentResponse(
                response_text=agent_response_text,
                source_attributions=source_attributions,
                confidence_score=confidence_score,
                processing_time=processing_time,
                session_id=query.session_id,
                query_text=query.query_text
            )

            self._logger.info(f"Query processed successfully in {processing_time:.3f}s")
            return response

        except Exception as e:
            self._logger.error(f"Error processing query with agent: {str(e)}")
            raise QueryProcessingError(f"Agent query processing failed: {str(e)}")

    def _format_context_for_agent(self, retrieved_results: List[RetrievalResult]) -> str:
        """
        Format retrieved results into a context string for the agent.

        Args:
            retrieved_results: List of retrieval results to format

        Returns:
            str: Formatted context string
        """
        if not retrieved_results:
            return "No relevant context found in the knowledge base."

        context_parts = []
        for i, result in enumerate(retrieved_results, 1):
            context_part = f"""
{i}. Source: {result.metadata.source_url}
   Section: {result.metadata.section_title}
   Chunk Index: {result.metadata.chunk_index}
   Relevance Score: {result.relevance_score:.3f}
   Content: {result.content}
            """
            context_parts.append(context_part.strip())

        return "\n\n".join(context_parts)

    def validate_agent_response(
        self,
        query: str,
        response: str,
        retrieved_context: List[RetrievalResult]
    ) -> bool:
        """
        Validate that the agent response is properly grounded in the context.

        Args:
            query: The original query
            response: The agent's response
            retrieved_context: The context provided to the agent

        Returns:
            bool: True if the response is properly grounded, False otherwise
        """
        # This is a basic implementation - in a full implementation, we would
        # use more sophisticated validation techniques
        if not retrieved_context:
            # If no context was provided, the response should acknowledge this
            return "no relevant information" in response.lower() or \
                   "cannot find" in response.lower() or \
                   "don't have enough information" in response.lower()

        # Check if the response contains content that's related to the context
        context_text = " ".join([result.content for result in retrieved_context])
        response_lower = response.lower()
        context_lower = context_text.lower()

        # Simple keyword overlap check as a basic grounding validation
        response_words = set(response_lower.split())
        context_words = set(context_lower.split())

        if not context_words:  # If context is empty
            return False

        overlap = len(response_words.intersection(context_words))
        overlap_ratio = overlap / len(response_words) if response_words else 0

        # Consider it properly grounded if there's at least 30% overlap
        return overlap_ratio >= 0.3

    def close(self):
        """Close the agent manager and clean up resources."""
        if hasattr(self, '_assistant') and self._assistant and self._agent_id:
            try:
                # In a real implementation, we might delete the assistant
                # self._client.beta.assistants.delete(self._agent_id)
                pass
            except Exception as e:
                self._logger.warning(f"Error cleaning up agent resources: {str(e)}")

        self._initialized = False
        self._logger.info("Agent manager closed and resources cleaned up")


# Global instance of AgentManager
agent_manager = AgentManager()


def get_agent_manager() -> AgentManager:
    """
    Get the global AgentManager instance.

    Returns:
        AgentManager: The global AgentManager instance
    """
    return agent_manager


def initialize_global_agent(model_name: Optional[str] = None) -> str:
    """
    Initialize the global agent instance.

    Args:
        model_name: Optional model name to use

    Returns:
        str: The agent ID
    """
    return agent_manager.initialize_agent(model_name)