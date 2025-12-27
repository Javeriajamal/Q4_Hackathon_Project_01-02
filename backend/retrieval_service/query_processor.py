"""
Query processing module for the RAG retrieval service.

This module handles converting natural language queries to embeddings
and classifying query types for the retrieval system.
"""

from typing import List, Optional
from .models import Query, QueryType
from .embeddings import get_embedding_generator, validate_embedding_quality
from .utils import format_query_type, validate_text_content
from .logging_config import get_logger
from .exceptions import QueryProcessingError, EmbeddingError


class QueryProcessor:
    """Processes natural language queries for the retrieval system."""

    def __init__(self):
        """Initialize the QueryProcessor."""
        self._embedding_generator = get_embedding_generator()
        self._logger = get_logger()

    def process_query(self, query_text: str, query_type: Optional[str] = None) -> Query:
        """
        Process a natural language query and convert it to embeddings.

        Args:
            query_text: The natural language query text
            query_type: Optional query type (conceptual, factual, section-based)

        Returns:
            Query: A Query object with embeddings and metadata
        """
        # Validate input
        if not query_text or not isinstance(query_text, str):
            raise QueryProcessingError("Query text must be a non-empty string")

        if len(query_text.strip()) == 0:
            raise QueryProcessingError("Query text cannot be empty or whitespace only")

        if len(query_text) > 500:
            raise QueryProcessingError("Query text must be 500 characters or less")

        # Validate text content
        if not validate_text_content(query_text):
            raise QueryProcessingError("Query text does not contain enough meaningful content")

        # Determine query type
        parsed_query_type = None
        if query_type:
            parsed_query_type = format_query_type(query_type)
            if not parsed_query_type:
                self._logger.warning(f"Invalid query type '{query_type}' provided, will attempt to classify automatically")

        # If no valid query type was provided, attempt to classify automatically
        if not parsed_query_type:
            parsed_query_type = self.classify_query_type(query_text)

        # Generate embedding
        try:
            embedding = self._embedding_generator.generate_single_embedding(query_text)
        except Exception as e:
            self._logger.error(f"Error generating embedding for query: {str(e)}")
            raise EmbeddingError(f"Failed to generate embedding: {str(e)}")

        # Validate embedding quality
        if not validate_embedding_quality(embedding):
            raise EmbeddingError("Generated embedding failed quality validation")

        # Create and return Query object
        query = Query(
            query_text=query_text,
            embedding=embedding,
            query_type=parsed_query_type
        )

        self._logger.info(f"Processed query with type '{parsed_query_type.value if parsed_query_type else 'unknown'}'")
        return query

    def generate_query_embedding(self, query_text: str) -> List[float]:
        """
        Generate an embedding for a query text.

        Args:
            query_text: The query text to convert to embedding

        Returns:
            List[float]: The embedding vector
        """
        try:
            embedding = self._embedding_generator.generate_single_embedding(query_text)
        except Exception as e:
            self._logger.error(f"Error generating embedding for query: {str(e)}")
            raise EmbeddingError(f"Failed to generate embedding: {str(e)}")

        # Validate embedding quality
        if not validate_embedding_quality(embedding):
            raise EmbeddingError("Generated embedding failed quality validation")

        return embedding

    def classify_query_type(self, query_text: str) -> Optional[QueryType]:
        """
        Classify the type of query based on its content.

        Args:
            query_text: The query text to classify

        Returns:
            Optional[QueryType]: The classified query type, or None if uncertain
        """
        query_lower = query_text.lower().strip()

        # Keywords that suggest different query types
        conceptual_keywords = [
            "what is", "what are", "what does", "explain", "describe", "define",
            "how does", "how can", "principle", "concept", "theory", "approach"
        ]

        factual_keywords = [
            "how many", "how much", "when", "where", "who", "what year",
            "what number", "quantify", "calculate", "measure", "count"
        ]

        section_keywords = [
            "chapter", "section", "module", "page", "reference", "see",
            "according to", "mentioned in", "found in", "part"
        ]

        # Count keyword matches
        conceptual_matches = sum(1 for keyword in conceptual_keywords if keyword in query_lower)
        factual_matches = sum(1 for keyword in factual_keywords if keyword in query_lower)
        section_matches = sum(1 for keyword in section_keywords if keyword in query_lower)

        # Determine the most likely query type based on keyword matches
        max_matches = max(conceptual_matches, factual_matches, section_matches)

        if max_matches == 0:
            # If no clear keywords, try to infer from sentence structure
            if query_lower.endswith('?') or 'what' in query_lower or 'how' in query_lower:
                return QueryType.CONCEPTUAL  # Default to conceptual for questions
            else:
                return QueryType.CONCEPTUAL  # Default to conceptual for general queries

        if max_matches == conceptual_matches:
            return QueryType.CONCEPTUAL
        elif max_matches == factual_matches:
            return QueryType.FACTUAL
        else:  # max_matches == section_matches
            return QueryType.SECTION_BASED


# Global instance of QueryProcessor
query_processor = QueryProcessor()


def get_query_processor() -> QueryProcessor:
    """
    Get the global QueryProcessor instance.

    Returns:
        QueryProcessor: The global QueryProcessor instance
    """
    return query_processor