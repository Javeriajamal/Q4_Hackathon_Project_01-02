"""
Query type classification module for the RAG retrieval service.

This module handles classifying queries into different types (conceptual, factual, section-based)
to enable specialized processing and improved retrieval.
"""

from typing import List, Optional
from .models import QueryType
from .logging_config import get_logger


class QueryClassifier:
    """Classifies queries into different types to enable specialized processing."""

    def __init__(self):
        """Initialize the QueryClassifier."""
        self._logger = get_logger()

        # Keywords that suggest different query types
        self.conceptual_keywords = [
            "what is", "what are", "what does", "explain", "describe", "define",
            "how does", "how can", "principle", "concept", "theory", "approach",
            "understand", "meaning of", "difference between", "relationship between",
            "benefits of", "importance of", "purpose of", "significance of"
        ]

        self.factual_keywords = [
            "how many", "how much", "when", "where", "who", "what year",
            "what number", "quantify", "calculate", "measure", "count",
            "amount of", "size of", "duration", "frequency", "percentage",
            "date", "time", "location", "name", "value", "statistic"
        ]

        self.section_keywords = [
            "chapter", "section", "module", "page", "reference", "see",
            "according to", "mentioned in", "found in", "part",
            "refer to", "look at", "from the", "in the", "specifically"
        ]

    def classify_query_type(self, query_text: str) -> Optional[QueryType]:
        """
        Classify the type of query based on its content.

        Args:
            query_text: The query text to classify

        Returns:
            Optional[QueryType]: The classified query type, or None if uncertain
        """
        if not query_text or not isinstance(query_text, str):
            return None

        query_lower = query_text.lower().strip()

        # Count keyword matches
        conceptual_matches = sum(1 for keyword in self.conceptual_keywords if keyword in query_lower)
        factual_matches = sum(1 for keyword in self.factual_keywords if keyword in query_lower)
        section_matches = sum(1 for keyword in self.section_keywords if keyword in query_lower)

        # Determine the most likely query type based on keyword matches
        max_matches = max(conceptual_matches, factual_matches, section_matches)

        if max_matches == 0:
            # If no clear keywords, try to infer from sentence structure
            if query_lower.endswith('?'):
                # Questions often seek conceptual or factual information
                if any(word in query_lower for word in ['what', 'how', 'why']):
                    return QueryType.CONCEPTUAL
                elif any(word in query_lower for word in ['when', 'where', 'who', 'which']):
                    return QueryType.FACTUAL
                else:
                    return QueryType.CONCEPTUAL  # Default to conceptual for general questions
            else:
                # For non-questions, default to conceptual
                return QueryType.CONCEPTUAL

        # If there are matches, return the type with the most matches
        if max_matches == conceptual_matches:
            return QueryType.CONCEPTUAL
        elif max_matches == factual_matches:
            return QueryType.FACTUAL
        else:  # max_matches == section_matches
            return QueryType.SECTION_BASED

    def classify_query_type_advanced(self, query_text: str) -> List[tuple[QueryType, float]]:
        """
        Perform advanced classification that returns multiple possible types with confidence scores.

        Args:
            query_text: The query text to classify

        Returns:
            List[Tuple[QueryType, float]]: List of (query_type, confidence_score) tuples,
                                          sorted by confidence score in descending order
        """
        if not query_text or not isinstance(query_text, str):
            return []

        query_lower = query_text.lower().strip()

        # Calculate scores for each type
        conceptual_score = sum(1 for keyword in self.conceptual_keywords if keyword in query_lower)
        factual_score = sum(1 for keyword in self.factual_keywords if keyword in query_lower)
        section_score = sum(1 for keyword in self.section_keywords if keyword in query_lower)

        # Calculate total score
        total_score = conceptual_score + factual_score + section_score

        if total_score == 0:
            # If no matches, return default classification
            query_type = self.classify_query_type(query_text)
            if query_type:
                return [(query_type, 0.5)]  # Default confidence of 0.5
            else:
                return []

        # Normalize scores to get confidence percentages
        results = []
        if conceptual_score > 0:
            results.append((QueryType.CONCEPTUAL, conceptual_score / total_score))
        if factual_score > 0:
            results.append((QueryType.FACTUAL, factual_score / total_score))
        if section_score > 0:
            results.append((QueryType.SECTION_BASED, section_score / total_score))

        # Sort by confidence score (descending)
        results.sort(key=lambda x: x[1], reverse=True)

        self._logger.debug(f"Advanced classification for '{query_text[:50]}...': {results}")
        return results

    def is_conceptual_query(self, query_text: str) -> bool:
        """
        Determine if a query is primarily conceptual.

        Args:
            query_text: The query text to evaluate

        Returns:
            bool: True if the query is conceptual, False otherwise
        """
        query_type = self.classify_query_type(query_text)
        return query_type == QueryType.CONCEPTUAL

    def is_factual_query(self, query_text: str) -> bool:
        """
        Determine if a query is primarily factual.

        Args:
            query_text: The query text to evaluate

        Returns:
            bool: True if the query is factual, False otherwise
        """
        query_type = self.classify_query_type(query_text)
        return query_type == QueryType.FACTUAL

    def is_section_based_query(self, query_text: str) -> bool:
        """
        Determine if a query is primarily section-based.

        Args:
            query_text: The query text to evaluate

        Returns:
            bool: True if the query is section-based, False otherwise
        """
        query_type = self.classify_query_type(query_text)
        return query_type == QueryType.SECTION_BASED

    def get_query_type_description(self, query_type: Optional[QueryType]) -> str:
        """
        Get a human-readable description of a query type.

        Args:
            query_type: The query type to describe

        Returns:
            str: Human-readable description of the query type
        """
        if not query_type:
            return "Unclassified or ambiguous query type"

        descriptions = {
            QueryType.CONCEPTUAL: "Conceptual query seeking explanations, definitions, or theoretical understanding",
            QueryType.FACTUAL: "Factual query seeking specific information, numbers, dates, or statistics",
            QueryType.SECTION_BASED: "Section-based query seeking information from specific parts of the content"
        }

        return descriptions.get(query_type, f"Unknown query type: {query_type}")


# Global instance of QueryClassifier
query_classifier = QueryClassifier()


def get_query_classifier() -> QueryClassifier:
    """
    Get the global QueryClassifier instance.

    Returns:
        QueryClassifier: The global QueryClassifier instance
    """
    return query_classifier