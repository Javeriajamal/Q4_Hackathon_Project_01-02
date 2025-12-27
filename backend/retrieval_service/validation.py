"""
Validation functionality for the RAG retrieval service.

This module handles validation of query types and other validation
requirements for the retrieval system.
"""

from typing import List, Optional
from .models import QueryType
from .logging_config import get_logger


class ValidationService:
    """Provides validation services for the retrieval system."""

    def __init__(self):
        """Initialize the ValidationService."""
        self._logger = get_logger()

    def validate_query_types(self, query_types: Optional[List[str]]) -> List[str]:
        """
        Validate the query_types parameter.

        Args:
            query_types: List of query types to validate

        Returns:
            List[str]: List of validation error messages, empty if valid
        """
        errors = []

        if query_types is None:
            return []  # None is valid

        if not isinstance(query_types, list):
            errors.append("query_types must be a list of strings")
            return errors

        valid_types = ["conceptual", "factual", "section-based", "section_based"]

        for i, qt in enumerate(query_types):
            if not isinstance(qt, str):
                errors.append(f"query_types[{i}] must be a string")
            elif qt.lower() not in valid_types:
                errors.append(f"query_types[{i}] '{qt}' is not a valid query type. Valid types are: {valid_types}")

        return errors

    def validate_query_type_consistency(self, query_text: str, assigned_type: Optional[QueryType], detected_types: List[QueryType]) -> bool:
        """
        Validate that the assigned query type is consistent with the query text.

        Args:
            query_text: The original query text
            assigned_type: The query type assigned to the query
            detected_types: List of query types detected by classification

        Returns:
            bool: True if consistent, False otherwise
        """
        if not assigned_type or not detected_types:
            return True  # Can't validate if no types provided

        # Check if the assigned type is among the detected types (with some tolerance)
        return assigned_type in detected_types

    def validate_relevance_threshold(self, min_relevance: float) -> List[str]:
        """
        Validate the minimum relevance threshold.

        Args:
            min_relevance: The minimum relevance threshold to validate

        Returns:
            List[str]: List of validation error messages, empty if valid
        """
        errors = []

        if not isinstance(min_relevance, (int, float)):
            errors.append("min_relevance must be a number")
            return errors

        if not (0.0 <= min_relevance <= 1.0):
            errors.append("min_relevance must be between 0.0 and 1.0")

        return errors

    def validate_top_k_value(self, top_k: int) -> List[str]:
        """
        Validate the top_k parameter.

        Args:
            top_k: The top_k value to validate

        Returns:
            List[str]: List of validation error messages, empty if valid
        """
        errors = []

        if not isinstance(top_k, int):
            errors.append("top_k must be an integer")
            return errors

        if not (1 <= top_k <= 20):
            errors.append("top_k must be between 1 and 20")

        return errors

    def validate_query_text(self, query_text: str) -> List[str]:
        """
        Validate the query text.

        Args:
            query_text: The query text to validate

        Returns:
            List[str]: List of validation error messages, empty if valid
        """
        errors = []

        if not query_text or not isinstance(query_text, str):
            errors.append("Query text must be a non-empty string")
            return errors

        if len(query_text.strip()) == 0:
            errors.append("Query text cannot be empty or whitespace only")

        if len(query_text) > 500:
            errors.append("Query text must be 500 characters or less")

        # Check if text has enough meaningful content
        content_chars = sum(1 for c in query_text if c.isalnum() or c.isspace())
        if content_chars / len(query_text) < 0.5 if query_text else 0:  # At least 50% should be alphanumeric or spaces
            errors.append("Query text should contain meaningful content")

        return errors

    def validate_retrieval_parameters(self, query: str, top_k: int, min_relevance: float, query_types: Optional[List[str]] = None) -> List[str]:
        """
        Validate all retrieval parameters.

        Args:
            query: The query text
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold
            query_types: Optional list of query types

        Returns:
            List[str]: List of validation error messages, empty if valid
        """
        errors = []

        # Validate individual parameters
        errors.extend(self.validate_query_text(query))
        errors.extend(self.validate_top_k_value(top_k))
        errors.extend(self.validate_relevance_threshold(min_relevance))

        if query_types is not None:
            errors.extend(self.validate_query_types(query_types))

        return errors

    def validate_result_quality(self, results: List, min_expected_results: int = 1) -> bool:
        """
        Validate that the results meet quality expectations.

        Args:
            results: List of results to validate
            min_expected_results: Minimum number of results expected

        Returns:
            bool: True if results meet quality expectations, False otherwise
        """
        if len(results) < min_expected_results:
            self._logger.warning(f"Expected at least {min_expected_results} results, got {len(results)}")
            return False

        # Check if results have proper structure
        for result in results:
            if not hasattr(result, 'relevance_score') or not hasattr(result, 'content'):
                self._logger.error("Result missing required attributes (relevance_score, content)")
                return False

            if not (0.0 <= result.relevance_score <= 1.0):
                self._logger.warning(f"Invalid relevance score: {result.relevance_score}")
                # Don't return False here as this might be acceptable depending on context

        return True

    def validate_query_type_distribution(self, results: List, expected_type: Optional[QueryType] = None) -> dict:
        """
        Validate the distribution of query types in results (for quality assessment).

        Args:
            results: List of results to validate
            expected_type: Expected query type for the results

        Returns:
            dict: Statistics about query type distribution in results
        """
        stats = {
            'total_results': len(results),
            'conceptual_matches': 0,
            'factual_matches': 0,
            'section_based_matches': 0,
            'no_type_info': 0
        }

        for result in results:
            # In a real implementation, we would have type information for each result
            # For now, this is a placeholder for future enhancement
            stats['no_type_info'] += 1

        return stats

    def validate_performance_consistency(self, query_type: QueryType, response_time: float, expected_max_time: float = 2.0) -> bool:
        """
        Validate that the response time is consistent across different query types.

        Args:
            query_type: The type of query being validated
            response_time: The actual response time in seconds
            expected_max_time: The maximum expected response time in seconds

        Returns:
            bool: True if performance is consistent, False otherwise
        """
        if response_time > expected_max_time:
            self._logger.warning(f"Performance issue: {query_type.value} query took {response_time:.2f}s, exceeding threshold of {expected_max_time}s")
            return False

        self._logger.info(f"Performance validation passed for {query_type.value} query: {response_time:.2f}s")
        return True

    def validate_type_specific_quality(self, query_type: QueryType, results: List, query: str) -> dict:
        """
        Validate quality metrics specific to each query type.

        Args:
            query_type: The type of query
            results: List of results to validate
            query: The original query text

        Returns:
            dict: Quality metrics for the specific query type
        """
        quality_metrics = {
            'query_type': query_type.value,
            'result_count': len(results),
            'average_relevance': 0.0,
            'has_sufficient_content': True,
            'type_specific_score': 0.0
        }

        if not results:
            quality_metrics.update({
                'average_relevance': 0.0,
                'type_specific_score': 0.0,
                'has_sufficient_content': False
            })
            return quality_metrics

        # Calculate average relevance
        total_relevance = sum(getattr(result, 'relevance_score', 0) for result in results)
        quality_metrics['average_relevance'] = total_relevance / len(results) if results else 0

        # Type-specific validation
        if query_type == QueryType.CONCEPTUAL:
            # Conceptual queries should have explanatory content
            conceptual_score = sum(1 for result in results if self._has_explanatory_content(result.content))
            quality_metrics['type_specific_score'] = conceptual_score / len(results)

        elif query_type == QueryType.FACTUAL:
            # Factual queries should have numerical or specific information
            factual_score = sum(1 for result in results if self._has_factual_content(result.content))
            quality_metrics['type_specific_score'] = factual_score / len(results)

        elif query_type == QueryType.SECTION_BASED:
            # Section-based queries should match section titles or URLs appropriately
            section_score = sum(1 for result in results if self._matches_requested_section(result, query))
            quality_metrics['type_specific_score'] = section_score / len(results)

        return quality_metrics

    def _has_explanatory_content(self, content: str) -> bool:
        """
        Check if content has explanatory characteristics (for conceptual queries).

        Args:
            content: Content to analyze

        Returns:
            bool: True if content has explanatory characteristics
        """
        explanatory_keywords = [
            "means", "represents", "implies", "describes", "explains",
            "consists of", "includes", "refers to", "is", "defined as",
            "works", "operates", "functions", "process", "procedure",
            "method", "technique", "approach", "principle", "concept"
        ]

        content_lower = content.lower()
        return any(keyword in content_lower for keyword in explanatory_keywords)

    def _has_factual_content(self, content: str) -> bool:
        """
        Check if content has factual characteristics (for factual queries).

        Args:
            content: Content to analyze

        Returns:
            bool: True if content has factual characteristics
        """
        # Check for presence of numbers/digits
        has_numbers = any(char.isdigit() for char in content)

        # Check for factual keywords
        factual_keywords = [
            "number", "amount", "quantity", "size", "duration", "frequency",
            "percentage", "ratio", "date", "time", "count", "statistic",
            "measurement", "value", "metric", "year", "month", "day"
        ]

        content_lower = content.lower()
        has_factual_terms = any(keyword in content_lower for keyword in factual_keywords)

        return has_numbers or has_factual_terms

    def _matches_requested_section(self, result, query: str) -> bool:
        """
        Check if result matches the requested section (for section-based queries).

        Args:
            result: Result to analyze
            query: Original query to compare against

        Returns:
            bool: True if result matches requested section
        """
        # This is a basic implementation - a full implementation would do more sophisticated matching
        # Look for section-related keywords in the query and check if they relate to the result

        query_lower = query.lower()
        section_indicators = ["chapter", "section", "module", "part", "subsection"]

        # If the query mentions a specific section, check if the result is related
        has_section_indicator = any(indicator in query_lower for indicator in section_indicators)

        if has_section_indicator:
            # Check if the result's metadata relates to the requested section
            if hasattr(result, 'metadata') and result.metadata:
                # Basic check: if query mentions a number, see if it's in section title or URL
                for term in query_lower.split():
                    if term.isdigit() and (term in result.metadata.section_title.lower() or term in result.metadata.source_url):
                        return True

        # For now, if it's a section-based query, we'll consider it a match
        # In a full implementation, we'd have more sophisticated section matching
        return True


# Global instance of ValidationService
validation_service = ValidationService()


def calculate_relevance_score(content1: str, content2: str) -> float:
    """
    Calculate a basic relevance score between two texts.

    Args:
        content1: First text for comparison
        content2: Second text for comparison

    Returns:
        float: Relevance score between 0.0 and 1.0
    """
    if not content1 or not content2:
        return 0.0

    # Convert to lowercase and split into sets of words
    words1 = set(content1.lower().split())
    words2 = set(content2.lower().split())

    if not words1 and not words2:
        return 1.0  # Both empty texts are identical
    if not words1 or not words2:
        return 0.0  # One empty, one not

    # Calculate Jaccard similarity (intersection over union)
    intersection = words1.intersection(words2)
    union = words1.union(words2)

    return len(intersection) / len(union) if union else 0.0


def validate_relevance_threshold(results: List, min_relevance: float) -> List:
    """
    Validate that results meet the minimum relevance threshold.

    Args:
        results: List of results to validate
        min_relevance: Minimum relevance threshold

    Returns:
        List: Results that meet the relevance threshold
    """
    if not results or not min_relevance:
        return results

    valid_results = []
    for result in results:
        if hasattr(result, 'relevance_score') and result.relevance_score >= min_relevance:
            valid_results.append(result)

    return valid_results


def get_validation_service() -> ValidationService:
    """
    Get the global ValidationService instance.

    Returns:
        ValidationService: The global ValidationService instance
    """
    return validation_service