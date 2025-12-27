"""
Validation module for the Agent-Based RAG Backend service.

This module provides functionality for validating query relevance,
response quality, and system performance metrics.
"""

from typing import List, Dict, Any, Optional
from .models import RetrievalResult, ValidationRequest, ValidationResponse
from .logging_config import get_logger
from .exceptions import ValidationError


class ValidationService:
    """Provides validation services for query relevance and response quality."""

    def __init__(self):
        """Initialize the ValidationService."""
        self._logger = get_logger()

    def validate_query_relevance(
        self,
        query: str,
        min_length: int = 3,
        max_length: int = 500
    ) -> List[str]:
        """
        Validate the relevance and quality of a query.

        Args:
            query: The query text to validate
            min_length: Minimum query length
            max_length: Maximum query length

        Returns:
            List[str]: List of validation errors (empty if valid)
        """
        errors = []

        if not query or not isinstance(query, str):
            errors.append("Query must be a non-empty string")
            return errors

        if len(query.strip()) < min_length:
            errors.append(f"Query must be at least {min_length} characters long")

        if len(query) > max_length:
            errors.append(f"Query must be no more than {max_length} characters long")

        # Check for meaningful content (not just special characters)
        content_chars = sum(1 for c in query if c.isalnum() or c.isspace())
        if content_chars / len(query) < 0.3 if query else 0:  # At least 30% should be alphanumeric or spaces
            errors.append("Query should contain meaningful content (letters and numbers)")

        return errors

    def validate_response_relevance(
        self,
        query: str,
        response: str,
        retrieved_context: List[Dict[str, Any]],
        min_relevance_threshold: float = 0.3
    ) -> ValidationResponse:
        """
        Validate that the response is relevant to the query and grounded in the context.

        Args:
            query: The original query text
            response: The agent's response
            retrieved_context: List of context chunks used to generate the response
            min_relevance_threshold: Minimum relevance threshold

        Returns:
            ValidationResponse: Validation results with metrics
        """
        start_time = time.time()

        if not query or not response:
            return ValidationResponse(
                is_properly_grounded=False,
                grounding_percentage=0.0,
                validation_notes="Query and response are required",
                validation_time=time.time() - start_time
            )

        # Calculate how well the response is grounded in the retrieved context
        grounding_score = self._calculate_response_grounding_score(query, response, retrieved_context)

        # Determine if the response meets the minimum threshold
        is_properly_grounded = grounding_score >= min_relevance_threshold

        # Generate validation notes
        if grounding_score < 0.1:
            validation_notes = "Response appears to be hallucinated with little connection to retrieved context"
        elif grounding_score < min_relevance_threshold:
            validation_notes = f"Response has low grounding score ({grounding_score:.2f}), below threshold of {min_relevance_threshold}"
        else:
            validation_notes = f"Response is well-grounded in context with score of {grounding_score:.2f}"

        validation_response = ValidationResponse(
            is_properly_grounded=is_properly_grounded,
            grounding_percentage=grounding_score,
            validation_notes=validation_notes,
            validation_time=time.time() - start_time
        )

        self._logger.info(
            f"Response validation completed: grounded={is_properly_grounded}, "
            f"score={grounding_score:.3f}, time={validation_response.validation_time:.3f}s"
        )

        return validation_response

    def _calculate_response_grounding_score(
        self,
        query: str,
        response: str,
        retrieved_context: List[Dict[str, Any]]
    ) -> float:
        """
        Calculate a score indicating how well the response is grounded in the context.

        Args:
            query: The original query
            response: The agent's response
            retrieved_context: List of context chunks used

        Returns:
            float: Grounding score between 0.0 and 1.0
        """
        if not retrieved_context:
            return 0.0  # No context means no grounding

        response_lower = response.lower()
        context_texts = [item.get('content', '').lower() for item in retrieved_context if 'content' in item]

        if not context_texts:
            return 0.0  # No content in context means no grounding

        # Calculate overlap between response and context
        response_words = set(response_lower.split())
        context_words = set()
        for text in context_texts:
            context_words.update(text.split())

        if not response_words:
            return 0.0

        # Calculate word overlap ratio
        overlapping_words = response_words.intersection(context_words)
        overlap_ratio = len(overlapping_words) / len(response_words)

        # Calculate semantic similarity based on common phrases
        common_phrases = 0
        total_phrases = 0

        # Check for 2-3 word phrases
        response_words_list = response_lower.split()
        for i in range(len(response_words_list) - 1):
            # 2-word phrases
            phrase = ' '.join(response_words_list[i:i+2])
            total_phrases += 1
            if any(phrase in context_text for context_text in context_texts):
                common_phrases += 1

            # 3-word phrases if possible
            if i < len(response_words_list) - 2:
                phrase = ' '.join(response_words_list[i:i+3])
                total_phrases += 1
                if any(phrase in context_text for context_text in context_texts):
                    common_phrases += 1

        phrase_similarity = common_phrases / total_phrases if total_phrases > 0 else 0.0

        # Combine metrics (adjust weights as needed)
        grounding_score = 0.6 * overlap_ratio + 0.4 * phrase_similarity

        # Clamp the score between 0 and 1
        grounding_score = max(0.0, min(1.0, grounding_score))

        return grounding_score

    def validate_retrieval_quality(
        self,
        query: str,
        retrieved_results: List[RetrievalResult],
        expected_sources: Optional[List[str]] = None,
        min_results_threshold: int = 1,
        min_avg_relevance: float = 0.5
    ) -> Dict[str, Any]:
        """
        Validate the quality of retrieval results.

        Args:
            query: The original query
            retrieved_results: List of retrieved results
            expected_sources: Optional list of expected source URLs
            min_results_threshold: Minimum number of results expected
            min_avg_relevance: Minimum average relevance score expected

        Returns:
            Dict[str, Any]: Quality validation results
        """
        if not retrieved_results:
            return {
                "is_quality_acceptable": False,
                "quality_score": 0.0,
                "issues": ["No results retrieved"],
                "metrics": {
                    "result_count": 0,
                    "average_relevance": 0.0,
                    "expected_sources_found": 0 if expected_sources else "N/A",
                    "relevance_distribution": {}
                }
            }

        # Calculate quality metrics
        result_count = len(retrieved_results)
        avg_relevance = sum(r.relevance_score for r in retrieved_results) / result_count
        relevance_distribution = self._calculate_relevance_distribution(retrieved_results)

        # Check if expected sources are found (if provided)
        expected_sources_found = 0
        if expected_sources:
            result_sources = {r.metadata.source_url for r in retrieved_results}
            expected_set = set(expected_sources)
            expected_sources_found = len(result_sources.intersection(expected_set))

        # Calculate quality score based on multiple factors
        quality_score = self._calculate_retrieval_quality_score(
            result_count, avg_relevance, relevance_distribution,
            expected_sources_found if expected_sources else 0,
            len(expected_sources) if expected_sources else 1
        )

        # Determine if quality is acceptable
        is_quality_acceptable = (
            result_count >= min_results_threshold and
            avg_relevance >= min_avg_relevance and
            quality_score >= 0.6  # 60% threshold for overall quality
        )

        # Identify any issues
        issues = []
        if result_count < min_results_threshold:
            issues.append(f"Insufficient results: {result_count} < {min_results_threshold}")
        if avg_relevance < min_avg_relevance:
            issues.append(f"Low average relevance: {avg_relevance:.3f} < {min_avg_relevance}")

        quality_results = {
            "is_quality_acceptable": is_quality_acceptable,
            "quality_score": quality_score,
            "issues": issues,
            "metrics": {
                "result_count": result_count,
                "average_relevance": avg_relevance,
                "expected_sources_found": expected_sources_found if expected_sources else "N/A",
                "relevance_distribution": relevance_distribution,
                "query_length": len(query)
            }
        }

        self._logger.info(
            f"Retrieval quality validation: acceptable={is_quality_acceptable}, "
            f"score={quality_score:.3f}, results={result_count}, avg_relevance={avg_relevance:.3f}"
        )

        return quality_results

    def _calculate_relevance_distribution(self, results: List[RetrievalResult]) -> Dict[str, int]:
        """
        Calculate the distribution of relevance scores.

        Args:
            results: List of retrieval results

        Returns:
            Dict[str, int]: Distribution of relevance scores in ranges
        """
        distribution = {
            "very_high": 0,    # 0.8-1.0
            "high": 0,        # 0.6-0.8
            "medium": 0,      # 0.4-0.6
            "low": 0,         # 0.2-0.4
            "very_low": 0     # 0.0-0.2
        }

        for result in results:
            score = result.relevance_score
            if 0.8 <= score <= 1.0:
                distribution["very_high"] += 1
            elif 0.6 <= score < 0.8:
                distribution["high"] += 1
            elif 0.4 <= score < 0.6:
                distribution["medium"] += 1
            elif 0.2 <= score < 0.4:
                distribution["low"] += 1
            else:
                distribution["very_low"] += 1

        return distribution

    def _calculate_retrieval_quality_score(
        self,
        result_count: int,
        avg_relevance: float,
        relevance_distribution: Dict[str, int],
        expected_sources_found: int,
        total_expected_sources: int
    ) -> float:
        """
        Calculate an overall quality score based on multiple metrics.

        Args:
            result_count: Number of results retrieved
            avg_relevance: Average relevance score
            relevance_distribution: Distribution of relevance scores
            expected_sources_found: Number of expected sources found
            total_expected_sources: Total number of expected sources

        Returns:
            float: Overall quality score between 0.0 and 1.0
        """
        # Normalize result count (assume 10 is sufficient for full score)
        result_score = min(result_count / 10.0, 1.0)

        # Average relevance score (already between 0 and 1)
        relevance_score = avg_relevance

        # Distribution score (favor more high-relevance results)
        total_results = sum(relevance_distribution.values())
        if total_results > 0:
            high_relevance_ratio = (relevance_distribution["very_high"] + relevance_distribution["high"]) / total_results
        else:
            high_relevance_ratio = 0.0

        # Expected sources score (if provided)
        expected_score = expected_sources_found / total_expected_sources if total_expected_sources > 0 else 1.0

        # Weighted combination of all factors
        quality_score = (
            0.3 * result_score +
            0.4 * relevance_score +
            0.2 * high_relevance_ratio +
            0.1 * expected_score
        )

        return quality_score

    def validate_query_type_classification(
        self,
        query_text: str,
        detected_type: str,
        confidence_threshold: float = 0.7
    ) -> bool:
        """
        Validate that the query type classification is reasonable.

        Args:
            query_text: The original query text
            detected_type: The detected query type
            confidence_threshold: Minimum confidence threshold

        Returns:
            bool: True if classification is valid, False otherwise
        """
        valid_types = ["conceptual", "factual", "section-based"]

        if detected_type not in valid_types:
            self._logger.warning(f"Invalid query type detected: {detected_type}")
            return False

        # Additional validation could be done based on query characteristics
        # For now, we'll just check if the type is in the valid set
        return True

    def validate_response_coherence(
        self,
        query: str,
        response: str,
        retrieved_context: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate the coherence and appropriateness of the response.

        Args:
            query: The original query
            response: The agent's response
            retrieved_context: Retrieved context used for the response

        Returns:
            Dict[str, Any]: Coherence validation results
        """
        validation_results = {
            "is_coherent": True,
            "coherence_score": 1.0,
            "issues": [],
            "metrics": {}
        }

        # Check if response is empty
        if not response or not response.strip():
            validation_results.update({
                "is_coherent": False,
                "coherence_score": 0.0,
                "issues": ["Response is empty or contains only whitespace"]
            })
            return validation_results

        # Check if response is too short
        if len(response.strip()) < 10:
            validation_results["issues"].append("Response is very short")
            validation_results["is_coherent"] = False
            validation_results["coherence_score"] = 0.3

        # Check for coherence with query
        query_lower = query.lower()
        response_lower = response.lower()

        # Look for signs of confusion or irrelevance
        if "i don't know" in response_lower or "i cannot find" in response_lower:
            if retrieved_context:  # If there was context but agent says it can't find info
                validation_results["issues"].append("Agent claims no information found despite context availability")
                validation_results["is_coherent"] = False
                validation_results["coherence_score"] = 0.4

        # Check for potential hallucinations (responses with high certainty but low grounding)
        grounding_score = self._calculate_response_grounding_score(query, response, retrieved_context)
        if grounding_score < 0.3 and any(phrase in response_lower for phrase in [
            "according to the text", "the document states", "as mentioned", "the source says"
        ]):
            validation_results["issues"].append("Potential hallucination detected: high confidence statements with low grounding")
            validation_results["is_coherent"] = False
            validation_results["coherence_score"] = 0.2

        # Calculate a simple coherence score based on the issues found
        issue_count = len(validation_results["issues"])
        if issue_count == 0:
            validation_results["coherence_score"] = 1.0
        elif issue_count == 1:
            validation_results["coherence_score"] = 0.7
        elif issue_count == 2:
            validation_results["coherence_score"] = 0.5
        else:
            validation_results["coherence_score"] = 0.2

        validation_results["metrics"]["grounding_score"] = grounding_score
        validation_results["metrics"]["response_length"] = len(response)

        return validation_results


# Global instance of ValidationService
validation_service = ValidationService()


def get_validation_service() -> ValidationService:
    """
    Get the global ValidationService instance.

    Returns:
        ValidationService: The global ValidationService instance
    """
    return validation_service