"""
Result ranking functionality for the RAG retrieval service.

This module handles ranking retrieval results by relevance score
and applying any additional ranking algorithms.
"""

from typing import List
from .models import RetrievalResult
from .logging_config import get_logger


class ResultRanker:
    """Ranks retrieval results by relevance and applies additional ranking algorithms."""

    def __init__(self):
        """Initialize the ResultRanker."""
        self._logger = get_logger()

    def rank_results(self, results: List[RetrievalResult]) -> List[RetrievalResult]:
        """
        Rank results by relevance score in descending order.

        Args:
            results: List of retrieval results to rank

        Returns:
            List[RetrievalResult]: Ranked list of results (highest relevance first)
        """
        if not results:
            return []

        # Sort results by relevance score in descending order
        sorted_results = sorted(results, key=lambda x: x.relevance_score, reverse=True)

        # Update the rank field to reflect the new ordering
        for i, result in enumerate(sorted_results):
            result.rank = i + 1

        self._logger.info(f"Ranked {len(results)} results by relevance score")
        return sorted_results

    def filter_by_relevance_threshold(
        self,
        results: List[RetrievalResult],
        min_relevance: float
    ) -> List[RetrievalResult]:
        """
        Filter results by a minimum relevance threshold.

        Args:
            results: List of retrieval results to filter
            min_relevance: Minimum relevance score threshold

        Returns:
            List[RetrievalResult]: Filtered list of results
        """
        filtered_results = [result for result in results if result.relevance_score >= min_relevance]

        self._logger.info(f"Filtered {len(results)} results to {len(filtered_results)} based on min_relevance {min_relevance}")
        return filtered_results

    def apply_diversity_ranking(
        self,
        results: List[RetrievalResult],
        top_k: int = 5
    ) -> List[RetrievalResult]:
        """
        Apply diversity ranking to ensure results cover different aspects of the query.
        This is a simplified implementation - a full implementation would use more
        sophisticated algorithms.

        Args:
            results: List of retrieval results to diversify
            top_k: Number of results to return after diversity ranking

        Returns:
            List[RetrievalResult]: Diversified list of results
        """
        if not results:
            return []

        # For now, just return the top_k results
        # In a full implementation, this would apply algorithms like MMR (Maximal Marginal Relevance)
        diversified_results = results[:top_k]

        self._logger.info(f"Applied diversity ranking, returning {len(diversified_results)} results")
        return diversified_results

    def rank_and_filter_results(
        self,
        results: List[RetrievalResult],
        min_relevance: float = 0.3,
        top_k: int = 5
    ) -> List[RetrievalResult]:
        """
        Rank results by relevance and apply filtering.

        Args:
            results: List of retrieval results to process
            min_relevance: Minimum relevance score threshold
            top_k: Maximum number of results to return

        Returns:
            List[RetrievalResult]: Processed list of results
        """
        # First, rank the results by relevance
        ranked_results = self.rank_results(results)

        # Then filter by relevance threshold
        filtered_results = self.filter_by_relevance_threshold(ranked_results, min_relevance)

        # Finally, limit to top_k results
        final_results = filtered_results[:top_k]

        self._logger.info(f"Processed {len(results)} results to {len(final_results)} final results "
                         f"(min_relevance: {min_relevance}, top_k: {top_k})")
        return final_results

    def calculate_normalized_scores(
        self,
        results: List[RetrievalResult]
    ) -> List[RetrievalResult]:
        """
        Calculate normalized relevance scores to provide better relative rankings.

        Args:
            results: List of retrieval results to normalize

        Returns:
            List[RetrievalResult]: Results with normalized scores
        """
        if not results:
            return []

        # If there's only one result, keep its score as is
        if len(results) == 1:
            return results

        # Get the min and max scores
        scores = [result.relevance_score for result in results]
        min_score = min(scores)
        max_score = max(scores)

        # If all scores are the same, return as is
        if max_score == min_score:
            return results

        # Normalize scores to 0-1 range based on the actual range
        normalized_results = []
        for result in results:
            normalized_score = (result.relevance_score - min_score) / (max_score - min_score)
            # Update the result with the normalized score
            new_result = RetrievalResult(
                chunk_id=result.chunk_id,
                content=result.content,
                relevance_score=normalized_score,
                metadata=result.metadata,
                rank=result.rank
            )
            normalized_results.append(new_result)

        self._logger.info(f"Normalized relevance scores for {len(results)} results")
        return normalized_results


# Global instance of ResultRanker
result_ranker = ResultRanker()


def get_result_ranker() -> ResultRanker:
    """
    Get the global ResultRanker instance.

    Returns:
        ResultRanker: The global ResultRanker instance
    """
    return result_ranker