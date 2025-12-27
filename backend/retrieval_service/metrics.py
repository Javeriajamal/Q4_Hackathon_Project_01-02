"""
Metrics calculation functionality for the RAG retrieval service.

This module calculates quality metrics like precision, recall, and F1 score
to evaluate retrieval performance.
"""

from typing import List, Dict, Tuple, Optional
from .models import RetrievalResult
from .logging_config import get_logger


class MetricsCalculator:
    """Calculates quality metrics for retrieval performance evaluation."""

    def __init__(self):
        """Initialize the MetricsCalculator."""
        self._logger = get_logger()

    def calculate_precision_recall_f1(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str],
        relevant_threshold: float = 0.7
    ) -> Dict[str, float]:
        """
        Calculate precision, recall, and F1 score for retrieval results.

        Args:
            retrieved_results: List of results returned by the system
            expected_results: List of expected result IDs that should have been retrieved
            relevant_threshold: Threshold for considering a result relevant

        Returns:
            Dict[str, float]: Dictionary with 'precision', 'recall', and 'f1' scores
        """
        if not expected_results:
            return {'precision': 0.0, 'recall': 0.0, 'f1': 0.0}

        if not retrieved_results:
            return {'precision': 0.0, 'recall': 0.0, 'f1': 0.0}

        # Get retrieved result IDs
        retrieved_ids = {result.chunk_id for result in retrieved_results}

        # Get expected result IDs
        expected_ids = set(expected_results)

        # Calculate true positives, false positives, and false negatives
        true_positives = retrieved_ids.intersection(expected_ids)
        false_positives = retrieved_ids.difference(expected_ids)
        false_negatives = expected_ids.difference(retrieved_ids)

        # Calculate precision, recall, and F1
        precision = len(true_positives) / len(retrieved_ids) if retrieved_ids else 0.0
        recall = len(true_positives) / len(expected_ids) if expected_ids else 0.0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

        self._logger.info(
            f"Metrics calculated: Precision={precision:.3f}, Recall={recall:.3f}, F1={f1:.3f}, "
            f"TP={len(true_positives)}, FP={len(false_positives)}, FN={len(false_negatives)}"
        )

        return {
            'precision': precision,
            'recall': recall,
            'f1': f1
        }

    def calculate_mean_reciprocal_rank(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str]
    ) -> float:
        """
        Calculate Mean Reciprocal Rank (MRR) for retrieval results.

        Args:
            retrieved_results: List of results returned by the system (ranked)
            expected_results: List of expected result IDs that should have been retrieved

        Returns:
            float: Mean Reciprocal Rank score
        """
        if not expected_results or not retrieved_results:
            return 0.0

        expected_ids = set(expected_results)

        reciprocal_ranks = []
        for rank, result in enumerate(retrieved_results, 1):
            if result.chunk_id in expected_ids:
                reciprocal_ranks.append(1.0 / rank)
                # Once we find an expected result, we stop (first occurrence matters most)
                break

        mrr = sum(reciprocal_ranks) / len(expected_results) if reciprocal_ranks else 0.0
        self._logger.info(f"Mean Reciprocal Rank calculated: {mrr:.3f}")

        return mrr

    def calculate_normalized_discounted_cumulative_gain(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str],
        relevance_scores: Optional[Dict[str, float]] = None
    ) -> float:
        """
        Calculate Normalized Discounted Cumulative Gain (NDCG) for retrieval results.

        Args:
            retrieved_results: List of results returned by the system (ranked)
            expected_results: List of expected result IDs that should have been retrieved
            relevance_scores: Optional mapping of result IDs to relevance scores

        Returns:
            float: NDCG score
        """
        if not retrieved_results:
            return 0.0

        # If no relevance scores provided, use relevance based on position in expected results
        if not relevance_scores:
            relevance_scores = {expected_id: 1.0 for expected_id in expected_results}

        # Calculate DCG (Discounted Cumulative Gain)
        dcg = 0.0
        for rank, result in enumerate(retrieved_results, 1):
            gain = relevance_scores.get(result.chunk_id, 0.0)
            dcg += gain / (rank ** 0.5)  # Standard DCG formula with log base 2

        # Calculate IDCG (Ideal DCG) - best possible DCG if results were perfectly ordered
        ideal_gains = sorted([relevance_scores.get(exp_id, 0.0) for exp_id in expected_results], reverse=True)
        idcg = 0.0
        for rank, gain in enumerate(ideal_gains, 1):
            idcg += gain / (rank ** 0.5)

        ndcg = dcg / idcg if idcg > 0 else 0.0
        self._logger.info(f"NDCG calculated: {ndcg:.3f}")

        return ndcg

    def calculate_hit_rate_at_k(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str],
        k: int = 5
    ) -> float:
        """
        Calculate Hit Rate at K (HR@K) - fraction of queries where at least one relevant result is in top-K.

        Args:
            retrieved_results: List of results returned by the system (ranked)
            expected_results: List of expected result IDs that should have been retrieved
            k: Top-K positions to consider

        Returns:
            float: Hit Rate at K score
        """
        if not expected_results or not retrieved_results:
            return 0.0

        # Get top-k retrieved results
        top_k_results = retrieved_results[:k]
        top_k_ids = {result.chunk_id for result in top_k_results}

        # Check if any expected result is in top-k
        hit = bool(top_k_ids.intersection(set(expected_results)))
        hit_rate = 1.0 if hit else 0.0

        self._logger.info(f"Hit Rate at {k} calculated: {hit_rate:.3f}")

        return hit_rate

    def calculate_average_precision(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str]
    ) -> float:
        """
        Calculate Average Precision (AP) for retrieval results.

        Args:
            retrieved_results: List of results returned by the system (ranked)
            expected_results: List of expected result IDs that should have been retrieved

        Returns:
            float: Average Precision score
        """
        if not expected_results or not retrieved_results:
            return 0.0

        expected_ids = set(expected_results)
        hits = 0
        sum_precisions = 0.0

        for rank, result in enumerate(retrieved_results, 1):
            if result.chunk_id in expected_ids:
                hits += 1
                precision_at_rank = hits / rank  # Precision at current rank
                sum_precisions += precision_at_rank

        avg_precision = sum_precisions / len(expected_results) if expected_ids else 0.0
        self._logger.info(f"Average Precision calculated: {avg_precision:.3f}")

        return avg_precision

    def calculate_comprehensive_metrics(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str],
        k_values: List[int] = [1, 3, 5, 10]
    ) -> Dict[str, float]:
        """
        Calculate comprehensive set of quality metrics.

        Args:
            retrieved_results: List of results returned by the system (ranked)
            expected_results: List of expected result IDs that should have been retrieved
            k_values: Values of K for which to calculate HR@K

        Returns:
            Dict[str, float]: Dictionary containing all calculated metrics
        """
        metrics = {}

        # Calculate basic metrics
        basic_metrics = self.calculate_precision_recall_f1(retrieved_results, expected_results)
        metrics.update(basic_metrics)

        # Calculate additional metrics
        metrics['mrr'] = self.calculate_mean_reciprocal_rank(retrieved_results, expected_results)
        metrics['ndcg'] = self.calculate_normalized_discounted_cumulative_gain(retrieved_results, expected_results)
        metrics['map'] = self.calculate_average_precision(retrieved_results, expected_results)

        # Calculate hit rates at different K values
        for k in k_values:
            metrics[f'hr_at_{k}'] = self.calculate_hit_rate_at_k(retrieved_results, expected_results, k)

        # Calculate additional statistics
        metrics['total_retrieved'] = len(retrieved_results)
        metrics['total_expected'] = len(expected_results)
        metrics['relevant_retrieved'] = len(set(r.chunk_id for r in retrieved_results).intersection(set(expected_results)))

        self._logger.info(f"Comprehensive metrics calculated: {metrics}")

        return metrics


# Global instance of MetricsCalculator
metrics_calculator = MetricsCalculator()


def get_metrics_calculator() -> MetricsCalculator:
    """
    Get the global MetricsCalculator instance.

    Returns:
        MetricsCalculator: The global MetricsCalculator instance
    """
    return metrics_calculator


class PerformanceMonitor:
    """Monitors system performance and tracks key metrics."""

    def __init__(self):
        """Initialize the PerformanceMonitor."""
        self._logger = get_logger()
        self._queries_processed = 0
        self._total_processing_time = 0.0
        self._retrieval_count = 0
        self._error_count = 0
        self._successful_retrievals = 0

    def record_retrieval_performance(
        self,
        processing_time: float,
        results_count: int,
        query_type: Optional[str] = None
    ) -> None:
        """
        Record performance metrics for a retrieval operation.

        Args:
            processing_time: Time taken for the retrieval operation
            results_count: Number of results returned
            query_type: Type of query processed
        """
        self._queries_processed += 1
        self._total_processing_time += processing_time
        self._retrieval_count += 1

        # Log individual performance
        self._logger.info(
            f"PERFORMANCE - Query Type: {query_type or 'unknown'}, "
            f"Processing Time: {processing_time:.3f}s, Results: {results_count}"
        )

        # Log aggregate performance periodically
        if self._queries_processed % 10 == 0:  # Every 10 queries
            avg_time = self._total_processing_time / self._queries_processed
            self._logger.info(
                f"AGGREGATE_PERFORMANCE - Total Queries: {self._queries_processed}, "
                f"Average Time: {avg_time:.3f}s, Successful Retrievals: {self._successful_retrievals}, "
                f"Errors: {self._error_count}"
            )

    def record_error(self, error_type: str, query: Optional[str] = None) -> None:
        """
        Record an error in the system.

        Args:
            error_type: Type of error that occurred
            query: Query that caused the error (optional)
        """
        self._error_count += 1
        self._logger.error(f"ERROR_RECORDED - Type: {error_type}, Query: {query[:50] if query else 'None'}")

    def get_performance_summary(self) -> Dict[str, float]:
        """
        Get a summary of performance metrics.

        Returns:
            Dict[str, float]: Dictionary with performance metrics
        """
        avg_processing_time = self._total_processing_time / self._queries_processed if self._queries_processed > 0 else 0.0
        success_rate = self._successful_retrievals / self._queries_processed if self._queries_processed > 0 else 0.0

        return {
            'total_queries_processed': self._queries_processed,
            'total_retrievals': self._retrieval_count,
            'total_errors': self._error_count,
            'average_processing_time': avg_processing_time,
            'success_rate': success_rate,
            'error_rate': 1.0 - success_rate if self._queries_processed > 0 else 0.0
        }

    def reset_counters(self) -> None:
        """Reset all performance counters."""
        self._queries_processed = 0
        self._total_processing_time = 0.0
        self._retrieval_count = 0
        self._error_count = 0
        self._successful_retrievals = 0

        self._logger.info("PERFORMANCE_COUNTERS_RESET - All performance counters reset")


# Global instance of PerformanceMonitor
performance_monitor = PerformanceMonitor()


def get_performance_monitor() -> PerformanceMonitor:
    """
    Get the global PerformanceMonitor instance.

    Returns:
        PerformanceMonitor: The global PerformanceMonitor instance
    """
    return performance_monitor