"""
Metrics calculation module for the Agent-Based RAG Backend service.

This module provides functionality for calculating quality metrics such as
precision, recall, F1 score, and other evaluation metrics for the retrieval system.
"""

import math
from typing import List, Dict, Any, Optional, Tuple
from collections import Counter
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
        expected_results: List[str]
    ) -> Dict[str, float]:
        """
        Calculate precision, recall, and F1 score for retrieval results.

        Args:
            retrieved_results: List of retrieval results returned by the system
            expected_results: List of expected result IDs that should have been retrieved

        Returns:
            Dict[str, float]: Dictionary with 'precision', 'recall', and 'f1' scores
        """
        if not expected_results:
            # If no expected results, return 0 for all metrics (or handle as appropriate)
            return {
                'precision': 0.0,
                'recall': 0.0,
                'f1': 0.0
            }

        if not retrieved_results:
            return {
                'precision': 0.0,
                'recall': 0.0,
                'f1': 0.0
            }

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
                break  # This is standard for MRR calculation

        mrr = sum(reciprocal_ranks) / len(expected_results) if expected_results else 0.0
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

        # If no relevance scores provided, use binary relevance based on whether result is expected
        if not relevance_scores:
            relevance_scores = {expected_id: 1.0 for expected_id in expected_results}

        # Calculate DCG (Discounted Cumulative Gain)
        dcg = 0.0
        for rank, result in enumerate(retrieved_results, 1):
            gain = relevance_scores.get(result.chunk_id, 0.0)
            discount = math.log2(rank + 1)  # Standard DCG discount factor
            dcg += gain / discount if discount != 0 else gain

        # Calculate IDCG (Ideal DCG) - best possible DCG if results were perfectly ordered
        ideal_gains = sorted([relevance_scores.get(exp_id, 0.0) for exp_id in expected_results], reverse=True)
        idcg = 0.0
        for rank, gain in enumerate(ideal_gains, 1):
            discount = math.log2(rank + 1)
            idcg += gain / discount if discount != 0 else gain

        ndcg = dcg / idcg if idcg > 0 else 0.0
        self._logger.info(f"NDCG calculated: {ndcg:.3f}")

        return ndcg

    def calculate_hit_rate_at_k(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str],
        k: int
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

    def calculate_recall_at_k(
        self,
        retrieved_results: List[RetrievalResult],
        expected_results: List[str],
        k: int
    ) -> float:
        """
        Calculate Recall at K (Recall@K) - fraction of expected results that appear in top-K.

        Args:
            retrieved_results: List of results returned by the system (ranked)
            expected_results: List of expected result IDs that should have been retrieved
            k: Top-K positions to consider

        Returns:
            float: Recall at K score
        """
        if not expected_results:
            return 0.0

        if not retrieved_results:
            return 0.0

        # Get top-k retrieved results
        top_k_results = retrieved_results[:k]
        top_k_ids = {result.chunk_id for result in top_k_results}

        # Get expected result IDs
        expected_ids = set(expected_results)

        # Calculate intersection
        relevant_retrieved = top_k_ids.intersection(expected_ids)

        recall_at_k = len(relevant_retrieved) / len(expected_ids) if expected_ids else 0.0
        self._logger.info(f"Recall at {k} calculated: {recall_at_k:.3f}")

        return recall_at_k

    def calculate_mean_average_precision(
        self,
        all_retrieved_results: List[List[RetrievalResult]],
        all_expected_results: List[List[str]]
    ) -> float:
        """
        Calculate Mean Average Precision (MAP) across multiple queries.

        Args:
            all_retrieved_results: List of retrieved results for each query
            all_expected_results: List of expected results for each query

        Returns:
            float: Mean Average Precision score
        """
        if not all_retrieved_results or not all_expected_results:
            return 0.0

        if len(all_retrieved_results) != len(all_expected_results):
            raise ValueError("Retrieved and expected results lists must have the same length")

        total_ap = 0.0
        valid_queries = 0

        for retrieved, expected in zip(all_retrieved_results, all_expected_results):
            ap = self.calculate_average_precision(retrieved, expected)
            if ap > 0:  # Only count queries that have expected results
                total_ap += ap
                valid_queries += 1

        map_score = total_ap / valid_queries if valid_queries > 0 else 0.0
        self._logger.info(f"Mean Average Precision calculated: {map_score:.3f}")

        return map_score

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
            k_values: Values of K for which to calculate Hit Rate and Recall@K

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

        # Calculate hit rates and recall at different K values
        for k in k_values:
            metrics[f'hr_at_{k}'] = self.calculate_hit_rate_at_k(retrieved_results, expected_results, k)
            metrics[f'recall_at_{k}'] = self.calculate_recall_at_k(retrieved_results, expected_results, k)

        # Calculate additional statistics
        metrics['total_retrieved'] = len(retrieved_results)
        metrics['total_expected'] = len(expected_results)
        metrics['relevant_retrieved'] = len(set(r.chunk_id for r in retrieved_results).intersection(set(expected_results)))

        self._logger.info(f"Comprehensive metrics calculated: {list(metrics.keys())}")

        return metrics

    def calculate_response_quality_metrics(
        self,
        query: str,
        response: str,
        retrieved_context: List[Dict[str, Any]],
        expected_answer_parts: Optional[List[str]] = None
    ) -> Dict[str, float]:
        """
        Calculate quality metrics specific to response quality (not just retrieval).

        Args:
            query: The original query
            response: The agent's response
            retrieved_context: Context used to generate the response
            expected_answer_parts: Optional list of expected answer components

        Returns:
            Dict[str, float]: Response quality metrics
        """
        metrics = {}

        # Calculate response length metrics
        response_length = len(response.split())
        metrics['response_word_count'] = response_length
        metrics['response_char_count'] = len(response)

        # Calculate grounding metrics
        grounding_score = self._calculate_response_grounding_score(query, response, retrieved_context)
        metrics['grounding_score'] = grounding_score

        # Calculate relevance to query
        query_response_relevance = self._calculate_query_response_relevance(query, response)
        metrics['query_response_relevance'] = query_response_relevance

        # If expected answer parts are provided, calculate answer completeness
        if expected_answer_parts:
            completeness_score = self._calculate_answer_completeness(response, expected_answer_parts)
            metrics['answer_completeness'] = completeness_score
        else:
            metrics['answer_completeness'] = 0.0  # Default when no expected parts provided

        # Calculate semantic coherence (how well the response flows)
        coherence_score = self._calculate_semantic_coherence(response)
        metrics['semantic_coherence'] = coherence_score

        # Calculate information density (meaningful content vs filler words)
        info_density = self._calculate_information_density(response)
        metrics['information_density'] = info_density

        self._logger.info(f"Response quality metrics calculated: {list(metrics.keys())}")

        return metrics

    def _calculate_response_grounding_score(
        self,
        query: str,
        response: str,
        retrieved_context: List[Dict[str, Any]]
    ) -> float:
        """
        Calculate how well the response is grounded in the retrieved context.

        Args:
            query: The original query
            response: The agent's response
            retrieved_context: Context used to generate the response

        Returns:
            float: Grounding score between 0.0 and 1.0
        """
        if not retrieved_context:
            return 0.0  # No context means no grounding

        response_lower = response.lower()
        context_texts = [item.get('content', '').lower() for item in retrieved_context if 'content' in item]

        if not context_texts:
            return 0.0  # No content in context means no grounding

        # Calculate lexical overlap between response and context
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

    def _calculate_query_response_relevance(
        self,
        query: str,
        response: str
    ) -> float:
        """
        Calculate the relevance between query and response.

        Args:
            query: The original query
            response: The agent's response

        Returns:
            float: Relevance score between 0.0 and 1.0
        """
        query_lower = query.lower()
        response_lower = response.lower()

        # Calculate word overlap
        query_words = set(query_lower.split())
        response_words = set(response_lower.split())

        if not query_words or not response_words:
            return 0.0

        # Calculate Jaccard similarity (intersection over union)
        intersection = query_words.intersection(response_words)
        union = query_words.union(response_words)

        jaccard_similarity = len(intersection) / len(union) if union else 0.0

        return jaccard_similarity

    def _calculate_answer_completeness(
        self,
        response: str,
        expected_parts: List[str]
    ) -> float:
        """
        Calculate how complete the answer is compared to expected parts.

        Args:
            response: The agent's response
            expected_parts: List of expected answer components

        Returns:
            float: Completeness score between 0.0 and 1.0
        """
        if not expected_parts:
            return 1.0  # If no expected parts, completeness is 100%

        response_lower = response.lower()
        covered_parts = 0

        for part in expected_parts:
            part_lower = part.lower()
            if part_lower in response_lower:
                covered_parts += 1

        completeness = covered_parts / len(expected_parts) if expected_parts else 0.0
        return completeness

    def _calculate_semantic_coherence(self, response: str) -> float:
        """
        Calculate the semantic coherence of the response.

        Args:
            response: The agent's response

        Returns:
            float: Coherence score between 0.0 and 1.0
        """
        # For a simple implementation, we'll check sentence structure and flow
        sentences = [s.strip() for s in response.split('.') if s.strip()]

        if len(sentences) < 2:
            # Single sentence responses can't be evaluated for coherence
            return 0.7 if len(sentences) > 0 else 0.0

        # In a more sophisticated implementation, we would use NLP models
        # to evaluate semantic coherence between sentences
        # For now, we'll return a moderate score assuming basic coherence
        return 0.75

    def _calculate_information_density(self, response: str) -> float:
        """
        Calculate the information density of the response.

        Args:
            response: The agent's response

        Returns:
            float: Information density score between 0.0 and 1.0
        """
        if not response:
            return 0.0

        # Count meaningful words (not stop words or filler)
        import re
        words = re.findall(r'\b\w+\b', response.lower())

        if not words:
            return 0.0

        # Common English stop words that don't add much information
        stop_words = {
            'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
            'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'being',
            'have', 'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could',
            'should', 'may', 'might', 'must', 'can', 'this', 'that', 'these', 'those',
            'i', 'you', 'he', 'she', 'it', 'we', 'they', 'me', 'him', 'her', 'us', 'them'
        }

        meaningful_words = [word for word in words if word not in stop_words]
        density = len(meaningful_words) / len(words)

        return min(density, 1.0)  # Clamp to 1.0


# Global instance of MetricsCalculator
metrics_calculator = MetricsCalculator()


def get_metrics_calculator() -> MetricsCalculator:
    """
    Get the global MetricsCalculator instance.

    Returns:
        MetricsCalculator: The global MetricsCalculator instance
    """
    return metrics_calculator