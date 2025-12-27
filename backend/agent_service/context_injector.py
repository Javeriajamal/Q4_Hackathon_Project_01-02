"""
Context Injection Module for the Agent-Based RAG Backend service.

This module handles the injection of retrieved context into the agent's
reasoning process, ensuring that responses are properly grounded in the
knowledge base.
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime

from .models import RetrievalResult, AgentQuery
from .logging_config import get_logger
from .exceptions import ContextInjectionError


class ContextInjector:
    """Handles injection of retrieved context into agent responses."""

    def __init__(self):
        """Initialize the ContextInjector."""
        self._logger = get_logger()

    def inject_context_into_agent_prompt(
        self,
        query: AgentQuery,
        retrieval_results: List[RetrievalResult],
        max_context_length: int = 2000
    ) -> str:
        """
        Inject retrieved context into the agent's prompt.

        Args:
            query: The original query object
            retrieval_results: List of retrieval results to inject
            max_context_length: Maximum length of context to include

        Returns:
            str: The augmented prompt with injected context
        """
        try:
            # Format the context from retrieval results
            formatted_context = self._format_retrieved_context(retrieval_results)

            # Truncate context if it exceeds the maximum length
            if len(formatted_context) > max_context_length:
                truncated_context = formatted_context[:max_context_length]
                self._logger.warning(
                    f"Context truncated from {len(formatted_context)} to {max_context_length} characters"
                )
            else:
                truncated_context = formatted_context

            # Create the augmented prompt with context
            augmented_prompt = f"""
            CONTEXT FROM KNOWLEDGE BASE:
            {truncated_context}

            USER QUERY:
            {query.query_text}

            INSTRUCTIONS:
            - Answer the user's query based ONLY on the provided context
            - Do not generate information not present in the context
            - If the context doesn't contain relevant information, clearly state this
            - Provide source attribution for any information you cite
            - Be concise but comprehensive in your response
            """

            self._logger.info(
                f"Context injected for query. Results: {len(retrieval_results)}, "
                f"Context length: {len(truncated_context)} characters"
            )

            return augmented_prompt

        except Exception as e:
            self._logger.error(f"Error injecting context: {str(e)}")
            raise ContextInjectionError(f"Context injection failed: {str(e)}")

    def _format_retrieved_context(self, retrieval_results: List[RetrievalResult]) -> str:
        """
        Format retrieved results into a context string suitable for the agent.

        Args:
            retrieval_results: List of retrieval results to format

        Returns:
            str: Formatted context string
        """
        if not retrieval_results:
            return "No relevant context found in the knowledge base."

        context_parts = []
        for i, result in enumerate(retrieval_results, 1):
            context_part = f"""
{i}. SOURCE: {result.metadata.source_url}
   SECTION: {result.metadata.section_title}
   CHUNK INDEX: {result.metadata.chunk_index}
   RELEVANCE SCORE: {result.relevance_score:.3f}
   CONTENT: {result.content}
            """
            context_parts.append(context_part.strip())

        formatted_context = "\n\n".join(context_parts)
        return formatted_context.strip()

    def prepare_context_for_multi_step_reasoning(
        self,
        query: str,
        initial_results: List[RetrievalResult],
        previous_context: Optional[str] = None
    ) -> str:
        """
        Prepare context for multi-step reasoning by combining initial results
        with previous context if available.

        Args:
            query: The current query being processed
            initial_results: Initial retrieval results
            previous_context: Previous context from earlier steps (optional)

        Returns:
            str: Combined context for multi-step reasoning
        """
        try:
            # Format the current retrieval results
            current_context = self._format_retrieved_context(initial_results)

            # Combine with previous context if available
            if previous_context:
                combined_context = f"""
PREVIOUS CONTEXT:
{previous_context}

NEWLY RETRIEVED CONTEXT:
{current_context}

COMBINED CONTEXT FOR REASONING:
Based on both previous information and newly retrieved information, answer the query: {query}
                """.strip()
            else:
                combined_context = f"""
RETRIEVED CONTEXT:
{current_context}

ANSWER THE FOLLOWING QUERY USING THE CONTEXT ABOVE:
{query}
                """.strip()

            self._logger.info(
                f"Multi-step reasoning context prepared. "
                f"Initial results: {len(initial_results)}, "
                f"Combined context length: {len(combined_context)} characters"
            )

            return combined_context

        except Exception as e:
            self._logger.error(f"Error preparing context for multi-step reasoning: {str(e)}")
            raise ContextInjectionError(f"Multi-step context preparation failed: {str(e)}")

    def validate_context_relevance(
        self,
        query: str,
        context: str,
        threshold: float = 0.3
    ) -> bool:
        """
        Validate that the context is relevant to the query.

        Args:
            query: The original query
            context: The context to validate
            threshold: Minimum relevance threshold

        Returns:
            bool: True if context is sufficiently relevant, False otherwise
        """
        if not context or not query:
            return False

        # Simple keyword overlap as a basic relevance check
        query_words = set(query.lower().split())
        context_words = set(context.lower().split())

        if not context_words:
            return False

        # Calculate overlap ratio
        overlap = len(query_words.intersection(context_words))
        overlap_ratio = overlap / len(query_words) if query_words else 0

        is_relevant = overlap_ratio >= threshold
        self._logger.debug(f"Context relevance check: {overlap_ratio:.3f} (threshold: {threshold}, result: {is_relevant})")

        return is_relevant

    def extract_source_attribution_from_context(self, context: str) -> List[Dict[str, str]]:
        """
        Extract source attribution information from the formatted context.

        Args:
            context: The formatted context string

        Returns:
            List[Dict[str, str]]: List of source attribution dictionaries
        """
        attributions = []

        # This is a simplified implementation - in a full implementation,
        # we would have more sophisticated parsing based on the context format
        lines = context.split('\n')

        current_attribution = {}
        for line in lines:
            line = line.strip()

            if line.startswith('SOURCE:') or 'SOURCE:' in line:
                # Extract URL from the line
                source_part = line.split('SOURCE:')[-1].strip()
                current_attribution['source_url'] = source_part
            elif line.startswith('SECTION:') or 'SECTION:' in line:
                section_part = line.split('SECTION:')[-1].strip()
                current_attribution['section_title'] = section_part
            elif line.startswith('CHUNK INDEX:') or 'CHUNK INDEX:' in line:
                index_part = line.split('CHUNK INDEX:')[-1].strip()
                try:
                    current_attribution['chunk_index'] = int(index_part)
                except ValueError:
                    current_attribution['chunk_index'] = 0
            elif line.startswith('RELEVANCE SCORE:') or 'RELEVANCE SCORE:' in line:
                score_part = line.split('RELEVANCE SCORE:')[-1].strip()
                try:
                    current_attribution['relevance_score'] = float(score_part)
                except ValueError:
                    current_attribution['relevance_score'] = 0.0

            # If we have a complete attribution, add it to the list
            if current_attribution.get('source_url'):
                attributions.append(current_attribution.copy())
                current_attribution = {}

        return attributions

    def calculate_context_adequacy_score(
        self,
        query: str,
        retrieval_results: List[RetrievalResult],
        min_results_needed: int = 1,
        min_avg_relevance: float = 0.5
    ) -> float:
        """
        Calculate a score representing how adequate the context is for answering the query.

        Args:
            query: The original query
            retrieval_results: List of retrieval results
            min_results_needed: Minimum number of results needed
            min_avg_relevance: Minimum average relevance needed

        Returns:
            float: Context adequacy score between 0.0 and 1.0
        """
        if not retrieval_results:
            return 0.0

        # Calculate number-based adequacy (0-1 scale)
        result_count_adequacy = min(len(retrieval_results) / min_results_needed, 1.0)

        # Calculate relevance-based adequacy (0-1 scale)
        avg_relevance = sum(r.relevance_score for r in retrieval_results) / len(retrieval_results)
        relevance_adequacy = min(avg_relevance / min_avg_relevance, 1.0) if min_avg_relevance > 0 else 1.0

        # Combine both factors (equal weight)
        adequacy_score = (result_count_adequacy + relevance_adequacy) / 2.0

        self._logger.info(
            f"Context adequacy calculated: {adequacy_score:.3f} "
            f"(results: {len(retrieval_results)}/{min_results_needed}, "
            f"relevance: {avg_relevance:.3f}/{min_avg_relevance})"
        )

        return adequacy_score


# Global instance of ContextInjector
context_injector = ContextInjector()


def get_context_injector() -> ContextInjector:
    """
    Get the global ContextInjector instance.

    Returns:
        ContextInjector: The global ContextInjector instance
    """
    return context_injector