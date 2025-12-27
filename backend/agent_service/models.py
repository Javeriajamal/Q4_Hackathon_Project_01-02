"""
Data models for the Agent-Based RAG Backend service.

This module defines the core data structures used throughout the agent service,
including query objects, retrieval results, and request/response models.
"""

from typing import List, Optional, Dict, Any
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
import json


class QueryType(Enum):
    """Enumeration of query types supported by the system."""
    CONCEPTUAL = "conceptual"
    FACTUAL = "factual"
    SECTION_BASED = "section-based"


@dataclass
class ChunkMetadata:
    """Metadata associated with a retrieved content chunk."""

    source_url: str
    section_title: str
    chunk_index: int

    def to_dict(self) -> Dict[str, Any]:
        """Convert the metadata to a dictionary."""
        return {
            "source_url": self.source_url,
            "section_title": self.section_title,
            "chunk_index": self.chunk_index
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ChunkMetadata':
        """Create a ChunkMetadata instance from a dictionary."""
        return cls(
            source_url=data.get('source_url', ''),
            section_title=data.get('section_title', ''),
            chunk_index=data.get('chunk_index', 0)
        )


@dataclass
class RetrievalResult:
    """Represents a matched content chunk with relevance score and metadata."""

    chunk_id: str
    content: str
    relevance_score: float  # Between 0.0 and 1.0
    metadata: ChunkMetadata
    rank: int = 1  # Position in the ranked results list (1-based)

    def __post_init__(self):
        """Validate the retrieval result after initialization."""
        if not 0.0 <= self.relevance_score <= 1.0:
            raise ValueError("Relevance score must be between 0.0 and 1.0")

        if self.rank <= 0:
            raise ValueError("Rank must be positive")


@dataclass
class AgentQuery:
    """Represents a natural language query from a user to the agent."""

    query_text: str
    session_id: str
    timestamp: datetime = field(default_factory=datetime.now)
    query_type: Optional[QueryType] = None
    retrieved_context: Optional[List[RetrievalResult]] = None
    agent_response: Optional[str] = None

    def __post_init__(self):
        """Validate the query after initialization."""
        if not self.query_text or len(self.query_text.strip()) == 0:
            raise ValueError("Query text cannot be empty or whitespace only")

        if len(self.query_text) > 500:
            raise ValueError("Query text must be 500 characters or less")

        if not self.session_id:
            raise ValueError("Session ID is required")


@dataclass
class RetrievalRequest:
    """Input parameters for the retrieval operation."""

    query: str
    top_k: int = 5
    min_relevance: float = 0.3
    query_types: Optional[List[str]] = None
    session_id: Optional[str] = None

    def __post_init__(self):
        """Validate the retrieval request after initialization."""
        if not self.query or len(self.query.strip()) == 0:
            raise ValueError("Query is required and cannot be empty")

        if len(self.query) > 500:
            raise ValueError("Query must be 500 characters or less")

        if not 1 <= self.top_k <= 20:
            raise ValueError("top_k must be between 1 and 20")

        if not 0.0 <= self.min_relevance <= 1.0:
            raise ValueError("min_relevance must be between 0.0 and 1.0")

        if self.query_types:
            valid_types = ["conceptual", "factual", "section-based"]
            for qt in self.query_types:
                if qt not in valid_types:
                    raise ValueError(f"Invalid query type: {qt}")


@dataclass
class AgentResponse:
    """Output from the agent after processing a query."""

    response_text: str
    source_attributions: List[Dict[str, Any]]
    confidence_score: float
    processing_time: float
    session_id: str
    timestamp: datetime = field(default_factory=datetime.now)
    query_text: Optional[str] = None
    quality_metrics: Optional[Dict[str, float]] = None

    def __post_init__(self):
        """Validate the agent response after initialization."""
        if not self.response_text:
            raise ValueError("Response text cannot be empty")

        if not 0.0 <= self.confidence_score <= 1.0:
            raise ValueError("Confidence score must be between 0.0 and 1.0")

        if self.processing_time < 0:
            raise ValueError("Processing time cannot be negative")

        if not self.session_id:
            raise ValueError("Session ID is required")


@dataclass
class AgentSession:
    """Stateful context for a multi-turn conversation with the agent."""

    session_id: str
    created_at: datetime = field(default_factory=datetime.now)
    last_interaction: datetime = field(default_factory=datetime.now)
    conversation_history: List[Dict[str, Any]] = field(default_factory=list)
    active: bool = True

    def __post_init__(self):
        """Validate the agent session after initialization."""
        if not self.session_id:
            raise ValueError("Session ID is required")

    def add_interaction(self, query: str, response: str):
        """Add an interaction to the conversation history."""
        interaction = {
            "query": query,
            "response": response,
            "timestamp": datetime.now().isoformat()
        }
        self.conversation_history.append(interaction)
        self.last_interaction = datetime.now()


@dataclass
class QueryLog:
    """Log entry for tracking retrieval requests and system performance."""

    log_id: str
    query: str
    query_type: str
    results_count: int
    avg_relevance: float
    processing_time: float
    timestamp: datetime = field(default_factory=datetime.now)
    user_agent: Optional[str] = None

    def __post_init__(self):
        """Validate the query log after initialization."""
        if not self.log_id:
            raise ValueError("Log ID is required")

        if self.processing_time < 0:
            raise ValueError("Processing time must be non-negative")

        if not 0.0 <= self.avg_relevance <= 1.0:
            raise ValueError("Average relevance must be between 0.0 and 1.0")


@dataclass
class ValidationRequest:
    """Request to validate agent response quality."""

    query: str
    response: str
    retrieved_context: List[Dict[str, Any]]
    expected_sources: Optional[List[str]] = None

    def __post_init__(self):
        """Validate the validation request after initialization."""
        if not self.query:
            raise ValueError("Query is required for validation")

        if not self.response:
            raise ValueError("Response is required for validation")

        if not self.retrieved_context:
            raise ValueError("Retrieved context is required for validation")


@dataclass
class ValidationResponse:
    """Response from the validation service."""

    is_properly_grounded: bool
    grounding_percentage: float
    validation_notes: Optional[str] = None
    validation_time: Optional[float] = None

    def __post_init__(self):
        """Validate the validation response after initialization."""
        if not 0.0 <= self.grounding_percentage <= 1.0:
            raise ValueError("Grounding percentage must be between 0.0 and 1.0")

        if self.validation_time is not None and self.validation_time < 0:
            raise ValueError("Validation time must be non-negative")


@dataclass
class PerformanceMetrics:
    """Performance metrics for the agent service."""

    total_queries_processed: int = 0
    average_response_time: float = 0.0
    success_rate: float = 1.0
    error_rate: float = 0.0
    average_relevance_score: float = 0.0
    total_retrievals: int = 0
    timestamp: datetime = field(default_factory=datetime.now)

    def update_with_result(self, response_time: float, success: bool, relevance_score: float = 0.0):
        """Update metrics with a new result."""
        self.total_queries_processed += 1

        # Update average response time
        total_time = self.average_response_time * (self.total_queries_processed - 1) + response_time
        self.average_response_time = total_time / self.total_queries_processed

        # Update success/error rates
        if success:
            successes = self.success_rate * (self.total_queries_processed - 1) + 1
            self.success_rate = successes / self.total_queries_processed
        else:
            errors = self.error_rate * (self.total_queries_processed - 1) + 1
            self.error_rate = errors / self.total_queries_processed

        # Update average relevance score
        total_relevance = self.average_relevance_score * (self.total_queries_processed - 1) + relevance_score
        self.average_relevance_score = total_relevance / self.total_queries_processed

        # Update retrieval count
        self.total_retrievals += 1