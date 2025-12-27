"""
Data models for the RAG retrieval service.

This module defines the data structures used throughout the service,
including query objects, retrieval results, and request/response models.
"""

from typing import List, Optional, Dict, Any
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum


class QueryType(Enum):
    """Enumeration of query types supported by the system."""
    CONCEPTUAL = "conceptual"
    FACTUAL = "factual"
    SECTION_BASED = "section-based"


@dataclass
class Query:
    """Represents a natural language query from a user."""

    query_text: str
    embedding: List[float] = field(default_factory=list)
    query_type: Optional[QueryType] = None
    timestamp: datetime = field(default_factory=datetime.now)
    processed_at: Optional[datetime] = None

    def __post_init__(self):
        """Validate the query after initialization."""
        if not 1 <= len(self.query_text) <= 500:
            raise ValueError("Query text must be between 1 and 500 characters")

        if self.embedding and len(self.embedding) != 1024:
            raise ValueError("Embedding must have exactly 1024 dimensions")


@dataclass
class ContentChunk:
    """Represents a segment of text from the knowledge base."""

    chunk_id: str  # Could be string or integer in practice
    text: str
    embedding: List[float] = field(default_factory=list)
    url: str = ""
    section_title: str = ""
    chunk_index: int = 0
    word_count: int = 0
    char_count: int = 0
    created_at: datetime = field(default_factory=datetime.now)

    def __post_init__(self):
        """Validate the content chunk after initialization."""
        if not self.chunk_id:
            raise ValueError("Chunk ID is required")

        if not 10 <= len(self.text) <= 2000:
            raise ValueError("Text must be between 10 and 2000 characters")

        if self.embedding and len(self.embedding) != 1024:
            raise ValueError("Embedding must have exactly 1024 dimensions")

        if self.chunk_index < 0:
            raise ValueError("Chunk index must be non-negative")


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


@dataclass
class RetrievalResult:
    """Represents a matched content chunk with relevance score and metadata."""

    chunk_id: str  # Could be string or integer in practice
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
class RetrievalRequest:
    """Input parameters for the retrieval operation."""

    query: str
    top_k: int = 5
    min_relevance: float = 0.3
    query_types: Optional[List[str]] = None

    def __post_init__(self):
        """Validate the retrieval request after initialization."""
        if not 1 <= len(self.query) <= 500:
            raise ValueError("Query must be between 1 and 500 characters")

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
class RetrievalResponse:
    """Output from the retrieval operation."""

    results: List[RetrievalResult]
    query_embedding_time: float
    search_time: float
    total_time: float
    retrieval_count: int
    query_text: str

    def __post_init__(self):
        """Validate the retrieval response after initialization."""
        if len(self.results) != self.retrieval_count:
            raise ValueError("Results array length must match retrieval_count")

        if any(time < 0 for time in [self.query_embedding_time, self.search_time, self.total_time]):
            raise ValueError("Timing values must be non-negative")

        # Results should be ordered by relevance score (descending)
        for i in range(len(self.results) - 1):
            if self.results[i].relevance_score < self.results[i + 1].relevance_score:
                raise ValueError("Results must be ordered by relevance_score (descending)")


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