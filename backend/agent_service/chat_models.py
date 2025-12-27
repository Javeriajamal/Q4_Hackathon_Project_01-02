"""
Data models for chatbot functionality in RAG Chatbot Integration feature.
"""
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
from enum import Enum


class SenderType(str, Enum):
    """Enumeration for message senders."""
    USER = "user"
    AI = "ai"


class MessageStatus(str, Enum):
    """Enumeration for message statuses."""
    SENT = "sent"
    SENDING = "sending"
    ERROR = "error"


class ChatMessage(BaseModel):
    """Represents a single message in the conversation."""
    id: str
    sender: SenderType
    content: str
    timestamp: datetime
    status: MessageStatus = MessageStatus.SENT


class ChatRequest(BaseModel):
    """Model for chat requests from the frontend."""
    query: str
    context: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Optional context constraints (e.g., selected text)"
    )
    session_id: Optional[str] = Field(
        default=None,
        description="Optional session identifier for continuity"
    )


class ChatResponse(BaseModel):
    """Model for chat responses from the backend."""
    response: str
    source_attributions: List[Dict[str, Any]] = Field(
        default=[],
        description="List of sources used in the response with metadata"
    )
    confidence_score: float = Field(
        default=0.0,
        description="Confidence in the response (0.0-1.0)"
    )
    processing_time: float = Field(
        default=0.0,
        description="Time taken to process the query in seconds"
    )
    session_id: str = Field(
        default="",
        description="Session identifier for continuity"
    )
    timestamp: str = Field(
        default="",
        description="ISO timestamp of the response"
    )


class ChatSession(BaseModel):
    """Model representing a chat session."""
    id: str
    messages: List[ChatMessage] = Field(
        default=[],
        description="List of messages in the session"
    )
    created_at: datetime
    context_mode: str = Field(
        default="full-book",
        description="Query context mode (full-book or selected-text)"
    )
    selected_text: Optional[str] = Field(
        default=None,
        description="Optional text that was selected (if context is selected-text)"
    )


class QueryContext(BaseModel):
    """Model for query context constraints."""
    type: str = Field(
        default="full-book",
        description="Type of context ('selected-text' or 'full-book')"
    )
    content: Optional[str] = Field(
        default=None,
        description="Content of the context (e.g., selected text)"
    )


class HealthResponse(BaseModel):
    """Model for health check responses."""
    status: str = Field(
        default="healthy",
        description="Overall health status"
    )
    timestamp: str = Field(
        default="",
        description="ISO timestamp of the health check"
    )
    services: Dict[str, str] = Field(
        default={},
        description="Status of individual services"
    )
    message: Optional[str] = Field(
        default=None,
        description="Additional health information"
    )


class ErrorResponse(BaseModel):
    """Model for error responses."""
    error: str = Field(
        default="",
        description="Error type"
    )
    message: str = Field(
        default="",
        description="Human-readable error message"
    )
    timestamp: str = Field(
        default="",
        description="ISO timestamp of the error"
    )
    status_code: Optional[int] = Field(
        default=None,
        description="HTTP status code"
    )