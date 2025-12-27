# API Contract: RAG Chatbot Integration

## Overview
This document defines the API contract between the frontend chatbot component and the backend RAG service.

## Base URL
- Development: `http://localhost:8000`
- Production: `[To be determined]`

## Authentication
- No authentication required for initial implementation
- May implement API keys in future iterations

## Endpoints

### POST /api/chat
Process a user query and return an AI-generated response.

#### Request
```json
{
  "query": "string, the user's question",
  "context": {
    "type": "selected-text | full-book",
    "content": "string, optional text content if type is selected-text"
  },
  "session_id": "string, optional session identifier"
}
```

#### Response
```json
{
  "response": "string, the AI-generated response",
  "source_attributions": [
    {
      "source_url": "string, URL of the source",
      "section_title": "string, title of the section",
      "chunk_index": "number, index of the content chunk",
      "relevance_score": "number, 0.0-1.0 relevance score",
      "extracted_text": "string, the relevant text chunk"
    }
  ],
  "confidence_score": "number, 0.0-1.0 confidence in response",
  "processing_time": "number, time taken in seconds",
  "session_id": "string, session identifier",
  "timestamp": "string, ISO 8601 timestamp"
}
```

#### Error Response
```json
{
  "error": "string, error type",
  "message": "string, human-readable error message",
  "timestamp": "string, ISO 8601 timestamp"
}
```

#### Example Request
```json
{
  "query": "What is digital twin simulation?",
  "context": {
    "type": "full-book"
  }
}
```

#### Example Response
```json
{
  "response": "Digital twin simulation is a virtual representation of a physical system...",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction",
      "section_title": "Introduction to Digital Twin Simulation",
      "chunk_index": 2,
      "relevance_score": 0.88,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring..."
    }
  ],
  "confidence_score": 0.85,
  "processing_time": 0.92,
  "session_id": "sess_1a2b3c4d5e6f",
  "timestamp": "2025-12-21T01:00:00Z"
}
```

## Error Codes
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server-side error
- `503 Service Unavailable`: Backend service temporarily unavailable

## Rate Limits
- Default: 100 requests per minute per IP
- May be adjusted based on deployment requirements

## Performance Expectations
- 95% of requests should respond within 10 seconds
- API endpoint should maintain 99.9% availability