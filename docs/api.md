# RAG Agent Service API Documentation

## Overview

The RAG Agent Service provides a REST API for interacting with an AI agent that performs retrieval-augmented generation using OpenAI's Agents SDK and the Physical AI & Humanoid Robotics knowledge base.

## Base URL

```
https://your-agent-service-domain.com/api/
```

## Authentication

The API uses Bearer token authentication. Include your API token in the Authorization header:

```
Authorization: Bearer YOUR_API_TOKEN
```

## Endpoints

### POST /api/chat

Process a chat query and return a response based on RAG functionality. This endpoint is designed for the embedded chatbot UI and supports both general queries and context-specific queries (e.g., based on selected text).

#### Request Body

```json
{
  "query": "Natural language query text",
  "context": {
    "type": "selected-text",
    "content": "Selected text content",
    "source": "current_page"
  },
  "session_id": "optional-session-id"
}
```

**Parameters:**
- `query` (string, required): The natural language query to process
- `context` (object, optional): Additional context for the query (e.g., selected text)
  - `type` (string): Type of context ("selected-text", "full-book", etc.)
  - `content` (string): Content of the context (e.g., selected text)
  - `source` (string): Source of the context (e.g., current page URL)
- `session_id` (string, optional): Session identifier for conversation continuity

#### Response

```json
{
  "response": "Generated response based on retrieved context",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction",
      "section_title": "Introduction to Digital Twin Simulation",
      "chunk_index": 2,
      "relevance_score": 0.85,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system..."
    }
  ],
  "confidence_score": 0.85,
  "processing_time": 0.85,
  "retrieval_time": 0.35,
  "retrieval_count": 5,
  "session_id": "session-abc123xyz",
  "timestamp": "2025-12-21T01:00:00Z",
  "query_text": "What is digital twin simulation?",
  "quality_metrics": {
    "is_properly_grounded": true,
    "grounding_percentage": 0.88,
    "validation_notes": "Response is well-grounded with strong evidence from provided context"
  }
}
```

**Response Fields:**
- `response` (string): The agent's response to the query
- `source_attributions` (array): List of sources used in the response with metadata
- `confidence_score` (float): Agent's confidence in the response (0.0-1.0)
- `processing_time` (float): Total time to process the query (seconds)
- `retrieval_time` (float): Time to retrieve relevant content (seconds)
- `retrieval_count` (integer): Number of content chunks retrieved
- `session_id` (string): Session identifier for conversation continuity
- `timestamp` (string): ISO 8601 timestamp of the response
- `query_text` (string): Original query text
- `quality_metrics` (object): Validation metrics for response quality

#### Example Request

```bash
curl -X POST https://your-agent-service-domain.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is digital twin simulation?",
    "context": {
      "type": "selected-text",
      "content": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring, analysis, and optimization of the physical counterpart.",
      "source": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction"
    },
    "session_id": "sess_1a2b3c4d5e6f"
  }'
```

#### Example Response

```json
{
  "response": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring, analysis, and optimization of the physical counterpart. It uses sensors and data to mirror the real-world behavior of the system, allowing for predictive maintenance, performance optimization, and what-if scenario analysis. Digital twins are widely used in manufacturing, healthcare, and robotics applications.",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction",
      "section_title": "Introduction to Digital Twin Simulation",
      "chunk_index": 2,
      "relevance_score": 0.88,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring..."
    },
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-2-applications",
      "section_title": "Applications of Digital Twin Simulation",
      "chunk_index": 1,
      "relevance_score": 0.82,
      "extracted_text": "Digital twins are widely used in manufacturing, healthcare, and robotics applications..."
    }
  ],
  "confidence_score": 0.85,
  "processing_time": 0.92,
  "retrieval_time": 0.35,
  "retrieval_count": 2,
  "session_id": "sess_1a2b3c4d5e6f",
  "timestamp": "2025-12-21T01:00:00Z",
  "query_text": "What is digital twin simulation?",
  "quality_metrics": {
    "is_properly_grounded": true,
    "grounding_percentage": 0.88,
    "validation_notes": "Response appropriately uses retrieved content with good attribution"
  }
}
```

### POST /api/agent/query

Submit a natural language query to the AI agent for processing with retrieval-augmented generation.

#### Request Body

```json
{
  "query": "Natural language query text",
  "top_k": 5,
  "min_relevance": 0.3,
  "query_types": ["conceptual", "factual", "section-based"],
  "session_id": "optional-session-id"
}
```

**Parameters:**
- `query` (string, required): The natural language query to process (1-500 characters)
- `top_k` (integer, optional): Number of results to return (1-20, default: 5)
- `min_relevance` (float, optional): Minimum relevance threshold (0.0-1.0, default: 0.3)
- `query_types` (array, optional): Types of queries to prioritize (conceptual, factual, section-based)
- `session_id` (string, optional): Session identifier for conversation continuity

#### Response

```json
{
  "response": "Generated response based on retrieved context",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction",
      "section_title": "Introduction to Digital Twin Simulation",
      "chunk_index": 2,
      "relevance_score": 0.85,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system..."
    }
  ],
  "confidence_score": 0.85,
  "processing_time": 0.85,
  "retrieval_time": 0.35,
  "retrieval_count": 5,
  "session_id": "session-abc123xyz",
  "timestamp": "2025-12-21T01:00:00Z",
  "query_text": "What is digital twin simulation?",
  "quality_metrics": {
    "is_properly_grounded": true,
    "grounding_percentage": 0.88,
    "validation_notes": "Response is well-grounded with strong evidence from provided context"
  }
}
```

**Response Fields:**
- `response` (string): The agent's response to the query
- `source_attributions` (array): List of sources used in the response with metadata
- `confidence_score` (float): Agent's confidence in the response (0.0-1.0)
- `processing_time` (float): Total time to process the query (seconds)
- `retrieval_time` (float): Time to retrieve relevant content (seconds)
- `retrieval_count` (integer): Number of content chunks retrieved
- `session_id` (string): Session identifier for conversation continuity
- `timestamp` (string): ISO 8601 timestamp of the response
- `query_text` (string): Original query text
- `quality_metrics` (object): Validation metrics for response quality

#### Example Request

```bash
curl -X POST https://your-agent-service-domain.com/api/agent/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_TOKEN" \
  -d '{
    "query": "Explain the principles of digital twin simulation",
    "top_k": 5,
    "min_relevance": 0.3
  }'
```

#### Example Response

```json
{
  "response": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring, analysis, and optimization of the physical counterpart. It uses sensors and data to mirror the real-world behavior of the system, allowing for predictive maintenance, performance optimization, and what-if scenario analysis. Digital twins are widely used in manufacturing, healthcare, and robotics applications.",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction",
      "section_title": "Introduction to Digital Twin Simulation",
      "chunk_index": 2,
      "relevance_score": 0.88,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring..."
    },
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-2-applications",
      "section_title": "Applications of Digital Twin Simulation",
      "chunk_index": 1,
      "relevance_score": 0.82,
      "extracted_text": "Digital twins are widely used in manufacturing, healthcare, and robotics applications..."
    }
  ],
  "confidence_score": 0.85,
  "processing_time": 0.92,
  "retrieval_time": 0.35,
  "retrieval_count": 2,
  "session_id": "sess_1a2b3c4d5e6f",
  "timestamp": "2025-12-21T01:00:00Z",
  "query_text": "Explain the principles of digital twin simulation",
  "quality_metrics": {
    "is_properly_grounded": true,
    "grounding_percentage": 0.88,
    "validation_notes": "Response appropriately uses retrieved content with good attribution"
  }
}
```

### POST /api/agent/validate-response

Validate that an agent response is properly grounded in retrieved context.

#### Request Body

```json
{
  "query": "Original query text",
  "response": "Agent response to validate",
  "retrieved_context": [
    {
      "content": "Retrieved content chunk",
      "source_url": "https://example.com",
      "relevance_score": 0.85
    }
  ]
}
```

**Parameters:**
- `query` (string, required): The original query that was processed
- `response` (string, required): The agent's response to validate
- `retrieved_context` (array, required): Context that was retrieved for the query

#### Response

```json
{
  "is_properly_grounded": true,
  "grounding_percentage": 0.88,
  "validation_notes": "Response is well-grounded with strong evidence from provided context",
  "validation_time": 0.08
}
```

**Response Fields:**
- `is_properly_grounded` (boolean): Whether the response is properly grounded in the context
- `grounding_percentage` (float): Percentage of response grounded in the context (0.0-1.0)
- `validation_notes` (string): Additional notes about the validation
- `validation_time` (float): Time taken to perform validation (seconds)

### GET /api/agent/health

Check the health status of the agent service and its dependencies.

#### Response

```json
{
  "status": "healthy",
  "timestamp": "2025-12-21T01:00:00Z",
  "services": {
    "openai_api": "available",
    "qdrant_db": "available",
    "cohere_api": "available"
  },
  "metrics": {
    "current_time": "2025-12-21T01:00:00Z"
  }
}
```

**Response Fields:**
- `status` (string): Overall health status (healthy, degraded, unhealthy)
- `timestamp` (string): ISO 8601 timestamp of the health check
- `services` (object): Status of individual services
- `metrics` (object): Additional health metrics

## Error Responses

The API returns standard HTTP error codes:

- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication token
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server-side error

### Error Response Format

```json
{
  "error": "error_type",
  "message": "Human-readable error message",
  "timestamp": "2025-12-21T01:00:00Z"
}
```

## Query Types

The system supports three types of queries:

1. **Conceptual** (`conceptual`): Questions seeking explanations, definitions, or theoretical understanding
   - Example: "Explain how machine learning works"

2. **Factual** (`factual`): Questions seeking specific information, numbers, dates, or statistics
   - Example: "How many layers does a neural network have?"

3. **Section-based** (`section-based`): Questions referencing specific parts of the content
   - Example: "According to chapter 2, what does it say about physics simulation?"

## Performance Guidelines

- **Response Time**: 95% of queries should respond within 10 seconds
- **Availability**: Service should maintain 99.9% uptime
- **Concurrency**: System supports up to 50 concurrent users
- **Reliability**: Error rate should be less than 1% under normal conditions

## Rate Limits

- Default rate limit: 100 requests per minute per IP
- For higher rate limits, contact support to upgrade your plan

## Best Practices

1. **Query Optimization**: Use specific and clear query text for better results
2. **Session Management**: Use session IDs for multi-turn conversations
3. **Parameter Tuning**: Adjust `top_k` and `min_relevance` based on your use case requirements
4. **Error Handling**: Implement proper error handling for API failures
5. **Response Validation**: Use the validation endpoint to assess response quality when needed

## Testing

### Sample Test Queries

```bash
# Basic chat query
curl -X POST https://your-agent-service-domain.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is digital twin simulation?"
  }'

# Chat query with selected text context
curl -X POST https://your-agent-service-domain.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept further",
    "context": {
      "type": "selected-text",
      "content": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring, analysis, and optimization of the physical counterpart.",
      "source": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction"
    }
  }'

# Agent query (legacy endpoint)
curl -X POST https://your-agent-service-domain.com/api/agent/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_TOKEN" \
  -d '{
    "query": "Explain ROS2 concepts",
    "top_k": 7,
    "min_relevance": 0.4
  }'

# Health check
curl https://your-agent-service-domain.com/api/agent/health
```

## Integration Examples

### Python Client

```python
import requests
import json

class RAGAgentClient:
    def __init__(self, base_url, api_token):
        self.base_url = base_url.rstrip('/')
        self.headers = {
            "Authorization": f"Bearer {api_token}",
            "Content-Type": "application/json"
        }

    def query(self, query_text, top_k=5, min_relevance=0.3, query_types=None, session_id=None):
        payload = {
            "query": query_text,
            "top_k": top_k,
            "min_relevance": min_relevance
        }

        if query_types:
            payload["query_types"] = query_types
        if session_id:
            payload["session_id"] = session_id

        response = requests.post(
            f"{self.base_url}/api/agent/query",
            headers=self.headers,
            json=payload
        )

        if response.status_code == 200:
            return response.json()
        else:
            raise Exception(f"API request failed: {response.status_code} - {response.text}")

    def chat(self, query, context=None, session_id=None):
        """
        Process a chat query with optional context (e.g., selected text).

        Args:
            query (str): The user's query
            context (dict, optional): Additional context for the query
            session_id (str, optional): Session identifier for continuity

        Returns:
            dict: The chat response with source attributions
        """
        payload = {
            "query": query
        }

        if context:
            payload["context"] = context
        if session_id:
            payload["session_id"] = session_id

        response = requests.post(
            f"{self.base_url}/api/chat",
            headers=self.headers,
            json=payload
        )

        if response.status_code == 200:
            return response.json()
        else:
            raise Exception(f"Chat API request failed: {response.status_code} - {response.text}")

    def validate_response(self, query, response, retrieved_context):
        payload = {
            "query": query,
            "response": response,
            "retrieved_context": retrieved_context
        }

        validation_response = requests.post(
            f"{self.base_url}/api/agent/validate-response",
            headers=self.headers,
            json=payload
        )

        if validation_response.status_code == 200:
            return validation_response.json()
        else:
            raise Exception(f"Validation request failed: {validation_response.status_code} - {validation_response.text}")

    def health_check(self):
        response = requests.get(
            f"{self.base_url}/api/agent/health",
            headers=self.headers
        )

        if response.status_code == 200:
            return response.json()
        else:
            raise Exception(f"Health check failed: {response.status_code} - {response.text}")

# Usage example
client = RAGAgentClient("https://your-agent-service-domain.com", "YOUR_API_TOKEN")

try:
    # Basic chat query
    result = client.chat("What is digital twin simulation?")
    print(f"Response: {result['response']}")
    print(f"Processing time: {result['processing_time']}s")

    # Chat query with context
    context = {
        "type": "selected-text",
        "content": "Digital twin simulation is a virtual representation of a physical system.",
        "source": "current_page"
    }
    result_with_context = client.chat(
        query="Explain this concept further",
        context=context,
        session_id="sess_1a2b3c4d5e6f"
    )
    print(f"Contextual response: {result_with_context['response']}")

except Exception as e:
    print(f"Error: {e}")
```

## Support

- **Documentation**: [Full API Documentation](https://docs.your-service.com)
- **Issues**: [GitHub Issues](https://github.com/your-org/your-repo/issues)
- **Contact**: [support@your-service.com](mailto:support@your-service.com)
- **Status Page**: [Service Status](https://status.your-service.com)