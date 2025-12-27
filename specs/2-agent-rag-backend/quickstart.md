# Quickstart Guide: Agent-Based RAG Backend Service

**Feature**: 2-agent-rag-backend
**Created**: 2025-12-21

## Overview

This guide will help you set up and run the Agent-Based RAG (Retrieval-Augmented Generation) service that uses OpenAI Agents SDK to provide intelligent responses based on the Physical AI & Humanoid Robotics textbook content.

## Prerequisites

- Python 3.9 or higher
- pip package manager
- Git (for cloning the repository)
- OpenAI API key
- Cohere API key (from existing setup)
- Qdrant Cloud instance access (from existing setup)

## Environment Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

If no requirements.txt exists with the needed dependencies, install them:

```bash
pip install openai fastapi uvicorn python-dotenv cohere qdrant-client requests beautifulsoup4
```

### 4. Configure Environment Variables

Update your `.env` file in the backend directory with the required API keys:

```env
OPENAI_API_KEY=your_openai_api_key_here
COHERE_API_KEY=your_cohere_api_key_here  # From existing setup
QDRANT_URL=your_qdrant_url_here          # From existing setup
QDRANT_API_KEY=your_qdrant_api_key_here  # From existing setup
QDRANT_COLLECTION_NAME=rag_embedding     # From existing setup
AGENT_MODEL_NAME=gpt-4-turbo
HOST=0.0.0.0
PORT=8001
DEFAULT_TOP_K=5
DEFAULT_MIN_RELEVANCE=0.3
SESSION_TIMEOUT_MINUTES=30
LOG_LEVEL=INFO
```

## Running the Service

### 1. Start the Agent Service

```bash
cd backend
uvicorn agent_service.main:app --host 0.0.0.0 --port 8001
```

Alternatively, if using a main.py approach:

```bash
cd backend
python -m agent_service.main
```

### 2. Verify Service is Running

Check the health endpoint:

```bash
curl http://localhost:8001/api/agent/health
```

You should receive a response indicating the service is healthy.

## Using the Agent API

### 1. Submit a Query to the Agent

```bash
curl -X POST http://localhost:8001/api/agent/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_TOKEN" \
  -d '{
    "query": "Explain digital twin simulation concepts",
    "top_k": 5,
    "min_relevance": 0.3
  }'
```

### 2. Example Response

```json
{
  "response": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring, analysis, and optimization of the physical counterpart. It uses sensors and data to mirror the real-world behavior of the system, allowing for predictive maintenance, performance optimization, and what-if scenario analysis...",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction-to-digital-twins",
      "section_title": "Introduction to Digital Twins",
      "chunk_index": 2,
      "relevance_score": 0.85,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system..."
    }
  ],
  "confidence_score": 0.92,
  "processing_time": 2.34,
  "session_id": "sess_abc123xyz",
  "timestamp": "2025-12-21T01:00:00Z",
  "quality_metrics": {
    "average_relevance_score": 0.82,
    "result_diversity": 0.65,
    "response_time": 2.34
  }
}
```

### 3. Validate an Agent Response

```bash
curl -X POST http://localhost:8001/api/agent/validate-response \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_TOKEN" \
  -d '{
    "query": "What is digital twin simulation?",
    "response": "Digital twin simulation is a virtual representation of a physical system...",
    "retrieved_context": [
      {
        "content": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring...",
        "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/",
        "relevance_score": 0.85
      }
    ]
  }'
```

### 4. Example Validation Response

```json
{
  "is_properly_grounded": true,
  "grounding_percentage": 0.85,
  "validation_notes": "Response appropriately uses retrieved content with good attribution",
  "validation_time": 0.12
}
```

## Session Management

### Get Session Details

```bash
curl -X GET http://localhost:8001/api/agent/session/sess_abc123xyz \
  -H "Authorization: Bearer YOUR_API_TOKEN"
```

### Terminate a Session

```bash
curl -X DELETE http://localhost:8001/api/agent/session/sess_abc123xyz \
  -H "Authorization: Bearer YOUR_API_TOKEN"
```

## Python Client Example

```python
import requests
import json

def query_agent(query, top_k=5, min_relevance=0.3, session_id=None):
    """
    Query the RAG agent with natural language.

    Args:
        query: Natural language query
        top_k: Number of results to retrieve (default: 5)
        min_relevance: Minimum relevance threshold (default: 0.3)
        session_id: Session ID for conversation continuity (optional)

    Returns:
        dict: Agent response with source attributions
    """
    url = "http://localhost:8001/api/agent/query"
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer YOUR_API_TOKEN"
    }

    payload = {
        "query": query,
        "top_k": top_k,
        "min_relevance": min_relevance
    }

    if session_id:
        payload["session_id"] = session_id

    response = requests.post(url, json=payload, headers=headers)

    if response.status_code == 200:
        return response.json()
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None

# Example usage
result = query_agent("Explain the principles of ROS2 for robotics applications")
if result:
    print(f"Response: {result['response'][:200]}...")
    print(f"Sources used: {len(result['source_attributions'])}")
    print(f"Processing time: {result['processing_time']}s")
```

## Configuration Options

### Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key for agent functionality
- `COHERE_API_KEY`: Your Cohere API key for embedding generation (from existing setup)
- `QDRANT_URL`: URL of your Qdrant Cloud instance (from existing setup)
- `QDRANT_API_KEY`: API key for Qdrant Cloud access (from existing setup)
- `QDRANT_COLLECTION_NAME`: Name of the collection containing embeddings (default: "rag_embedding")
- `AGENT_MODEL_NAME`: OpenAI model to use for the agent (default: "gpt-4-turbo")
- `HOST`: Host address for the API server (default: "0.0.0.0")
- `PORT`: Port number for the API server (default: 8001)
- `DEFAULT_TOP_K`: Default number of results to return (default: 5)
- `DEFAULT_MIN_RELEVANCE`: Default minimum relevance threshold (default: 0.3)
- `SESSION_TIMEOUT_MINUTES`: Minutes of inactivity before session expires (default: 30)
- `LOG_LEVEL`: Logging level (default: "INFO")

### Runtime Parameters

The API accepts the following parameters:

- `query` (required): Natural language query text (1-500 characters)
- `top_k` (optional): Number of results to retrieve (1-20, default: 5)
- `min_relevance` (optional): Minimum relevance threshold (0.0-1.0, default: 0.3)
- `session_id` (optional): Session identifier for conversation continuity
- `query_types` (optional): Types of queries to prioritize ["conceptual", "factual", "section-based"]

## Troubleshooting

### Common Issues

1. **Authentication errors**: Verify your OpenAI API key is correctly set in environment variables
2. **Qdrant connection errors**: Check that QDRANT_URL and QDRANT_API_KEY are correctly configured
3. **Empty results**: Ensure the knowledge base has been properly ingested using the existing pipeline
4. **Slow responses**: Check if the OpenAI or Cohere APIs are experiencing high latency

### Health Check Endpoints

- `GET /api/agent/health`: Overall service health check
- `GET /api/agent/health/openai`: OpenAI API connectivity check
- `GET /api/agent/health/qdrant`: Qdrant database connectivity check

## Performance Tips

1. **Caching**: Frequently asked queries can be cached to improve response times
2. **Batching**: Multiple related queries can be processed together for efficiency
3. **Session Management**: Use session IDs to maintain context across multiple queries
4. **Parameter Tuning**: Adjust top_k and min_relevance based on your use case requirements

## Next Steps

1. Integrate the agent API into your application
2. Set up monitoring for agent performance and quality metrics
3. Fine-tune parameters based on user feedback
4. Implement rate limiting and user authentication as needed