# Quickstart Guide: RAG Content Retrieval Service

**Feature**: 1-rag-retrieval
**Created**: 2025-12-21

## Overview

This guide will help you set up and run the RAG content retrieval service that performs semantic similarity search against stored textbook content.

## Prerequisites

- Python 3.9 or higher
- pip package manager
- Git (for cloning the repository)
- Cohere API key
- Qdrant Cloud instance access

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

If no requirements.txt exists, install the required packages:

```bash
pip install fastapi uvicorn cohere qdrant-client python-dotenv requests beautifulsoup4
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
COLLECTION_NAME=rag_embedding
HOST=0.0.0.0
PORT=8000
```

## Running the Service

### 1. Start the Retrieval Service

```bash
cd backend
uvicorn retrieval_service.api:app --host 0.0.0.0 --port 8000
```

Or if using the main.py approach:

```bash
python -m retrieval_service.main
```

### 2. Verify Service is Running

Open your browser or use curl to check if the service is running:

```bash
curl http://localhost:8000/health
```

You should receive a response indicating the service is healthy.

## Using the API

### 1. Perform Content Retrieval

```bash
curl -X POST http://localhost:8000/api/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is digital twin simulation?",
    "top_k": 5,
    "min_relevance": 0.3
  }'
```

### 2. Example Response

```json
{
  "results": [
    {
      "chunk_id": "123456789",
      "content": "Digital twin simulation is a virtual representation of a physical system that enables real-time monitoring, analysis, and optimization of the physical counterpart.",
      "relevance_score": 0.85,
      "metadata": {
        "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction-to-digital-twins",
        "section_title": "Introduction to Digital Twins",
        "chunk_index": 2
      },
      "rank": 1
    }
  ],
  "query_embedding_time": 0.12,
  "search_time": 0.23,
  "total_time": 0.35,
  "retrieval_count": 5,
  "query_text": "What is digital twin simulation?"
}
```

### 3. Advanced Query with Specific Parameters

```bash
curl -X POST http://localhost:8000/api/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain ROS2 concepts for robotics applications",
    "top_k": 3,
    "min_relevance": 0.5,
    "query_types": ["conceptual", "factual"]
  }'
```

## Testing Retrieval Quality

### Validate Retrieval Performance

```bash
curl -X POST http://localhost:8000/api/validate-retrieval \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "expected_chunks": ["987654321", "111222333"],
    "top_k": 5
  }'
```

### Example Validation Response

```json
{
  "precision": 0.8,
  "recall": 0.75,
  "f1_score": 0.77,
  "retrieved_chunks": ["987654321", "444555666", "777888999"],
  "relevant_retrieved": 3,
  "validation_time": 0.45
}
```

## Python Client Example

```python
import requests
import json

def retrieve_content(query, top_k=5, min_relevance=0.3):
    url = "http://localhost:8000/api/retrieve"
    payload = {
        "query": query,
        "top_k": top_k,
        "min_relevance": min_relevance
    }

    response = requests.post(url, json=payload)

    if response.status_code == 200:
        return response.json()
    else:
        print(f"Error: {response.status_code} - {response.text}")
        return None

# Example usage
results = retrieve_content("What is digital twin simulation?")
if results:
    for result in results['results']:
        print(f"Score: {result['relevance_score']}")
        print(f"Content: {result['content'][:100]}...")
        print(f"Source: {result['metadata']['source_url']}")
        print("---")
```

## Configuration Options

### Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL of your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud access
- `COLLECTION_NAME`: Name of the Qdrant collection (default: "rag_embedding")
- `HOST`: Host address for the API server (default: "0.0.0.0")
- `PORT`: Port number for the API server (default: 8000)
- `DEFAULT_TOP_K`: Default number of results to return (default: 5)
- `DEFAULT_MIN_RELEVANCE`: Default minimum relevance threshold (default: 0.3)

### Runtime Parameters

The API accepts the following parameters:

- `top_k`: Number of results to return (1-20, default: 5)
- `min_relevance`: Minimum relevance score threshold (0.0-1.0, default: 0.3)
- `query_types`: List of query types to prioritize (conceptual, factual, section-based)

## Troubleshooting

### Common Issues

1. **Connection errors to Qdrant**:
   - Verify your QDRANT_URL and QDRANT_API_KEY are correct
   - Check that your Qdrant Cloud instance is accessible

2. **Cohere API errors**:
   - Verify your COHERE_API_KEY is valid
   - Check that you have sufficient API quota

3. **No results returned**:
   - Ensure the "rag_embedding" collection exists and has content
   - Verify that content was properly ingested using the main.py script

### Health Check Endpoints

- `GET /health`: Basic service health check
- `GET /health/database`: Check Qdrant connection
- `GET /health/embeddings`: Check Cohere API connection

## Next Steps

1. Integrate the retrieval API into your RAG chatbot application
2. Set up monitoring for retrieval performance
3. Fine-tune parameters based on usage patterns
4. Add caching for frequently requested queries