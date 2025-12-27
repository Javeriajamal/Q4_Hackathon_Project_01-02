# RAG Content Retrieval Service

This service provides semantic search capabilities for the Physical AI & Humanoid Robotics textbook content using Cohere embeddings and Qdrant vector database.

## Overview

The retrieval service enables AI engineers and backend developers to:
- Perform semantic search against ingested textbook content
- Retrieve relevant text chunks with proper metadata attribution
- Validate retrieval quality and system performance
- Support multiple query types (conceptual, factual, section-based)

## Architecture

The service follows a modular design with the following components:
- **Query Processor**: Converts natural language queries to embeddings
- **Vector Search**: Performs similarity search against Qdrant vectors
- **Result Ranker**: Orders results by relevance score
- **API Service**: Exposes retrieval functionality via REST endpoints

## Configuration

The service uses environment variables for configuration:

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: URL of your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud access
- `COLLECTION_NAME`: Name of the Qdrant collection (default: "rag_embedding")
- `DEFAULT_TOP_K`: Default number of results to return (default: 5)
- `DEFAULT_MIN_RELEVANCE`: Default minimum relevance threshold (default: 0.3)

## API Endpoints

- `POST /api/retrieve`: Retrieve relevant content chunks for a query
- `POST /api/validate-retrieval`: Validate retrieval quality metrics

## Development

To run the service locally:

```bash
cd backend
python -m retrieval_service.main
```

## Testing

Run the test suite:

```bash
pytest tests/
```

## Dependencies

- Python 3.9+
- Cohere API client
- Qdrant client
- FastAPI
- python-dotenv