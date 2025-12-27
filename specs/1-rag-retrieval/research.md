# Research Findings: Vector Retrieval and Pipeline Validation for RAG Chatbot

**Feature**: 1-rag-retrieval
**Created**: 2025-12-21
**Research Phase**: Phase 0

## Research Tasks Completed

### Task 1: Qdrant Collection Configuration

**Objective**: Determine the exact collection name and configuration used for storing embeddings

**Findings**:
- From examining the existing ingestion code in `backend/main.py`, the collection name is "rag_embedding"
- The collection is created with 1024-dimensional vectors using cosine distance
- Vector configuration: `VectorParams(size=1024, distance=models.Distance.COSINE)`
- The collection name can be configured via command line arguments but defaults to "rag_embedding"

**Decision**: Use "rag_embedding" as the default collection name with option to configure differently

### Task 2: Optimal Search Parameters

**Objective**: Determine the best top-k value and relevance thresholds for quality retrieval

**Research**:
- Standard RAG implementations typically use top-k values between 3-10
- For content retrieval, top-k=5 provides good balance between result quantity and relevance
- Relevance thresholds typically range from 0.1-0.5, with 0.3 being a common minimum
- Higher thresholds may miss relevant content, lower thresholds may include irrelevant results

**Decision**: Use top-k=5 with minimum relevance threshold of 0.3 for initial implementation
**Alternatives considered**:
- Top-k=3: Fewer results but higher precision - good for focused queries
- Top-k=10: More results but potential for lower relevance - good for exploratory queries
- Threshold 0.5: Higher precision but risk of missing relevant content
- Threshold 0.1: More recall but potential for irrelevant results

### Task 3: Cohere Embedding Compatibility

**Objective**: Verify embedding model compatibility between ingestion and retrieval

**Findings**:
- Both ingestion (main.py) and retrieval will use Cohere's "embed-multilingual-v3.0" model
- This ensures consistency in embedding space between stored content and query processing
- The model supports multiple languages and is optimized for retrieval tasks
- Input type for queries should be "search_query" while stored content uses "search_document"

**Decision**: Use "embed-multilingual-v3.0" model for query embeddings with "search_query" input type to match stored content using "search_document" type

### Task 4: Performance and Scalability Considerations

**Objective**: Identify performance parameters for efficient retrieval

**Research**:
- Qdrant provides efficient similarity search with configurable indexing options
- For 1000-10000 vectors, search response times are typically under 100ms
- Batch processing can improve performance for multiple queries
- Connection pooling and caching can optimize repeated queries

**Decision**: Implement basic search functionality first, with optimization options for future scaling

### Task 5: Error Handling and Edge Cases

**Objective**: Identify potential error scenarios and handling strategies

**Findings**:
- Network timeouts during API calls (Cohere/Qdrant)
- Rate limiting from external services
- Empty query results when no relevant content exists
- Malformed or extremely long queries
- Qdrant collection not found or temporarily unavailable

**Decision**: Implement comprehensive error handling with appropriate user feedback and logging

## Key Decisions Made

### 1. Collection Name and Configuration
- **Decision**: Use "rag_embedding" as default collection name
- **Rationale**: Consistent with existing ingestion pipeline
- **Implementation**: Make configurable via environment variable or parameter

### 2. Search Parameters
- **Decision**: Top-k=5 with minimum relevance 0.3
- **Rationale**: Balances result quantity with quality
- **Implementation**: Make configurable for different use cases

### 3. Embedding Model Consistency
- **Decision**: Use same Cohere model for queries as stored content
- **Rationale**: Ensures compatibility in embedding space
- **Implementation**: Use "search_query" input type for queries vs "search_document" for stored content

### 4. API Design
- **Decision**: REST API with JSON requests/responses
- **Rationale**: Simple, standard interface that's easy to integrate
- **Implementation**: FastAPI for automatic documentation and validation

## Unknowns Resolved

All previously identified unknowns have been resolved through research:

- ✅ Qdrant collection name: "rag_embedding" (from existing code)
- ✅ Optimal top-k value: 5 (based on best practices)
- ✅ Relevance threshold: 0.3 (based on best practices)
- ✅ Embedding model compatibility: Same model for query and stored content
- ✅ Error handling strategies: Comprehensive approach identified

## Next Steps

1. Begin implementation of query processing module
2. Create vector search functionality
3. Build API endpoints
4. Implement validation and error handling
5. Create test cases based on research findings