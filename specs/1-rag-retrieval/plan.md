# Implementation Plan: Vector Retrieval and Pipeline Validation for RAG Chatbot

**Feature**: 1-rag-retrieval
**Created**: 2025-12-21
**Status**: Draft
**Input**: User requirements for RAG vector retrieval system

## Technical Context

### Current State
- RAG ingestion pipeline exists (main.py in backend/)
- Qdrant vector database is configured with Cohere embeddings
- Website content has been ingested with chunked text and metadata
- Existing codebase uses Python, Cohere API, and Qdrant client

### Architecture Components
- **Query Processing**: Natural language query → Cohere embedding generation
- **Vector Database**: Qdrant Cloud instance with stored content embeddings
- **Retrieval Engine**: Similarity search and result ranking functionality
- **Metadata Management**: Source tracking and chunk identification
- **Validation Layer**: Relevance scoring and result quality assessment

### Dependencies
- **Cohere API**: For query embedding generation
- **Qdrant Cloud**: Vector database for similarity search
- **Python Environment**: Backend processing requirements
- **Configuration**: API keys and endpoint URLs

### Technology Stack
- **Backend**: Python 3.9+
- **Embeddings**: Cohere multilingual-v3.0 model
- **Vector DB**: Qdrant Cloud instance
- **Framework**: Standard Python libraries with Cohere and Qdrant clients

### Unknowns
- Specific Qdrant collection name for retrieval [NEEDS CLARIFICATION]
- Optimal top-k value for similarity search [NEEDS CLARIFICATION]
- Relevance threshold for result filtering [NEEDS CLARIFICATION]

## Constitution Check

### Alignment with Project Principles
- **Accuracy**: Retrieval system must return only content from the textbook
- **Clarity**: Results should be clearly presented with source attribution
- **Consistency**: Uniform metadata format across all retrieved chunks
- **Reproducibility**: Retrieval process must be deterministic and testable
- **Integrity**: System must not generate hallucinated content
- **Modularity**: Retrieval functionality should be modular and reusable

### Compliance Verification
- ✅ RAG system will only retrieve from ingested textbook content
- ✅ Results will include proper source attribution (URL, section, chunk)
- ✅ Metadata will be consistent across all retrieved content
- ✅ Retrieval process will be testable and measurable

### Potential Violations
- Need to ensure retrieved content is limited to textbook sources only
- Must prevent system from generating responses based on external knowledge

## Gates

### Gate 1: Technical Feasibility
- **Status**: PASS
- **Verification**: All required technologies (Cohere, Qdrant) are available and accessible
- **Dependencies**: API keys and database access confirmed in existing codebase

### Gate 2: Constitution Compliance
- **Status**: PASS
- **Verification**: Design aligns with project principles and technical standards
- **Alignment**: Uses specified technology stack (Cohere, Qdrant Cloud)

### Gate 3: Resource Availability
- **Status**: PASS
- **Verification**: All required services have free tier or existing access
- **Capacity**: Current infrastructure supports retrieval functionality

## Phase 0: Outline & Research

### Research Tasks

#### Task 1: Qdrant Collection Configuration
**Objective**: Determine the exact collection name and configuration used for storing embeddings
**Research**: Examine existing ingestion code to identify collection naming convention and schema

#### Task 2: Optimal Search Parameters
**Objective**: Determine the best top-k value and relevance thresholds for quality retrieval
**Research**: Best practices for semantic search in RAG systems and typical parameter ranges

#### Task 3: Cohere Embedding Compatibility
**Objective**: Verify embedding model compatibility between ingestion and retrieval
**Research**: Confirm that query embeddings use the same model as stored content embeddings

### Research Findings

#### Decision: Qdrant Collection Name
**Rationale**: Based on existing main.py code, the collection name is "rag_embedding" which is created during ingestion
**Implementation**: Use "rag_embedding" as the default collection name with option to configure

#### Decision: Search Parameters
**Rationale**: Top-k=5 provides good balance between result quantity and relevance; relevance threshold of 0.3 filters low-quality matches
**Alternatives considered**:
- Top-k=3: Fewer results but higher precision
- Top-k=10: More results but potential for lower relevance
**Chosen**: Top-k=5 with threshold 0.3 for initial implementation

#### Decision: Embedding Model Consistency
**Rationale**: Using the same Cohere model (embed-multilingual-v3.0) for both ingestion and retrieval ensures compatibility
**Implementation**: Use "embed-multilingual-v3.0" model for query embeddings to match stored content

## Phase 1: Design & Contracts

### Data Model: data-model.md

#### Query Entity
- **query_text**: String (required) - The original natural language query
- **embedding**: Array<Float> (required) - Vector representation of the query
- **query_type**: Enum (conceptual, factual, section-based) - Classification of query intent
- **timestamp**: DateTime (required) - When the query was processed

#### RetrievalResult Entity
- **chunk_id**: String (required) - Unique identifier for the content chunk
- **content**: String (required) - The retrieved text content
- **relevance_score**: Float (required) - Similarity score between query and chunk
- **metadata**: Object (required) - Source information (URL, section, chunk_index)
- **source_url**: String (required) - Original document URL
- **section_title**: String (required) - Title of the section containing the chunk
- **chunk_index**: Integer (required) - Position of chunk in original document

#### RetrievalRequest Entity
- **query**: Query (required) - The input query to process
- **top_k**: Integer (optional, default=5) - Number of results to return
- **min_relevance**: Float (optional, default=0.3) - Minimum relevance threshold
- **query_types**: Array<String> (optional) - Types of queries to prioritize

#### RetrievalResponse Entity
- **results**: Array<RetrievalResult> (required) - Ranked list of retrieved chunks
- **query_embedding_time**: Float (required) - Time taken to generate query embedding
- **search_time**: Float (required) - Time taken for vector search
- **total_time**: Float (required) - Total processing time
- **retrieval_count**: Integer (required) - Number of chunks returned

### API Contracts

#### Endpoint: POST /api/retrieve
**Purpose**: Process natural language query and return relevant content chunks

**Request Body**:
```json
{
  "query": "Natural language query text",
  "top_k": 5,
  "min_relevance": 0.3,
  "query_types": ["conceptual", "factual", "section-based"]
}
```

**Response**:
```json
{
  "results": [
    {
      "chunk_id": "numeric_id",
      "content": "Retrieved text content...",
      "relevance_score": 0.85,
      "metadata": {
        "source_url": "https://physicalairobotics.netlify.app/...",
        "section_title": "Section Name",
        "chunk_index": 0
      }
    }
  ],
  "query_embedding_time": 0.12,
  "search_time": 0.23,
  "total_time": 0.35,
  "retrieval_count": 5
}
```

**Error Responses**:
- 400: Invalid query parameters
- 429: Rate limit exceeded (from Cohere or Qdrant)
- 500: Internal server error

### System Architecture

#### Components
1. **Query Processor**: Converts natural language to embeddings
2. **Vector Search Engine**: Performs similarity search in Qdrant
3. **Result Ranker**: Orders results by relevance score
4. **Metadata Enricher**: Adds source attribution to results
5. **Validation Layer**: Ensures quality and relevance thresholds

#### Flow
1. User submits natural language query
2. Query Processor generates embedding using Cohere API
3. Vector Search Engine performs similarity search in Qdrant
4. Result Ranker orders results by relevance score
5. Metadata Enricher adds source information
6. Validation Layer filters results by minimum relevance
7. System returns ranked list of content chunks

### Quickstart Guide: quickstart.md

#### Setting up RAG Retrieval Service

1. **Prerequisites**
   - Python 3.9+
   - Cohere API key
   - Qdrant Cloud instance access
   - Environment variables configured

2. **Environment Setup**
   ```bash
   # Set environment variables
   export COHERE_API_KEY="your_cohere_api_key"
   export QDRANT_URL="your_qdrant_url"
   export QDRANT_API_KEY="your_qdrant_api_key"
   ```

3. **Install Dependencies**
   ```bash
   pip install cohere qdrant-client python-dotenv
   ```

4. **Run Retrieval Service**
   ```bash
   python -m retrieval_service.main
   ```

5. **Test Retrieval**
   ```bash
   curl -X POST http://localhost:8000/api/retrieve \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is digital twin simulation?",
       "top_k": 5
     }'
   ```

## Phase 2: Implementation Plan

### Implementation Components

#### 1. Query Processing Module
- **Purpose**: Convert natural language queries to embeddings
- **Location**: `retrieval_service/query_processor.py`
- **Dependencies**: Cohere client
- **Functions**:
  - `generate_query_embedding(query_text: str) -> List[float]`
  - `classify_query_type(query_text: str) -> str`

#### 2. Vector Search Module
- **Purpose**: Perform similarity search in Qdrant
- **Location**: `retrieval_service/vector_search.py`
- **Dependencies**: Qdrant client
- **Functions**:
  - `search_similar_chunks(query_embedding: List[float], top_k: int = 5, min_relevance: float = 0.3) -> List[RetrievalResult]`
  - `get_qdrant_client() -> QdrantClient`

#### 3. Result Validation Module
- **Purpose**: Validate and filter results
- **Location**: `retrieval_service/validation.py`
- **Functions**:
  - `validate_relevance(results: List[RetrievalResult], min_score: float) -> List[RetrievalResult]`
  - `calculate_relevance_metrics(results: List[RetrievalResult]) -> Dict`

#### 4. API Service Module
- **Purpose**: Expose retrieval functionality via REST API
- **Location**: `retrieval_service/api.py`
- **Dependencies**: FastAPI
- **Functions**:
  - `retrieve_endpoint(query_request: RetrievalRequest) -> RetrievalResponse`

### Development Tasks
1. Implement query processing module with Cohere integration
2. Create vector search module with Qdrant client
3. Build result validation and ranking functionality
4. Develop REST API with proper error handling
5. Add logging and performance monitoring
6. Create comprehensive test suite
7. Document API endpoints and usage examples

## Constitution Re-Check (Post-Design)

### Updated Compliance Verification
- ✅ All retrieved content limited to textbook sources (metadata tracking)
- ✅ Proper source attribution maintained in all results
- ✅ Consistent metadata format across all retrieved content
- ✅ Retrieval process is deterministic and testable
- ✅ System prevents hallucinated content generation through source verification

### Architecture Alignment
- ✅ Uses specified technology stack (Cohere, Qdrant Cloud)
- ✅ Modular design supports future enhancements
- ✅ Performance requirements can be met with current architecture
- ✅ Logging and monitoring support quality assurance requirements

## Next Steps

### Immediate Actions
1. Begin implementation of Query Processing Module
2. Set up development environment with required dependencies
3. Create initial test cases for retrieval functionality
4. Implement basic vector search functionality

### Success Criteria Validation
- [ ] Query processing module successfully converts text to embeddings
- [ ] Vector search returns relevant results within 2 seconds
- [ ] Top-5 results contain relevant information for 85% of test queries
- [ ] System handles multiple query types (conceptual, factual, section-based)
- [ ] Proper error handling for edge cases and unavailable services