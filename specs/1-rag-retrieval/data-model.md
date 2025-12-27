# Data Model: Vector Retrieval and Pipeline Validation for RAG Chatbot

**Feature**: 1-rag-retrieval
**Created**: 2025-12-21

## Entity Definitions

### Query
**Description**: Natural language input from user that needs to be matched against stored content

**Attributes**:
- `query_text` (String, required): The original natural language query text
- `embedding` (Array<Float>, required): Vector representation of the query (1024-dimensional)
- `query_type` (Enum, required): Classification of query intent ["conceptual", "factual", "section-based"]
- `timestamp` (DateTime, required): When the query was processed (ISO 8601 format)
- `processed_at` (DateTime, required): When the query was processed by the system

**Validation Rules**:
- `query_text` must be 1-500 characters
- `embedding` must have exactly 1024 dimensions
- `query_type` must be one of the allowed values
- `timestamp` must be in the past or present

### ContentChunk
**Description**: Segments of text from the knowledge base that have been processed and stored in the vector database with metadata

**Attributes**:
- `chunk_id` (String/Integer, required): Unique identifier for the content chunk (numeric ID)
- `text` (String, required): The actual text content of the chunk
- `embedding` (Array<Float>, required): Vector representation of the content (1024-dimensional)
- `url` (String, required): Original document URL where this content originated
- `section_title` (String, required): Title of the section containing the chunk
- `chunk_index` (Integer, required): Position of chunk in original document (0-based)
- `word_count` (Integer, required): Number of words in the text
- `char_count` (Integer, required): Number of characters in the text
- `created_at` (DateTime, required): When the chunk was created (ISO 8601 format)

**Validation Rules**:
- `chunk_id` must be unique within the system
- `text` must be 10-2000 characters
- `embedding` must have exactly 1024 dimensions
- `url` must be a valid URL format
- `chunk_index` must be non-negative

### RetrievalResult
**Description**: Matched content chunk with relevance score and metadata returned in response to a query

**Attributes**:
- `chunk_id` (String/Integer, required): Unique identifier for the content chunk
- `content` (String, required): The retrieved text content
- `relevance_score` (Float, required): Similarity score between query and chunk (0.0-1.0)
- `metadata` (Object, required): Source information object
  - `source_url` (String, required): Original document URL
  - `section_title` (String, required): Title of the section containing the chunk
  - `chunk_index` (Integer, required): Position of chunk in original document
- `rank` (Integer, required): Position in the ranked results list (1-based)

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- `rank` must be positive
- `metadata` object must contain all required fields

### RetrievalRequest
**Description**: Input parameters for the retrieval operation

**Attributes**:
- `query` (String, required): The natural language query text to process
- `top_k` (Integer, optional): Number of results to return (default: 5, range: 1-20)
- `min_relevance` (Float, optional): Minimum relevance threshold (default: 0.3, range: 0.0-1.0)
- `query_types` (Array<String>, optional): Types of queries to prioritize (default: all types)

**Validation Rules**:
- `query` must be 1-500 characters
- `top_k` must be between 1 and 20
- `min_relevance` must be between 0.0 and 1.0
- `query_types` values must be from allowed set

### RetrievalResponse
**Description**: Output from the retrieval operation

**Attributes**:
- `results` (Array<RetrievalResult>, required): Ranked list of retrieved content chunks
- `query_embedding_time` (Float, required): Time taken to generate query embedding (seconds)
- `search_time` (Float, required): Time taken for vector search (seconds)
- `total_time` (Float, required): Total processing time (seconds)
- `retrieval_count` (Integer, required): Number of chunks returned
- `query_text` (String, required): The original query that was processed

**Validation Rules**:
- `results` array length must match `retrieval_count`
- All timing values must be non-negative
- `results` must be ordered by relevance_score (descending)

### QueryLog
**Description**: Log entry for tracking retrieval requests and system performance

**Attributes**:
- `log_id` (String/Integer, required): Unique identifier for the log entry
- `query` (String, required): The original query text
- `query_type` (String, required): Classification of the query
- `results_count` (Integer, required): Number of results returned
- `avg_relevance` (Float, required): Average relevance score of returned results
- `processing_time` (Float, required): Total time to process the query (seconds)
- `timestamp` (DateTime, required): When the query was processed
- `user_agent` (String, optional): Information about the client making the request

**Validation Rules**:
- `log_id` must be unique
- `processing_time` must be non-negative
- `avg_relevance` must be between 0.0 and 1.0

## Entity Relationships

### Query → RetrievalResult
- **Relationship**: One-to-Many
- **Description**: One query can produce multiple retrieval results
- **Cardinality**: 1 query → 0 to N results

### ContentChunk → RetrievalResult
- **Relationship**: One-to-One (for each result)
- **Description**: Each retrieval result corresponds to one content chunk
- **Cardinality**: 1 chunk → 1 result (in a specific retrieval)

### RetrievalRequest → RetrievalResponse
- **Relationship**: One-to-One
- **Description**: Each retrieval request produces exactly one response
- **Cardinality**: 1 request → 1 response

## State Transitions

### Query Processing States
1. **Received**: Query has been received by the system
2. **Embedding**: Query is being converted to vector embedding
3. **Searching**: Vector search is being performed in Qdrant
4. **Ranked**: Results have been ranked by relevance
5. **Validated**: Results have been validated against quality thresholds
6. **Completed**: Response has been prepared and returned

## Data Validation

### Input Validation
- All text inputs are sanitized to prevent injection attacks
- Query length is limited to prevent system overload
- Embedding dimensions are validated to match expected size

### Output Validation
- Retrieved results are verified to come from approved content sources
- Relevance scores are validated to be within expected range
- Metadata completeness is verified before response

### Performance Validation
- Response times are monitored and logged
- Quality metrics are calculated for each retrieval
- System performance indicators are tracked