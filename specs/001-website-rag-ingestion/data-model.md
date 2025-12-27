# Data Model: Website RAG Ingestion Pipeline

## Content Chunk Entity

**Definition**: A segment of extracted text from the technical book with associated metadata

**Fields**:
- `id` (string): Unique identifier for the chunk (UUID format)
- `text` (string): The actual text content of the chunk
- `url` (string): Source URL where the content was extracted from
- `section_title` (string): Title of the section from which content was extracted
- `chunk_index` (integer): Sequential index of this chunk within the source document
- `created_at` (datetime): Timestamp when the chunk was created
- `word_count` (integer): Number of words in the chunk
- `token_count` (integer): Estimated number of tokens in the chunk

**Validation Rules**:
- `text` must not be empty
- `url` must be a valid URL format
- `chunk_index` must be non-negative
- `word_count` and `token_count` must be positive integers

## Embedding Vector Entity

**Definition**: High-dimensional vector representation of content chunk with semantic meaning

**Fields**:
- `id` (string): Unique identifier matching the source content chunk
- `vector` (array[float]): The embedding vector (dimension depends on Cohere model)
- `metadata` (object): Associated metadata including URL, section title, chunk index
- `created_at` (datetime): Timestamp when the embedding was generated

**Validation Rules**:
- `vector` must have consistent dimensions based on the embedding model
- `id` must match a corresponding content chunk
- `vector` values must be finite numbers (not NaN or Infinity)

## Crawl Result Entity

**Definition**: Result of crawling a single URL with extracted content and processing status

**Fields**:
- `url` (string): The URL that was crawled
- `title` (string): Page title extracted from HTML
- `content` (string): Clean text content extracted from the page
- `status` (string): Processing status (success, error, skipped)
- `error_message` (string, optional): Error details if processing failed
- `word_count` (integer): Number of words in the extracted content
- `processed_at` (datetime): Timestamp when crawling was completed

**Validation Rules**:
- `url` must be a valid URL
- `status` must be one of: "success", "error", "skipped"
- If `status` is "error", `error_message` must not be empty

## Qdrant Point Structure

**Definition**: Structure for storing embeddings in Qdrant with metadata

**Fields**:
- `id` (string): Unique identifier for the point
- `vector` (array[float]): The embedding vector
- `payload` (object): Metadata object with:
  - `url` (string): Source URL
  - `section_title` (string): Section title
  - `chunk_index` (integer): Chunk sequence number
  - `text` (string): Original text content
  - `created_at` (datetime): Creation timestamp

**Validation Rules**:
- `id` must be unique within the collection
- `vector` dimensions must match collection configuration
- `payload` must contain all required metadata fields