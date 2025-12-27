# API Contract: RAG Ingestion Pipeline Functions

## Function: get_all_urls

**Purpose**: Crawls the target website and returns all discoverable URLs

**Input**:
- `base_url` (string): The base URL to start crawling from

**Output**:
- `urls` (array[string]): List of all discovered URLs

**Errors**:
- NetworkError: When unable to reach the website
- InvalidURLError: When the base URL is malformed

## Function: extract_text_from_url

**Purpose**: Extracts clean text content from a given URL

**Input**:
- `url` (string): The URL to extract text from

**Output**:
- `title` (string): The page title
- `text` (string): Clean extracted text content
- `status` (string): Processing status

**Errors**:
- NetworkError: When unable to fetch the URL
- ContentExtractionError: When unable to extract clean text

## Function: chunk_text

**Purpose**: Splits text into appropriately sized chunks for embedding

**Input**:
- `text` (string): The text to chunk
- `chunk_size` (integer): Maximum size of each chunk
- `chunk_overlap` (integer): Overlap between chunks

**Output**:
- `chunks` (array[object]): Array of text chunks with metadata

**Errors**:
- InvalidInputError: When text is empty or parameters are invalid

## Function: embed

**Purpose**: Generates embeddings for text chunks using Cohere API

**Input**:
- `texts` (array[string]): Array of text chunks to embed

**Output**:
- `embeddings` (array[array[float]]): Array of embedding vectors

**Errors**:
- APIError: When Cohere API is unavailable or returns an error
- RateLimitError: When API rate limits are exceeded

## Function: create_collection

**Purpose**: Creates a Qdrant collection for storing embeddings

**Input**:
- `collection_name` (string): Name of the collection to create

**Output**:
- `success` (boolean): Whether the collection was created successfully

**Errors**:
- CollectionExistsError: When collection already exists
- ConnectionError: When unable to connect to Qdrant

## Function: save_chunk_to_qdrant

**Purpose**: Saves a single chunk with its embedding to Qdrant

**Input**:
- `chunk_id` (string): Unique identifier for the chunk
- `embedding` (array[float]): The embedding vector
- `metadata` (object): Metadata to store with the chunk

**Output**:
- `success` (boolean): Whether the chunk was saved successfully

**Errors**:
- StorageError: When unable to save to Qdrant
- ValidationError: When input data is invalid