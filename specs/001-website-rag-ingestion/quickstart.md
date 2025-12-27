# Quickstart: Website RAG Ingestion Pipeline

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud cluster URL and API key

## Setup

1. **Create backend directory and initialize UV package**:
   ```bash
   mkdir backend
   cd backend
   uv init
   ```

2. **Install required dependencies**:
   ```bash
   uv add requests beautifulsoup4 cohere qdrant-client python-dotenv
   ```

3. **Set up environment variables**:
   Create a `.env` file in the backend directory with:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   TARGET_URL=https://physicalairobotics.netlify.app/
   ```

4. **Create the main.py file** with the required functions as specified in the implementation plan.

## Running the Pipeline

1. **Execute the main function**:
   ```bash
   cd backend
   python main.py
   ```

2. **Monitor the output** for progress and any errors during processing.

## Configuration

The pipeline can be configured through environment variables:
- `TARGET_URL`: The base URL to crawl (default: https://physicalairobotics.netlify.app/)
- `CHUNK_SIZE`: Maximum characters per text chunk (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks in characters (default: 100)
- `BATCH_SIZE`: Number of chunks to process at once (default: 10)

## Expected Output

- Creates a Qdrant collection named `rag_embedding`
- Stores all processed content chunks with metadata
- Provides progress updates during crawling and embedding
- Logs any errors encountered during processing