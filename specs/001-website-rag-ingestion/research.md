# Research: Website RAG Ingestion Pipeline

## Decision: Technology Stack Selection
**Rationale**: Based on user requirements and project constitution, selected Python with specific libraries for the RAG ingestion pipeline.

**Alternatives considered**:
- Node.js with LangChain: Would require different skill set, not aligned with existing Python focus
- Java/Spring Boot: More complex for this specific use case
- Go: Good performance but less suitable for ML/LLM integrations

## Decision: Single File Architecture
**Rationale**: User explicitly requested all functionality in a single main.py file with specific function names. This simplifies deployment and maintenance for the ingestion pipeline.

**Alternatives considered**:
- Modular architecture with separate files: More maintainable but violates user requirements
- Package structure: Better organization but not what was requested

## Decision: Docusaurus Website Crawling Approach
**Rationale**: Docusaurus sites have predictable structure with navigation elements that need to be excluded. Will use requests + BeautifulSoup for crawling and extraction, focusing on content areas while avoiding headers, footers, and navigation.

**Alternatives considered**:
- Selenium: More robust for dynamic content but slower and more complex
- Scrapy: More powerful but overkill for single site extraction
- Playwright: Good for JavaScript-heavy sites but not necessary here

## Decision: Text Chunking Strategy
**Rationale**: Will implement recursive character-based chunking to maintain semantic coherence while fitting within Cohere's token limits. Target chunks of approximately 500-1000 tokens to balance context and retrieval precision.

**Alternatives considered**:
- Sentence-based chunking: May create chunks that are too small or break semantic coherence
- Fixed character length: May split sentences or paragraphs inappropriately
- Semantic chunking: More advanced but requires additional processing

## Decision: Cohere Embedding Model
**Rationale**: Cohere's embed-multilingual-v3.0 model provides good performance for technical content and handles various languages. Will use the appropriate input type for the content being processed.

**Alternatives considered**:
- OpenAI embeddings: Different pricing model and API structure
- Sentence Transformers: Self-hosted but requires more infrastructure
- Hugging Face models: More complex deployment and management

## Decision: Qdrant Cloud Integration
**Rationale**: Aligns with project constitution requirement for Qdrant Cloud. Will use the Python client to store embeddings with metadata including URL, section title, and chunk index.

**Alternatives considered**:
- Pinecone: Different API and pricing structure
- Weaviate: Self-hosted option but requires more setup
- Chroma: Local-first but doesn't meet cloud requirement

## Decision: Error Handling and Resilience
**Rationale**: Implement comprehensive error handling for network requests, API failures, and rate limiting. Include retry logic and graceful degradation to ensure pipeline reliability.

**Alternatives considered**:
- Minimal error handling: Faster to implement but less robust
- Fail-fast approach: Simpler but could interrupt entire pipeline for single issues

## Key Unknowns Resolved:

1. **Website structure**: The target site https://physicalairobotics.netlify.app/ is a Docusaurus site with predictable content structure
2. **Cohere API access**: Will require API key configuration via environment variables
3. **Qdrant Cloud setup**: Will require cluster URL and API key configuration
4. **Rate limiting**: Need to implement appropriate delays to respect website terms
5. **Content extraction selectors**: Need to identify proper CSS selectors for Docusaurus content areas