# Implementation Plan: Website RAG Ingestion Pipeline

**Branch**: `001-website-rag-ingestion` | **Date**: 2025-12-20 | **Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a RAG ingestion pipeline that crawls Docusaurus website content (https://physicalairobotics.netlify.app/), extracts clean text, chunks it, generates Cohere embeddings, and stores them with metadata in Qdrant Cloud vector database. The implementation will be a single Python file (main.py) with modular functions for URL crawling, text extraction, content chunking, embedding generation, and vector storage.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, uv (for package management)
**Storage**: Qdrant Cloud vector database (for embeddings), local filesystem (for temporary processing)
**Testing**: pytest (for unit tests)
**Target Platform**: Linux server environment
**Project Type**: Single backend script with modular functions
**Performance Goals**: Process 100+ pages within 4 hours, <500ms response time for similarity search
**Constraints**: Must handle rate limiting, <100MB memory usage during processing, maintain 99% embedding success rate
**Scale/Scope**: Single website with ~100-500 pages, 100MB-1GB total content
**SiteMap URL**: https://physicalairobotics.netlify.app/sitemap.xml

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The implementation aligns with the project constitution:
- Uses Qdrant Cloud as specified in constitution (Section 33)
- Will maintain original content integrity during extraction
- Implementation will be reproducible with proper dependency management via UV
- Code will be structured modularly for maintainability
- All external dependencies will be documented and version-controlled

**Post-design constitution check**:
- All technology choices align with constitution requirements
- Qdrant Cloud integration confirmed as per constitution
- Cohere API usage for embeddings aligns with project goals
- Single-file architecture approved as per user requirements
- Data models and contracts support the RAG system functionality

## Project Structure

### Documentation (this feature)

```text
specs/001-website-rag-ingestion/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── pyproject.toml       # UV package configuration
├── main.py              # Main ingestion pipeline with all required functions
└── requirements.txt     # Dependencies list
```

**Structure Decision**: The implementation will follow a single-file architecture with all required functionality in main.py as specified in the user requirements. The backend folder will contain the pyproject.toml for UV package management and the main.py file with the following functions:
- get_all_urls: Crawls the website and returns all relevant URLs
- extract_text_from_url: Extracts clean text from a given URL
- chunk_text: Splits text into appropriate chunks for embedding
- embed: Generates embeddings using Cohere API
- create_collection: Creates Qdrant collection named 'rag_embedding'
- save_chunk_to_qdrant: Saves individual chunks with metadata to Qdrant
- main function: Orchestrates the entire pipeline

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Single file architecture | User requirement specified single main.py file | Splitting into modules would violate explicit requirement |
