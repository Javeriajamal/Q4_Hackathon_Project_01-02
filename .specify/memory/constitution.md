# Physical AI & Humanoid Robotics Textbook Constitution

## Project Overview
Create a comprehensive, structured Docusaurus book covering Physical AI & Humanoid Robotics with 4 main modules (3 chapters each), Introduction, and Conclusion. Integrate a RAG chatbot that answers questions based only on selected book text.

## Core Principles
### I. Accuracy
All technical concepts and examples must be correct and verified against authoritative robotics and AI sources. Content must be factually accurate and properly cited.

### II. Clarity
Content should be accessible to students and researchers in robotics and AI, written in a clear and structured manner. Academic yet approachable style with diagrams or examples where appropriate.

### III. Consistency
Maintain uniform terminology, formatting, and style across all chapters. Cross-references between chapters must be consistent.

### IV. Reproducibility
Where relevant, examples and exercises should be reproducible, with references to software versions and libraries. Code examples must be tested and verified.

### V. Integrity
Content must be original and free of plagiarism. Zero tolerance for plagiarism. All content must be original work with proper citations.

### VI. Modularity
Structure book into logical modules and chapters following the specified format: Introduction, 4 Modules (each with 3 chapters), and Conclusion.

## Technical Standards
### Content Structure
- **Format**: Markdown with LaTeX support for equations
- **Organization**: Introduction, 4 Modules (3 chapters each), Conclusion - totaling 12+ chapters
- **Citations**: Reference authoritative sources where needed, using inline notes

### RAG System Integration
- **Technology Stack**: OpenAI Agents/ChatKit SDKs, FastAPI backend
- **Databases**: Neon Serverless Postgres for metadata, Qdrant Cloud Free Tier for vector storage
- **Functionality**: Must answer user questions only based on book text, no external knowledge

### Deployment Requirements
- **Framework**: Docusaurus for static site generation
- **Platform**: GitHub Pages deployment
- **Build Process**: Must build without errors and deploy successfully

## Development Workflow
### Quality Assurance
- All content must be factually accurate and properly cited
- Code examples must be tested and verified
- All external dependencies must be documented and version-controlled
- Cross-references between chapters must be consistent

### Review Process
- Content must undergo technical review for accuracy
- Code examples must be validated for correctness
- RAG system must be tested to ensure it only responds to book content

## Governance
This constitution serves as the authoritative guide for the Physical AI & Humanoid Robotics textbook project. All development activities must align with these principles and standards. Any deviations require explicit approval and documentation of rationale.

All contributors must adhere to the core principles of accuracy, clarity, consistency, reproducibility, and integrity. Content must be original and properly attributed when referencing external sources.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
