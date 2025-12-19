---
id: 1
title: Update Textbook Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, textbook, physical-ai, humanoid-robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

```
/sp.constitution

Project: Physical AI & Humanoid Robotics textbook

Purpose:
- Create a comprehensive, structured Docusaurus book covering Physical AI & Humanoid Robotics.
- The book will have 4 main modules with 3 chapters each, plus an Introduction and Conclusion.
- Integrate a RAG chatbot within the book that can answer questions based only on text selected by the user.

Core Principles:
- Accuracy: All technical concepts and examples must be correct and verified against authoritative robotics and AI sources.
- Clarity: Content should be accessible to students and researchers in robotics and AI, written in a clear and structured manner.
- Consistency: Maintain uniform terminology, formatting, and style across all chapters.
- Reproducibility: Where relevant, examples and exercises should be reproducible, with references to software versions and libraries.
- Integrity: Content must be original and free of plagiarism.

Key Standards:
- Structure: Book divided into Introduction, 4 Modules (with 3 chapters each), and Conclusion.
- Citations: Reference authoritative sources where needed, using inline notes.
- RAG Chatbot: Must answer user questions only based on book text; integrate using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier.
- Code examples: Must be functional, properly formatted, and use standard conventions.

Constraints:
- Book content: 10+ chapters covering all modules
- Style: Academic yet approachable; include diagrams or examples where appropriate
- Deployment: Must be deployable via GitHub Pages using Docusaurus
- Plagiarism: 0% tolerance

Success Criteria:
- Book follows the module/chapter structure and meets academic clarity standards
- RAG Chatbot responds correctly to questions based only on the book's content
- Successfully builds and deploys to GitHub Pages without errors
- All code and examples work as intended
```

## Response snapshot

Updated the constitution file (.specify/memory/constitution.md) with specific details for the Physical AI & Humanoid Robotics textbook project, replacing the generic template with project-specific principles, technical standards, and governance guidelines.

## Outcome

- ✅ Impact: Updated project constitution with specific requirements for Physical AI & Humanoid Robotics textbook
- 🧪 Tests: None required for constitution update
- 📁 Files: Modified .specify/memory/constitution.md
- 🔁 Next prompts: Begin creating book structure and content
- 🧠 Reflection: Constitution now properly defines the project scope and standards

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing book structure