# Research: RAG Chatbot Integration

## Frontend Integration Research

### Docusaurus Theme Customization
- Docusaurus allows custom themes through swizzling or theme components
- Can inject components at the layout level using `<Layout>` wrapper
- Alternatively, can use MDX components for page-level injection
- For global chatbot, layout-level injection is recommended

### Text Selection APIs
- `window.getSelection()` provides cross-browser text selection access
- `document.getSelection()` is the standard method
- Need to handle different selection scenarios (single word, multiple paragraphs)
- Consider mobile text selection differences

### React Integration
- Docusaurus is built on React, so standard React components work seamlessly
- Can use React hooks for state management
- CSS modules or styled-components for styling

## Backend API Analysis

### Existing Backend Structure
- The existing backend uses FastAPI with endpoints for RAG functionality
- Current API endpoints are in backend/agent_service/api.py
- Need to add chat-specific endpoints while maintaining existing functionality
- Authentication and rate limiting patterns should be consistent

### API Design Considerations
- Need to support both general queries and context-specific queries
- Response format should be consistent with existing patterns
- Error handling should follow existing patterns
- Consider session management for chat history

## Technical Decisions

### Component Architecture
- Main Chatbot component managing state and layout
- ChatMessage component for individual messages
- ChatInput component with text selection capabilities
- ChatHistory component for message display
- Custom hook for API communication and state management

### State Management
- React hooks for local component state
- Context API if needed for cross-component state sharing
- Consider using a state management library if complexity grows