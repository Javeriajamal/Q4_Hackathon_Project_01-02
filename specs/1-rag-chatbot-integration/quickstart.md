# Quickstart: RAG Chatbot Integration

## Prerequisites

- Node.js 18+ for frontend development
- Python 3.11+ for backend development
- Access to the existing backend API (FastAPI service)
- Git for version control

## Local Development Setup

### 1. Clone and Setup Backend
```bash
# Backend should already be set up from previous RAG implementation
cd backend
pip install -r requirements.txt  # if available
# Or install required packages:
pip install fastapi uvicorn python-dotenv cohere qdrant-client
```

### 2. Start Backend Service
```bash
cd backend
uvicorn agent_service.api:app --reload --port 8000
```

### 3. Setup Frontend Integration
```bash
# Navigate to Docusaurus project (assuming it's in the root)
# Install dependencies if needed
npm install
```

### 4. Environment Configuration
Create/update `.env` file with:
```
REACT_APP_BACKEND_URL=http://localhost:8000  # For local development
# In production, this would point to the deployed backend
```

## Running Both Services Simultaneously

### Option 1: Separate Terminals
Terminal 1 (Backend):
```bash
cd backend
uvicorn agent_service.api:app --reload --port 8000
```

Terminal 2 (Frontend):
```bash
npm run start  # or whatever command starts Docusaurus
```

### Option 2: Using a Process Manager
Consider using `concurrently` or similar tools to run both services:
```bash
npm install -g concurrently
concurrently "cd backend && uvicorn agent_service.api:app --reload --port 8000" "npm run start"
```

## Key Integration Points

### 1. Chatbot Component Injection
The chatbot component will be injected at the layout level in Docusaurus to appear on all pages.

### 2. Text Selection Feature
The component will detect text selection on the page and offer to ask questions about the selected text.

### 3. API Communication
- Frontend sends queries to backend via REST API
- Backend returns responses with source attributions
- Frontend displays responses in the chat interface

## Testing the Integration

### Basic Functionality Test
1. Start both frontend and backend services
2. Navigate to any book page
3. Type a question in the chatbot interface
4. Verify that you receive a relevant response from the backend

### Text Selection Test
1. Select text on any book page
2. Use the chatbot to ask a question about the selected text
3. Verify the response is contextually relevant to the selected text

## API Endpoints

### Chat Endpoint
- Method: `POST`
- Path: `/api/chat` (or similar path to be determined)
- Request: `{ query: string, context?: { type: "selected-text" | "full-book", content?: string } }`
- Response: `{ response: string, sourceAttributions: [...], confidenceScore: number, ... }`

## Troubleshooting

### CORS Issues
- Ensure backend is configured to allow requests from frontend origin
- Check that `--reload` mode doesn't interfere with CORS settings

### Network Timeout
- Verify backend service is running on expected port
- Check network connectivity between frontend and backend

### Text Selection Not Working
- Verify browser compatibility
- Check for JavaScript errors in console