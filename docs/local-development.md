# Local Development Guide for RAG Chatbot Integration

This guide provides instructions for setting up and running both the frontend and backend services locally to work with the embedded RAG chatbot.

## Prerequisites

- Node.js 18+ for frontend development
- Python 3.11+ for backend development
- Git for version control
- Access to API keys (Cohere, Qdrant, OpenAI) for backend functionality

## Project Structure

```
Physical AI_And_Humanoid_Robotics_Book/
├── website/                 # Docusaurus frontend
│   ├── src/
│   │   ├── components/Chatbot/  # Chatbot components
│   │   ├── hooks/useChatbot.js  # Chatbot hook
│   │   └── services/api.js      # API service
│   └── docusaurus.config.js
├── backend/                 # FastAPI backend
│   └── agent_service/
│       ├── api.py          # API endpoints (includes /api/chat)
│       ├── chatbot.py      # Chatbot logic
│       └── chat_models.py  # Data models
└── docs/                   # Documentation
    └── local-development.md # This file
```

## Backend Setup

### 1. Navigate to the backend directory

```bash
cd backend
```

### 2. Create a virtual environment (optional but recommended)

```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
# Or install required packages:
pip install fastapi uvicorn python-dotenv cohere qdrant-client openai
```

### 4. Set up environment variables

Create a `.env` file in the backend directory with your API keys:

```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,http://localhost:3002
```

### 5. Start the backend server

```bash
cd backend
uvicorn agent_service.api:app --reload --port 8000
```

The backend server will be available at `http://localhost:8000`.

## Frontend Setup

### 1. Navigate to the website directory

```bash
cd website
```

### 2. Install dependencies

```bash
npm install
```

### 3. Set up environment variables

The frontend uses the following environment variable in `website/.env`:

```env
REACT_APP_BACKEND_URL=http://localhost:8000
```

This tells the frontend where to send API requests during development.

### 4. Start the frontend server

```bash
cd website
npm start
```

The frontend will be available at `http://localhost:3000` (or another available port).

## Running Both Services Simultaneously

### Option 1: Using separate terminals

Terminal 1 (Backend):
```bash
cd backend
uvicorn agent_service.api:app --reload --port 8000
```

Terminal 2 (Frontend):
```bash
cd website
npm start
```

### Option 2: Using concurrently

Install concurrently globally:
```bash
npm install -g concurrently
```

Then run both services:
```bash
concurrently "cd backend && uvicorn agent_service.api:app --reload --port 8000" "cd website && npm start"
```

### Option 3: Using a custom script

Create a `start-dev.sh` script in the project root:

```bash
#!/bin/bash
# Start backend in background
cd backend && uvicorn agent_service.api:app --reload --port 8000 &
BACKEND_PID=$!

# Start frontend in background
cd website && npm start &
FRONTEND_PID=$!

# Function to clean up background processes
cleanup() {
    echo "Shutting down services..."
    kill $BACKEND_PID $FRONTEND_PID
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for processes to finish
wait $BACKEND_PID $FRONTEND_PID
```

Make it executable and run:
```bash
chmod +x start-dev.sh
./start-dev.sh
```

## Chatbot Integration

The embedded chatbot is integrated into the Docusaurus layout through:

1. **Custom Layout**: `website/src/theme/Layout.jsx` wraps all pages with the chatbot
2. **Floating Button**: `website/src/components/Chatbot/FloatingChatButton.jsx` provides the UI toggle
3. **API Service**: `website/src/services/api.js` handles communication with backend
4. **State Management**: `website/src/hooks/useChatbot.js` manages chat state

## Testing the Integration

### Basic Functionality Test

1. Start both frontend and backend services
2. Navigate to any page in the documentation
3. Click the floating chat button in the bottom-right corner
4. Type a question in the chat interface
5. Verify that you receive a relevant response from the backend
6. Check the browser console for any errors

### Text Selection Test

1. Select text on any documentation page
2. Click the "Ask AI" button that appears near the selection (if implemented)
3. Or copy the selected text and paste it into the chat with appropriate context
4. Verify the response is relevant to the selected text

### API Endpoint Test

Verify the chat endpoint is working:

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is digital twin simulation?",
    "context": {
      "type": "full-book"
    }
  }'
```

## Troubleshooting

### Common Issues

#### CORS Errors
- **Problem**: "Access to fetch at 'http://localhost:8000/...' from origin 'http://localhost:3000' has been blocked by CORS policy"
- **Solution**: Verify that the backend is configured with the correct allowed origins in the `.env` file

#### Backend Not Responding
- **Problem**: Frontend shows "Failed to send query" or network timeout
- **Solution**:
  - Verify the backend server is running on the correct port
  - Check that `REACT_APP_BACKEND_URL` is set correctly in `website/.env`
  - Confirm the backend is accessible by visiting `http://localhost:8000/health`

#### API Key Issues
- **Problem**: 401 or 500 errors related to API services
- **Solution**: Verify all required API keys are set in the backend `.env` file

#### Chatbot Not Appearing
- **Problem**: The floating chat button doesn't appear on pages
- **Solution**: Check that the custom Layout wrapper is properly implemented and the component is being loaded

#### Text Selection Not Working
- **Problem**: Text selection features don't respond
- **Solution**: Verify that the text selection utilities are properly imported and event listeners are set up

### Development Tips

1. **Hot Reloading**: Both frontend (Docusaurus) and backend (FastAPI with `--reload`) support hot reloading
2. **Environment Consistency**: Use the same backend URL format in both development and production environments
3. **Error Logging**: Check both frontend console and backend server logs for debugging information
4. **API Testing**: Use tools like Postman or curl to test API endpoints directly

## Configuration Notes

### Backend Configuration

The backend configuration is managed in `backend/agent_service/config.py` and can be customized via environment variables in the `.env` file.

### Frontend Configuration

The frontend configuration is managed through:
- Environment variables in `website/.env`
- Docusaurus configuration in `website/docusaurus.config.js`
- Component styling in `website/src/components/Chatbot/chatbot.css`

## Performance Considerations

- The backend is configured with a 30-second timeout for API requests
- The frontend shows loading indicators during query processing
- Error handling is implemented to gracefully handle network failures
- Session management maintains context across page navigation

## Next Steps

Once local development is set up successfully, you can:
- Customize the chatbot UI to match your documentation theme
- Extend the functionality with additional features
- Test with different types of content and queries
- Deploy to a staging environment for further testing