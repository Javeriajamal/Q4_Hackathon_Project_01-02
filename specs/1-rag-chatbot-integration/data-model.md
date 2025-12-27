# Data Model: RAG Chatbot Integration

## Frontend Data Models

### ChatMessage
Represents a single message in the conversation

```javascript
{
  id: string,           // Unique identifier for the message
  sender: "user" | "ai", // Who sent the message
  content: string,      // The message content
  timestamp: Date,      // When the message was created
  status: "sent" | "sending" | "error" // Message transmission status
}
```

### ChatSession
Represents a single chat session (not persisted, browser session only)

```javascript
{
  id: string,           // Session identifier
  messages: ChatMessage[], // Array of messages in the session
  createdAt: Date,      // When the session started
  context: "full-book" | "selected-text", // Query context mode
  selectedText?: string // Optional text that was selected (if context is selected-text)
}
```

### API Request/Response Models

#### ChatRequest
```javascript
{
  query: string,        // The user's query
  context?: {           // Optional context constraints
    type: "selected-text" | "full-book",
    content?: string    // Selected text content if applicable
  },
  sessionId?: string    // Optional session identifier for continuity
}
```

#### ChatResponse
```javascript
{
  response: string,     // The AI-generated response
  sourceAttributions: [ // Sources used in the response
    {
      sourceUrl: string,
      sectionTitle: string,
      chunkIndex: number,
      relevanceScore: number,
      extractedText: string
    }
  ],
  confidenceScore: number, // Confidence in the response
  processingTime: number,  // Time taken to process the query
  sessionId: string,       // Session identifier for continuity
  timestamp: string        // ISO timestamp of the response
}
```

## Backend Data Models (Python)

### ChatMessage (Pydantic Model)
```python
class ChatMessage(BaseModel):
    id: str
    sender: Literal["user", "ai"]
    content: str
    timestamp: datetime
    status: Optional[Literal["sent", "sending", "error"]] = "sent"
```

### ChatRequest (Pydantic Model)
```python
class ChatRequest(BaseModel):
    query: str
    context: Optional[Dict[str, Any]] = None
    session_id: Optional[str] = None
```

### ChatResponse (Pydantic Model)
```python
class ChatResponse(BaseModel):
    response: str
    source_attributions: List[Dict[str, Any]]
    confidence_score: float
    processing_time: float
    session_id: str
    timestamp: str
```

## API Endpoints Data Flow

### POST /api/chat
- Request: ChatRequest
- Response: ChatResponse
- Purpose: Process a user query and return AI response
- Context handling: Supports both full-book and selected-text contexts

### Error Response Model
```javascript
{
  error: string,        // Error type
  message: string,      // Human-readable error message
  timestamp: string     // ISO timestamp of the error
}
```