// Frontend data models for ChatMessage and ChatSession

// ChatMessage type definition
export const ChatMessage = {
  id: '',           // Unique identifier for the message
  sender: '',       // "user" | "ai" - Who sent the message
  content: '',      // The message content
  timestamp: null,  // Date - When the message was created
  status: '',       // "sent" | "sending" | "error" - Message transmission status
  sourceAttributions: [], // Array of source attributions for AI responses
  confidenceScore: 0,     // Number - Confidence in AI response
  processingTime: 0,      // Number - Time taken to process the query
};

// ChatSession type definition
export const ChatSession = {
  id: '',                    // Session identifier
  messages: [],              // Array of ChatMessage objects
  createdAt: null,           // Date - When the session started
  context: '',               // "full-book" | "selected-text" - Query context mode
  selectedText: '',          // Optional string - Text that was selected (if context is selected-text)
};

// Create type instances with default values
export const createDefaultChatMessage = (overrides = {}) => ({
  id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
  sender: 'user',
  content: '',
  timestamp: new Date(),
  status: 'sent',
  sourceAttributions: [],
  confidenceScore: 0,
  processingTime: 0,
  ...overrides,
});

export const createDefaultChatSession = (overrides = {}) => ({
  id: `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
  messages: [],
  createdAt: new Date(),
  context: 'full-book',
  selectedText: '',
  ...overrides,
});

// Context type definition
export const QueryContext = {
  type: '',        // "selected-text" | "full-book"
  content: '',     // Optional content for selected text
};

export const createDefaultQueryContext = (overrides = {}) => ({
  type: 'full-book',
  content: '',
  ...overrides,
});