/*
import { useState, useEffect } from 'react';
import { sendChatQuery } from '../services/api';

// Custom hook for chatbot state management
const useChatbot = () => {
  // State for chat messages
  const [messages, setMessages] = useState([]);

  // State for current input
  const [input, setInput] = useState('');

  // State for loading status
  const [isLoading, setIsLoading] = useState(false);

  // State for error messages
  const [error, setError] = useState(null);

  // State for session ID
  const [sessionId, setSessionId] = useState(() => {
    // Generate a unique session ID or retrieve from localStorage
    return localStorage.getItem('chatbot-session-id') || `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  });

  // Save session ID to localStorage
  useEffect(() => {
    localStorage.setItem('chatbot-session-id', sessionId);
  }, [sessionId]);

  // Function to add a message to the chat
  const addMessage = (message) => {
    setMessages(prev => [...prev, message]);
  };

  // Function to handle sending a message
  const sendMessage = async (query, context = null) => {
    if (!query.trim()) return;

    // Add user message to chat
    const userMessage = {
      id: `msg-${Date.now()}`,
      sender: 'user',
      content: query,
      timestamp: new Date(),
      status: 'sent'
    };

    addMessage(userMessage);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      // Send query to backend
      const result = await sendChatQuery(query, context, sessionId);
      console.log('Backend response:', result.data);

      if (result.success) {
        // Add AI response to chat
        const aiMessage = {
          id: `msg-${Date.now() + 1}`,
          sender: 'ai',
          content: result.data.response,
          timestamp: new Date(),
          sourceAttributions: result.data.source_attributions || [],
          confidenceScore: result.data.confidence_score,
          processingTime: result.data.processing_time,
          status: 'sent'
        };

        addMessage(aiMessage);

        // Update session ID if it was returned from the backend
        if (result.data.session_id) {
          setSessionId(result.data.session_id);
        }
      } else {
        // Handle error response from backend
        setError(result.error);

        // Add error message to chat
        const errorMessage = {
          id: `msg-${Date.now() + 1}`,
          sender: 'system',
          content: `Error: ${result.error}`,
          timestamp: new Date(),
          status: 'error'
        };

        addMessage(errorMessage);
      }
    } catch (err) {
      console.error('Failed to send message:', err);
      setError('Failed to send message. Please try again.');

      // Add error message to chat
      const errorMessage = {
        id: `msg-${Date.now() + 1}`,
        sender: 'system',
        content: 'Error: Failed to send message. Please try again.',
        timestamp: new Date(),
        status: 'error'
      };

      addMessage(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to clear chat history
  const clearChat = () => {
    setMessages([]);
    setError(null);
  };

  // Function to get selected text context
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  return {
    messages,
    input,
    setInput,
    isLoading,
    error,
    sessionId,
    sendMessage,
    addMessage,
    clearChat,
    getSelectedText,
  };
};

export default useChatbot;
*/

import { useState, useEffect } from 'react';
import { sendChatQuery } from '../services/api';

// Custom hook for chatbot state management
const useChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Generate or retrieve session ID
  const [sessionId, setSessionId] = useState(() => {
    return localStorage.getItem('chatbot-session-id') || `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  });

  useEffect(() => {
    localStorage.setItem('chatbot-session-id', sessionId);
  }, [sessionId]);

  // Add a message to chat
  const addMessage = (message) => {
    setMessages(prev => [...prev, message]);
  };

  // Send user query to backend
  const sendMessage = async (query, context = null) => {
    if (!query.trim()) return;

    // Add user message
    const userMessage = {
      id: `msg-${Date.now()}`,
      sender: 'user',
      content: query,
      timestamp: new Date(),
      status: 'sent',
    };
    addMessage(userMessage);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      const result = await sendChatQuery(query, context, sessionId);
      console.log('Backend response:', result.data);

      if (result.success) {
        // Use fallback keys in case response format is different
        const aiContent = result.data.response || result.data.message || "No response from backend";
        const aiSources = result.data.source_attributions || result.data.sources || [];

        const aiMessage = {
          id: `msg-${Date.now() + 1}`,
          sender: 'ai',
          content: aiContent,
          timestamp: new Date(),
          sourceAttributions: aiSources,
          confidenceScore: result.data.confidence_score || result.data.confidence || null,
          processingTime: result.data.processing_time || null,
          status: 'sent',
        };

        addMessage(aiMessage);

        // Update session ID if returned
        if (result.data.session_id) {
          setSessionId(result.data.session_id);
        }
      } else {
        handleError(result.error);
      }
    } catch (err) {
      console.error('Failed to send message:', err);
      handleError('Failed to send message. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  // Error handling helper
  const handleError = (message) => {
    setError(message);
    const errorMessage = {
      id: `msg-${Date.now()}`,
      sender: 'system',
      content: `Error: ${message}`,
      timestamp: new Date(),
      status: 'error',
    };
    addMessage(errorMessage);
  };

  // Clear chat history
  const clearChat = () => {
    setMessages([]);
    setError(null);
  };

  // Get selected text from page
  const getSelectedText = () => {
    const selection = window.getSelection();
    return selection.toString().trim();
  };

  return {
    messages,
    input,
    setInput,
    isLoading,
    error,
    sessionId,
    sendMessage,
    addMessage,
    clearChat,
    getSelectedText,
  };
};

export default useChatbot;
