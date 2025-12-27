import React, { useState, useEffect } from 'react';
import { Box, Paper, Typography, Divider } from '@mui/material';
import ChatHistory from './ChatHistory';
import ChatInput from './ChatInput';
import useChatbot from '../../hooks/useChatbot';
import { setupTextSelectionListener, getSelectedText } from './utils';
import ErrorBoundary from './ErrorBoundary';
import './chatbot.css';

const Chatbot = ({ initialContext = null }) => {
  const {
    messages,
    input,
    setInput,
    isLoading,
    error,
    sendMessage,
    clearChat
  } = useChatbot();

  const [selectedText, setSelectedText] = useState('');

  // Set up text selection listener
  useEffect(() => {
    const cleanup = setupTextSelectionListener((newSelection) => {
      setSelectedText(newSelection);
    });

    return cleanup;
  }, []);

  const handleSend = async () => {
    if (input.trim()) {
      // Use the currently selected text or the selected text from state
      const currentSelectedText = getSelectedText() || selectedText;
      const context = currentSelectedText ? {
        type: 'selected-text',
        content: currentSelectedText
      } : (initialContext || { type: 'full-book' });

      await sendMessage(input, context);
      // Clear the selected text after sending
      setSelectedText('');
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <ErrorBoundary>
      <Paper
        role="complementary"
        aria-label="AI Assistant Chat Interface"
        elevation={3}
        sx={{
          height: '500px',
          display: 'flex',
          flexDirection: 'column',
          maxWidth: '1000px',
          width: '90%',
          margin: '0 auto',
          overflow: 'hidden',
          borderRadius: '8px',
          border: '1px solid #e0e0e0'
        }}
      >
        {/* Chat Header */}
        <Box
          sx={{
            p: 2,
            bgcolor: 'primary.main',
            color: 'white',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}
          role="banner"
        >
          <Typography variant="h6" component="h2" id="chatbot-title">
            AI Chatbot
          </Typography>
          <button
            onClick={clearChat}
            style={{
              background: 'none',
              border: 'none',
              color: 'white',
              cursor: 'pointer',
              fontSize: '1rem'
            }}
            title="Clear chat"
            aria-label="Clear chat conversation"
          >
            ✕
          </button>
        </Box>

        <Divider />

        {/* Chat Messages */}
        <Box
          sx={{
            flex: 1,
            overflowY: 'auto',
            p: 2,
            bgcolor: '#fafafa'
          }}
          role="log"
          aria-live="polite"
          aria-labelledby="chatbot-title"
        >
          <ChatHistory messages={messages}  isLoading={isLoading}/>
        </Box>

        <Divider />

        {/* Error Display */}
        {error && (
          <Box
            sx={{
              p: 1,
              bgcolor: 'error.light',
              color: 'error.main',
              textAlign: 'center',
              fontSize: '0.8rem'
            }}
            role="alert"
            aria-live="assertive"
          >
            {error}
          </Box>
        )}

        {/* Chat Input */}
        <Box
          sx={{ p: 2, bgcolor: 'white' }}
          role="form"
          aria-label="Chat input area"
        >
          <ChatInput
            input={input}
            setInput={setInput}
            isLoading={isLoading}
            onSend={handleSend}
            onKeyDown={handleKeyDown}
            selectedText={selectedText}
          />
        </Box>
      </Paper>
    </ErrorBoundary>
  );
};

export default Chatbot;