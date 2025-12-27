/*import React from 'react';
import { Box } from '@mui/material';
import ChatMessage from './ChatMessage';
import ErrorBoundary from './ErrorBoundary';
import './chatbot.css';

const ChatHistory = ({ messages }) => {
  const messagesEndRef = React.useRef(null);

  // Scroll to bottom when messages change
  React.useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  return (
    <Box sx={{
      height: '100%',
      overflowY: 'auto',
      display: 'flex',
      flexDirection: 'column'
    }}>
      {messages.length === 0 ? (
        <Box sx={{
          flex: 1,
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          color: 'text.secondary',
          p: 2,
          textAlign: 'center'
        }}>
          
        </Box>
      ) : (
        <>
          {messages.map((message) => (
            <ErrorBoundary key={message.id}>
              <ChatMessage message={message} />
            </ErrorBoundary>
          ))}
          <div ref={messagesEndRef} />
        </>
      )}
    </Box>
  );
};

export default ChatHistory;
*/





import React, { useEffect, useRef } from 'react';
import { Box, Typography } from '@mui/material';

const ChatHistory = ({ messages, isLoading }) => {
  const bottomRef = useRef(null);

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      {messages.map((msg, index) => {
        /*const isUser = msg.role === 'user';*/
        const isUser =
          msg.role === 'user' ||
          msg.sender === 'user' ||
          msg.type === 'user';

        return (
          <Box
            key={index}
            sx={{
              display: 'flex',
              width: '100%',
              justifyContent: isUser ? 'flex-end' : 'flex-start',
            }}
          >
            <Box
              sx={{
                maxWidth: isUser ? '60%' : '85%',
                px: 2,
                py: 1.5,
                borderRadius: '14px',
                fontSize: '0.95rem',
                lineHeight: 1.6,
                backgroundColor: isUser
                  ? '#0b1220'
                  : '#111827',
                color: '#e5e7eb',
                border: isUser
                  ? '1px solid rgba(56,189,248,0.4)'
                  : '1px solid rgba(255,255,255,0.08)',
                boxShadow: !isUser
                  ? '0 0 12px rgba(56,189,248,0.15)'
                  : '0 0 8px rgba(56,189,248,0.25)',
              }}
            >
              {msg.content}
            </Box>
          </Box>
        );
      })}

      {/* Thinking indicator */}
      {isLoading && (
        <Typography
          sx={{
            textAlign: 'center',
            fontSize: '0.85rem',
            fontStyle: 'italic',
            color: 'rgba(255,255,255,0.6)',
            mt: 1,
          }}
        >
          Thinking...
        </Typography>
      )}

      <div ref={bottomRef} />
    </Box>
  );
};

export default ChatHistory;
