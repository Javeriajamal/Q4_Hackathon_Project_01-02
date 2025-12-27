import React, { useState, useEffect } from 'react';
import { Box, TextField, Button, CircularProgress, IconButton, Tooltip, Chip } from '@mui/material';
import SendIcon from '@mui/icons-material/Send';
import ContentCopyIcon from '@mui/icons-material/ContentCopy';
import './chatbot.css';
import { FormatListBulleted } from '@mui/icons-material';

const ChatInput = ({ input, setInput, isLoading, onSend, onKeyDown, selectedText = null }) => {
  const [showSelectedText, setShowSelectedText] = useState(!!selectedText);

  useEffect(() => {
    setShowSelectedText(!!selectedText);
  }, [selectedText]);

  const handleInputChange = (e) => {
    setInput(e.target.value);
  };

  const handleButtonClick = () => {
    onSend();
  };

  const handleClearSelection = () => {
    setShowSelectedText(false);
  };

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', width: '100%' }}>
      {/* Show selected text indicator if there's selected text */}
      {showSelectedText && selectedText && (
        <Box
          sx={{
            mb: 1,
            p: 1,
            bgcolor: 'info.light',
            borderRadius: '4px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between'
          }}
          role="status"
          aria-live="polite"
        >
          <Box sx={{ display: 'flex', alignItems: 'center', flexWrap: 'wrap' }}>
            <ContentCopyIcon sx={{ fontSize: '1rem', mr: 1 }} />
            <span style={{ fontSize: '0.85rem', mr: 1 }}>
              Asking about selected text:
            </span>
            <Chip
              label={selectedText.length > 60 ? selectedText.substring(0, 60) + '...' : selectedText}
              size="small"
              variant="outlined"
              sx={{ fontSize: '0.75rem', maxWidth: '200px' }}
              aria-label={`Selected text: ${selectedText}`}
            />
          </Box>
          <Button
            size="small"
            onClick={handleClearSelection}
            sx={{ minWidth: 'auto', padding: '4px' }}
            aria-label="Clear selected text"
          >
            ✕
          </Button>
        </Box>
      )}

      <Box sx={{ display: 'flex', alignItems: 'flex-end' }}>
        <TextField
          value={input}
          onChange={handleInputChange}
          onKeyDown={onKeyDown}
          placeholder={selectedText ? "Ask about the selected text..." : "Ask me a question..."}
          multiline
          maxRows={4}
          variant="outlined"
          fullWidth
          disabled={isLoading}
          sx={{
            mr: 1,
            '& .MuiOutlinedInput-root': {
              bgcolor: 'white'
            }
          }}
          aria-label="Chat message input"
          role="textbox"
          aria-multiline="true"
          id="chat-input"
        />
        <Button
          variant="contained"
          color="primary"
          onClick={handleButtonClick}
          disabled={isLoading}
          sx={{
            height: '54px',
            background: 'linear-gradient(135deg, #2563eb, #38bdf8)',
            boxShadow: '0 0 12px rgba(56, 189, 248, 0.5)',
            transition: 'all 0.2s ease',

            '&:hover': {
              transform: 'translateY(-2px) scale(1.05)',
              boxShadow: '0 0 20px rgba(56, 189, 248, 0.9)',
              background: 'linear-gradient(135deg, #2563eb, #38bdf8)',
            },

            '&:active': {
              transform: 'scale(0.95)',
            },

            '&.Mui-disabled': {
              background: '#020617',
              boxShadow: 'none',
              transform: 'none',
            },
           }}
          aria-label={isLoading ? "Sending message" : "Send message"}
        >
          {isLoading ? (
            <CircularProgress size={24} />
          ) : (
            <SendIcon />
            
          )}
        </Button>
      </Box>
    </Box>
  );
};

export default ChatInput;