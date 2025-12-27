import React, { useState } from 'react';
import { Fab, Badge, Box } from '@mui/material';
import SupportAgentIcon from '@mui/icons-material/SupportAgent';
import CloseIcon from '@mui/icons-material/Close';
import Chatbot from './Chatbot';
import './chatbot.css';

const FloatingChatButton = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [hasUnread, setHasUnread] = useState(false);

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
    if (!isChatOpen) {
      setHasUnread(false); // Clear unread status when opening
    }
  };

  // Function to be called when new messages arrive
  const notifyNewMessage = () => {
    if (!isChatOpen) {
      setHasUnread(true);
    }
  };

  return (
    <Box sx={{ position: 'fixed', bottom: 20, right: 20, zIndex: 1000 }}>
      {isChatOpen ? (
        <Box sx={{
          position: 'absolute',
          bottom: 70,
          right: 0,
          width: '400px',
          height: '500px',
          display: 'flex',
          flexDirection: 'column'
        }}>
          <Chatbot />
        </Box>
      ) : null}

      <Badge color="secondary" variant="dot" invisible={!hasUnread}>
        <Fab
          color="primary"
          onClick={toggleChat}
          sx={{
            boxShadow: '0 0 12px rgba(56, 189, 248, 0.6)',
            background: '#007bff',
            color: '#fff',
            transition: 'all 0.3s ease-in-out',
            animation: 'chatbot-breathe 3s ease-in-out infinite',

           '&:hover': {
             transform: 'scale(1.15) rotate(15deg)',
             boxShadow: '0 0 22px rgba(56, 189, 248, 0.9)',
            },

           '& svg': {
             fontSize: '1.8rem',
            }
          }}
        >
          {isChatOpen ? <CloseIcon /> : <SupportAgentIcon />}
        </Fab>
      </Badge>
    </Box>
  );
};

export default FloatingChatButton;