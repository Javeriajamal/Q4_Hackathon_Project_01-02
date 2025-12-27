import React from 'react';
import { Box, Typography, Chip, Tooltip } from '@mui/material';
import './chatbot.css';

const ChatMessage = ({ message }) => {
  const isUser = message.sender === 'user';
  const isAI = message.sender === 'ai';
  const isSystem = message.sender === 'system';

  const getMessageStyle = () => {
    if (isUser) {
      return {
        bgcolor: 'primary.main',
        color: 'white',
        alignSelf: 'flex-end',
        ml: '20%',
        mr: '10px'
      };
    } else if (isSystem) {
      return {
        bgcolor: 'error.light',
        color: 'error.main',
        alignSelf: 'center',
        mx: 'auto',
        textAlign: 'center'
      };
    } else {
      return {
        bgcolor: 'grey.100',
        color: 'text.primary',
        alignSelf: 'flex-start',
        ml: '10px',
        mr: '20%'
      };
    }
  };

  const getAvatar = () => {
    if (isUser) return '👤';
    if (isAI) return '🤖';
    return '⚠️';
  };

  return (
    <Box
      sx={{
        display: 'flex',
        flexDirection: 'column',
        mb: 2,
        ...getMessageStyle()
      }}
      role="listitem"
      aria-label={`${isUser ? 'User' : isAI ? 'AI Assistant' : 'System'} message`}
    >
      <Box sx={{ display: 'flex', alignItems: 'center', mb: 0.5 }}>
        <span style={{ marginRight: '8px', fontSize: '1.2em' }} aria-hidden="true">{getAvatar()}</span>
        <Typography variant="subtitle2" sx={{ fontWeight: 'bold' }}>
          {isUser ? 'You' : isAI ? 'AI Assistant' : 'System'}
        </Typography>
      </Box>

      <Typography variant="body1" sx={{ whiteSpace: 'pre-wrap' }} role="paragraph">
        {message.content}
      </Typography>

      {/* Show source attributions for AI responses */}
      {isAI && message.sourceAttributions && message.sourceAttributions.length > 0 && (
        <Box sx={{ mt: 1 }} role="region" aria-label="Source attributions">
          <Typography variant="caption" color="text.secondary" sx={{ fontStyle: 'italic' }}>
            Sources:
          </Typography>
          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5, mt: 0.5 }}>
            {message.sourceAttributions.slice(0, 3).map((source, index) => (
              <Tooltip
                key={index}
                title={`${source.section_title || 'Section'} - Relevance: ${(source.relevance_score * 100).toFixed(1)}%`}
                placement="top"
              >
                <Chip
                  size="small"
                  label={source.section_title || `Source ${index + 1}`}
                  component="a"
                  href={source.source_url}
                  target="_blank"
                  rel="noopener noreferrer"
                  clickable
                  sx={{
                    fontSize: '0.7rem',
                    height: '20px'
                  }}
                  aria-label={`Source: ${source.section_title || `Source ${index + 1}`}`}
                />
              </Tooltip>
            ))}
            {message.sourceAttributions.length > 3 && (
              <Typography variant="caption" color="text.secondary">
                +{message.sourceAttributions.length - 3} more
              </Typography>
            )}
          </Box>
        </Box>
      )}

      {/* Show confidence score for AI responses */}
      {isAI && message.confidenceScore !== undefined && (
        <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }} aria-label={`Confidence score: ${(message.confidenceScore * 100).toFixed(1)}%`}>
          Confidence: {(message.confidenceScore * 100).toFixed(1)}%
        </Typography>
      )}

      {/* Show timestamp */}
      {message.timestamp && (
        <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }} aria-label={`Message time: ${new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}`}>
          {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </Typography>
      )}
    </Box>
  );
};

export default ChatMessage;