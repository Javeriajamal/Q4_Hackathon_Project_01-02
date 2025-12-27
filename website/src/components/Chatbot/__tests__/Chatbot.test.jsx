import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import Chatbot from '../Chatbot';

// Mock the API service
jest.mock('../../services/api', () => ({
  sendChatQuery: jest.fn(),
  getHealthStatus: jest.fn()
}));

// Mock child components to test Chatbot in isolation
jest.mock('../ChatMessage', () => ({ message }) => (
  <div data-testid="chat-message">{message.content}</div>
));
jest.mock('../ChatInput', () => ({ onSendMessage, selectedText, onClearSelection }) => (
  <div data-testid="chat-input">
    <input
      data-testid="message-input"
      onChange={(e) => onSendMessage(e.target.value)}
    />
  </div>
));
jest.mock('../ChatHistory', () => ({ messages }) => (
  <div data-testid="chat-history">
    {messages.length > 0 ? 'Messages present' : 'No messages'}
  </div>
));

describe('Chatbot Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders chatbot component with initial state', () => {
    render(<Chatbot />);

    expect(screen.getByTestId('chat-input')).toBeInTheDocument();
    expect(screen.getByTestId('chat-history')).toBeInTheDocument();
    expect(screen.getByText('No messages')).toBeInTheDocument();
  });

  test('handles sending a message', async () => {
    const { sendChatQuery } = require('../../services/api');
    sendChatQuery.mockResolvedValue({
      response: 'Test response',
      source_attributions: [],
      confidence_score: 0.9
    });

    render(<Chatbot />);

    const input = screen.getByTestId('message-input');
    fireEvent.change(input, { target: { value: 'Hello' } });

    // Simulate sending the message
    const chatInput = screen.getByTestId('chat-input');
    // Trigger the send functionality (would be done through ChatInput component)

    await waitFor(() => {
      expect(sendChatQuery).toHaveBeenCalled();
    });
  });

  test('shows error when API call fails', async () => {
    const { sendChatQuery } = require('../../services/api');
    sendChatQuery.mockRejectedValue(new Error('API Error'));

    render(<Chatbot />);

    const input = screen.getByTestId('message-input');
    fireEvent.change(input, { target: { value: 'Hello' } });

    await waitFor(() => {
      expect(screen.getByText(/error/i)).toBeInTheDocument();
    });
  });

  test('handles text selection context', () => {
    render(<Chatbot />);

    // Test that component can handle selected text context
    expect(screen.getByTestId('chat-input')).toBeInTheDocument();
  });
});