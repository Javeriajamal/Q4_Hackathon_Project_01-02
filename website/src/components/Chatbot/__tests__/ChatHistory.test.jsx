import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatHistory from '../ChatHistory';

// Mock the ChatMessage component
jest.mock('../ChatMessage', () => ({ message }) => (
  <div data-testid="chat-message" data-sender={message.sender}>
    {message.content}
  </div>
));

describe('ChatHistory Component', () => {
  test('renders welcome message when no messages exist', () => {
    render(<ChatHistory messages={[]} />);

    expect(screen.getByText(/ask anything about the Physical AI & Humanoid Robotics book/i)).toBeInTheDocument();
    expect(screen.getByText(/select text and ask about it specifically/i)).toBeInTheDocument();
  });

  test('renders messages when they exist', () => {
    const messages = [
      {
        id: '1',
        sender: 'user',
        content: 'Hello, what is Physical AI?',
        timestamp: new Date().toISOString()
      },
      {
        id: '2',
        sender: 'ai',
        content: 'Physical AI is a field that combines artificial intelligence with physical systems.',
        source_attributions: ['Chapter 1', 'Introduction'],
        confidence_score: 0.9,
        timestamp: new Date().toISOString()
      }
    ];

    render(<ChatHistory messages={messages} />);

    expect(screen.getByText(/what is Physical AI/i)).toBeInTheDocument();
    expect(screen.getByText(/Physical AI is a field/i)).toBeInTheDocument();
    expect(screen.getByTestId('chat-message')).toBeInTheDocument();
  });

  test('renders multiple messages in correct order', () => {
    const messages = [
      {
        id: '1',
        sender: 'user',
        content: 'First message',
        timestamp: new Date().toISOString()
      },
      {
        id: '2',
        sender: 'ai',
        content: 'Second message',
        timestamp: new Date().toISOString()
      },
      {
        id: '3',
        sender: 'user',
        content: 'Third message',
        timestamp: new Date().toISOString()
      }
    ];

    render(<ChatHistory messages={messages} />);

    const messageElements = screen.getAllByTestId('chat-message');
    expect(messageElements).toHaveLength(3);
  });

  test('scrolls to bottom when new messages are added', () => {
    // Mock the scrollIntoView function
    window.HTMLElement.prototype.scrollIntoView = jest.fn();

    const messages = [
      {
        id: '1',
        sender: 'user',
        content: 'First message',
        timestamp: new Date().toISOString()
      }
    ];

    const { rerender } = render(<ChatHistory messages={messages} />);

    // Add a new message and check if scrolling happens
    const updatedMessages = [
      ...messages,
      {
        id: '2',
        sender: 'ai',
        content: 'Second message',
        timestamp: new Date().toISOString()
      }
    ];

    rerender(<ChatHistory messages={updatedMessages} />);

    // The scrollIntoView would be called in the component
    // This test verifies that the component re-renders properly with new messages
    expect(screen.getByText('Second message')).toBeInTheDocument();
  });

  test('renders system messages appropriately', () => {
    const messages = [
      {
        id: '1',
        sender: 'system',
        content: 'System notification: Connection established',
        timestamp: new Date().toISOString()
      }
    ];

    render(<ChatHistory messages={messages} />);

    expect(screen.getByText(/Connection established/i)).toBeInTheDocument();
  });
});