import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatMessage from '../ChatMessage';

describe('ChatMessage Component', () => {
  test('renders user message correctly', () => {
    const message = {
      id: '1',
      sender: 'user',
      content: 'Hello, this is a test message',
      timestamp: new Date().toISOString()
    };

    render(<ChatMessage message={message} />);

    expect(screen.getByText('Hello, this is a test message')).toBeInTheDocument();
    expect(screen.getByTestId('user-message')).toBeInTheDocument();
  });

  test('renders AI message correctly', () => {
    const message = {
      id: '2',
      sender: 'ai',
      content: 'This is an AI response',
      source_attributions: ['Source 1', 'Source 2'],
      confidence_score: 0.85,
      timestamp: new Date().toISOString()
    };

    render(<ChatMessage message={message} />);

    expect(screen.getByText('This is an AI response')).toBeInTheDocument();
    expect(screen.getByTestId('ai-message')).toBeInTheDocument();
    expect(screen.getByText('Source 1')).toBeInTheDocument();
    expect(screen.getByText('Source 2')).toBeInTheDocument();
  });

  test('renders system message correctly', () => {
    const message = {
      id: '3',
      sender: 'system',
      content: 'System notification',
      timestamp: new Date().toISOString()
    };

    render(<ChatMessage message={message} />);

    expect(screen.getByText('System notification')).toBeInTheDocument();
    expect(screen.getByTestId('system-message')).toBeInTheDocument();
  });

  test('shows confidence score for AI messages', () => {
    const message = {
      id: '4',
      sender: 'ai',
      content: 'AI response with confidence',
      confidence_score: 0.95,
      timestamp: new Date().toISOString()
    };

    render(<ChatMessage message={message} />);

    expect(screen.getByText('AI response with confidence')).toBeInTheDocument();
  });

  test('formats timestamp correctly', () => {
    const timestamp = new Date('2023-01-01T12:00:00Z').toISOString();
    const message = {
      id: '5',
      sender: 'user',
      content: 'Message with timestamp',
      timestamp: timestamp
    };

    render(<ChatMessage message={message} />);

    expect(screen.getByText('Message with timestamp')).toBeInTheDocument();
  });
});