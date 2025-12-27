import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatInput from '../ChatInput';

describe('ChatInput Component', () => {
  test('renders input field and send button', () => {
    render(<ChatInput onSendMessage={jest.fn()} />);

    expect(screen.getByRole('textbox')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /send/i })).toBeInTheDocument();
  });

  test('calls onSendMessage when message is submitted', () => {
    const mockOnSendMessage = jest.fn();
    render(<ChatInput onSendMessage={mockOnSendMessage} />);

    const input = screen.getByRole('textbox');
    fireEvent.change(input, { target: { value: 'Test message' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    expect(mockOnSendMessage).toHaveBeenCalledWith('Test message');
    expect(input.value).toBe('');
  });

  test('handles Enter key press to send message', () => {
    const mockOnSendMessage = jest.fn();
    render(<ChatInput onSendMessage={mockOnSendMessage} />);

    const input = screen.getByRole('textbox');
    fireEvent.change(input, { target: { value: 'Test message' } });
    fireEvent.keyPress(input, { key: 'Enter', code: 'Enter', charCode: 13 });

    expect(mockOnSendMessage).toHaveBeenCalledWith('Test message');
  });

  test('handles Shift+Enter for new line', () => {
    const mockOnSendMessage = jest.fn();
    render(<ChatInput onSendMessage={mockOnSendMessage} />);

    const input = screen.getByRole('textbox');
    fireEvent.change(input, { target: { value: 'Test message' } });
    fireEvent.keyPress(input, { key: 'Enter', code: 'Enter', charCode: 13, shiftKey: true });

    expect(mockOnSendMessage).not.toHaveBeenCalled();
  });

  test('disables send button when input is empty', () => {
    render(<ChatInput onSendMessage={jest.fn()} />);

    const sendButton = screen.getByRole('button', { name: /send/i });
    expect(sendButton).toBeDisabled();
  });

  test('enables send button when input has text', () => {
    render(<ChatInput onSendMessage={jest.fn()} />);

    const input = screen.getByRole('textbox');
    fireEvent.change(input, { target: { value: 'Test' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    expect(sendButton).not.toBeDisabled();
  });

  test('shows selected text indicator when context is provided', () => {
    render(<ChatInput
      onSendMessage={jest.fn()}
      selectedText="Selected text content"
      onClearSelection={jest.fn()}
    />);

    expect(screen.getByText(/selected text:/i)).toBeInTheDocument();
    expect(screen.getByText('Selected text content')).toBeInTheDocument();
  });

  test('calls onClearSelection when clear button is clicked', () => {
    const mockOnClearSelection = jest.fn();
    render(<ChatInput
      onSendMessage={jest.fn()}
      selectedText="Selected text content"
      onClearSelection={mockOnClearSelection}
    />);

    const clearButton = screen.getByRole('button', { name: /clear/i });
    fireEvent.click(clearButton);

    expect(mockOnClearSelection).toHaveBeenCalled();
  });
});