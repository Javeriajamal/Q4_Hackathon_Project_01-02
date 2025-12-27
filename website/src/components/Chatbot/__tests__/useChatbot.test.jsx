import { renderHook, act } from '@testing-library/react';
import useChatbot from '../../hooks/useChatbot';

// Mock the API service
jest.mock('../../services/api', () => ({
  sendChatQuery: jest.fn(),
  getHealthStatus: jest.fn()
}));

describe('useChatbot Hook', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('initializes with correct default state', () => {
    const { result } = renderHook(() => useChatbot());

    expect(result.current.messages).toEqual([]);
    expect(result.current.input).toBe('');
    expect(result.current.isLoading).toBe(false);
    expect(result.current.error).toBe(null);
    expect(result.current.sessionId).toBeDefined();
    expect(result.current.selectedText).toBe('');
  });

  test('handles sending a message', async () => {
    const { sendChatQuery } = require('../../services/api');
    sendChatQuery.mockResolvedValue({
      response: 'Test response',
      source_attributions: ['Source 1'],
      confidence_score: 0.8,
      session_id: 'test-session-123'
    });

    const { result } = renderHook(() => useChatbot());

    await act(async () => {
      await result.current.handleSendMessage('Hello, world!');
    });

    // Check that the message was added to the history
    expect(result.current.messages).toHaveLength(2); // User message + AI response
    expect(result.current.messages[0].sender).toBe('user');
    expect(result.current.messages[0].content).toBe('Hello, world!');
    expect(result.current.messages[1].sender).toBe('ai');
    expect(result.current.messages[1].content).toBe('Test response');
  });

  test('handles API errors gracefully', async () => {
    const { sendChatQuery } = require('../../services/api');
    sendChatQuery.mockRejectedValue(new Error('API Error'));

    const { result } = renderHook(() => useChatbot());

    await act(async () => {
      await result.current.handleSendMessage('Hello, world!');
    });

    expect(result.current.error).toBe('Failed to get response: API Error');
  });

  test('updates input value correctly', () => {
    const { result } = renderHook(() => useChatbot());

    act(() => {
      result.current.setInput('New input text');
    });

    expect(result.current.input).toBe('New input text');
  });

  test('handles text selection', () => {
    const { result } = renderHook(() => useChatbot());

    act(() => {
      result.current.setSelectedText('Selected text content');
    });

    expect(result.current.selectedText).toBe('Selected text content');
  });

  test('clears selection', () => {
    const { result } = renderHook(() => useChatbot());

    act(() => {
      result.current.setSelectedText('Selected text content');
    });

    expect(result.current.selectedText).toBe('Selected text content');

    act(() => {
      result.current.clearSelection();
    });

    expect(result.current.selectedText).toBe('');
  });

  test('loads messages from existing session', () => {
    // Mock localStorage
    const mockMessages = [
      {
        id: '1',
        sender: 'user',
        content: 'Previous message',
        timestamp: new Date().toISOString()
      }
    ];

    Storage.prototype.getItem = jest.fn(() => JSON.stringify(mockMessages));

    const { result } = renderHook(() => useChatbot());

    expect(result.current.messages).toEqual(mockMessages);
  });

  test('saves messages to session storage', () => {
    const mockSetItem = jest.fn();
    Storage.prototype.setItem = mockSetItem;

    const { result } = renderHook(() => useChatbot());

    const newMessage = {
      id: '1',
      sender: 'user',
      content: 'New message',
      timestamp: new Date().toISOString()
    };

    act(() => {
      result.current.setMessages([newMessage]);
    });

    expect(mockSetItem).toHaveBeenCalledWith(
      `chatbot_messages_${result.current.sessionId}`,
      JSON.stringify([newMessage])
    );
  });
});