import axios from 'axios';
import { sendChatQuery, getHealthStatus } from '../api';

// Mock axios
jest.mock('axios');

describe('API Service Integration Tests', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('sendChatQuery', () => {
    test('successfully sends chat query to backend', async () => {
      const mockResponse = {
        data: {
          response: 'Test response from backend',
          source_attributions: ['Source 1', 'Source 2'],
          confidence_score: 0.85,
          session_id: 'test-session-123'
        }
      };

      axios.post.mockResolvedValue(mockResponse);

      const result = await sendChatQuery('Test query', null, 'test-session-123');

      expect(axios.post).toHaveBeenCalledWith(
        expect.stringContaining('/api/chat'),
        {
          query: 'Test query',
          context: null,
          session_id: 'test-session-123'
        },
        {
          headers: {
            'Content-Type': 'application/json'
          }
        }
      );
      expect(result).toEqual(mockResponse.data);
    });

    test('handles API error gracefully', async () => {
      const mockError = new Error('Network error');
      axios.post.mockRejectedValue(mockError);

      await expect(sendChatQuery('Test query')).rejects.toThrow('Network error');
    });

    test('sends query with context when provided', async () => {
      const mockResponse = {
        data: {
          response: 'Contextual response',
          source_attributions: [],
          confidence_score: 0.9
        }
      };

      axios.post.mockResolvedValue(mockResponse);

      const context = {
        type: 'selected_text',
        content: 'Selected text content',
        source: 'current_page'
      };

      await sendChatQuery('Test query', context, 'test-session-456');

      expect(axios.post).toHaveBeenCalledWith(
        expect.stringContaining('/api/chat'),
        expect.objectContaining({
          query: 'Test query',
          context: context
        }),
        expect.any(Object)
      );
    });
  });

  describe('getHealthStatus', () => {
    test('successfully retrieves health status from backend', async () => {
      const mockHealthResponse = {
        data: {
          status: 'healthy',
          timestamp: new Date().toISOString(),
          services: {
            agent: true,
            retrieval: true,
            config_loaded: true
          }
        }
      };

      axios.get.mockResolvedValue(mockHealthResponse);

      const result = await getHealthStatus();

      expect(axios.get).toHaveBeenCalledWith(
        expect.stringContaining('/health')
      );
      expect(result).toEqual(mockHealthResponse.data);
    });

    test('handles health check error', async () => {
      const mockError = new Error('Health check failed');
      axios.get.mockRejectedValue(mockError);

      await expect(getHealthStatus()).rejects.toThrow('Health check failed');
    });
  });

  describe('error handling', () => {
    test('handles 400 errors from backend', async () => {
      const mockError = {
        response: {
          status: 400,
          data: {
            detail: 'Invalid request parameters'
          }
        }
      };
      axios.post.mockRejectedValue(mockError);

      await expect(sendChatQuery('')).rejects.toThrow('Invalid request parameters');
    });

    test('handles 500 errors from backend', async () => {
      const mockError = {
        response: {
          status: 500,
          data: {
            detail: 'Internal server error'
          }
        }
      };
      axios.post.mockRejectedValue(mockError);

      await expect(sendChatQuery('Test query')).rejects.toThrow('Internal server error');
    });

    test('handles network errors', async () => {
      const mockError = new Error('Network Error');
      axios.post.mockRejectedValue(mockError);

      await expect(sendChatQuery('Test query')).rejects.toThrow('Network Error');
    });
  });
});