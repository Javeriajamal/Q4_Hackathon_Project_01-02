
// API service for backend communication
/*
import axios from 'axios';

// Create an axios instance with base configuration
const apiClient = axios.create({
  /*baseURL: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
  baseURL: import.meta.env.VITE_BACKEND_URL || 'http://localhost:8000',
  timeout: 30000, // 30 second timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

// Function to send a chat query to the backend
export const sendChatQuery = async (query, context = null, sessionId = null) => {
  try {
    const response = await apiClient.post('/api/chat', {
      query,
      context: context || { type: 'full-book' },
      session_id: sessionId,
    });

    return {
      success: true,
      data: response.data,
    };
  } catch (error) {
    console.error('Error sending chat query:', error);

    // Return a structured error response
    return {
      success: false,
      error: error.response?.data?.message || error.message || 'Failed to send query',
      status: error.response?.status || 500,
    };
  }
};

// Function to get health status of the backend
export const getHealthStatus = async () => {
  try {
    const response = await apiClient.get('/api/health');
    return {
      success: true,
      data: response.data.answer,
    };
  } catch (error) {
    console.error('Error getting health status:', error);
    return {
      success: false,
      error: error.response?.data?.message || error.message || 'Failed to get health status',
      status: error.response?.status || 500,
    };
  }
};

export default apiClient;
*/


// API service for backend communication 
import axios from 'axios';

// Create an axios instance with base configuration
const apiClient = axios.create({
  baseURL: import.meta.env.VITE_BACKEND_URL || 'http://localhost:8000',
  timeout: 120000, // 30 second timeout
  headers: {
    'Content-Type': 'application/json',
  },
});

// Function to send a chat query to the backend
export const sendChatQuery = async (query, context = null, sessionId = null) => {
  try {
    const response = await apiClient.post('/api/chat', {
      query,
      context: context || { type: 'full-book' },
      session_id: sessionId,
    });

    console.log('Raw backend response:', response.data); // <-- debug line to check key

    // Safely extract response message
    const message =
      response.data.response ||
      response.data.message ||
      response.data.answer ||
      'No response from backend';

    return {
      success: true,
      data: {
        ...response.data,
        message, // add standardized message key
      },
    };
  } catch (error) {
    console.error('Error sending chat query:', error);

    // Return a structured error response
    return {
      success: false,
      error: error.response?.data?.message || error.message || 'Failed to send query',
      status: error.response?.status || 500,
    };
  }
};

// Function to get health status of the backend
export const getHealthStatus = async () => {
  try {
    const response = await apiClient.get('/api/health');

    const healthMessage =
      response.data.answer ||
      response.data.message ||
      'Backend health unknown';

    return {
      success: true,
      data: healthMessage,
    };
  } catch (error) {
    console.error('Error getting health status:', error);
    return {
      success: false,
      error: error.response?.data?.message || error.message || 'Failed to get health status',
      status: error.response?.status || 500,
    };
  }
};

export default apiClient;
