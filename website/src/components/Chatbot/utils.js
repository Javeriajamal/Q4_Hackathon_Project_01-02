/**
 * Utility functions for text selection and chatbot functionality
 */

/**
 * Get the currently selected text from the page
 * @returns {string} The selected text, or empty string if no text is selected
 */
export const getSelectedText = () => {
  const selection = window.getSelection();
  return selection.toString().trim();
};

/**
 * Get the context of the selected text including surrounding content
 * @returns {Object} Object containing selected text and additional context
 */
export const getSelectedTextWithContext = () => {
  const selection = window.getSelection();

  if (!selection.toString().trim()) {
    return null;
  }

  const range = selection.getRangeAt(0);
  const selectedText = selection.toString().trim();

  // Get the start and end containers of the selection
  const startContainer = range.startContainer;
  const endContainer = range.endContainer;

  // Try to find a containing element that might provide context
  let contextElement = startContainer.parentElement;
  while (contextElement && contextElement !== document.body) {
    if (contextElement.textContent.length > selectedText.length * 3) {
      break;
    }
    contextElement = contextElement.parentElement;
  }

  return {
    text: selectedText,
    contextElement: contextElement ? contextElement.tagName : null,
    containerText: contextElement ? contextElement.textContent.substring(0, 200) + '...' : null,
    rect: range.getBoundingClientRect() // For positioning UI elements
  };
};

/**
 * Check if text is currently selected on the page
 * @returns {boolean} True if text is selected, false otherwise
 */
export const isTextSelected = () => {
  return getSelectedText().length > 0;
};

/**
 * Highlight text selection visually (if needed for UI feedback)
 * @param {string} color - Color to highlight the selection with
 */
export const highlightSelection = (color = 'yellow') => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0) {
    const range = selection.getRangeAt(0);
    const span = document.createElement('span');
    span.style.backgroundColor = color;
    range.surroundContents(span);
  }
};

/**
 * Remove highlighting from text selection
 */
export const removeHighlight = () => {
  // Remove any highlighting spans we may have added
  const highlightedSpans = document.querySelectorAll('span[style*="background-color"]');
  highlightedSpans.forEach(span => {
    const parent = span.parentNode;
    while (span.firstChild) {
      parent.insertBefore(span.firstChild, span);
    }
    parent.removeChild(span);
  });
};

/**
 * Format context for API request
 * @param {string} selectedText - The text that was selected
 * @returns {Object} Formatted context object for API requests
 */
export const formatContextForAPI = (selectedText) => {
  if (!selectedText) {
    return {
      type: 'full-book',
      content: null
    };
  }

  return {
    type: 'selected-text',
    content: selectedText
  };
};

/**
 * Create a visual indicator for text selection
 * @param {DOMRect} rect - The rectangle of the selected text
 * @returns {HTMLElement} The indicator element
 */
export const createSelectionIndicator = (rect) => {
  // Create a floating button that appears near the selection
  const indicator = document.createElement('div');
  indicator.style.position = 'fixed';
  indicator.style.left = (rect.right + 5) + 'px';
  indicator.style.top = rect.top + 'px';
  indicator.style.zIndex = '10000';
  indicator.style.backgroundColor = '#1976d2';
  indicator.style.color = 'white';
  indicator.style.borderRadius = '16px';
  indicator.style.padding = '6px 12px';
  indicator.style.fontSize = '14px';
  indicator.style.cursor = 'pointer';
  indicator.style.boxShadow = '0 2px 10px rgba(0,0,0,0.2)';
  indicator.style.fontWeight = '500';
  indicator.textContent = 'Ask AI';
  indicator.className = 'chatbot-selection-indicator';

  // Add click event to trigger chat with selected text
  indicator.onclick = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      // This would trigger the chatbot component to use the selected text as context
      console.log('Asking about selected text:', selectedText);
      // In a real implementation, this would communicate with the chatbot component
      // For now, we'll trigger a global event
      window.dispatchEvent(new CustomEvent('chatbot-ask-about-selection', {
        detail: { text: selectedText }
      }));
    }
  };

  return indicator;
};

/**
 * Show text selection indicator near selected text
 */
export const showSelectionIndicator = () => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0 && selection.toString().trim()) {
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Remove any existing indicators
    hideSelectionIndicator();

    // Create and add new indicator
    const indicator = createSelectionIndicator(rect);
    document.body.appendChild(indicator);

    // Store reference for later removal
    window.chatbotSelectionIndicator = indicator;
  }
};

/**
 * Hide text selection indicator
 */
export const hideSelectionIndicator = () => {
  if (window.chatbotSelectionIndicator) {
    document.body.removeChild(window.chatbotSelectionIndicator);
    window.chatbotSelectionIndicator = null;
  }
};

/**
 * Add visual feedback for selected text (subtle highlight)
 */
export const addSelectionHighlight = () => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0 && selection.toString().trim()) {
    // Remove any existing highlights
    removeSelectionHighlight();

    const range = selection.getRangeAt(0);
    const selectedContents = range.extractContents();

    // Create a highlight span
    const highlightSpan = document.createElement('span');
    highlightSpan.style.backgroundColor = 'rgba(25, 118, 210, 0.2)';
    highlightSpan.style.borderBottom = '2px dotted rgba(25, 118, 210, 0.5)';
    highlightSpan.appendChild(selectedContents);

    // Insert the highlighted content back
    range.insertNode(highlightSpan);

    // Store the highlight element for later removal
    window.chatbotTextHighlight = highlightSpan;
  }
};

/**
 * Remove visual feedback for selected text
 */
export const removeSelectionHighlight = () => {
  if (window.chatbotTextHighlight) {
    const parent = window.chatbotTextHighlight.parentNode;
    const contents = window.chatbotTextHighlight.childNodes;

    // Move the contents back to the parent
    while (contents.length > 0) {
      parent.insertBefore(contents[0], window.chatbotTextHighlight);
    }

    // Remove the highlight span
    parent.removeChild(window.chatbotTextHighlight);
    window.chatbotTextHighlight = null;
  }
};

/**
 * Set up text selection event listeners
 * @param {Function} onSelectionChange - Callback function when selection changes
 */
export const setupTextSelectionListener = (onSelectionChange) => {
  let currentSelection = '';

  const handleSelectionChange = () => {
    const newSelection = getSelectedText();

    if (newSelection !== currentSelection) {
      currentSelection = newSelection;

      if (onSelectionChange && typeof onSelectionChange === 'function') {
        onSelectionChange(newSelection);
      }
    }
  };

  // Add event listeners for selection changes
  document.addEventListener('selectionchange', handleSelectionChange);
  document.addEventListener('mouseup', handleSelectionChange);
  document.addEventListener('keyup', handleSelectionChange);

  // Return cleanup function
  return () => {
    document.removeEventListener('selectionchange', handleSelectionChange);
    document.removeEventListener('mouseup', handleSelectionChange);
    document.removeEventListener('keyup', handleSelectionChange);
  };
};