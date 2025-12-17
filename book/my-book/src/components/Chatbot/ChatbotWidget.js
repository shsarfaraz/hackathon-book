import React, { useState, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';

// Check if bookIndex is available (for client-side functionality)
let bookIndex = [];
try {
  // Try to import book data for client-side search
  bookIndex = require('../../data/bookIndex.json');
} catch (e) {
  // bookIndex not available, will use backend in dev mode
  console.warn('Book index not available, will use backend in development mode');
}

/**
 * Floating chatbot widget component
 * @param {Object} props - Component properties
 * @param {string} [props.position='bottom-right'] - Position of the widget
 * @param {string[]} [props.showOnPages=['/']] - Pages where widget should appear
 * @param {boolean} [props.isOpen=false] - Initial open state
 * @param {Function} [props.onToggle] - Callback when toggle state changes
 */
const ChatbotWidget = ({
  position = 'bottom-right',
  showOnPages = ['/'],
  isOpen = false,
  onToggle
}) => {
  const [isVisible, setIsVisible] = useState(true);
  const [isMinimized, setIsMinimized] = useState(false);
  const [messages, setMessages] = useState([{ text: "Welcome! How can I help you today?", sender: "bot" }]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  // Determine if we're running in production (Vercel) or development (local)
  const isProduction = typeof window !== 'undefined' &&
    window.location.hostname !== 'localhost' &&
    !window.location.hostname.includes('127.0.0.1') &&
    !window.location.hostname.includes('0.0.0.0');

  useEffect(() => {
    setIsVisible(true);
  }, []);

  const toggleWidget = () => {
    setIsMinimized(!isMinimized);
    if (onToggle) onToggle();
  };

  // Function to search bookIndex (client-side)
  const searchBook = (query) => {
    if (!bookIndex || bookIndex.length === 0) {
      return "Sorry, the book content is not available for searching right now.";
    }

    query = query.toLowerCase();
    const results = bookIndex.filter(
      item =>
        item.chapter.toLowerCase().includes(query) ||
        item.title.toLowerCase().includes(query) ||
        item.content.toLowerCase().includes(query)
    );
    if (results.length > 0) {
      // Return the most relevant results (first 3)
      const relevantResults = results.slice(0, 3);
      return relevantResults.map(item => `${item.chapter}: ${item.title}. ${item.content}`).join('\n\n');
    } else {
      return "Sorry, I couldn't find anything related to your question in the book. Try asking about ROS, Gazebo, Isaac Sim, or Humanoid Robotics.";
    }
  };

  // Function to send message to backend API (for development)
  const sendToBackend = async (message) => {
    const response = await fetch('http://localhost:8000/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ message: message }),
    });

    if (response.ok) {
      const data = await response.json();
      return data.response_text || data.response || "I found some information about that in the book.";
    } else {
      return "Sorry, I couldn't process your request through the backend.";
    }
  };

  // Send message handler - chooses between backend (dev) and client-side (production)
  const sendMessage = async (message) => {
    if (!message.trim() || isLoading) return;

    setIsLoading(true);
    const userMessage = { text: message, sender: "user" };
    setMessages(prev => [...prev, userMessage]);

    try {
      let responseText;

      if (isProduction) {
        // In production (Vercel), use client-side search
        responseText = searchBook(message);
      } else {
        // In development (localhost), try backend first, fallback to client-side
        try {
          responseText = await sendToBackend(message);
        } catch (backendError) {
          console.warn('Backend unavailable, falling back to client-side search:', backendError);
          responseText = searchBook(message);
        }
      }

      const botMessage = { text: responseText, sender: "bot" };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        text: isProduction
          ? "I encountered an issue while searching the book content. Please try rephrasing your question."
          : "Connection error with backend. Make sure the server is running or check client-side content.",
        sender: "bot"
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSend = () => {
    if (inputValue.trim()) {
      sendMessage(inputValue);
      setInputValue("");
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  if (!isVisible) return null;

  return (
    <div className={`${styles['chatbot-widget']} ${styles['chatbot-widget--fixed']} ${isMinimized ? styles['chatbot-widget--minimized'] : styles['chatbot-widget--open']}`}>
      <button
        className={styles['chatbot-widget__toggle']}
        onClick={toggleWidget}
        aria-label={isMinimized ? "Open chatbot" : "Minimize chatbot"}
      >
        {isMinimized ? '💬' : '✕'}
      </button>
      {!isMinimized && (
        <div className={styles['chatbot-widget__content']}>
          <div className={styles['chatbot-widget__header']}>
            <h3>{isProduction ? "Book Assistant" : "Chatbot"}</h3>
          </div>
          <div className={styles['chatbot-widget__body']}>
            <div className={styles['chat-messages']}>
              {messages.map((msg, index) => (
                <div
                  key={index}
                  className={`${styles['message']} ${styles[msg.sender]}`}
                >
                  {msg.text}
                </div>
              ))}
              {isLoading && (
                <div className={styles['message'] + ' ' + styles['bot']}>
                  <em>{isProduction ? "Searching book content..." : "Thinking..."}</em>
                </div>
              )}
            </div>
            <div className={styles['chat-input-container']}>
              <input
                type="text"
                className={styles['chat-input']}
                placeholder={isProduction ? "Ask about the book content..." : "Ask about the book..."}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                disabled={isLoading}
              />
              <button
                className={styles['send-button']}
                onClick={handleSend}
                disabled={isLoading}
              >
                {isLoading ? (isProduction ? 'Searching...' : 'Sending...') : 'Send'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;
