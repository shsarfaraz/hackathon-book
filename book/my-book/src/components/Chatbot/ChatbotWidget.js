import React, { useState, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';

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
  const [isVisible, setIsVisible] = useState(true); // Always visible since we're using Root.js
  const [isMinimized, setIsMinimized] = useState(false); // Start in open state to show chat interface
  const [messages, setMessages] = useState([{ text: "Welcome! How can I help you today?", sender: "bot" }]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  // Since we're using Root.js, widget should always be visible
  useEffect(() => {
    setIsVisible(true);
  }, []);

  // Toggle widget state
  const toggleWidget = () => {
    setIsMinimized(!isMinimized);
    if (onToggle) {
      onToggle();
    }
  };

  // Function to send message to backend
  const sendMessage = async (message) => {
    if (!message.trim() || isLoading) return;

    setIsLoading(true);

    // Add user message to chat
    const userMessage = { text: message, sender: "user" };
    setMessages(prev => [...prev, userMessage]);

    try {
      // Call backend API
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: message }),
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = { text: data.response_text || data.response || "I'm not sure how to respond to that.", sender: "bot" };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = { text: "Sorry, I couldn't process your request. Please try again.", sender: "bot" };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { text: "Connection error. Please make sure the backend server is running.", sender: "bot" };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle send button click
  const handleSend = () => {
    if (inputValue.trim()) {
      sendMessage(inputValue);
      setInputValue("");
    }
  };

  // Handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // If not on the right page, don't render
  if (!isVisible) {
    return null;
  }

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
            <h3>Chatbot</h3>
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
                  <em>Thinking...</em>
                </div>
              )}
            </div>
            <div className={styles['chat-input-container']}>
              <input
                type="text"
                className={styles['chat-input']}
                placeholder="Ask about the book..."
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
                {isLoading ? 'Sending...' : 'Send'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;