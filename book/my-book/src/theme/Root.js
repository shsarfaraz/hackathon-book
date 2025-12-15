import React from 'react';
import ChatbotWidget from '@site/src/components/Chatbot/ChatbotWidget';

// This component wraps the entire app and adds the chatbot widget to all pages
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}