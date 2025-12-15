import React from 'react';
import Chatbot from '../components/Chatbot/Chatbot';
import Layout from '@theme/Layout';

function ChatPage() {
  return (
    <Layout title="Chat with RAG Bot" description="Chat with our RAG document query bot">
      <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
        <div style={{ textAlign: 'center', marginBottom: '20px' }}>
          <h1>Chat with RAG Document Query Bot</h1>
          <p>Ask questions about the documents in our knowledge base</p>
        </div>
        <Chatbot />
      </div>
    </Layout>
  );
}

export default ChatPage;