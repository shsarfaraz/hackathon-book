// API service for RAG chatbot
class RAGChatAPI {
    constructor(baseURL = 'http://localhost:8000/api') {
        this.baseURL = baseURL;
    }

    // Send a message to the chat endpoint
    async sendMessage(message, conversationId = null, options = {}) {
        const requestBody = {
            message: message,
            conversation_id: conversationId || null,
            top_k: options.topK || 5,
            response_type: options.responseType || 'standard'
        };

        try {
            const response = await fetch(`${this.baseURL}/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(requestBody)
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            return await response.json();
        } catch (error) {
            console.error('Error sending message:', error);
            throw error;
        }
    }

    // Search documents without AI generation
    async searchDocuments(query, topK = 5) {
        try {
            const response = await fetch(`${this.baseURL}/search`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    query: query,
                    top_k: topK
                })
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            return await response.json();
        } catch (error) {
            console.error('Error searching documents:', error);
            throw error;
        }
    }

    // Upload a document
    async uploadDocument(file) {
        const formData = new FormData();
        formData.append('file', file);

        try {
            const response = await fetch(`${this.baseURL}/documents/upload`, {
                method: 'POST',
                body: formData
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            return await response.json();
        } catch (error) {
            console.error('Error uploading document:', error);
            throw error;
        }
    }

    // Get list of documents
    async getDocuments() {
        try {
            const response = await fetch(`${this.baseURL}/documents`);

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            return await response.json();
        } catch (error) {
            console.error('Error fetching documents:', error);
            throw error;
        }
    }
}

// Export for use in frontend
if (typeof module !== 'undefined' && module.exports) {
    module.exports = RAGChatAPI;
}