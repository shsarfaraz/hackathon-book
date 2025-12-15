# Quickstart Guide for RAG Document Query System

**Feature**: RAG Document Query System
**Date**: 2025-12-14
**Version**: 1.0

## Overview

This guide provides step-by-step instructions to quickly set up and run the RAG Document Query System. The system enables semantic search and AI-powered responses based on uploaded documents using Cohere for embeddings and generation, Qdrant for vector storage, and Neon DB for metadata.

## Prerequisites

- Python 3.8 or higher
- Pip package manager
- Git (for cloning the repository)
- API keys and connection details (provided in the project configuration)

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Set up Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

The required packages include:
- qdrant-client
- psycopg2-binary
- cohere
- fastapi
- uvicorn
- python-multipart

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```env
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.g2XnURC4Y1nt8Vhl_KJVAj2j3PS1yy__f2XUtoZzJWE
QDRANT_URL=https://f22366ed-d4f4-46c4-9240-b9c602ce070d.europe-west3-0.gcp.cloud.qdrant.io:6333
NEON_DB_URL=postgresql://neondb_owner:npg_GByY4x9bRVWj@ep-quiet-cloud-a1mwquji-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
COHERE_API_KEY=8TRHBV6QYzj0dB7VpLhxfTPrS6FEILs97Dl4X6vL
```

### 5. Initialize the Application

Run database migrations and set up the Qdrant collection:

```bash
python src/scripts/setup.py
```

## Basic Usage

### 1. Start the API Server

```bash
uvicorn src.api.main:app --reload --port 8000
```

### 2. Upload a Document

Use the API to upload a document for processing:

```bash
curl -X POST "http://localhost:8000/api/v1/documents/upload" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@path/to/your/document.pdf"
```

### 3. Query the Document

Once the document is processed, submit a query:

```bash
curl -X POST "http://localhost:8000/api/v1/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main topic of the document?",
    "top_k": 3
  }'
```

## Jupyter Notebook Usage

For experimentation in a Jupyter notebook environment:

1. Install Jupyter: `pip install jupyter`
2. Launch: `jupyter notebook`
3. Navigate to `notebooks/rag_implementation.ipynb`
4. Follow the step-by-step implementation guide

## Testing the System

Run the unit tests to ensure everything is working correctly:

```bash
pytest tests/unit/ -v
```

Run integration tests:

```bash
pytest tests/integration/ -v
```

## Troubleshooting

### Common Issues

1. **API Limit Exceeded**: If you encounter rate limiting, implement backoff strategies or reduce query frequency.
2. **Connection Issues**: Verify all API keys and connection URLs are correct.
3. **Memory Issues**: For large documents, consider increasing chunk overlap or reducing concurrent operations.

### Verification Steps

1. Verify Qdrant connection: Check that you can connect to your Qdrant instance
2. Verify Neon DB connection: Test database connectivity with provided credentials
3. Verify Cohere API: Test that you can generate embeddings and responses
4. Verify document processing: Upload a small document and confirm it processes successfully