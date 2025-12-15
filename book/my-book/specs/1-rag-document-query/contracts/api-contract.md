# API Contract for RAG Document Query System

**Feature**: RAG Document Query System
**Date**: 2025-12-14
**Version**: 1.0

## Base URL
```
http://localhost:8000/api/v1
```

## Endpoints

### 1. Document Ingestion

#### POST /documents/upload
Upload a document for processing and indexing.

**Request**:
- **Content-Type**: multipart/form-data
- **Body**:
  - `file`: Document file (PDF, DOCX, or TXT)
  - `metadata` (optional): JSON string with additional document metadata

**Response**:
- **200 OK**: 
  ```json
  {
    "document_id": "uuid-string",
    "filename": "document.pdf",
    "status": "processing",
    "message": "Document uploaded successfully and processing started"
  }
  ```
- **400 Bad Request**: Invalid file type or format
- **413 Payload Too Large**: Document exceeds size limits

**Headers**:
- `Authorization`: Bearer {api_token} (if implemented)

#### GET /documents/{document_id}
Get the processing status of a document.

**Path Parameters**:
- `document_id`: UUID of the document

**Response**:
- **200 OK**:
  ```json
  {
    "document_id": "uuid-string",
    "filename": "document.pdf",
    "status": "complete",
    "upload_date": "2023-12-14T10:30:00Z",
    "size_bytes": 1024000,
    "page_count": 25,
    "message": "Document processing complete"
  }
  ```
- **404 Not Found**: Document with ID not found

### 2. Document Querying

#### POST /query
Submit a query to search through documents and generate an AI response.

**Request**:
- **Content-Type**: application/json
- **Body**:
  ```json
  {
    "query": "What does the document say about climate change?",
    "top_k": 5,
    "document_ids": ["uuid1", "uuid2"]  // optional, default all docs
  }
  ```

**Response**:
- **200 OK**:
  ```json
  {
    "query_id": "uuid-string",
    "response": "The document discusses various aspects of climate change...",
    "sources": [
      {
        "document_id": "uuid-string",
        "filename": "document.pdf",
        "content": "Relevant snippet from document...",
        "similarity_score": 0.87,
        "chunk_number": 3
      }
    ],
    "confidence_score": 0.92
  }
  ```
- **400 Bad Request**: Query is too short or invalid

### 3. Document Management

#### GET /documents
List all processed documents.

**Query Parameters**:
- `status`: Filter by processing status (optional)
- `limit`: Number of results to return (default 20, max 100)
- `offset`: Number of results to skip (default 0)

**Response**:
- **200 OK**:
  ```json
  {
    "documents": [
      {
        "document_id": "uuid-string",
        "filename": "document.pdf",
        "status": "complete",
        "upload_date": "2023-12-14T10:30:00Z",
        "size_bytes": 1024000,
        "page_count": 25
      }
    ],
    "total_count": 1
  }
  ```

#### DELETE /documents/{document_id}
Delete a document and its associated chunks.

**Path Parameters**:
- `document_id`: UUID of the document to delete

**Response**:
- **200 OK**:
  ```json
  {
    "message": "Document and associated data deleted successfully"
  }
  ```
- **404 Not Found**: Document with ID not found

## Error Responses

All error responses follow the same structure:

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Additional error details if applicable"
  }
}
```

## Authentication

For simplicity in this iteration, authentication is deferred to a future implementation. All endpoints are publicly accessible.