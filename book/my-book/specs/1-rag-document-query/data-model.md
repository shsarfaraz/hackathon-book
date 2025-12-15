# Data Model for RAG Document Query System

**Feature**: RAG Document Query System
**Date**: 2025-12-14
**Modeler**: Assistant

## Entity: Document

**Description**: Represents an ingested document with metadata and processing status

**Fields**:
- `id` (UUID): Unique identifier for the document
- `filename` (str): Original filename
- `upload_date` (datetime): Timestamp when document was uploaded
- `size_bytes` (int): Size of the document in bytes
- `file_type` (str): Document format (PDF, DOCX, TXT, etc.)
- `page_count` (int): Number of pages in the document (if applicable)
- `status` (str): Processing status (PENDING, PROCESSING, COMPLETE, FAILED)
- `metadata` (json): Additional metadata extracted from the document

**Validation Rules**:
- `id` must be unique
- `filename` must not exceed 255 characters
- `size_bytes` must be positive
- `file_type` must be one of allowed formats
- `status` must be one of the defined enum values

## Entity: DocumentChunk

**Description**: A segment of document content with its vector embedding and reference to the parent document

**Fields**:
- `id` (UUID): Unique identifier for the chunk
- `document_id` (UUID): Reference to the parent Document
- `chunk_number` (int): Sequential number of this chunk in the document
- `content` (text): Text content of the chunk
- `embedding_vector` (list[float]): Vector representation of the content
- `char_start_pos` (int): Starting character position in original document
- `char_end_pos` (int): Ending character position in original document
- `metadata` (json): Additional metadata for this chunk

**Validation Rules**:
- `document_id` must reference an existing Document
- `chunk_number` must be positive
- `content` length must be within Cohere's context limits
- `char_start_pos` and `char_end_pos` must be valid for the parent document

**Relationships**:
- One Document to Many DocumentChunks (one-to-many relationship)

## Entity: Query

**Description**: A user's search request that gets processed to find relevant document chunks

**Fields**:
- `id` (UUID): Unique identifier for the query
- `query_text` (text): Original user query
- `created_at` (datetime): Timestamp when query was created
- `embedding_vector` (list[float]): Vector representation of the query
- `user_id` (UUID, optional): Identifier for the user making the query (if implemented later)

**Validation Rules**:
- `query_text` must not be empty
- `embedding_vector` should be properly formatted

## Entity: SearchResult

**Description**: Contains relevant document chunks with relevance scores and source references

**Fields**:
- `id` (UUID): Unique identifier for the search result
- `query_id` (UUID): Reference to the original query
- `document_chunk_id` (UUID): Reference to the matching document chunk
- `similarity_score` (float): Similarity score between query and chunk (0-1)
- `rank` (int): Rank of this result in the search results list
- `retrieved_at` (datetime): When this result was retrieved

**Validation Rules**:
- `query_id` must reference an existing Query
- `document_chunk_id` must reference an existing DocumentChunk
- `similarity_score` must be between 0 and 1
- `rank` must be positive

**Relationships**:
- One Query to Many SearchResults (one-to-many relationship)
- One DocumentChunk to Many SearchResults (one-to-many relationship)

## Entity: Response

**Description**: AI-generated answer based on retrieved document content with source citations

**Fields**:
- `id` (UUID): Unique identifier for the response
- `query_id` (UUID): Reference to the original query
- `response_text` (text): Generated response text
- `generated_at` (datetime): When response was generated
- `source_chunks` (list[UUID]): List of document chunks used to generate the response
- `confidence_score` (float): Confidence in the response accuracy (0-1)

**Validation Rules**:
- `query_id` must reference an existing Query
- `response_text` must not be empty
- `confidence_score` must be between 0 and 1
- `source_chunks` must reference existing DocumentChunks

**Relationships**:
- One Query to One Response (one-to-one relationship)