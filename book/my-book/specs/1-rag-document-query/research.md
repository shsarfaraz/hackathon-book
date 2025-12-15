# Research for RAG Document Query System

**Feature**: RAG Document Query System
**Date**: 2025-12-14
**Researcher**: Assistant

## Decision: Document Processing Approach

**Rationale**: Using PyPDF2 and similar libraries to extract text from documents, then chunking with semantic boundaries to optimize for Cohere's context window and Qdrant's vector storage.

**Alternatives considered**: 
- Direct ingestion without chunking (rejected due to context limitations)
- Using different chunking algorithms (settled on semantic boundaries for better retrieval)

## Decision: Embedding Strategy

**Rationale**: Using Cohere's embed-multilingual-v3.0 model for generating document embeddings. This model works well for various languages and document types, and integrates seamlessly with the Cohere generation API.

**Alternatives considered**:
- OpenAI embeddings (not allowed per constitution)
- Sentence Transformers (would require additional dependencies)
- Custom embeddings (violates "no custom model training" constraint)

## Decision: Vector Database Collection Structure

**Rationale**: Creating a dedicated Qdrant collection for document chunks with metadata fields for document ID, source, and chunk number. This enables efficient semantic search and proper attribution.

**Alternatives considered**:
- Single collection with all documents (would complicate retrieval)
- Multiple collections per document (would complicate queries)

## Decision: Metadata Storage Schema

**Rationale**: Storing document metadata in Neon Postgres with fields for document ID, filename, upload date, page count, and processing status. The relational structure allows for complex queries and proper indexing.

**Alternatives considered**:
- Storing metadata in Qdrant only (would limit query capabilities)
- No relational database (violates spec requirements)

## Decision: API Rate Limiting Strategy

**Rationale**: Implementing client-side rate limiting with exponential backoff to handle Cohere API rate limits gracefully without exceeding quotas. This ensures consistent performance while staying within free tier limits.

**Alternatives considered**:
- No rate limiting (would risk exceeding API limits)
- Server-side queue (adds complexity beyond scope)