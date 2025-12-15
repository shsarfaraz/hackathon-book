# Implementation Plan: Embedding Pipeline

## Technical Context

### Feature Overview
Implementation of an embedding pipeline that crawls the deployed Docusaurus website at https://hackathon-book-sigma.vercel.app/, extracts and cleans text, chunks it, generates Cohere embeddings, and stores vectors in Qdrant for RAG-based retrieval.

### Architecture Components
- **Backend Framework**: UV Package (Python package manager)
- **Embedding Service**: Cohere API for text embeddings
- **Vector Database**: Qdrant for storing embeddings
- **Target Website**: https://hackathon-book-sigma.vercel.app/
- **Sitemap URL**: https://hackathon-book-sigma.vercel.app/sitemap.xml


### Core Functions to Implement
- `get_all_URLs`: Discover and collect all URLs from the target website
- `extract_text_from_URLs`: Extract and clean text content from each URL
- `create_collection`: Create a Qdrant collection named 'rag_embedding'
- `save_chunk_to_qdrant`: Store text chunks with embeddings and metadata in Qdrant
- `main`: Execute the complete pipeline

### Technology Stack
- **Language**: Python
- **Package Manager**: UV
- **Web Scraping**: requests/BeautifulSoup or similar
- **Text Processing**: Clean and chunk text content
- **Embeddings**: Cohere API
- **Vector DB**: Qdrant client

### Dependencies
- Cohere Python client
- Qdrant Python client
- Web scraping libraries (requests, BeautifulSoup, etc.)
- Text processing libraries
- Environment management for API keys

### Integration Points
- Cohere API for embedding generation
- Qdrant vector database for storage
- Target website (https://hackathon-book-sigma.vercel.app/) for content extraction

### Known Unknowns
- Specific Cohere model to use for embeddings
- Qdrant collection schema and configuration details
- Rate limits for Cohere API calls
- Optimal text chunk size for embeddings
- Website structure of target URL for scraping

## Constitution Check

### Code Quality Standards
- Follow Python PEP 8 style guidelines
- Include type hints for function parameters and return values
- Write comprehensive docstrings for all functions
- Use meaningful variable and function names

### Security Considerations
- Store API keys in environment variables, not in code
- Validate and sanitize all inputs from web scraping
- Implement proper error handling for API calls

### Performance Requirements
- Efficient web scraping to avoid overwhelming target server
- Proper handling of API rate limits
- Memory-efficient processing of large documents

### Architecture Principles
- Single-file implementation as requested (main.py)
- Clear separation of concerns within functions
- Configurable parameters for flexibility

## Phase 0: Research & Unknown Resolution

### Research Tasks

#### 1. Cohere Embedding Model Selection
- Decision: Use Cohere's embed-multilingual-v3.0 model for documentation content
- Rationale: Good for technical documentation, supports multiple languages
- Alternatives considered: embed-english-v3.0 (limited to English)

#### 2. Text Chunking Strategy
- Decision: Use 512-token chunks with 50-token overlap
- Rationale: Balances context preservation with embedding efficiency
- Alternatives considered: Fixed character length chunks, sentence-based chunks

#### 3. Qdrant Collection Schema
- Decision: Create collection with 1024-dimension vectors (for Cohere embeddings)
- Rationale: Cohere's embed-multilingual-v3.0 produces 1024-dim vectors
- Configuration: Enable indexing for efficient similarity search

#### 4. Web Scraping Approach
- Decision: Use requests + BeautifulSoup for HTML parsing
- Rationale: Reliable, well-documented approach for static content
- Alternatives considered: Selenium (for JS-rendered content - not needed for Docusaurus)

#### 5. Rate Limiting Strategy
- Decision: Implement exponential backoff for Cohere API calls
- Rationale: Prevents hitting rate limits while maintaining throughput
- Configuration: Start with 1s delay, max 60s, with jitter

## Phase 1: Design & Contracts

### Data Model

#### Document Chunk Entity
- `id` (string): Unique identifier for the chunk
- `content` (string): Cleaned text content
- `source_url` (string): Original URL where content was found
- `timestamp` (datetime): When chunk was processed
- `embedding` (list[float]): Vector representation of content (1024 dimensions)
- `metadata` (dict): Additional information (section, page title, etc.)

#### Processing Configuration
- `target_url` (string): Root URL to crawl
- `max_depth` (int): Maximum recursion depth for links
- `chunk_size` (int): Maximum size of text chunks
- `overlap_size` (int): Overlap between chunks to preserve context
- `cohere_model` (string): Model name for embeddings
- `qdrant_collection` (string): Name of Qdrant collection

### API Contracts (Internal Functions)

#### Function: get_all_URLs
- **Input**: `base_url` (string), `max_depth` (int)
- **Output**: `urls` (list[string])
- **Purpose**: Discover all accessible URLs from the base URL up to specified depth
- **Error Handling**: Returns empty list if base URL is inaccessible

#### Function: extract_text_from_URLs
- **Input**: `urls` (list[string])
- **Output**: `documents` (list[dict]) with keys: url, content, metadata
- **Purpose**: Extract clean text content from each URL
- **Error Handling**: Skips URLs that return errors, logs failures

#### Function: create_collection
- **Input**: `collection_name` (string), `vector_size` (int)
- **Output**: Success/failure status
- **Purpose**: Create Qdrant collection with appropriate schema
- **Error Handling**: Check if collection exists, handle creation errors

#### Function: save_chunk_to_qdrant
- **Input**: `chunk_data` (dict) with content, embedding, metadata
- **Output**: Success/failure status
- **Purpose**: Store a single chunk with its embedding in Qdrant
- **Error Handling**: Retry logic for failed insertions

#### Function: main
- **Input**: Configuration parameters
- **Output**: Execution summary
- **Purpose**: Execute the complete pipeline end-to-end
- **Error Handling**: Comprehensive error handling and logging

### Quickstart Guide

1. Install dependencies: `uv pip install cohere qdrant-client requests beautifulsoup4 python-dotenv`
2. Set environment variables: `COHERE_API_KEY` and `QDRANT_URL`
3. Run: `python main.py`
4. Monitor progress and check Qdrant for stored embeddings

## Phase 2: Implementation Tasks

### Task 1: Project Setup
- Create backend directory
- Initialize with UV package manager
- Install required dependencies

### Task 2: Configuration Management
- Create environment variable handling
- Set up configuration class/object

### Task 3: URL Discovery
- Implement `get_all_URLs` function
- Handle crawling with appropriate depth

### Task 4: Text Extraction
- Implement `extract_text_from_URLs` function
- Clean HTML and extract meaningful content

### Task 5: Qdrant Integration
- Implement `create_collection` function
- Set up Qdrant client connection

### Task 6: Embedding Generation
- Integrate Cohere API
- Generate embeddings for text chunks

### Task 7: Storage Implementation
- Implement `save_chunk_to_qdrant` function
- Handle metadata storage with embeddings

### Task 8: Main Pipeline
- Implement `main` function
- Orchestrate the complete pipeline

## Re-evaluated Constitution Check Post-Design

### Code Quality
- All functions will have clear docstrings
- Type hints will be included
- Error handling is built into each component

### Security
- API keys handled via environment variables
- Input validation included in text processing

### Performance
- Rate limiting implemented for API calls
- Memory-efficient processing strategies

### Architecture
- Single-file implementation as requested
- Clear function separation
- Configurable parameters