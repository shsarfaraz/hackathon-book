# Embedding Pipeline for Docusaurus Websites

This project implements a complete pipeline that crawls Docusaurus websites, extracts and cleans text content, chunks it, generates embeddings using Cohere, and stores vectors with metadata in Qdrant for RAG-based retrieval.

## Features

- **URL Discovery**: Crawls Docusaurus websites with configurable depth
- **Text Extraction**: Extracts clean text content while preserving semantic structure
- **Content Chunking**: Splits text into configurable chunks with overlap
- **Embedding Generation**: Uses Cohere's multilingual model for embeddings
- **Vector Storage**: Stores embeddings and metadata in Qdrant vector database
- **Error Handling**: Comprehensive error handling and retry mechanisms
- **Progress Reporting**: Detailed progress reporting during pipeline execution

## Prerequisites

- Python 3.8+
- UV package manager
- Access to Cohere API
- Access to Qdrant vector database

## Setup

1. Create a virtual environment and install dependencies:
   ```bash
   cd backend
   uv venv
   source .venv/Scripts/activate  # On Windows: .venv\Scripts\activate
   uv pip install cohere qdrant-client requests beautifulsoup4 python-dotenv
   ```

2. Set up your environment variables by copying the template:
   ```bash
   cp .env .env.local
   ```
   Then edit `.env.local` with your API keys and configuration.

## Configuration

The pipeline can be configured using environment variables in the `.env` file or command-line arguments:

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant instance URL
- `QDRANT_API_KEY`: Your Qdrant API key (if required)
- `TARGET_URL`: The website to crawl (default: https://hackathon-book-sigma.vercel.app/)
- `MAX_DEPTH`: Maximum crawling depth (default: 2)
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `OVERLAP_SIZE`: Overlap size between chunks (default: 50)
- `QDRANT_COLLECTION`: Qdrant collection name (default: rag_embedding)

## Usage

### Command Line Interface

Run the pipeline with default settings:
```bash
python main.py
```

Run with custom parameters:
```bash
python main.py --url https://example.com --max-depth 3 --chunk-size 1024 --verbose
```

Available command-line options:
- `--url`: Target URL to crawl
- `--max-depth`: Maximum crawling depth
- `--chunk-size`: Size of text chunks
- `--overlap-size`: Overlap size between chunks
- `--collection`: Qdrant collection name
- `--verbose`: Enable verbose logging

### As a Module

You can also use the pipeline programmatically:

```python
from main import main

config = {
    'target_url': 'https://hackathon-book-sigma.vercel.app/',
    'max_depth': 2,
    'chunk_size': 512,
    'overlap_size': 50,
    'qdrant_collection': 'rag_embedding'
}

result = main(config)
print(result)
```

## Pipeline Stages

1. **URL Discovery**: Discovers all accessible URLs from the target website
2. **Text Extraction**: Extracts and cleans text content from each URL
3. **Content Chunking**: Splits content into appropriately sized chunks
4. **Embedding Generation**: Generates vector embeddings using Cohere
5. **Vector Storage**: Stores embeddings and metadata in Qdrant

## Architecture

The pipeline follows a modular architecture with the following key functions:

- `get_all_URLs()`: Discovers URLs to crawl
- `extract_text_from_URLs()`: Extracts clean text content
- `chunk_text()`: Splits content into chunks
- `generate_embeddings_for_chunks()`: Creates embeddings using Cohere
- `save_chunk_to_qdrant()`: Stores embeddings in Qdrant
- `main()`: Orchestrates the complete pipeline

## Error Handling

The pipeline includes comprehensive error handling:

- Graceful degradation when individual URLs fail
- Rate limiting and exponential backoff for API calls
- Validation of embedding dimensions
- Detailed logging for debugging

## Performance Considerations

- Batching requests to Cohere API for efficiency
- Respectful crawling with rate limiting
- Memory-efficient processing of large documents
- Configurable batch sizes for optimal performance

## Security Considerations

- API keys stored in environment variables
- Input validation for all user-provided parameters
- Sanitized content extraction from web pages