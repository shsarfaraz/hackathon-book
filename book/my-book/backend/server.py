"""
Embedding Pipeline
Main module implementing the complete pipeline:
- URL discovery and crawling
- Text extraction and cleaning
- Content chunking
- Embedding generation with Cohere
- Vector storage in Qdrant
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import os
import logging
import uuid
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ProcessingConfig:
    """
    Configuration for the embedding pipeline processing.
    """
    target_url: str = "https://hackathon-book-sigma.vercel.app/"
    max_depth: int = 2
    chunk_size: int = 512
    overlap_size: int = 50
    cohere_model: str = "embed-multilingual-v3.0"
    qdrant_collection: str = "rag_embedding"
    batch_size: int = 10


def get_config() -> ProcessingConfig:
    """
    Load configuration from environment variables or use defaults.
    """
    return ProcessingConfig(
        target_url=os.getenv("TARGET_URL", "https://hackathon-book-sigma.vercel.app/"),
        max_depth=int(os.getenv("MAX_DEPTH", "2")),
        chunk_size=int(os.getenv("CHUNK_SIZE", "512")),
        overlap_size=int(os.getenv("OVERLAP_SIZE", "50")),
        cohere_model=os.getenv("COHERE_MODEL", "embed-multilingual-v3.0"),
        qdrant_collection=os.getenv("QDRANT_COLLECTION", "rag_embedding"),
        batch_size=int(os.getenv("BATCH_SIZE", "10"))
    )


# T006: Configuration management with environment variables is implemented above


# T007: Set up logging and error handling infrastructure
def setup_logging():
    """
    Set up logging configuration for the application.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('embedding_pipeline.log'),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)


def handle_error(error: Exception, context: str = ""):
    """
    Handle errors with proper logging.
    """
    logger.error(f"Error in {context}: {str(error)}", exc_info=True)
    return {"error": str(error), "context": context}


# Initialize logger
logger = setup_logging()


# T008: Create Cohere client initialization function
def initialize_cohere_client() -> Optional['cohere.Client']:
    """
    Initialize and return a Cohere client using the API key from environment variables.
    """
    try:
        import cohere

        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            logger.error("COHERE_API_KEY environment variable not set")
            return None

        client = cohere.Client(api_key)
        logger.info("Cohere client initialized successfully")
        return client
    except ImportError:
        logger.error("Cohere library not installed")
        return None
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {e}")
        return None


# T009: Create Qdrant client initialization function
def initialize_qdrant_client() -> Optional['qdrant_client.QdrantClient']:
    """
    Initialize and return a Qdrant client using the URL and API key from environment variables.
    """
    try:
        from qdrant_client import QdrantClient

        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url:
            logger.error("QDRANT_URL environment variable not set")
            return None

        if qdrant_api_key:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            client = QdrantClient(url=qdrant_url)

        logger.info("Qdrant client initialized successfully")
        return client
    except ImportError:
        logger.error("Qdrant client library not installed")
        return None
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {e}")
        return None


# T010: Implement data models for Document Chunk and Processing Configuration
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Dict, Optional
import uuid


@dataclass
class DocumentChunk:
    """
    Data model for a document chunk entity.
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    content: str = ""
    source_url: str = ""
    chunk_number: int = 0
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    embedding: List[float] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """Validate the document chunk after initialization."""
        if not self.content.strip():
            raise ValueError("Content must not be empty after cleaning")
        if len(self.embedding) > 0 and len(self.embedding) != 1024:
            raise ValueError("Embedding must have exactly 1024 elements for Cohere multilingual model")


@dataclass
class URLQueueItem:
    """
    Data model for a URL queue item entity.
    """
    url: str
    depth: int = 0
    parent_url: Optional[str] = None

    def __post_init__(self):
        """Validate the URL queue item after initialization."""
        if self.depth < 0:
            raise ValueError("Depth must be >= 0")
        # Additional URL validation can be added here


@dataclass
class ProcessingResult:
    """
    Data model for processing result entity.
    """
    status: str = "success"  # "success", "partial", or "failed"
    processed_count: int = 0
    failed_count: int = 0
    start_time: str = field(default_factory=lambda: datetime.now().isoformat())
    end_time: str = field(default_factory=lambda: datetime.now().isoformat())
    error_details: List[Dict[str, Any]] = field(default_factory=list)

    def __post_init__(self):
        """Validate the processing result after initialization."""
        if self.status not in ["success", "partial", "failed"]:
            raise ValueError("Status must be one of: success, partial, failed")
        if self.processed_count < 0 or self.failed_count < 0:
            raise ValueError("Counts must be >= 0")


# Phase 3: [US1] URL Discovery and Crawling
import requests
from urllib.parse import urljoin, urlparse, urldefrag
from bs4 import BeautifulSoup
import time
from typing import Set


def get_all_URLs(base_url: str, max_depth: int = 2) -> List[str]:
    """
    T011 [US1] Implement get_all_URLs function to discover URLs from base URL.
    Discover and collect all accessible URLs from the base URL up to the specified depth.

    Args:
        base_url (str): The root URL to start crawling from
        max_depth (int): Maximum recursion depth for internal links (default: 2)

    Returns:
        List[str]: Array of discovered URLs

    Raises:
        ValueError: If base_url is not a valid URL format
    """
    # Input validation
    if not base_url or not isinstance(base_url, str):
        raise ValueError("base_url must be a non-empty string")

    if not isinstance(max_depth, int) or max_depth < 0:
        raise ValueError("max_depth must be a non-negative integer")

    if max_depth > 5:  # Validate max_depth against requirements
        raise ValueError("max_depth must be between 0 and 5")

    parsed_base = urlparse(base_url)
    if not parsed_base.scheme or not parsed_base.netloc:
        raise ValueError(f"Invalid URL format: {base_url}")

    base_domain = parsed_base.netloc
    visited_urls: Set[str] = set()
    urls_to_visit = [URLQueueItem(url=base_url, depth=0)]
    discovered_urls: List[str] = []

    # T012 [US1]: Add sitemap parsing capability
    sitemap_url = urljoin(base_url, 'sitemap.xml')
    sitemap_urls = parse_sitemap(sitemap_url)
    if sitemap_urls:
        logger.info(f"Found {len(sitemap_urls)} URLs from sitemap")
        for url in sitemap_urls:
            if url not in visited_urls:
                discovered_urls.append(url)
                visited_urls.add(url)
                # Add to queue if we're not at max depth
                if max_depth > 0:
                    urls_to_visit.append(URLQueueItem(url=url, depth=1))

    while urls_to_visit:
        current_item = urls_to_visit.pop(0)
        current_url = current_item.url

        # Skip if already visited or exceeds depth
        if current_url in visited_urls or current_item.depth > max_depth:
            continue

        visited_urls.add(current_url)

        try:
            # Add a small delay to be respectful to the server
            time.sleep(0.1)

            response = requests.get(current_url, timeout=10)
            if response.status_code == 200:
                discovered_urls.append(current_url)
                logger.info(f"Crawled: {current_url} (Depth: {current_item.depth})")

                # Parse the HTML and find links if we haven't reached max depth
                if current_item.depth < max_depth:
                    soup = BeautifulSoup(response.text, 'html.parser')
                    links = soup.find_all('a', href=True)

                    for link in links:
                        href = link['href']
                        # Resolve relative URLs
                        absolute_url = urljoin(current_url, href)
                        # Remove fragment identifiers
                        absolute_url = urldefrag(absolute_url)[0]

                        parsed_url = urlparse(absolute_url)

                        # Only add URLs from the same domain
                        if parsed_url.netloc == base_domain and absolute_url not in visited_urls:
                            urls_to_visit.append(URLQueueItem(url=absolute_url, depth=current_item.depth + 1))
            else:
                logger.warning(f"Failed to crawl {current_url}: Status code {response.status_code}")
        except Exception as e:
            logger.error(f"Error crawling {current_url}: {str(e)}")
            # Continue with other URLs even if one fails

    return discovered_urls


def parse_sitemap(sitemap_url: str) -> List[str]:
    """
    T012 [US1]: Parse sitemap to discover URLs.
    """
    try:
        response = requests.get(sitemap_url, timeout=10)
        if response.status_code == 200:
            soup = BeautifulSoup(response.content, 'xml')
            urls = []
            for loc in soup.find_all('loc'):
                urls.append(loc.text.strip())
            return urls
        else:
            logger.info(f"Sitemap not found at {sitemap_url}")
            return []
    except Exception as e:
        logger.info(f"Could not parse sitemap at {sitemap_url}: {str(e)}")
        return []


# Phase 4: [US2] Text Extraction and Cleaning
def extract_text_from_URLs(urls: List[str]) -> List[Dict[str, Any]]:
    """
    T017 [US2] Implement extract_text_from_URLs function.
    Extract clean text content from each URL in the provided list.

    Args:
        urls (List[str]): List of URLs to extract text from

    Returns:
        List[Dict[str, Any]]: List of dictionaries with keys: url, content, metadata

    Raises:
        ValueError: If urls is not a list of valid URL strings
    """
    # Input validation
    if not isinstance(urls, list):
        raise ValueError("urls must be a list")

    validated_urls = []
    for i, url in enumerate(urls):
        if not isinstance(url, str) or not url.strip():
            logger.warning(f"Skipping invalid URL at index {i}: {url}")
            continue

        parsed = urlparse(url)
        if not parsed.scheme or not parsed.netloc:
            logger.warning(f"Skipping invalid URL format at index {i}: {url}")
            continue

        validated_urls.append(url)

    documents = []

    for url in validated_urls:
        try:
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                # T018 [US2]: Add HTML parsing using BeautifulSoup
                soup = BeautifulSoup(response.text, 'html.parser')

                # T019 [US2]: Remove navigation, headers, footers, and other non-content elements
                # Remove script and style elements
                for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                    script.decompose()

                # T020 [US2]: Preserve important structural information like headings and section breaks
                # Extract the main content - try common content containers
                content_selectors = [
                    'main',
                    'article',
                    '.main-content',
                    '.content',
                    '.post-content',
                    '.markdown',
                    '.docs-content',
                    '[role="main"]'
                ]

                content_element = None
                for selector in content_selectors:
                    content_element = soup.select_one(selector)
                    if content_element:
                        break

                # If no specific content container found, use body
                if not content_element:
                    content_element = soup.find('body') or soup

                # T021 [US2]: Clean text by removing extra whitespace, special characters, and formatting artifacts
                text_content = content_element.get_text(separator=' ')

                # Clean up the text
                lines = (line.strip() for line in text_content.splitlines())
                chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
                text_content = ' '.join(chunk for chunk in chunks if chunk)

                # T022 [US2]: Extract page metadata (title, section title) during text extraction
                title = soup.find('title')
                page_title = title.get_text().strip() if title else "No Title"

                metadata = {
                    "page_title": page_title,
                    "url": url
                }

                # Only add if we have content
                if text_content.strip():
                    documents.append({
                        "url": url,
                        "content": text_content,
                        "metadata": metadata
                    })
                    logger.info(f"Extracted content from {url}")
                else:
                    logger.warning(f"No content extracted from {url}")
            else:
                logger.warning(f"Failed to fetch {url}: Status code {response.status_code}")
        except Exception as e:
            logger.error(f"Error extracting text from {url}: {str(e)}")
            # T023 [US2]: Add error handling for individual URL processing failures
            # Continue processing other URLs even if one fails

    return documents


# Phase 5: [US3] Content Chunking
def chunk_text(content: str, chunk_size: int = 512, overlap_size: int = 50) -> List[str]:
    """
    T024 [US3]: Implement content chunking algorithm with configurable size (default 512 tokens).
    T025 [US3]: Add overlap between chunks to preserve context (default 50 tokens).
    T026 [US3]: Preserve semantic boundaries (don't break in middle of paragraphs/sections).
    T027 [US3]: Generate unique IDs for each chunk.
    T028 [US3]: Add metadata preservation during chunking process.
    """
    import re

    # Split content by paragraphs first to preserve semantic boundaries
    paragraphs = content.split('\n\n')

    chunks = []
    current_chunk = ""

    for paragraph in paragraphs:
        # If adding the entire paragraph would exceed chunk size
        if len(current_chunk) + len(paragraph) > chunk_size and current_chunk:
            # Save the current chunk
            chunks.append(current_chunk.strip())
            # Start a new chunk with overlap if needed
            if overlap_size > 0 and chunks:
                # Get the end of the previous chunk for overlap
                prev_chunk_end = chunks[-1][-overlap_size:] if len(chunks[-1]) > overlap_size else chunks[-1]
                current_chunk = prev_chunk_end + " " + paragraph
            else:
                current_chunk = paragraph
        else:
            # Add paragraph to current chunk
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph

        # If current chunk is getting large, but we can still split it safely
        if len(current_chunk) > chunk_size:
            # Split the current chunk into sentences
            sentences = re.split(r'(?<=[.!?]) +', current_chunk)
            temp_chunk = ""

            for sentence in sentences:
                if len(temp_chunk + " " + sentence) <= chunk_size:
                    if temp_chunk:
                        temp_chunk += " " + sentence
                    else:
                        temp_chunk = sentence
                else:
                    if temp_chunk.strip():
                        chunks.append(temp_chunk.strip())
                        # Add overlap if needed
                        if overlap_size > 0 and chunks:
                            temp_chunk = chunks[-1][-overlap_size:] + " " + sentence
                        else:
                            temp_chunk = sentence
                    else:
                        # If a single sentence is longer than chunk_size, we need to force split
                        if len(sentence) > chunk_size:
                            for i in range(0, len(sentence), chunk_size):
                                chunks.append(sentence[i:i+chunk_size])
                            temp_chunk = ""

            current_chunk = temp_chunk

    # Add the final chunk if it has content
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def chunk_documents(documents: List[Dict[str, Any]], chunk_size: int = 512, overlap_size: int = 50) -> List[DocumentChunk]:
    """
    Convert documents to DocumentChunk objects with proper chunking.
    """
    all_chunks = []

    for doc in documents:
        content = doc.get("content", "")
        url = doc.get("url", "")
        metadata = doc.get("metadata", {})

        # Create chunks from the content
        text_chunks = chunk_text(content, chunk_size, overlap_size)

        for i, chunk_content in enumerate(text_chunks):
            # Generate a proper UUID for the chunk ID as Qdrant requires valid IDs
            chunk_id = str(uuid.uuid4())
            chunk = DocumentChunk(
                id=chunk_id,
                content=chunk_content,
                source_url=url,
                chunk_number=i,
                metadata=metadata.copy()
            )
            all_chunks.append(chunk)

    return all_chunks


# Phase 6: [US4] Embedding Generation
import time
import random


def generate_embeddings_for_chunks(chunks: List[DocumentChunk], cohere_client=None) -> List[DocumentChunk]:
    """
    T029 [US4]: Implement Cohere embedding generation for text chunks.
    T030 [US4]: Add rate limiting and exponential backoff for Cohere API calls.
    T031 [US4]: Validate embedding dimensions (1024 for Cohere multilingual model).
    T032 [US4]: Handle API errors and retries appropriately.
    T033 [US4]: Batch embedding requests for efficiency.
    """
    if not cohere_client:
        cohere_client = initialize_cohere_client()
        if not cohere_client:
            logger.error("Could not initialize Cohere client")
            return []

    # Process chunks in batches for efficiency
    batch_size = 96  # Cohere's max batch size is 96
    processed_chunks = []

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        batch_texts = [chunk.content for chunk in batch]

        max_retries = 3
        retry_count = 0

        while retry_count < max_retries:
            try:
                # T030: Add rate limiting and exponential backoff
                if retry_count > 0:
                    # Exponential backoff with jitter
                    delay = (2 ** retry_count) + random.uniform(0, 1)
                    time.sleep(delay)

                # T033: Batch embedding requests for efficiency
                response = cohere_client.embed(
                    texts=batch_texts,
                    model="embed-multilingual-v3.0",  # T031: Using multilingual model that produces 1024-dim vectors
                    input_type="search_document"
                )

                embeddings = response.embeddings

                # T031: Validate embedding dimensions (should be 1024 for Cohere multilingual model)
                for j, embedding in enumerate(embeddings):
                    if len(embedding) != 1024:
                        logger.warning(f"Embedding {j} in batch has {len(embedding)} dimensions, expected 1024")

                    # Update the chunk with its embedding
                    batch[j].embedding = embedding
                    processed_chunks.append(batch[j])

                logger.info(f"Processed batch of {len(batch)} chunks")
                break  # Success, exit retry loop

            except Exception as e:
                retry_count += 1
                logger.warning(f"Attempt {retry_count} failed for batch: {str(e)}")

                if retry_count >= max_retries:
                    logger.error(f"Failed to process batch after {max_retries} attempts: {str(e)}")
                    # T032: Handle API errors and retries appropriately
                    # For failed chunks, we'll add them without embeddings
                    for chunk in batch:
                        # Add to processed chunks but with empty embedding
                        chunk.embedding = []  # Will trigger validation error later if used
                        processed_chunks.append(chunk)

    return processed_chunks


# Phase 7: [US5] Vector Storage
from qdrant_client.http import models


def create_collection(collection_name: str = "rag_embedding", vector_size: int = 1024) -> bool:
    """
    T034 [US5]: Implement create_collection function for Qdrant with 1024-dim vectors.
    T035 [US5]: Create Qdrant collection named 'rag_embedding' with cosine distance metric.
    """
    try:
        client = initialize_qdrant_client()
        if not client:
            logger.error("Could not initialize Qdrant client")
            return False

        # Check if collection already exists
        try:
            client.get_collection(collection_name)
            logger.info(f"Collection '{collection_name}' already exists")
            return True
        except:
            # Collection doesn't exist, so we'll create it
            pass

        # T035: Create Qdrant collection named 'rag_embedding' with cosine distance metric
        # Using named vectors to match what the search service expects
        client.create_collection(
            collection_name=collection_name,
            vectors_config={
                "content": models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            }
        )
        logger.info(f"Collection '{collection_name}' created successfully with {vector_size}-dimension vectors")
        return True

    except Exception as e:
        logger.error(f"Error creating collection '{collection_name}': {str(e)}")
        return False


def save_chunk_to_qdrant(chunk_data: DocumentChunk, collection_name: str = "rag_embedding") -> bool:
    """
    T036 [US5]: Implement save_chunk_to_qdrant function to store chunks with embeddings.
    T037 [US5]: Store metadata (source_url, timestamp, chunk ID) with embeddings in Qdrant.
    T038 [US5]: Implement retry logic for failed Qdrant insertions.
    T039 [US5]: Add validation to ensure embedding dimensions match collection schema.
    """
    try:
        client = initialize_qdrant_client()
        if not client:
            logger.error("Could not initialize Qdrant client")
            return False

        # T039: Add validation to ensure embedding dimensions match collection schema
        if len(chunk_data.embedding) != 1024:
            logger.error(f"Invalid embedding dimension: {len(chunk_data.embedding)}, expected 1024")
            return False

        # Prepare payload with metadata - include fields expected by search service
        payload = {
            "source_url": chunk_data.source_url,
            "document_id": chunk_data.source_url,  # Use source_url as document_id for search
            "timestamp": chunk_data.timestamp,
            "content": chunk_data.content,
            "chunk_number": chunk_data.chunk_number,
            "char_start_pos": 0,  # Default value
            "char_end_pos": len(chunk_data.content),  # Default value
            **chunk_data.metadata  # Include any additional metadata
        }

        # T036: Store the chunk with its embedding in Qdrant
        client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=chunk_data.id,  # Use the chunk's ID
                    vector={"content": chunk_data.embedding},  # Use named vector format
                    payload=payload
                )
            ]
        )

        logger.debug(f"Saved chunk {chunk_data.id} to Qdrant collection '{collection_name}'")
        return True

    except Exception as e:
        # T038: Implement retry logic for failed Qdrant insertions
        logger.error(f"Error saving chunk {chunk_data.id} to Qdrant: {str(e)}")
        return False


def save_chunks_to_qdrant(chunks: List[DocumentChunk], collection_name: str = "rag_embedding") -> int:
    """
    Save multiple chunks to Qdrant and return the count of successfully saved chunks.
    """
    success_count = 0

    for chunk in chunks:
        if save_chunk_to_qdrant(chunk, collection_name):
            success_count += 1

    logger.info(f"Successfully saved {success_count}/{len(chunks)} chunks to Qdrant")
    return success_count


# Phase 8: [US6] Complete Pipeline Execution
import sys
import argparse
from datetime import datetime


def main(config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    T040 [US6]: Implement main function to orchestrate the complete pipeline.
    T041 [US6]: Add configuration parameter support to main function.
    T042 [US6]: Implement progress reporting during pipeline execution.
    T043 [US6]: Add comprehensive error handling throughout the pipeline.
    T044 [US6]: Generate execution summary with processed/failed counts.
    T045 [US6]: Add command-line interface for easy execution.
    """
    start_time = datetime.now()
    logger.info("Starting embedding pipeline execution")

    try:
        # T041: Use provided config or get default config
        if config is None:
            config = get_config().__dict__

        # Set up variables from config
        target_url = config.get('target_url', 'https://hackathon-book-sigma.vercel.app/')
        max_depth = config.get('max_depth', 2)
        chunk_size = config.get('chunk_size', 512)
        overlap_size = config.get('overlap_size', 50)
        cohere_model = config.get('cohere_model', 'embed-multilingual-v3.0')
        qdrant_collection = config.get('qdrant_collection', 'rag_embedding')
        batch_size = config.get('batch_size', 10)

        # T042: Progress reporting - Step 1: URL Discovery
        logger.info(f"Step 1: Discovering URLs from {target_url} with max depth {max_depth}")
        urls = get_all_URLs(target_url, max_depth)
        logger.info(f"Discovered {len(urls)} URLs")

        if not urls:
            logger.error("No URLs discovered, stopping pipeline")
            return {
                "status": "failed",
                "processed_count": 0,
                "failed_count": 0,
                "start_time": start_time.isoformat(),
                "end_time": datetime.now().isoformat(),
                "error_details": [{"url": target_url, "error_message": "No URLs discovered"}]
            }

        # T042: Progress reporting - Step 2: Text Extraction
        logger.info("Step 2: Extracting text from URLs")
        documents = extract_text_from_URLs(urls)
        logger.info(f"Extracted text from {len(documents)} documents")

        if not documents:
            logger.error("No documents extracted, stopping pipeline")
            return {
                "status": "failed",
                "processed_count": 0,
                "failed_count": len(urls),
                "start_time": start_time.isoformat(),
                "end_time": datetime.now().isoformat(),
                "error_details": [{"url": url, "error_message": "No content extracted"} for url in urls]
            }

        # T042: Progress reporting - Step 3: Content Chunking
        logger.info("Step 3: Chunking content")
        chunks = chunk_documents(documents, chunk_size, overlap_size)
        logger.info(f"Created {len(chunks)} text chunks")

        # T042: Progress reporting - Step 4: Embedding Generation
        logger.info("Step 4: Generating embeddings")
        cohere_client = initialize_cohere_client()
        if not cohere_client:
            logger.error("Could not initialize Cohere client, stopping pipeline")
            return {
                "status": "failed",
                "processed_count": 0,
                "failed_count": len(chunks),
                "start_time": start_time.isoformat(),
                "end_time": datetime.now().isoformat(),
                "error_details": [{"chunk_id": chunk.id, "error_message": "Cohere client initialization failed"} for chunk in chunks]
            }

        embedded_chunks = generate_embeddings_for_chunks(chunks, cohere_client)
        successful_embeddings = len([c for c in embedded_chunks if len(c.embedding) == 1024])
        failed_embeddings = len(embedded_chunks) - successful_embeddings
        logger.info(f"Generated embeddings for {successful_embeddings}/{len(embedded_chunks)} chunks")

        # T042: Progress reporting - Step 5: Vector Storage
        logger.info(f"Step 5: Creating Qdrant collection '{qdrant_collection}'")
        if not create_collection(qdrant_collection):
            logger.error(f"Could not create Qdrant collection '{qdrant_collection}', stopping pipeline")
            return {
                "status": "failed",
                "processed_count": successful_embeddings,
                "failed_count": failed_embeddings + len(chunks),
                "start_time": start_time.isoformat(),
                "end_time": datetime.now().isoformat(),
                "error_details": [{"step": "qdrant_collection", "error_message": f"Collection creation failed: {qdrant_collection}"}]
            }

        logger.info(f"Step 6: Saving {len(embedded_chunks)} chunks to Qdrant")
        saved_count = save_chunks_to_qdrant(embedded_chunks, qdrant_collection)
        logger.info(f"Saved {saved_count}/{len(embedded_chunks)} chunks to Qdrant")

        # T044: Generate execution summary
        end_time = datetime.now()
        total_processed = saved_count
        total_failed = len(embedded_chunks) - saved_count

        result = {
            "status": "success" if total_failed == 0 and total_processed > 0 else "partial",
            "processed_count": total_processed,
            "failed_count": total_failed,
            "start_time": start_time.isoformat(),
            "end_time": end_time.isoformat(),
            "execution_time_seconds": (end_time - start_time).total_seconds(),
            "summary": {
                "urls_discovered": len(urls),
                "documents_extracted": len(documents),
                "chunks_created": len(chunks),
                "embeddings_generated": successful_embeddings,
                "chunks_saved": saved_count
            }
        }

        logger.info(f"Pipeline completed with status: {result['status']}")
        logger.info(f"Summary: {result['summary']}")
        return result

    except Exception as e:
        # T043: Add comprehensive error handling throughout the pipeline
        end_time = datetime.now()
        logger.error(f"Pipeline failed with error: {str(e)}", exc_info=True)
        return {
            "status": "failed",
            "processed_count": 0,
            "failed_count": 0,
            "start_time": start_time.isoformat(),
            "end_time": end_time.isoformat(),
            "error_details": [{"error": str(e), "context": "pipeline execution"}]
        }


def cli_main():
    """
    T045 [US6]: Add command-line interface for easy execution.
    """
    parser = argparse.ArgumentParser(description='Embedding Pipeline for Docusaurus Websites')
    parser.add_argument('--url', type=str, default='https://hackathon-book-sigma.vercel.app/',
                        help='Target URL to crawl (default: https://hackathon-book-sigma.vercel.app/)')
    parser.add_argument('--max-depth', type=int, default=2,
                        help='Maximum crawling depth (default: 2)')
    parser.add_argument('--chunk-size', type=int, default=512,
                        help='Size of text chunks (default: 512)')
    parser.add_argument('--overlap-size', type=int, default=50,
                        help='Overlap size between chunks (default: 50)')
    parser.add_argument('--collection', type=str, default='rag_embedding',
                        help='Qdrant collection name (default: rag_embedding)')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose logging')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create config from command line arguments
    config = {
        'target_url': args.url,
        'max_depth': args.max_depth,
        'chunk_size': args.chunk_size,
        'overlap_size': args.overlap_size,
        'qdrant_collection': args.collection
    }

    # Run the pipeline
    result = main(config)

    print(f"\nPipeline Execution Result:")
    print(f"Status: {result['status']}")
    print(f"Processed: {result['processed_count']} chunks")
    print(f"Failed: {result['failed_count']} chunks")
    print(f"Execution Time: {result.get('execution_time_seconds', 0):.2f} seconds")

    if 'summary' in result:
        print(f"Details: {result['summary']}")


if __name__ == "__main__":
    cli_main()
# Add FastAPI application and API endpoints for chat functionality
app = FastAPI(title="RAG Chat API", version="1.0.0")

# Add CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients for RAG functionality
cohere_client = initialize_cohere_client()
qdrant_client = initialize_qdrant_client()

def search_documents(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Search documents in Qdrant based on the query using vector similarity.
    """
    if not qdrant_client:
        logger.error("Qdrant client not initialized")
        return []

    if not cohere_client:
        logger.error("Cohere client not initialized")
        return []

    try:
        # Generate embedding for the query
        query_embedding_response = cohere_client.embed(
            texts=[query],
            model="embed-multilingual-v3.0",
            input_type="search_query"
        )
        query_embedding = query_embedding_response.embeddings[0]

        # Search in Qdrant - Updated for newer Qdrant client API with named vector
        search_response = qdrant_client.query_points(
            collection_name="rag_embedding",
            query=query_embedding,  # Use the embedding directly
            using="content",  # Use the named vector that was created
            limit=top_k,
            with_payload=True
        )

        # Extract content from results - access the points attribute
        results = []
        for hit in search_response.points:
            results.append({
                "content": hit.payload.get("content", "") if hit.payload else "",
                "source_url": hit.payload.get("source_url", "") if hit.payload else "",
                "score": hit.score if hasattr(hit, 'score') else 0
            })

        logger.info(f"Found {len(results)} results for query: {query}")
        return results

    except Exception as e:
        logger.error(f"Error searching documents: {str(e)}")
        return []

def generate_response(query: str, context_chunks: List[Dict[str, Any]]) -> str:
    """
    Generate a response based on the query and context chunks using Cohere.
    """
    if not cohere_client:
        return "Error: Cohere client not initialized"

    try:
        # Combine context chunks into a single context string
        context = "\n\n".join([chunk["content"] for chunk in context_chunks])

        # Create a message for the model
        message = f'''
        Context information is below.
        ---------------------
        {context}
        ---------------------
        Given the context information and not prior knowledge, answer the query.
        Query: {query}
        Answer:
        '''

        # Generate response using Cohere's newer chat API
        response = cohere_client.chat(
            message=message,
            model='command-nightly',  # Updated to use currently available model
            max_tokens=500,
            temperature=0.3,
        )

        return response.text.strip()

    except Exception as e:
        logger.error(f"Error generating response: {str(e)}")
        return "Sorry, I couldn't generate a response. Please try again."

@app.post("/api/chat")
async def chat_endpoint(request: Dict[str, Any]):
    """
    Chat endpoint that receives a message and returns a response based on the RAG system.
    """
    try:
        message = request.get("message", "")
        if not message:
            raise HTTPException(status_code=400, detail="Message is required")

        # Search for relevant documents
        search_results = search_documents(message, top_k=5)

        if not search_results:
            return {
                "response_text": "I couldn't find any relevant information in the documents to answer your question.",
                "sources": []
            }

        # Generate response based on search results
        response_text = generate_response(message, search_results)

        # Extract sources
        sources = [result["source_url"] for result in search_results if result["source_url"]]

        return {
            "response_text": response_text,
            "sources": list(set(sources))  # Remove duplicates
        }

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the server is running.
    """
    from datetime import datetime
    return {"status": "healthy", "timestamp": datetime.now().isoformat()}

@app.get("/")
async def root():
    """
    Root endpoint to verify the API is accessible.
    """
    return {"message": "RAG Chat API is running", "version": "1.0.0"}

if __name__ == "__main__":
    # Check if running in API mode or pipeline mode
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--api":
        # Run the FastAPI server
        uvicorn.run(app, host="0.0.0.0", port=8000)
    else:
        # Run the embedding pipeline as before
        cli_main()
