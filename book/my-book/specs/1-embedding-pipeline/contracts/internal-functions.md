# Internal Function Contracts: Embedding Pipeline

## Function: get_all_URLs

### Signature
```python
def get_all_URLs(base_url: str, max_depth: int = 2) -> List[str]
```

### Purpose
Discover and collect all accessible URLs from the base URL up to the specified depth.

### Parameters
- **base_url** (string, required): The root URL to start crawling from
  - Format: Valid URL string (e.g., "https://example.com")
  - Validation: Must be a reachable URL
- **max_depth** (int, optional): Maximum recursion depth for internal links
  - Default: 2
  - Range: 1 to 5
  - Validation: Must be positive integer

### Return Value
- **Type**: List of strings
- **Content**: Array of discovered URLs
- **Format**: ["https://example.com/page1", "https://example.com/page2", ...]
- **Validation**: Each URL must be valid and accessible

### Behavior
- Performs breadth-first crawling up to max_depth
- Respects robots.txt and rate limiting
- Filters out external links (same domain only)
- Handles HTTP errors gracefully

### Error Handling
- Returns empty list if base_url is inaccessible
- Logs errors for individual URLs that fail
- Continues processing even if some URLs fail

### Example
```python
urls = get_all_URLs("https://hackathon-book-sigma.vercel.app/", max_depth=2)
# Returns: ["https://hackathon-book-sigma.vercel.app/", "https://hackathon-book-sigma.vercel.app/module1", ...]
```

---

## Function: extract_text_from_URLs

### Signature
```python
def extract_text_from_URLs(urls: List[str]) -> List[Dict[str, Any]]
```

### Purpose
Extract clean text content from each URL in the provided list.

### Parameters
- **urls** (list of strings, required): List of URLs to extract text from
  - Format: Array of valid URL strings
  - Validation: Each URL must be reachable

### Return Value
- **Type**: List of dictionaries
- **Content**: Each dictionary contains:
  - url (string): The source URL
  - content (string): Cleaned text content
  - metadata (dict): Additional information including:
    - page_title (string): Title of the page
    - section_title (string): Current section (if applicable)
- **Validation**: Content must be non-empty after cleaning

### Behavior
- Removes HTML tags, navigation elements, headers, footers
- Preserves semantic structure (headings, paragraphs)
- Handles various HTML structures gracefully
- Skips URLs that return errors

### Error Handling
- Logs failures for individual URLs
- Continues processing remaining URLs
- Returns partial results if some URLs fail

### Example
```python
documents = extract_text_from_URLs(["https://example.com/page1"])
# Returns: [{"url": "https://example.com/page1", "content": "Cleaned text...", "metadata": {...}}, ...]
```

---

## Function: create_collection

### Signature
```python
def create_collection(collection_name: str, vector_size: int = 1024) -> bool
```

### Purpose
Create a Qdrant collection with the specified name and vector size.

### Parameters
- **collection_name** (string, required): Name for the Qdrant collection
  - Format: Valid Qdrant collection name
  - Validation: Must follow Qdrant naming conventions
- **vector_size** (int, optional): Dimension of vectors to be stored
  - Default: 1024 (for Cohere embeddings)
  - Validation: Must be positive integer

### Return Value
- **Type**: Boolean
- **True**: Collection created successfully
- **False**: Collection creation failed
- **Validation**: Checks if collection already exists

### Behavior
- Checks if collection already exists
- Creates new collection with cosine distance metric
- Configures indexing for efficient search
- Sets up payload schema for metadata storage

### Error Handling
- Returns False if collection already exists
- Logs specific error details
- Validates Qdrant connection before attempting creation

### Example
```python
success = create_collection("rag_embedding", 1024)
# Returns: True if successful
```

---

## Function: save_chunk_to_qdrant

### Signature
```python
def save_chunk_to_qdrant(chunk_data: Dict[str, Any]) -> bool
```

### Purpose
Store a single text chunk with its embedding and metadata in Qdrant.

### Parameters
- **chunk_data** (dict, required): Data to store
  - Required keys:
    - id (string): Unique identifier for the chunk
    - content (string): Text content
    - embedding (list[float]): Vector representation (1024 elements for Cohere)
    - source_url (string): Original URL
    - timestamp (string): ISO datetime
  - Optional keys:
    - metadata (dict): Additional information

### Return Value
- **Type**: Boolean
- **True**: Chunk saved successfully
- **False**: Save operation failed
- **Validation**: Embedding must match expected dimensions

### Behavior
- Validates embedding dimensions (1024 for Cohere)
- Stores content as payload with metadata
- Uses provided ID for the vector record
- Implements retry logic for transient failures

### Error Handling
- Returns False if embedding dimensions don't match
- Logs detailed error information
- Implements exponential backoff for retries

### Example
```python
chunk = {
    "id": "chunk-123",
    "content": "Sample text content...",
    "embedding": [0.1, 0.2, ..., 0.5],  # 1024 elements
    "source_url": "https://example.com/page",
    "timestamp": "2025-12-10T10:00:00Z",
    "metadata": {"section_title": "Introduction"}
}
success = save_chunk_to_qdrant(chunk)
# Returns: True if successful
```

---

## Function: main

### Signature
```python
def main(config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]
```

### Purpose
Execute the complete embedding pipeline end-to-end.

### Parameters
- **config** (dict, optional): Configuration parameters
  - Keys:
    - target_url (string): Root URL to crawl (default: "https://hackathon-book-sigma.vercel.app/")
    - max_depth (int): Crawling depth (default: 2)
    - chunk_size (int): Text chunk size (default: 512)
    - overlap_size (int): Chunk overlap (default: 50)
    - cohere_model (string): Embedding model (default: "embed-multilingual-v3.0")
    - qdrant_collection (string): Collection name (default: "rag_embedding")
    - batch_size (int): Processing batch size (default: 10)

### Return Value
- **Type**: Dictionary with execution summary
- **Keys**:
  - status (string): "success", "partial", or "failed"
  - processed_count (int): Number of chunks successfully processed
  - failed_count (int): Number of chunks that failed
  - start_time (string): ISO datetime when processing started
  - end_time (string): ISO datetime when processing ended
  - error_details (list): Details about any errors that occurred

### Behavior
- Orchestrates the complete pipeline:
  1. Discovers URLs from target site
  2. Extracts and cleans text content
  3. Chunks the text appropriately
  4. Generates embeddings using Cohere
  5. Stores results in Qdrant
- Implements comprehensive error handling
- Provides progress feedback during execution

### Error Handling
- Catches and logs errors at each stage
- Returns partial results if pipeline fails partway through
- Provides detailed error information in return value

### Example
```python
result = main({
    "target_url": "https://hackathon-book-sigma.vercel.app/",
    "max_depth": 2,
    "qdrant_collection": "rag_embedding"
})
# Returns: {"status": "success", "processed_count": 45, "failed_count": 0, ...}
```