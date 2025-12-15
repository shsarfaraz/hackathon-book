# Data Model: Embedding Pipeline

## Document Chunk Entity

### Attributes
- **id** (string, required)
  - Type: UUID or auto-generated string
  - Purpose: Unique identifier for the chunk
  - Validation: Must be unique within the system

- **content** (string, required)
  - Type: Text (up to 10000 characters)
  - Purpose: Cleaned text content extracted from the source
  - Validation: Must not be empty after cleaning

- **source_url** (string, required)
  - Type: URL string
  - Purpose: Original URL where the content was found
  - Validation: Must be a valid URL format

- **timestamp** (datetime, required)
  - Type: ISO 8601 datetime string
  - Purpose: When the chunk was processed
  - Validation: Must be current time or past

- **embedding** (list[float], required)
  - Type: Array of 1024 floats
  - Purpose: Vector representation of the content
  - Validation: Must have exactly 1024 elements

- **metadata** (dict, optional)
  - Type: Key-value pairs
  - Purpose: Additional information about the document
  - Standard keys:
    - section_title (string): Title of the section this chunk came from
    - page_title (string): Title of the full page
    - crawl_depth (int): How deep in the site structure this was found

### Relationships
- None (standalone entity stored in Qdrant)

### Validation Rules
1. Content must be cleaned of HTML tags and formatting
2. Embedding must match the expected dimension (1024 for Cohere)
3. Source URL must match the target domain
4. Timestamp must be set during processing

## Processing Configuration Entity

### Attributes
- **target_url** (string, required)
  - Type: URL string
  - Purpose: Root URL to start crawling from
  - Validation: Must be a valid URL format

- **max_depth** (int, required)
  - Type: Integer, default: 2
  - Purpose: Maximum recursion depth for internal links
  - Validation: Must be between 1 and 5

- **chunk_size** (int, required)
  - Type: Integer, default: 512
  - Purpose: Maximum size of text chunks in tokens
  - Validation: Must be between 100 and 1000

- **overlap_size** (int, required)
  - Type: Integer, default: 50
  - Purpose: Overlap between chunks to preserve context
  - Validation: Must be less than chunk_size

- **cohere_model** (string, required)
  - Type: String, default: "embed-multilingual-v3.0"
  - Purpose: Model name for embeddings
  - Validation: Must be a valid Cohere model name

- **qdrant_collection** (string, required)
  - Type: String, default: "rag_embedding"
  - Purpose: Name of Qdrant collection to store vectors
  - Validation: Must follow Qdrant collection naming rules

- **batch_size** (int, required)
  - Type: Integer, default: 10
  - Purpose: Number of chunks to process in each batch
  - Validation: Must be between 1 and 100

## URL Queue Item Entity

### Attributes
- **url** (string, required)
  - Type: URL string
  - Purpose: The URL to be processed
  - Validation: Must be a valid URL format

- **depth** (int, required)
  - Type: Integer
  - Purpose: Current depth in the crawling process
  - Validation: Must be >= 0

- **parent_url** (string, optional)
  - Type: URL string
  - Purpose: URL that linked to this one
  - Validation: If provided, must be valid URL

### Validation Rules
- URLs in queue must not exceed max_depth
- Duplicate URLs should be filtered out
- URLs must be from the same domain as target_url

## Processing Result Entity

### Attributes
- **status** (string, required)
  - Type: Enum ["success", "partial", "failed"]
  - Purpose: Overall status of the processing run
  - Validation: Must be one of the allowed values

- **processed_count** (int, required)
  - Type: Integer
  - Purpose: Number of chunks successfully processed
  - Validation: Must be >= 0

- **failed_count** (int, required)
  - Type: Integer
  - Purpose: Number of chunks that failed processing
  - Validation: Must be >= 0

- **start_time** (datetime, required)
  - Type: ISO 8601 datetime string
  - Purpose: When the processing started
  - Validation: Must be current time or past

- **end_time** (datetime, required)
  - Type: ISO 8601 datetime string
  - Purpose: When the processing ended
  - Validation: Must be after start_time

- **error_details** (list[dict], optional)
  - Type: Array of error objects
  - Purpose: Details about any errors that occurred
  - Structure: {url, error_message, timestamp}

### Validation Rules
- processed_count + failed_count should equal total chunks processed
- end_time must be after start_time
- If status is "failed", error_details should not be empty