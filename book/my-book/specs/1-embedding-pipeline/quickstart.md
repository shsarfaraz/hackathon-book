# Quickstart Guide: Embedding Pipeline

## Prerequisites

- Python 3.8 or higher
- UV package manager
- Access to Cohere API (valid API key)
- Access to Qdrant vector database (local or remote)

## Setup

### 1. Create Backend Directory
```bash
mkdir backend
cd backend
```

### 2. Initialize Project with UV
```bash
# Create virtual environment
uv venv

# Activate virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies
```bash
uv pip install cohere qdrant-client requests beautifulsoup4 python-dotenv
```

### 4. Set Up Environment Variables
Create a `.env` file in the backend directory:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # e.g., http://localhost:6333 or https://your-cluster.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key_if_required
```

## Implementation

Create a `main.py` file with the following structure:

```python
import os
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from urllib.parse import urljoin, urlparse
from dotenv import load_dotenv
import time
import logging
from typing import List, Dict, Any, Optional
import uuid
from datetime import datetime

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_all_urls(base_url: str, max_depth: int = 2) -> List[str]:
    """
    Discover and collect all accessible URLs from the base URL up to the specified depth.
    """
    # Implementation here
    pass

def extract_text_from_urls(urls: List[str]) -> List[Dict[str, Any]]:
    """
    Extract clean text content from each URL in the provided list.
    """
    # Implementation here
    pass

def create_collection(collection_name: str, vector_size: int = 1024) -> bool:
    """
    Create a Qdrant collection with the specified name and vector size.
    """
    # Implementation here
    pass

def save_chunk_to_qdrant(chunk_data: Dict[str, Any]) -> bool:
    """
    Store a single text chunk with its embedding and metadata in Qdrant.
    """
    # Implementation here
    pass

def main(config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Execute the complete embedding pipeline end-to-end.
    """
    # Implementation here
    pass

if __name__ == "__main__":
    result = main()
    print(result)
```

## Running the Pipeline

### 1. Execute the Pipeline
```bash
cd backend
python main.py
```

### 2. Monitor Progress
The pipeline will output progress information as it:
- Discovers URLs from the target site
- Extracts and cleans text content
- Generates embeddings
- Stores results in Qdrant

### 3. Check Results
After completion, verify that embeddings are stored in your Qdrant collection named `rag_embedding`.

## Configuration Options

The pipeline can be customized by modifying the configuration passed to the `main` function:

```python
config = {
    "target_url": "https://hackathon-book-sigma.vercel.app/",
    "max_depth": 2,
    "chunk_size": 512,
    "overlap_size": 50,
    "cohere_model": "embed-multilingual-v3.0",
    "qdrant_collection": "rag_embedding",
    "batch_size": 10
}

result = main(config)
```

## Troubleshooting

### Common Issues

1. **API Key Errors**: Verify that your Cohere and Qdrant API keys are correct and have the necessary permissions.

2. **Connection Issues**: Check that your Qdrant instance is accessible and that the URL is correct.

3. **Rate Limits**: If you encounter rate limit errors, the pipeline implements exponential backoff, but you may need to adjust your Cohere plan for higher limits.

4. **Crawling Errors**: If specific URLs are failing, check if they require authentication or have special access requirements.

### Logging
The pipeline logs detailed information at each stage. Check the console output for progress and error information.