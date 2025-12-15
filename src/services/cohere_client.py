import cohere
import time
import asyncio
from typing import List, Dict, Any
from ..config.settings import settings


class CohereClient:
    def __init__(self):
        self.client = cohere.Client(api_key=settings.COHERE_API_KEY)
        self.last_request_time = 0
        self.rate_limit_delay = 1.0  # Minimum delay between requests in seconds to stay within rate limits

    def _rate_limit(self):
        """
        Simple rate limiting to avoid exceeding API limits
        """
        current_time = time.time()
        time_since_last = current_time - self.last_request_time
        if time_since_last < self.rate_limit_delay:
            time.sleep(self.rate_limit_delay - time_since_last)
        self.last_request_time = time.time()

    def generate_embeddings(self, texts: List[str], model: str = "embed-multilingual-v3.0") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        self._rate_limit()
        
        response = self.client.embed(
            texts=texts,
            model=model,
            input_type="search_document"  # Using search_document for document chunks
        )
        
        return [embedding for embedding in response.embeddings]

    def generate_response(self, prompt: str, model: str = "command-r-08-2024") -> str:
        """
        Generate a response to a prompt using Cohere Chat API
        """
        self._rate_limit()

        response = self.client.chat(
            message=prompt,
            model=model,
            max_tokens=300,
            temperature=0.3,
        )

        return response.text.strip()

    def rerank(self, query: str, documents: List[str], top_n: int = 5) -> List[Dict[str, Any]]:
        """
        Rerank documents based on relevance to query
        """
        self._rate_limit()

        response = self.client.rerank(
            model="rerank-english-v3.0",  # Updated model
            query=query,
            documents=documents,
            top_n=top_n,
        )

        # Handle both possible response formats
        if hasattr(response, 'results'):
            # New format with .results attribute
            rerank_results = response.results
        else:
            # Old format where response is the list directly
            rerank_results = response

        return [
            {
                "index": rank.index,
                "relevance_score": rank.relevance_score,
                "document": rank.document["text"] if isinstance(rank.document, dict) else rank.document
            }
            for rank in rerank_results
        ]


# Global instance
cohere_client = CohereClient()