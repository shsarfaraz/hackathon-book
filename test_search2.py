import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.config.vector_db import qdrant_client
from src.services.embedding import EmbeddingService
import cohere

def test_search_with_embeddings():
    print("Testing search with embeddings...")

    # Initialize Cohere client directly to generate an embedding
    import os
    from dotenv import load_dotenv
    load_dotenv()

    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        print("COHERE_API_KEY not found in environment")
        return

    co = cohere.Client(cohere_api_key)

    # Generate embedding for test query
    response = co.embed(
        texts=["intro"],
        model="embed-multilingual-v3.0",
        input_type="search_query"
    )
    query_embedding = response.embeddings[0]

    print(f"Generated embedding for 'intro': {len(query_embedding)} dimensions")

    # Perform search using the correct Qdrant API
    try:
        search_results = qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=("content", query_embedding),  # Using named vector
            limit=5,
            with_payload=True
        )

        print(f"Search results: {len(search_results)} items found")
        for i, result in enumerate(search_results):
            print(f"Result {i+1}:")
            print(f"  ID: {result.id}")
            print(f"  Score: {result.score}")
            print(f"  Content preview: {result.payload['content'][:100]}...")
            print(f"  Document ID: {result.payload['document_id']}")
            print(f"  Source URL: {result.payload['source_url']}")
            print()
    except Exception as e:
        print(f"Search error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_search_with_embeddings()