import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.config.vector_db import qdrant_client
from src.services.embedding import EmbeddingService
import cohere

def test_new_search_api():
    print("Testing new search API with query_points...")

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

    # Perform search using the new query_points API
    try:
        search_results = qdrant_client.query_points(
            collection_name="rag_embedding",
            query=query_embedding,  # Using the embedding directly
            using="content",  # Specify the named vector
            limit=5,
            with_payload=True
        )

        print(f"Query points results type: {type(search_results)}")
        print(f"QueryResponse attributes: {dir(search_results)}")

        # Access the results from the QueryResponse object
        results_list = search_results.points
        print(f"Results list type: {type(results_list)}")
        print(f"Number of results: {len(results_list)}")

        if results_list:
            first_result = results_list[0]
            print(f"First result type: {type(first_result)}")
            print(f"First result attributes: {dir(first_result)}")
            print(f"First result ID: {first_result.id}")
            print(f"First result score: {first_result.score}")
            print(f"First result payload keys: {list(first_result.payload.keys()) if first_result.payload else 'None'}")

    except Exception as e:
        print(f"Query points error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_new_search_api()