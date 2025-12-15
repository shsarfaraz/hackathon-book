import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.config.vector_db import qdrant_client

def test_raw_search():
    print("Testing raw Qdrant search...")

    # Test if the collection exists
    try:
        collection_info = qdrant_client.get_collection("rag_embedding")
        print(f"Collection exists: {collection_info}")
    except Exception as e:
        print(f"Collection error: {e}")
        return

    # Count points in collection
    try:
        count = qdrant_client.count("rag_embedding")
        print(f"Points in collection: {count}")
    except Exception as e:
        print(f"Count error: {e}")

    # Try to list some points to see the structure
    try:
        records = qdrant_client.scroll(
            collection_name="rag_embedding",
            limit=2
        )
        print(f"Sample records: {records}")
    except Exception as e:
        print(f"Scroll error: {e}")

if __name__ == "__main__":
    test_raw_search()