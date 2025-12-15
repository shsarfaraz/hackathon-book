import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def delete_collection():
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        print("QDRANT_URL environment variable not set")
        return False

    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    # Delete the collection
    try:
        client.delete_collection("rag_embedding")
        print("Collection 'rag_embedding' deleted successfully")
        return True
    except Exception as e:
        print(f"Error deleting collection: {e}")
        return False

if __name__ == "__main__":
    delete_collection()