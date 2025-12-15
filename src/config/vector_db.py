from qdrant_client import QdrantClient
from ..config.settings import settings


def get_qdrant_client():
    """
    Creates and returns a Qdrant client instance
    """
    if settings.QDRANT_URL:
        # Use URL-based connection (for cloud)
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=False,  # Set to True for better performance if available
        )
    else:
        # Use host/port-based connection (for local)
        client = QdrantClient(
            host=settings.QDRANT_HOST,
            port=settings.QDRANT_PORT,
            api_key=settings.QDRANT_API_KEY,
        )
    
    return client


# Default client instance
qdrant_client = get_qdrant_client()


def create_collection_if_not_exists(
    collection_name: str,
    vector_size: int = 1024,  # Default vector size for Cohere embeddings
    distance="Cosine"
):
    """
    Creates a collection in Qdrant if it doesn't exist
    """
    try:
        # Try to get collection info to see if it exists
        qdrant_client.get_collection(collection_name)
    except:
        # Collection doesn't exist, create it
        from qdrant_client.http import models
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config={
                "content": models.VectorParams(
                    size=vector_size,
                    distance=distance,
                )
            }
        )