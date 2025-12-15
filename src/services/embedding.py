from typing import List, Dict, Any
from ..services.cohere_client import cohere_client
from ..config.vector_db import qdrant_client, create_collection_if_not_exists


class EmbeddingService:
    """
    Service for generating and managing embeddings
    """
    
    def __init__(self, collection_name: str = "documents"):
        self.collection_name = collection_name
        # Ensure the collection exists
        create_collection_if_not_exists(self.collection_name)
    
    def generate_embeddings(self, texts: List[str], model: str = "embed-multilingual-v3.0") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        return cohere_client.generate_embeddings(texts, model)
    
    def store_embeddings(self, 
                         texts: List[str], 
                         embeddings: List[List[float]], 
                         metadata_list: List[Dict[str, Any]] = None,
                         ids: List[str] = None) -> bool:
        """
        Store embeddings in Qdrant vector database
        """
        try:
            # If IDs are not provided, generate them
            if ids is None:
                import uuid
                ids = [str(uuid.uuid4()) for _ in texts]
            
            # If metadata is not provided, create empty metadata for each text
            if metadata_list is None:
                metadata_list = [{} for _ in texts]
            
            # Prepare points for Qdrant
            points = []
            for i, (text, embedding, metadata) in enumerate(zip(texts, embeddings, metadata_list)):
                points.append({
                    "id": ids[i],
                    "vector": {"content": embedding},  # Using named vector "content"
                    "payload": {
                        "text": text,
                        "metadata": metadata
                    }
                })
            
            # Upload points to Qdrant
            qdrant_client.upload_points(
                collection_name=self.collection_name,
                points=points
            )
            
            return True
            
        except Exception as e:
            print(f"Error storing embeddings: {str(e)}")
            return False
    
    def search_similar(self, query: str, top_k: int = 5, model: str = "embed-multilingual-v3.0") -> List[Dict[str, Any]]:
        """
        Search for similar texts based on a query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.generate_embeddings([query], model)[0]
            
            # Search in Qdrant using query_points (newer API)
            search_response = qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,  # Using the embedding directly
                using="content",  # Specify the named vector
                limit=top_k,
                with_payload=True
            )

            # Format results - access the points from the QueryResponse
            formatted_results = []
            for result in search_response.points:  # Access the .points attribute
                formatted_results.append({
                    "id": result.id,
                    "text": result.payload["text"],
                    "similarity_score": result.score,
                    "metadata": result.payload.get("metadata", {})
                })

            return formatted_results
            
        except Exception as e:
            print(f"Error searching similar texts: {str(e)}")
            return []
    
    def delete_embeddings(self, ids: List[str]) -> bool:
        """
        Delete embeddings by IDs from the vector database
        """
        try:
            qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=ids
            )
            return True
        except Exception as e:
            print(f"Error deleting embeddings: {str(e)}")
            return False
    
    def update_embedding(self, id: str, text: str, metadata: Dict[str, Any] = None, model: str = "embed-multilingual-v3.0") -> bool:
        """
        Update an existing embedding with new text
        """
        try:
            # Generate new embedding
            new_embedding = self.generate_embeddings([text], model)[0]
            
            # Prepare the point to update
            point = {
                "id": id,
                "vector": {"content": new_embedding},
                "payload": {
                    "text": text,
                    "metadata": metadata or {}
                }
            }
            
            # Update in Qdrant
            qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )
            
            return True
        except Exception as e:
            print(f"Error updating embedding: {str(e)}")
            return False