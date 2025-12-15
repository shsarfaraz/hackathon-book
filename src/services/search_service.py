from typing import List, Dict, Any, Optional
from ..services.embedding import EmbeddingService
from ..config.vector_db import qdrant_client
from ..services.cohere_client import cohere_client


class SearchService:
    """
    Service for performing semantic search on documents
    """
    
    def __init__(self, collection_name: str = "documents"):
        self.embedding_service = EmbeddingService(collection_name)
        self.collection_name = collection_name

    def semantic_search(self, 
                       query: str, 
                       top_k: int = 5, 
                       document_ids: List[str] = None,
                       filters: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Perform semantic search using vector similarity
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embedding_service.generate_embeddings([query])[0]
            
            # Prepare search filters if provided
            search_filter = None
            if filters:
                from qdrant_client.http import models
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )
                
                if filter_conditions:
                    search_filter = models.Filter(
                        must=filter_conditions
                    )
            
            # Search in Qdrant using query_points (newer API)
            search_response = qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,  # Using the embedding directly
                using="content",  # Specify the named vector
                limit=top_k,
                query_filter=search_filter,
                with_payload=True
            )

            # Format results - access the points from the QueryResponse
            formatted_results = []
            for result in search_response.points:  # Access the .points attribute
                result_data = {
                    "id": result.id,
                    "content": result.payload["content"],
                    "document_id": result.payload["document_id"],
                    "chunk_number": result.payload["chunk_number"],
                    "similarity_score": result.score,
                    "char_start_pos": result.payload["char_start_pos"],
                    "char_end_pos": result.payload["char_end_pos"]
                }

                # If specific document IDs were requested, filter results
                if document_ids and result.payload["document_id"] not in document_ids:
                    continue

                formatted_results.append(result_data)

            return formatted_results
            
        except Exception as e:
            print(f"Error in semantic search: {str(e)}")
            return []

    def hybrid_search(self, 
                     query: str, 
                     top_k: int = 5,
                     keyword_weight: float = 0.3,
                     semantic_weight: float = 0.7) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining semantic and keyword-based search
        """
        try:
            # For now, using semantic search as primary approach
            # In a full implementation, this would combine both semantic and keyword search results
            return self.semantic_search(query, top_k)
        except Exception as e:
            print(f"Error in hybrid search: {str(e)}")
            return []

    def search_with_reranking(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search documents and rerank results for better relevance
        """
        try:
            # Get initial search results
            initial_results = self.semantic_search(query, top_k * 2)  # Get more results for reranking
            
            if not initial_results:
                return []
            
            # Extract content from results for reranking
            contents = [result["content"] for result in initial_results]
            
            # Use Cohere's rerank functionality
            reranked_results = cohere_client.rerank(
                query=query,
                documents=contents,
                top_n=top_k
            )
            
            # Handle both possible response formats from cohere_client.rerank
            if hasattr(reranked_results, 'results'):
                # New format with .results attribute
                rerank_results_list = reranked_results.results
            else:
                # Old format where response is the list directly
                rerank_results_list = reranked_results

            # Map reranked results back to original format with scores
            final_results = []
            for rank_result in rerank_results_list:
                original_result = initial_results[rank_result["index"]]
                original_result["rerank_score"] = rank_result["relevance_score"]
                final_results.append(original_result)
            
            # Sort by rerank score to ensure proper ordering
            final_results.sort(key=lambda x: x["rerank_score"], reverse=True)
            
            return final_results[:top_k]
            
        except Exception as e:
            print(f"Error in search with reranking: {str(e)}")
            # Fallback to regular semantic search
            return self.semantic_search(query, top_k)

    def get_document_chunks(self, document_id: str, top_k: int = None) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks for a specific document
        """
        try:
            from qdrant_client.http import models
            
            # Create filter for specific document ID
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.document_id",
                        match=models.MatchValue(value=document_id)
                    )
                ]
            )
            
            # Search in Qdrant with the filter
            search_results = qdrant_client.search(
                collection_name=self.collection_name,
                query_filter=search_filter,
                limit=top_k or 1000,  # Default to 1000, or specific number if provided
                with_payload=True
            )
            
            # Format results
            formatted_results = []
            for result in search_results:
                formatted_results.append({
                    "id": result.id,
                    "content": result.payload["content"],
                    "chunk_number": result.payload["chunk_number"],
                    "char_start_pos": result.payload["char_start_pos"],
                    "char_end_pos": result.payload["char_end_pos"]
                })
                
            # Sort by chunk number to maintain document order
            formatted_results.sort(key=lambda x: x["chunk_number"])
            
            return formatted_results
            
        except Exception as e:
            print(f"Error retrieving document chunks: {str(e)}")
            return []