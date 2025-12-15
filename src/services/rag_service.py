from typing import List, Dict, Any
from src.services.search_service import SearchService
from src.services.cohere_client import cohere_client


class RAGService:
    """
    Service for Retrieval-Augmented Generation
    Combines search results with AI generation to create contextual responses
    """
    
    def __init__(self, collection_name: str = "documents"):
        self.search_service = SearchService(collection_name)

    def generate_response(self, 
                         query: str, 
                         top_k: int = 5, 
                         document_ids: List[str] = None,
                         include_sources: bool = True) -> Dict[str, Any]:
        """
        Generate a response based on retrieved documents
        """
        try:
            # Search for relevant documents
            search_results = self.search_service.search_with_reranking(query, top_k)
            
            if not search_results:
                return {
                    "response_text": "I couldn't find any relevant information in the documents to answer your query.",
                    "sources": [],
                    "confidence_score": 0.0
                }
            
            # Prepare context from search results
            context_parts = []
            sources = []
            
            for result in search_results:
                context_parts.append(result["content"])
                
                if include_sources:
                    sources.append({
                        "document_id": result["document_id"],
                        "content": result["content"],
                        "similarity_score": result.get("rerank_score", result["similarity_score"]),
                        "chunk_number": result["chunk_number"]
                    })
            
            # Combine context
            context = "\n\n".join(context_parts)
            
            # Create prompt for the AI model
            prompt = self._create_prompt(query, context)
            
            # Generate response using Cohere
            ai_response = cohere_client.generate_response(prompt)
            
            # Calculate confidence based on similarity scores
            avg_similarity = sum([src["similarity_score"] for src in sources]) / len(sources) if sources else 0.0
            
            return {
                "response_text": ai_response,
                "sources": sources,
                "confidence_score": avg_similarity
            }
            
        except Exception as e:
            print(f"Error in RAG generation: {str(e)}")
            return {
                "response_text": "Sorry, I encountered an error while processing your request.",
                "sources": [],
                "confidence_score": 0.0
            }

    def _create_prompt(self, query: str, context: str) -> str:
        """
        Create a prompt for the AI model with context and query
        """
        prompt = f"""
        Based on the following context, please answer the question. 
        If the answer is not in the context, say "I don't have enough information to answer that based on the provided documents."
        
        Context: {context}
        
        Question: {query}
        
        Answer:
        """
        return prompt.strip()

    def generate_detailed_response(self, 
                                  query: str, 
                                  top_k: int = 5, 
                                  document_ids: List[str] = None,
                                  response_type: str = "standard") -> Dict[str, Any]:
        """
        Generate a more detailed response with different response types
        """
        try:
            # Search for relevant documents
            search_results = self.search_service.search_with_reranking(query, top_k)
            
            if not search_results:
                return {
                    "response_text": "I couldn't find any relevant information in the documents to answer your query.",
                    "sources": [],
                    "confidence_score": 0.0,
                    "response_type": response_type
                }
            
            # Prepare context from search results
            context_parts = []
            sources = []
            
            for result in search_results:
                context_parts.append(result["content"])
                sources.append({
                    "document_id": result["document_id"],
                    "content": result["content"],
                    "similarity_score": result.get("rerank_score", result["similarity_score"]),
                    "chunk_number": result["chunk_number"]
                })
            
            # Combine context
            context = "\n\n".join(context_parts)
            
            # Create specialized prompt based on response type
            prompt = self._create_specialized_prompt(query, context, response_type)
            
            # Generate response using Cohere
            ai_response = cohere_client.generate_response(prompt)
            
            # Calculate confidence
            avg_similarity = sum([src["similarity_score"] for src in sources]) / len(sources) if sources else 0.0
            
            return {
                "response_text": ai_response,
                "sources": sources,
                "confidence_score": avg_similarity,
                "response_type": response_type
            }
            
        except Exception as e:
            print(f"Error in detailed RAG generation: {str(e)}")
            return {
                "response_text": "Sorry, I encountered an error while processing your request.",
                "sources": [],
                "confidence_score": 0.0,
                "response_type": response_type
            }

    def _create_specialized_prompt(self, query: str, context: str, response_type: str) -> str:
        """
        Create a specialized prompt based on response type
        """
        if response_type == "summary":
            prompt = f"""
            Please provide a concise summary of the information related to this query: {query}
            
            Base your summary solely on the following context:
            {context}
            
            Summary:
            """
        elif response_type == "list":
            prompt = f"""
            Please provide a numbered list of key points related to this query: {query}
            
            Base your list solely on the following context:
            {context}
            
            Key Points:
            1. 
            """
        elif response_type == "detailed":
            prompt = f"""
            Please provide a detailed response to this query: {query}
            
            Use the following context to inform your response:
            {context}
            
            Provide a thorough explanation with specific details from the context:
            
            Detailed Response:
            """
        else:  # standard response
            prompt = f"""
            Based on the following context, please answer the question. 
            If the answer is not in the context, say "I don't have enough information to answer that based on the provided documents."
            
            Context: {context}
            
            Question: {query}
            
            Answer:
            """
        
        return prompt.strip()