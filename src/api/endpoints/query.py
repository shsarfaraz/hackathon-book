from fastapi import APIRouter, HTTPException
from typing import Optional, List
import uuid

# Import with path relative to project root when running from api directory
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from src.services.rag_service import RAGService
from src.utils.exceptions import http_exception_400, http_exception_404, logger

router = APIRouter()

# Initialize services
rag_service = RAGService()


@router.post("/query")
async def query_documents(
    query: str,
    top_k: int = 5,
    document_ids: Optional[List[str]] = None,
    response_type: str = "standard"
):
    """
    Submit a query to search through documents and generate an AI response
    """
    try:
        # Validate inputs
        if not query or len(query.strip()) < 3:
            raise http_exception_400("Query must be at least 3 characters long")
        
        if top_k <= 0 or top_k > 20:
            raise http_exception_400("top_k must be between 1 and 20")
        
        # Convert document IDs to UUIDs if provided
        if document_ids:
            validated_doc_ids = []
            for doc_id in document_ids:
                try:
                    uuid.UUID(doc_id)
                    validated_doc_ids.append(doc_id)
                except ValueError:
                    raise http_exception_400(f"Invalid document ID format: {doc_id}")
        else:
            validated_doc_ids = None
        
        # Generate response using RAG service
        result = rag_service.generate_detailed_response(
            query=query,
            top_k=top_k,
            document_ids=validated_doc_ids,
            response_type=response_type
        )
        
        # Format the response
        formatted_response = {
            "query_id": str(uuid.uuid4()),  # In a full implementation, this would be stored
            "response": result["response_text"],
            "sources": result["sources"],
            "confidence_score": result["confidence_score"],
            "response_type": result["response_type"]
        }
        
        return formatted_response
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise http_exception_400(f"Error processing query: {str(e)}")


@router.post("/search")
async def search_documents(query: str, top_k: int = 5, document_ids: Optional[List[str]] = None):
    """
    Perform a search without AI generation, just return the relevant document chunks
    """
    try:
        # Validate inputs
        if not query or len(query.strip()) < 3:
            raise http_exception_400("Query must be at least 3 characters long")
        
        if top_k <= 0 or top_k > 20:
            raise http_exception_400("top_k must be between 1 and 20")
        
        # Convert document IDs to UUIDs if provided
        if document_ids:
            validated_doc_ids = []
            for doc_id in document_ids:
                try:
                    uuid.UUID(doc_id)
                    validated_doc_ids.append(doc_id)
                except ValueError:
                    raise http_exception_400(f"Invalid document ID format: {doc_id}")
        else:
            validated_doc_ids = None
        
        # Perform search using the search service
        from ..services.search_service import SearchService
        search_service = SearchService()
        
        search_results = search_service.search_with_reranking(query, top_k, validated_doc_ids)
        
        return {
            "query": query,
            "results": search_results
        }
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in search: {str(e)}")
        raise http_exception_400(f"Error performing search: {str(e)}")