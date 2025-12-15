from fastapi import APIRouter, HTTPException
from typing import List, Dict, Optional
import uuid
from pydantic import BaseModel

# Import with path relative to project root when running from api directory
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from src.services.rag_service import RAGService
from src.utils.exceptions import http_exception_400, http_exception_404, logger

router = APIRouter()

# Initialize services
rag_service = RAGService(collection_name="rag_embedding")


class ChatMessage(BaseModel):
    """
    Model for a chat message
    """
    role: str  # "user" or "assistant"
    content: str


class ChatRequest(BaseModel):
    """
    Model for a chat request
    """
    message: str
    conversation_id: Optional[str] = None
    history: List[ChatMessage] = []
    top_k: int = 5
    document_ids: Optional[List[str]] = None
    response_type: str = "standard"


class ChatResponse(BaseModel):
    """
    Model for a chat response
    """
    conversation_id: str
    response: str
    sources: List[Dict]
    confidence_score: float
    response_type: str


@router.post("/chat", response_model=ChatResponse)
async def chat_with_bot(request: ChatRequest):
    """
    Main chat endpoint that handles conversation with the RAG bot
    """
    try:
        # Validate inputs
        if not request.message or len(request.message.strip()) < 1:
            raise http_exception_400("Message cannot be empty")

        if request.top_k <= 0 or request.top_k > 20:
            raise http_exception_400("top_k must be between 1 and 20")

        # Use provided conversation ID or generate a new one
        conversation_id = request.conversation_id or str(uuid.uuid4())

        # Prepare context from history if provided
        context_from_history = ""
        if request.history:
            # Include recent conversation history (limit to last 5 exchanges)
            recent_history = request.history[-5:]
            for msg in recent_history:
                context_from_history += f"{msg.role}: {msg.content}\n"

        # Generate response using RAG service
        result = rag_service.generate_detailed_response(
            query=request.message,
            top_k=request.top_k,
            document_ids=request.document_ids,
            response_type=request.response_type
        )

        # Format the response
        response = ChatResponse(
            conversation_id=conversation_id,
            response=result["response_text"],
            sources=result["sources"],
            confidence_score=result["confidence_score"],
            response_type=result["response_type"]
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in chat: {str(e)}")
        raise http_exception_400(f"Error processing chat message: {str(e)}")


@router.post("/chat/stream")
async def chat_with_bot_stream(request: ChatRequest):
    """
    Streaming chat endpoint that returns responses in chunks
    """
    from fastapi.responses import StreamingResponse
    import asyncio

    async def event_generator():
        try:
            # Generate response using RAG service
            result = rag_service.generate_detailed_response(
                query=request.message,
                top_k=request.top_k,
                document_ids=request.document_ids,
                response_type=request.response_type
            )

            # Simulate streaming by yielding response in chunks
            response_text = result["response_text"]
            chunk_size = 20  # characters per chunk

            for i in range(0, len(response_text), chunk_size):
                chunk = response_text[i:i + chunk_size]
                yield f"data: {chunk}\n\n"
                await asyncio.sleep(0.01)  # Small delay to simulate streaming

            # Send sources and metadata at the end
            yield f"data: [DONE]\n\n"

        except Exception as e:
            yield f"error: {str(e)}\n\n"

    return StreamingResponse(event_generator(), media_type="text/plain")


@router.post("/chat/conversation/start")
async def start_conversation(document_ids: Optional[List[str]] = None):
    """
    Start a new conversation with optional document context
    """
    try:
        conversation_id = str(uuid.uuid4())

        return {
            "conversation_id": conversation_id,
            "message": "New conversation started",
            "document_ids": document_ids
        }
    except Exception as e:
        logger.error(f"Error starting conversation: {str(e)}")
        raise http_exception_400(f"Error starting conversation: {str(e)}")


@router.get("/chat/conversation/{conversation_id}")
async def get_conversation(conversation_id: str):
    """
    Get conversation history (placeholder - would need actual storage in a real implementation)
    """
    try:
        # In a real implementation, this would retrieve from a database
        # For now, returning a mock response
        return {
            "conversation_id": conversation_id,
            "messages": [],
            "created_at": "2025-12-14T10:30:00Z",
            "status": "active"
        }
    except Exception as e:
        logger.error(f"Error retrieving conversation: {str(e)}")
        raise http_exception_404("Conversation not found")