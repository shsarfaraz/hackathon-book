import pytest
from unittest.mock import Mock, patch
from src.services.rag_service import RAGService


def test_rag_service_initialization():
    """Test that the RAG service initializes correctly"""
    service = RAGService()
    assert service is not None
    assert service.search_service is not None


def test_create_prompt():
    """Test the prompt creation functionality"""
    service = RAGService()
    
    query = "What is AI?"
    context = "AI is artificial intelligence."
    
    prompt = service._create_prompt(query, context)
    
    assert query in prompt
    assert context in prompt
    assert "Answer:" in prompt


def test_create_specialized_prompt():
    """Test the specialized prompt creation functionality"""
    service = RAGService()
    
    query = "What is AI?"
    context = "AI is artificial intelligence."
    
    # Test summary prompt
    summary_prompt = service._create_specialized_prompt(query, context, "summary")
    assert "summary" in summary_prompt.lower()
    
    # Test list prompt
    list_prompt = service._create_specialized_prompt(query, context, "list")
    assert "key points:" in list_prompt.lower() or "1." in list_prompt
    
    # Test detailed prompt
    detailed_prompt = service._create_specialized_prompt(query, context, "detailed")
    assert "detailed" in detailed_prompt.lower() or "thorough" in detailed_prompt
    
    # Test standard prompt
    standard_prompt = service._create_specialized_prompt(query, context, "standard")
    assert "Answer:" in standard_prompt