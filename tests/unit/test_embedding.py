import pytest
from src.services.embedding import EmbeddingService


def test_embedding_service_initialization():
    """Test that the embedding service initializes correctly"""
    service = EmbeddingService()
    assert service is not None
    assert service.collection_name == "documents"


def test_generate_embeddings():
    """Test the embedding generation functionality"""
    service = EmbeddingService()
    
    # Test with a simple text
    texts = ["This is a test sentence."]
    embeddings = service.generate_embeddings(texts)
    
    # Should return a list
    assert isinstance(embeddings, list)
    assert len(embeddings) == 1
    
    # Each embedding should be a list of floats
    assert isinstance(embeddings[0], list)
    assert len(embeddings[0]) > 0  # Should have some dimensions
    assert all(isinstance(val, float) for val in embeddings[0])