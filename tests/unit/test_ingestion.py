import pytest
from src.services.ingestion import IngestionService


def test_ingestion_service_initialization():
    """Test that the ingestion service initializes correctly"""
    service = IngestionService()
    assert service is not None
    assert hasattr(service, 'logger')


def test_chunk_text():
    """Test the text chunking functionality"""
    service = IngestionService()
    
    # This test would need to access the private _chunk_text method
    # For now, we'll just verify the method exists
    assert hasattr(service, '_chunk_text')
    
    # Test with a simple text
    text = "This is a test. This is only a test. If this had been an actual emergency, you would have been instructed where to go and what to do."
    chunks = service._chunk_text(text)
    
    # There should be at least one chunk
    assert len(chunks) > 0
    
    # Each chunk should have required keys
    for chunk in chunks:
        assert 'text' in chunk
        assert 'start_pos' in chunk
        assert 'end_pos' in chunk
        assert 'chunk_number' in chunk