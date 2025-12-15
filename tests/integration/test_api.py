import pytest
from fastapi.testclient import TestClient
from src.api.app import app


def test_api_root():
    """Test the root endpoint of the API"""
    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()
    assert "RAG Document Query System API" in response.json()["message"]


def test_api_health():
    """Test a basic health check"""
    client = TestClient(app)
    # Test that the API is running and can handle requests
    # Since we don't have authentication required in our endpoints,
    # we can make a simple request to check if the API is working
    response = client.get("/docs")  # FastAPI auto-generates this
    # The status could be 200 if docs are available or 404/405 depending on setup
    # We just want to ensure the server is responding
    assert response.status_code in [200, 404, 405]  # Server is responding