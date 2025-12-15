import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import with path relative to project root (like the API does)
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from src.services.rag_service import RAGService
from src.services.search_service import SearchService

def test_api_initialization():
    print("Testing API-style initialization...")

    # This is how it's done in the API endpoint
    rag_service = RAGService(collection_name="rag_embedding")

    # Check what collection the internal search service is using
    print(f"RAG Service collection name: rag_embedding")
    print(f"Internal search service collection name: {rag_service.search_service.collection_name}")

    # Test with query that should find Docusaurus content
    result = rag_service.generate_response("How easy is Docusaurus to use?", top_k=5)

    print(f"RAG Service Result for 'How easy is Docusaurus to use?' with API-style init:")
    print(f"  Response: {result.get('response_text', 'N/A')}")
    print(f"  Confidence: {result.get('confidence_score', 'N/A')}")
    print(f"  Sources: {len(result.get('sources', []))} sources")

    for i, source in enumerate(result.get('sources', [])):
        content = source.get('content', '')[:100]
        # Handle potential Unicode characters that can't be printed in the console
        safe_content = content.encode('ascii', errors='ignore').decode('ascii')
        print(f"  Source {i+1}: {safe_content}...")

if __name__ == "__main__":
    test_api_initialization()