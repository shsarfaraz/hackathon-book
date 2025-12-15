import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.services.rag_service import RAGService

def test_api_exact_method():
    print("Testing with exact same method as API...")

    # Create RAG service instance (same as in chat endpoint)
    rag_service = RAGService(collection_name="rag_embedding")

    # Call the exact same method as used in the API
    result = rag_service.generate_detailed_response("How easy is Docusaurus to use?", top_k=5, response_type="standard")

    print(f"RAG Service Detailed Result (API method):")
    print(f"  Response: {result.get('response_text', 'N/A')}")
    print(f"  Confidence: {result.get('confidence_score', 'N/A')}")
    print(f"  Sources: {len(result.get('sources', []))} sources")
    print(f"  Response Type: {result.get('response_type', 'N/A')}")

    for i, source in enumerate(result.get('sources', [])):
        content = source.get('content', '')[:100]
        # Handle potential Unicode characters that can't be printed in the console
        safe_content = content.encode('ascii', errors='ignore').decode('ascii')
        print(f"  Source {i+1}: {safe_content}...")

if __name__ == "__main__":
    test_api_exact_method()