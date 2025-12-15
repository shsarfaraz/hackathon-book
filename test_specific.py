import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.services.rag_service import RAGService

def test_specific_query():
    print("Testing specific query...")

    # Create RAG service instance
    rag_service = RAGService(collection_name="rag_embedding")

    # Test with query that should find Docusaurus content
    result = rag_service.generate_response("How easy is Docusaurus to use?", top_k=5)

    print(f"RAG Service Result for 'How easy is Docusaurus to use?':")
    print(f"  Response: {result.get('response_text', 'N/A')}")
    print(f"  Confidence: {result.get('confidence_score', 'N/A')}")
    print(f"  Sources: {len(result.get('sources', []))} sources")

    for i, source in enumerate(result.get('sources', [])):
        content = source.get('content', '')[:150]
        # Handle potential Unicode characters that can't be printed in the console
        safe_content = content.encode('ascii', errors='ignore').decode('ascii')
        print(f"  Source {i+1}: {safe_content}...")

if __name__ == "__main__":
    test_specific_query()