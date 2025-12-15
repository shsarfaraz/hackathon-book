import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.services.search_service import SearchService

def test_search_service():
    print("Testing search service directly...")

    # Create search service instance
    search_service = SearchService(collection_name="rag_embedding")

    # Test search
    results = search_service.semantic_search("Docusaurus", top_k=5)

    print(f"Search results: {len(results)} items found")
    for i, result in enumerate(results):
        print(f"Result {i+1}:")
        print(f"  ID: {result.get('id', 'N/A')}")
        content = result.get('content', 'N/A')[:100]
        safe_content = content.encode('ascii', errors='ignore').decode('ascii')
        print(f"  Content: {safe_content}...")
        print(f"  Similarity: {result.get('similarity_score', 'N/A')}")
        print(f"  Document ID: {result.get('document_id', 'N/A')}")
        print()

    # Also try search_with_reranking
    print("Testing search with reranking...")
    results_rerank = search_service.search_with_reranking("Docusaurus", top_k=5)
    print(f"Reranked search results: {len(results_rerank)} items found")
    for i, result in enumerate(results_rerank):
        print(f"Reranked Result {i+1}:")
        print(f"  ID: {result.get('id', 'N/A')}")
        content = result.get('content', 'N/A')[:100]
        safe_content = content.encode('ascii', errors='ignore').decode('ascii')
        print(f"  Content: {safe_content}...")
        print(f"  Similarity: {result.get('similarity_score', 'N/A')}")
        print(f"  Rerank Score: {result.get('rerank_score', 'N/A')}")
        print()

if __name__ == "__main__":
    test_search_service()