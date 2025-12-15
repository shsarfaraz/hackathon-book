import uuid
from typing import List, Dict, Any
from ..config.settings import settings


class ChunkingService:
    """
    Service for handling document chunking logic
    """
    
    @staticmethod
    def chunk_text(text: str, chunk_size: int = None, overlap_size: int = None) -> List[Dict[str, Any]]:
        """
        Split text into chunks with configurable size and overlap
        """
        if chunk_size is None:
            chunk_size = settings.CHUNK_SIZE
        if overlap_size is None:
            overlap_size = settings.OVERLAP_SIZE
        
        chunks = []
        start = 0
        chunk_number = 0
        
        while start < len(text):
            # Determine the end position
            end = start + chunk_size
            
            # Adjust end if it exceeds text length
            if end > len(text):
                end = len(text)
            
            chunk_text = text[start:end]
            
            # Create chunk with metadata
            chunk = {
                "id": str(uuid.uuid4()),
                "text": chunk_text,
                "start_pos": start,
                "end_pos": end,
                "chunk_number": chunk_number,
                "metadata": {}
            }
            
            chunks.append(chunk)
            
            # Move start to create overlap
            start = end - overlap_size
            chunk_number += 1
            
            # If we've reached the end, break
            if end == len(text):
                break
        
        return chunks

    @staticmethod
    def chunk_text_by_sentences(text: str, max_chunk_size: int = None) -> List[Dict[str, Any]]:
        """
        Split text into sentence-aware chunks that respect sentence boundaries
        """
        if max_chunk_size is None:
            max_chunk_size = settings.CHUNK_SIZE
        
        # Split text into sentences
        import re
        sentences = re.split(r'[.!?]+\s+', text)
        
        chunks = []
        current_chunk = ""
        current_start_pos = 0
        chunk_number = 0
        
        for i, sentence in enumerate(sentences):
            # Check if adding this sentence would exceed the size limit
            if len(current_chunk) + len(sentence) <= max_chunk_size:
                current_chunk += sentence + ". "  # Add sentence delimiter back
            else:
                # If the current chunk is not empty, save it
                if current_chunk.strip():
                    chunks.append({
                        "id": str(uuid.uuid4()),
                        "text": current_chunk.strip(),
                        "start_pos": current_start_pos,
                        "end_pos": current_start_pos + len(current_chunk),
                        "chunk_number": chunk_number,
                        "metadata": {"sentence_aware": True}
                    })
                    
                    # Update positions for next chunk
                    current_start_pos = current_start_pos + len(current_chunk)
                    current_chunk = sentence + ". "  # Start new chunk with current sentence
                    chunk_number += 1
                else:
                    # Handle the case where a single sentence is larger than max_chunk_size
                    # In this case, fall back to character-based chunking for this sentence
                    fallback_chunks = ChunkingService.chunk_text(
                        sentence, 
                        chunk_size=max_chunk_size, 
                        overlap_size=settings.OVERLAP_SIZE
                    )
                    for fallback_chunk in fallback_chunks:
                        fallback_chunk["metadata"]["fallback_to_char_chunking"] = True
                        chunks.append(fallback_chunk)
                    chunk_number += len(fallback_chunks)
        
        # Add the final chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "id": str(uuid.uuid4()),
                "text": current_chunk.strip(),
                "start_pos": current_start_pos,
                "end_pos": current_start_pos + len(current_chunk),
                "chunk_number": chunk_number,
                "metadata": {"sentence_aware": True}
            })
        
        return chunks

    @staticmethod
    def merge_chunks(chunks: List[Dict[str, Any]], max_chunk_size: int = None) -> List[Dict[str, Any]]:
        """
        Merge small chunks to approach the target size, when possible
        """
        if not chunks or max_chunk_size is None:
            return chunks
            
        merged_chunks = []
        current_chunk = None
        
        for chunk in chunks:
            if current_chunk is None:
                current_chunk = chunk.copy()
            else:
                # Check if merging would exceed the size limit
                combined_size = len(current_chunk["text"]) + len(chunk["text"])
                if combined_size <= max_chunk_size:
                    # Merge the chunks
                    current_chunk["text"] += " " + chunk["text"]
                    current_chunk["end_pos"] = chunk["end_pos"]
                    # Update metadata to indicate merging
                    if "merged_from" in current_chunk["metadata"]:
                        current_chunk["metadata"]["merged_from"].append(chunk["id"])
                    else:
                        current_chunk["metadata"]["merged_from"] = [current_chunk["id"], chunk["id"]]
                    # Update ID to reflect the merged nature
                    current_chunk["id"] = str(uuid.uuid4())
                else:
                    # Can't merge, add current chunk to results and start new one
                    merged_chunks.append(current_chunk)
                    current_chunk = chunk.copy()
        
        # Add the final chunk if it exists
        if current_chunk:
            merged_chunks.append(current_chunk)
        
        return merged_chunks