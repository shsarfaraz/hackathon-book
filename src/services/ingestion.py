import uuid
import logging
from datetime import datetime
from typing import List, Optional, Tuple
from pathlib import Path

from .cohere_client import cohere_client
from ..models.document import Document, DocumentCreate, DocumentInDB
from ..models.document_chunk import DocumentChunk, DocumentChunkCreate
from ..config.settings import settings
from ..utils.exceptions import DocumentProcessingError
from ..config.vector_db import qdrant_client, create_collection_if_not_exists

# Import PyPDF2 for PDF processing
try:
    import PyPDF2
    PDF_AVAILABLE = True
except ImportError:
    PDF_AVAILABLE = False
    print("PyPDF2 not available. PDF processing will not work.")


class IngestionService:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        # Create the documents collection in Qdrant if it doesn't exist
        create_collection_if_not_exists("documents")

    async def process_document(self, file_path: str, original_filename: str) -> DocumentInDB:
        """
        Process a document: extract text, chunk, embed, and store
        """
        try:
            # Create document record
            document = DocumentCreate(
                filename=original_filename,
                size_bytes=Path(file_path).stat().st_size,
                file_type=Path(file_path).suffix,
                status="PROCESSING"
            )
            
            # Extract text from the document
            text_content = await self._extract_text(file_path)
            
            # Chunk the document
            chunks = self._chunk_text(text_content)
            
            # Generate embeddings for chunks
            chunk_texts = [chunk["text"] for chunk in chunks]
            embeddings = cohere_client.generate_embeddings(chunk_texts)
            
            # Store document in vector database
            await self._store_document_chunks(document.id, chunks, embeddings)
            
            # Update document status to complete
            document.status = "COMPLETE"
            
            # Convert to DocumentInDB format for return
            return DocumentInDB(
                id=document.id,
                filename=document.filename,
                size_bytes=document.size_bytes,
                file_type=document.file_type,
                upload_date=datetime.utcnow(),
                page_count=len(chunks),  # Using number of chunks as page count
                status=document.status,
                metadata=None
            )
            
        except Exception as e:
            self.logger.error(f"Error processing document {file_path}: {str(e)}")
            raise DocumentProcessingError(f"Failed to process document: {str(e)}")

    async def _extract_text(self, file_path: str) -> str:
        """
        Extract text from various file types
        """
        file_ext = Path(file_path).suffix.lower()
        
        if file_ext == ".pdf" and PDF_AVAILABLE:
            return self._extract_text_from_pdf(file_path)
        elif file_ext == ".txt":
            return self._extract_text_from_txt(file_path)
        else:
            raise DocumentProcessingError(
                f"Unsupported file type: {file_ext}. Supported types: .pdf, .txt"
            )

    def _extract_text_from_pdf(self, file_path: str) -> str:
        """
        Extract text from PDF file
        """
        text = ""
        try:
            with open(file_path, 'rb') as file:
                pdf_reader = PyPDF2.PdfReader(file)
                for page_num in range(len(pdf_reader.pages)):
                    page = pdf_reader.pages[page_num]
                    text += page.extract_text() + "\n"
        except Exception as e:
            raise DocumentProcessingError(f"Error reading PDF file: {str(e)}")
        
        return text

    def _extract_text_from_txt(self, file_path: str) -> str:
        """
        Extract text from TXT file
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                return file.read()
        except Exception as e:
            raise DocumentProcessingError(f"Error reading TXT file: {str(e)}")

    def _chunk_text(self, text: str) -> List[dict]:
        """
        Chunk text into smaller pieces for embedding
        """
        chunks = []
        chunk_size = settings.CHUNK_SIZE
        overlap_size = settings.OVERLAP_SIZE
        
        start = 0
        chunk_number = 0
        
        while start < len(text):
            end = start + chunk_size
            
            # Adjust end if it exceeds text length
            if end > len(text):
                end = len(text)
            
            chunk_text = text[start:end]
            
            chunks.append({
                "text": chunk_text,
                "start_pos": start,
                "end_pos": end,
                "chunk_number": chunk_number
            })
            
            # Move start to create overlap
            start = end - overlap_size
            chunk_number += 1
            
            # If we've reached the end, break
            if end == len(text):
                break
        
        return chunks

    async def _store_document_chunks(self, document_id: uuid.UUID, chunks: List[dict], embeddings: List[List[float]]):
        """
        Store document chunks in vector database
        """
        try:
            # Prepare points for Qdrant
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                points.append({
                    "id": str(uuid.uuid4()),
                    "vector": {"content": embedding},  # Using named vector "content"
                    "payload": {
                        "document_id": str(document_id),
                        "chunk_number": chunk["chunk_number"],
                        "content": chunk["text"],
                        "char_start_pos": chunk["start_pos"],
                        "char_end_pos": chunk["end_pos"]
                    }
                })
            
            # Upload points to Qdrant
            qdrant_client.upload_points(
                collection_name="documents",
                points=points
            )
            
        except Exception as e:
            raise DocumentProcessingError(f"Error storing document chunks: {str(e)}")

    def search_documents(self, query_text: str, top_k: int = 5) -> List[dict]:
        """
        Search for relevant document chunks based on query
        """
        try:
            # Generate embedding for the query
            query_embedding = cohere_client.generate_embeddings([query_text])[0]
            
            # Search in Qdrant
            search_results = qdrant_client.search(
                collection_name="documents",
                query_vector=("content", query_embedding),  # Using named vector
                limit=top_k,
                with_payload=True
            )
            
            # Format results
            formatted_results = []
            for result in search_results:
                formatted_results.append({
                    "document_id": result.payload["document_id"],
                    "content": result.payload["content"],
                    "chunk_number": result.payload["chunk_number"],
                    "similarity_score": result.score,
                    "char_start_pos": result.payload["char_start_pos"],
                    "char_end_pos": result.payload["char_end_pos"]
                })
                
            return formatted_results
            
        except Exception as e:
            raise DocumentProcessingError(f"Error searching documents: {str(e)}")