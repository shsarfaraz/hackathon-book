from fastapi import APIRouter, UploadFile, File, HTTPException, BackgroundTasks
from typing import Optional
import uuid
from pathlib import Path

# Import with path relative to project root when running from api directory
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from src.services.ingestion import IngestionService
from src.services.file_processor import FileProcessor
from src.models.document import Document
from src.utils.exceptions import http_exception_400, http_exception_404, logger

router = APIRouter()

# Initialize services
ingestion_service = IngestionService()


@router.post("/documents/upload")
async def upload_document(background_tasks: BackgroundTasks, file: UploadFile = File(...)):
    """
    Upload a document for processing and indexing
    """
    try:
        # Validate the uploaded file
        FileProcessor.validate_file(file.file, file.filename)
        
        # Create a temporary file to save the upload
        temp_path = FileProcessor.create_temp_file(file.filename)
        
        # Save the uploaded file to temporary location
        FileProcessor.save_upload_file(file.file, temp_path)
        
        # Process the document asynchronously 
        # (In a real implementation, this would be a background task)
        document_record = await ingestion_service.process_document(
            str(temp_path), 
            file.filename
        )
        
        # Clean up the temporary file after processing
        FileProcessor.cleanup_temp_file(temp_path)
        
        return {
            "document_id": str(document_record.id),
            "filename": document_record.filename,
            "status": document_record.status,
            "message": "Document uploaded successfully and processing completed"
        }
        
    except Exception as e:
        logger.error(f"Error uploading document: {str(e)}")
        # Clean up temp file if it exists in case of error
        if 'temp_path' in locals():
            FileProcessor.cleanup_temp_file(temp_path)
        raise http_exception_400(f"Error processing document: {str(e)}")


@router.get("/documents/{document_id}")
async def get_document_status(document_id: str):
    """
    Get the processing status of a document
    """
    try:
        # Convert string ID to UUID
        doc_uuid = uuid.UUID(document_id)
        
        # In a full implementation, this would retrieve from a database
        # For now, we're returning a mock response
        # In the actual implementation, you'd have a database record for each document
        
        # Since we don't have a persistent document store yet, return a mock response
        # indicating the document exists with a complete status
        return {
            "document_id": document_id,
            "filename": "document.pdf",
            "status": "complete",
            "upload_date": "2025-12-14T10:30:00Z",
            "size_bytes": 1024000,
            "page_count": 25,
            "message": "Document processing complete"
        }
        
    except ValueError:
        raise http_exception_400("Invalid document ID format")
    except Exception as e:
        logger.error(f"Error getting document status: {str(e)}")
        raise http_exception_404("Document not found")


@router.get("/documents")
async def list_documents(status: Optional[str] = None, limit: int = 20, offset: int = 0):
    """
    List all processed documents
    """
    try:
        # In a full implementation, this would query the database
        # For now, returning mock data
        documents = [
            {
                "document_id": str(uuid.uuid4()),
                "filename": "sample_document.pdf",
                "status": "complete",
                "upload_date": "2025-12-14T10:30:00Z",
                "size_bytes": 1024000,
                "page_count": 25
            }
        ]
        
        return {
            "documents": documents,
            "total_count": len(documents)
        }
        
    except Exception as e:
        logger.error(f"Error listing documents: {str(e)}")
        raise http_exception_400("Error retrieving documents")


@router.delete("/documents/{document_id}")
async def delete_document(document_id: str):
    """
    Delete a document and its associated chunks
    """
    try:
        # Convert string ID to UUID
        doc_uuid = uuid.UUID(document_id)
        
        # In a full implementation, this would:
        # 1. Delete from the database
        # 2. Delete from the vector store
        # For now, returning mock success
        
        return {
            "message": "Document and associated data deleted successfully"
        }
        
    except ValueError:
        raise http_exception_400("Invalid document ID format")
    except Exception as e:
        logger.error(f"Error deleting document: {str(e)}")
        raise http_exception_404("Document not found")