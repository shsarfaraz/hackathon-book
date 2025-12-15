import os
import tempfile
from pathlib import Path
from typing import BinaryIO, Optional
from ..utils.exceptions import DocumentProcessingError
from ..config.settings import settings


class FileProcessor:
    """
    Service for handling file uploads and validation
    """
    
    @staticmethod
    def validate_file(file: BinaryIO, filename: str) -> bool:
        """
        Validate the uploaded file based on size and extension
        """
        # Check file extension
        file_ext = Path(filename).suffix.lower()
        if file_ext not in settings.ALLOWED_EXTENSIONS:
            raise DocumentProcessingError(
                f"File type '{file_ext}' not allowed. "
                f"Allowed types: {', '.join(settings.ALLOWED_EXTENSIONS)}"
            )
        
        # Check file size by seeking to end and getting position
        file.seek(0, os.SEEK_END)
        file_size = file.tell()
        file.seek(0)  # Reset file pointer to beginning
        
        if file_size > settings.MAX_FILE_SIZE:
            raise DocumentProcessingError(
                f"File size {file_size} bytes exceeds maximum allowed size of {settings.MAX_FILE_SIZE} bytes"
            )
        
        return True

    @staticmethod
    def save_upload_file(file: BinaryIO, destination: Path) -> bool:
        """
        Save the uploaded file to the specified destination
        """
        try:
            with open(destination, "wb") as buffer:
                # Reset file pointer to beginning
                file.seek(0)
                # Write file content in chunks to handle large files
                chunk_size = 8192  # 8KB chunks
                while True:
                    chunk = file.read(chunk_size)
                    if not chunk:
                        break
                    buffer.write(chunk)
            return True
        except Exception as e:
            raise DocumentProcessingError(f"Error saving file: {str(e)}")

    @staticmethod
    def create_temp_file(filename: str) -> Path:
        """
        Create a temporary file path for processing
        """
        temp_dir = Path(tempfile.gettempdir())
        unique_filename = f"rag_upload_{filename}"
        return temp_dir / unique_filename

    @staticmethod
    def cleanup_temp_file(filepath: Path) -> bool:
        """
        Remove temporary file after processing
        """
        try:
            if filepath.exists():
                filepath.unlink()
                return True
            return False
        except Exception as e:
            print(f"Error cleaning up temp file {filepath}: {str(e)}")
            return False