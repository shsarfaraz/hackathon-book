import logging
from logging.handlers import RotatingFileHandler
import os
from typing import Optional
from fastapi import HTTPException, status
from fastapi.responses import JSONResponse


# Configure logging
def setup_logging(log_file: str = "app.log", log_level: int = logging.INFO):
    # Create logs directory if it doesn't exist
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Configure root logger
    logger = logging.getLogger()
    logger.setLevel(log_level)
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # File handler with rotation
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, log_file),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(log_level)
    file_handler.setFormatter(formatter)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(log_level)
    console_handler.setFormatter(formatter)
    
    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


# Create the main logger
logger = setup_logging()


# Custom exceptions
class DocumentProcessingError(Exception):
    """Raised when document processing fails"""
    pass


class VectorDBError(Exception):
    """Raised when vector database operations fail"""
    pass


class AIGenerationError(Exception):
    """Raised when AI generation fails"""
    pass


# HTTP Exception helpers
def http_exception_400(detail: str):
    return HTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=detail
    )


def http_exception_404(detail: str = "Item not found"):
    return HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=detail
    )


def http_exception_409(detail: str = "Conflict"):
    return HTTPException(
        status_code=status.HTTP_409_CONFLICT,
        detail=detail
    )


def http_exception_500(detail: str = "Internal server error"):
    return HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=detail
    )


# Response helpers
def success_response(data: any, message: str = "Success"):
    return JSONResponse(
        status_code=200,
        content={
            "success": True,
            "message": message,
            "data": data
        }
    )


def error_response(message: str, status_code: int = 400):
    return JSONResponse(
        status_code=status_code,
        content={
            "success": False,
            "message": message
        }
    )