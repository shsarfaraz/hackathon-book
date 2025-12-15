import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    # Qdrant settings
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_HOST: str = os.getenv("QDRANT_HOST", "")
    QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", 6333))
    
    # Neon DB settings
    DATABASE_URL: str = os.getenv("NEON_DB_URL", "")
    
    # Cohere settings
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    
    # Application settings
    APP_NAME: str = "RAG Document Query System"
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "RAG System"
    
    # Security
    SECRET_KEY: str = os.getenv("SECRET_KEY", "dev-secret-key-change-in-production")
    ALGORITHM = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES = 30
    
    # File upload settings
    MAX_FILE_SIZE = 10 * 1024 * 1024  # 10MB
    ALLOWED_EXTENSIONS = {'.pdf', '.txt', '.docx'}
    
    # Document processing
    CHUNK_SIZE = 512  # Number of characters per chunk
    OVERLAP_SIZE = 50  # Number of characters to overlap between chunks

settings = Settings()