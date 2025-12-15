from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Import with path relative to project root when running from api directory
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from api.endpoints import chat

app = FastAPI(
    title="RAG Document Query System",
    version="1.0.0"
)

# Add CORS middleware to allow frontend requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the chat API route
app.include_router(chat.router, prefix="/api", tags=["chat"])

@app.get("/")
def root():
    return {"message": "Hello FastAPI"}

# Health check endpoint
@app.get("/health")
def health_check():
    return {"status": "healthy", "message": "RAG Document Query System is running"}
