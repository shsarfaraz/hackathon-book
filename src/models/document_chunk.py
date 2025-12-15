from pydantic import BaseModel
from typing import Optional, Dict, Any, List
from datetime import datetime
from uuid import UUID


class DocumentChunkBase(BaseModel):
    document_id: UUID
    chunk_number: int
    content: str
    embedding_vector: Optional[List[float]] = None
    char_start_pos: int
    char_end_pos: int
    metadata: Optional[Dict[str, Any]] = None


class DocumentChunkCreate(DocumentChunkBase):
    pass


class DocumentChunkUpdate(BaseModel):
    embedding_vector: Optional[List[float]] = None


class DocumentChunkInDBBase(DocumentChunkBase):
    id: UUID


class DocumentChunk(DocumentChunkInDBBase):
    pass


class DocumentChunkInDB(DocumentChunkInDBBase):
    pass