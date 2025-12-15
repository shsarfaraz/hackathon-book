from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID


class DocumentBase(BaseModel):
    filename: str
    size_bytes: int
    file_type: str
    page_count: Optional[int] = None
    status: str = "PENDING"  # PENDING, PROCESSING, COMPLETE, FAILED
    metadata: Optional[Dict[str, Any]] = None


class DocumentCreate(DocumentBase):
    pass


class DocumentUpdate(BaseModel):
    status: Optional[str] = None


class DocumentInDBBase(DocumentBase):
    id: UUID
    upload_date: datetime


class Document(DocumentInDBBase):
    pass


class DocumentInDB(DocumentInDBBase):
    pass