from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
from uuid import UUID


class ResponseBase(BaseModel):
    query_id: UUID
    response_text: str
    source_chunks: List[UUID]
    confidence_score: Optional[float] = None  # 0-1


class ResponseCreate(ResponseBase):
    pass


class ResponseUpdate(BaseModel):
    pass


class ResponseInDBBase(ResponseBase):
    id: UUID
    generated_at: datetime


class Response(ResponseInDBBase):
    pass


class ResponseInDB(ResponseInDBBase):
    pass