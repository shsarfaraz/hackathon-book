from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
from uuid import UUID


class QueryBase(BaseModel):
    query_text: str
    embedding_vector: Optional[List[float]] = None
    user_id: Optional[UUID] = None


class QueryCreate(QueryBase):
    pass


class QueryUpdate(BaseModel):
    pass


class QueryInDBBase(QueryBase):
    id: UUID
    created_at: datetime


class Query(QueryInDBBase):
    pass


class QueryInDB(QueryInDBBase):
    pass