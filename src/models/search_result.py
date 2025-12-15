from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class SearchResultBase(BaseModel):
    query_id: UUID
    document_chunk_id: UUID
    similarity_score: float  # 0-1
    rank: int
    retrieved_at: Optional[datetime] = None


class SearchResultCreate(SearchResultBase):
    pass


class SearchResultUpdate(BaseModel):
    pass


class SearchResultInDBBase(SearchResultBase):
    id: UUID


class SearchResult(SearchResultInDBBase):
    pass


class SearchResultInDB(SearchResultInDBBase):
    pass