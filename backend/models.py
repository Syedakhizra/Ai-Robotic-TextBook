from pydantic import BaseModel
from typing import List, Dict, Any, Optional

class QueryRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5
    score_threshold: Optional[float] = 0.7

class RetrievalResult(BaseModel):
    text: str
    score: float
    metadata: Dict[str, Any]

class ChatRequest(BaseModel):
    query: str

class ChatResponse(BaseModel):
    response: str
    error: Optional[str] = None