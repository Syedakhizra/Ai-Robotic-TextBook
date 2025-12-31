import sys
import os

# Add the directory containing main.py to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import main # Now import main after sys.path is adjusted
from models import ChatRequest, ChatResponse

# Test client for FastAPI app
client = TestClient(main.app) # Use main.app directly

@pytest.fixture
def mock_agent_invocation():
    with patch('main.invoke_rag_agent') as mock_invoke_rag_agent:
        mock_invoke_rag_agent.return_value = "Mocked agent response to the query."
        yield mock_invoke_rag_agent

def test_chat_endpoint_success(mock_agent_invocation):
    query = "What is a URDF file?"
    response = client.post("/chat", json={"query": query})
    
    assert response.status_code == 200
    chat_response = ChatResponse(**response.json())
    assert chat_response.response == "Mocked agent response to the query."
    assert chat_response.error is None
    
    mock_agent_invocation.assert_called_once_with(query)

def test_chat_endpoint_internal_error(mock_agent_invocation):
    mock_agent_invocation.side_effect = Exception("Agent internal error")
    
    query = "Another test query."
    response = client.post("/chat", json={"query": query})
    
    assert response.status_code == 500
    assert "Internal server error" in response.json().get("detail", "")
    
    mock_agent_invocation.assert_called_once_with(query)