# Quickstart: Frontend-Backend Integration with FastAPI

This document outlines the steps to get the FastAPI backend running and interact with it.

## Prerequisites

-   Python 3.9+
-   uv
-   An OpenAI API key
-   A Cohere API key
-   A Qdrant Cloud API key and URL
-   A populated Qdrant collection (`reg_embedding`).

## 1. Setup

1.  **Navigate to the project root directory**:
    ```bash
    cd <project-root>
    ```

2.  **Ensure Python virtual environment is active** (if you created one with `uv venv` in the `backend` directory):
    ```bash
    .\backend\.venv\Scripts\Activate.ps1 # On Windows PowerShell
    source backend/.venv/bin/activate # On Linux/macOS Bash
    ```

3.  **Ensure dependencies are installed**:
    ```bash
    uv pip install -r backend/requirements.txt
    ```

4.  **Configure environment variables**:
    Ensure your `.env` file in the project root contains:
    ```
    OPENAI_API_KEY=your_openai_api_key
    COHERE_API_KEY=your_cohere_api_key
    QDRANT_API_KEY=your_qdrant_api_key
    QDRANT_URL=your_qdrant_url
    ```

## 2. Running the FastAPI Application

To start the FastAPI server:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://127.0.0.1:8000`.

## 3. Testing the `/chat` Endpoint

You can test the chat endpoint using `curl` or a tool like Postman/Insomnia.

### Example Query

Send a POST request to `http://127.0.0.1:8000/chat`:

```bash
Invoke-RestMethod -Uri "http://127.0.0.1:8000/chat" -Method Post -Body '{"query": "What is a URDF file?"}' -ContentType "application/json"
```

This will send a query to the agent and return its response.
