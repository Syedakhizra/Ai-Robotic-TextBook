# RAG Ingestion and Retrieval Pipeline

This directory contains the Python-based RAG (Retrieval-Augmented Generation) pipeline for the Physical AI & Humanoid Robotics Textbook. This includes:

1.  **Ingestion Pipeline**: A CLI command to crawl content, generate embeddings, and store them in Qdrant.
2.  **Retrieval API**: A FastAPI endpoint to perform semantic search on the ingested content.
3.  **Chat API**: A FastAPI endpoint to interact with the RAG agent.

## Setup

1.  **Navigate to the `backend` directory**:
    ```bash
    cd backend
    ```

2.  **Create a Python virtual environment** (if you haven't already):
    ```bash
    uv venv
    ```

3.  **Install dependencies**:
    ```bash
    uv pip install -r requirements.txt
    ```

4.  **Configure environment variables**:
    Create a `.env` file in the `backend` directory with your API keys and Qdrant URL:
    ```
    OPENAI_API_KEY=your_openai_api_key_here
    COHERE_API_KEY=your_cohere_api_key_here
    QDRANT_API_KEY=your_qdrant_api_key_here
    QDRANT_URL=your_qdrant_url_here
    ```
    Replace `your_openai_api_key_here`, `your_cohere_api_key_here`, `your_qdrant_api_key_here`, and `your_qdrant_url_here` with your actual credentials.

## Running the Ingestion Pipeline

To run the ingestion pipeline, execute the `ingest` command:

```bash
uv run python main.py ingest
```

The pipeline will:
-   Attempt to fetch URLs from the configured sitemap URL.
-   If sitemap fetching fails, it will fall back to reading local markdown files from the `docs/` directory of the project root.
-   Create (or recreate during development) the Qdrant collection named `reg_embedding`.
-   Ingest the processed chunks and their embeddings into Qdrant.

## Running the Retrieval and Chat API

To start the FastAPI server:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://127.0.0.1:8000`.

### Testing the Retrieval Endpoint

Send a POST request to `http://127.0.0.1:8000/retrieve`:

```bash
Invoke-RestMethod -Uri "http://127.0.0.1:8000/retrieve" -Method Post -Body '{"query": "What is a URDF file?", "top_k": 3}' -ContentType "application/json"
```

### Testing the Chat Endpoint

Send a POST request to `http://127.0.0.1:8000/chat`:

```bash
Invoke-RestMethod -Uri "http://127.0.0.1:8000/chat" -Method Post -Body '{"query": "What is ROS 2?"}' -ContentType "application/json"
```

## Testing

To run the unit and integration tests:

```bash
uv run pytest backend/tests/
```
