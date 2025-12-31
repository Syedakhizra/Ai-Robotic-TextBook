# Quickstart: RAG Ingestion Pipeline

This document outlines the steps to get the RAG ingestion pipeline running locally.

## Prerequisites

-   Python 3.9+
-   uv
-   A Cohere API key
-   A Qdrant Cloud API key and URL

## 1. Setup

1.  **Install dependencies**:
    ```bash
    uv pip install -r requirements.txt
    ```
2.  **Set up environment variables**:
    Create a `.env` file in the `backend` directory with the following content:
    ```
    COHERE_API_KEY=your_cohere_api_key
    QDRANT_API_KEY=your_qdrant_api_key
    QDRANT_URL=your_qdrant_url
    ```

## 2. Running the Ingestion Pipeline

To run the ingestion pipeline, execute the `main.py` script from within the `backend` directory:
```bash
python main.py
```
