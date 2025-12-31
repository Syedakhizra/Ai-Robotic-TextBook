# Quickstart: Retrieval & Pipeline Validation

This document outlines the steps to run the retrieval and pipeline validation script.

## Prerequisites

-   Python 3.9+
-   uv
-   A Cohere API key
-   A Qdrant Cloud API key and URL
-   A running Qdrant instance with the `reg_embedding` collection populated (from the ingestion pipeline).

## 1. Setup

1.  **Navigate to the project root directory**:
    ```bash
    cd <project-root>
    ```

2.  **Ensure Python virtual environment is active** (if you created one with `uv venv`):
    ```bash
    .venv/Scripts/activate # On Windows PowerShell
    source .venv/bin/activate # On Linux/macOS Bash
    ```

3.  **Ensure dependencies are installed**:
    ```bash
    uv pip install -r backend/requirements.txt
    ```

4.  **Configure environment variables**:
    Ensure your `.env` file in the project root contains:
    ```
    COHERE_API_KEY=your_cohere_api_key
    QDRANT_API_KEY=your_qdrant_api_key
    QDRANT_URL=your_qdrant_url
    ```

## 2. Running the Retrieval Script

To run the retrieval script, execute the `retrieve.py` script with your query:

```bash
python retrieve.py "What is ROS 2?"
```

You can also specify the `top_k` and `score_threshold` arguments:

```bash
python retrieve.py "What is a URDF file?" --top_k 3 --score_threshold 0.8
```

The script will output the retrieved results to the console.