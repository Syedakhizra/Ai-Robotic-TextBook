# Quickstart: AI Agent with Retrieval-Augmented Capabilities

This document outlines the steps to run the AI agent locally.

## Prerequisites

-   Python 3.9+
-   uv
-   An OpenAI API key
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

3.  **Install dependencies**:
    ```bash
    uv pip install -r backend/requirements.txt
    uv pip install openai
    ```

4.  **Configure environment variables**:
    Ensure your `.env` file in the project root contains:
    ```
    OPENAI_API_KEY=your_openai_api_key
    COHERE_API_KEY=your_cohere_api_key
    QDRANT_API_KEY=your_qdrant_api_key
    QDRANT_URL=your_qdrant_url
    ```

## 2. Running the Agent

To run the agent, execute the `agent.py` script:

```bash
python agent.py "What is a URDF file?"
```

The script will output the agent's response to the console.
