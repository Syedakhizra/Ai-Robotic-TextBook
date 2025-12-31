# Quickstart: Integrate Chatbot UI into Docusaurus Site

This document outlines the steps to get the Chatbot UI running and integrated into your Docusaurus site.

## Prerequisites

-   A running FastAPI backend with the `/chat` endpoint exposed (from previous features).
-   Node.js and npm/yarn installed for Docusaurus development.

## 1. Setup

1.  **Navigate to the project root directory**:
    ```bash
    cd <project-root>
    ```

2.  **Ensure Docusaurus dependencies are installed**:
    ```bash
    npm install
    ```

## 2. Running the Docusaurus Site with Chatbot UI

1.  **Start the FastAPI backend server** in a separate terminal (from the `backend` directory):
    ```bash
    uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000
    ```

2.  **Start the Docusaurus development server** (from the project root directory):
    ```bash
    npm start
    ```

3.  **Open your browser** to `http://localhost:3000`. You should see the Docusaurus site with the chatbot widget integrated.

## 3. Interacting with the Chatbot

1.  Click on the chatbot widget to expand it.
2.  Type a query into the input field and press Enter.
3.  Observe the chatbot sending the query to the FastAPI backend and displaying the response.
