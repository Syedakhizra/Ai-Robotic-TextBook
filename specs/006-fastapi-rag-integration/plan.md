# Implementation Plan: Frontend-Backend Integration with FastAPI

**Branch**: `006-fastapi-rag-integration` | **Date**: 2025-12-10 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/006-fastapi-rag-integration/spec.md`

## Summary

This plan outlines the integration of the backend RAG system with a frontend using FastAPI. It involves setting up a FastAPI server that exposes a query endpoint, which in turn calls the RAG agent (from Spec-5 logic) and returns its response to the frontend.

## High-Level Architecture

The integration will follow this flow:

1.  **Frontend Request**: The frontend sends a user query to the FastAPI backend's `/chat` endpoint.
2.  **FastAPI Endpoint**: The FastAPI endpoint receives the query.
3.  **Agent Invocation**: The FastAPI endpoint invokes the `agent.py` script's `main` function with the user's query.
4.  **Agent Processing**: The RAG agent (using OpenAI Agents SDK) processes the query, potentially using its retrieval tool to query Qdrant.
5.  **Agent Response**: The agent returns its generated response.
6.  **FastAPI Response**: The FastAPI endpoint returns the agent's response to the frontend in JSON format.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: uv, fastapi, uvicorn, openai-agents, openai, cohere, qdrant-client, python-dotenv
**Data Source**: Qdrant collection (`reg_embedding`) populated by the ingestion pipeline.
**Storage**: Qdrant Cloud
**Testing**: Manual API calls via `curl`/`Invoke-RestMethod` and potentially automated tests.
**Target Platform**: FastAPI backend service
**Project Type**: Backend API
**Performance Goals**: API response time < 1000ms. Handle concurrent requests gracefully.
**Constraints**: Backend-only implementation for this feature. Uses existing backend folder and uv-managed environment. JSON-based request/response. Securely handle API keys.

## Constitution Check

- [X] **High Technical Accuracy**: All technical claims, specs, and examples are verifiable against primary sources.
- [X] **Clear, Modular Writing**: The plan is broken down into clear, modular components.
- [X] **Reproducible Examples**: The plan accounts for creating examples that are reproducible on the target platforms.
- [X] **Embodied Intelligence Focus**: The plan ensures the final output connects AI concepts directly to physical robot actions.
- [X] **AI-Native Authoring**: The design supports structured, machine-readable content.
- [X] **Standards Compliance**: The plan adheres to all key standards.
- [X] **Book Constraints**: The proposed work fits within the book's structural constraints.

## Project Structure

### Documentation (this feature)

```text
specs/006-fastapi-rag-integration/
├── plan.md              # This file
├── research.md          # Research findings
├── data-model.md        # Data models for API requests/responses
├── quickstart.md        # Setup and execution instructions
└── tasks.md             # Implementation tasks (to be created)
```

### Source Code (repository root)
```text
backend/main.py          # FastAPI application
agent.py                 # RAG agent script (already exists)
```

**Structure Decision**: The existing `backend/main.py` will be modified to include the FastAPI endpoint. The `agent.py` script will be imported and its main function (`agent.main`) will be called.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                 |