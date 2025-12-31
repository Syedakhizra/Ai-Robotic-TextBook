# Data Model: Frontend-Backend Integration with FastAPI

**Version**: 1.0
**Status**: In Progress

This document describes the data model for the FastAPI integration with the RAG agent.

## 1. Chat Request

The API will accept a POST request to `/chat` with a JSON payload:

### 1.1. Fields

| Field   | Type   | Description                                     |
|---------|--------|-------------------------------------------------|
| `query` | String | The natural language query from the frontend.   |

## 2. Chat Response

The API will return a JSON payload containing the agent's response:

### 2.1. Fields

| Field      | Type   | Description                                     |
|------------|--------|-------------------------------------------------|
| `response` | String | The agent's generated response to the query.    |
| `error`    | String | Optional: An error message if an issue occurred.|
