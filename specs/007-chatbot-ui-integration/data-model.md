# Data Model: Chatbot UI Integration into Docusaurus Site

**Version**: 1.0
**Status**: In Progress

This document describes the data model for the Chatbot UI and its interaction with the FastAPI backend.

## 1. Frontend Chat Message (Internal UI State)

Represents a single message in the chatbot's conversation history.

### 1.1. Fields

| Field   | Type   | Description                                     |
|---------|--------|-------------------------------------------------|
| `id`    | String | Unique identifier for the message.              |
| `text`  | String | The content of the message.                     |
| `sender`| String | Who sent the message ("user" or "bot").         |
| `timestamp`| Date/Time | When the message was sent.                    |
| `is_loading`| Boolean | True if the bot is currently typing a response. |

## 2. Frontend-Backend Communication

### 2.1. Request (from Frontend to FastAPI)

Corresponds to the `ChatRequest` Pydantic model in the backend.

| Field   | Type   | Description                                     |
|---------|--------|-------------------------------------------------|
| `query` | String | The user's natural language query.              |

### 2.2. Response (from FastAPI to Frontend)

Corresponds to the `ChatResponse` Pydantic model in the backend.

| Field      | Type   | Description                                     |
|------------|--------|-------------------------------------------------|
| `response` | String | The agent's generated response.                 |
| `error`    | String | Optional: An error message if an issue occurred.|
