# Data Model: RAG Ingestion Pipeline

**Version**: 1.0
**Status**: In Progress

This document describes the data model for the RAG ingestion pipeline.

## 1. Chunk

The primary data entity is the "chunk", which represents a piece of text that has been extracted from the book content. Each chunk will be stored as a vector in the Qdrant collection.

### 1.1. Fields

| Field     | Type   | Description                                     |
|-----------|--------|-------------------------------------------------|
| `id`      | UUID   | A unique identifier for the chunk.              |
| `content` | String | The text content of the chunk.                    |
| `vector`  | Array  | The embedding vector for the chunk.               |
| `metadata`| Object | A collection of metadata associated with the chunk.|

### 1.2. Metadata

The `metadata` object will contain the following fields:

| Field     | Type   | Description                                     |
|-----------|--------|-------------------------------------------------|
| `url`     | String | The URL of the page where the chunk was extracted.|
| `section` | String | The section of the page where the chunk was extracted.|
| `heading` | String | The heading of the section where the chunk was extracted.|
| `module`  | String | The module that the chunk belongs to.             |
| `version` | String | The version of the book content.                  |
