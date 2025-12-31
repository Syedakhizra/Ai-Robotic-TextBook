# Implementation Plan: Content Ingestion & Vectorization Pipeline

**Branch**: `003-rag-ingestion-pipeline` | **Date**: 2025-12-10 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/003-rag-ingestion-pipeline/spec.md`

## Summary

This plan outlines the steps to build a content ingestion and vectorization pipeline for the Docusaurus-based textbook. The pipeline will crawl the deployed website, extract content, chunk it, generate embeddings using the Cohere API, and store the resulting vectors in a Qdrant collection.

## High-Level Architecture

The ingestion pipeline follows a linear data flow:

1.  **Crawl URLs**: Fetch all content URLs from the deployed Docusaurus site.
2.  **Extract Text**: For each URL, extract the main textual content.
3.  **Chunk Text**: Break down the extracted text into smaller, semantically meaningful chunks.
4.  **Embed Chunks**: Generate vector embeddings for each chunk using the Cohere API.
5.  **Upsert to Qdrant**: Store the vectors and their associated metadata in a Qdrant collection named `reg_embedding`.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: uv, FastAPI, requests, BeautifulSoup, cohere, qdrant-client
**Data Source**: Deployed Docusaurus Website (`https://ai-robotic-text-book.vercel.app/`)
**Sitemap URL**: `https://ai-robotic-text-book.vercel.app/sitemap.xml`
**Storage**: Qdrant Cloud
**Testing**: pytest
**Target Platform**: Backend script/service
**Project Type**: Backend
**Constraints**: Must be idempotent.

## Constitution Check

- [X] **High Technical Accuracy**: All technical claims, specs, and examples are verifiable against primary sources.
- [X] **Clear, Modular Writing**: The plan is broken down into clear, modular components.
- [X] **Reproducible Examples**: The plan accounts for creating examples that are reproducible on the target platforms.
- [ ] **Embodied Intelligence Focus**: The plan ensures the final output connects AI concepts directly to physical robot actions.
- [X] **AI-Native Authoring**: The design supports structured, machine-readable content.
- [X] **Standards Compliance**: The plan adheres to all key standards.
- [X] **Book Constraints**: The proposed work fits within the book's structural constraints.

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-ingestion-pipeline/
├── plan.md              # This file
├── research.md          # Research findings
├── data-model.md        # Data model for Qdrant
├── quickstart.md        # Setup and execution instructions
└── tasks.md             # Implementation tasks (to be created)
```

### Source Code (repository root)
```text
backend/
├── main.py
└── pyproject.toml
```

**Structure Decision**: A new `backend` directory will be created to house the Python-based ingestion pipeline. The dependencies will be managed by `uv` using a `pyproject.toml` file.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                 |