# Tasks for: Content Ingestion & Vectorization Pipeline

**Branch**: `003-rag-ingestion-pipeline` | **Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

This document outlines the tasks required to build the content ingestion and vectorization pipeline.

## Phase 1: Setup

- [X] T001 Install dependencies using `uv pip install -r requirements.txt` in the `backend` directory.
- [X] T002 Load environment variables from the `.env` file in the `backend` directory.

## Phase 2: Foundational

- [X] T003 Implement a function `get_all_urls` in `backend/main.py` to fetch all URLs from the sitemap.
- [X] T004 Implement a function `extract_text_from_url` in `backend/main.py` to extract the main content from a given URL.

## Phase 3: User Story 1 (Initial Data Ingestion)

- [X] T005 [US1] Implement a function `chunk_text` in `backend/main.py` to split the extracted text into semantic chunks.
- [X] T006 [US1] Implement a function `embed` in `backend/main.py` to generate embeddings for the text chunks using the Cohere API.
- [X] T007 [US1] Implement a function `create_collection` in `backend/main.py` to create a new collection in Qdrant.
- [X] T008 [US1] Implement a function `save_chunk_to_qdrant` in `backend/main.py` to save a chunk and its metadata to the Qdrant collection.
- [X] T009 [US1] Implement a `main` function in `backend/main.py` to orchestrate the entire ingestion pipeline.

## Phase 4: User Story 2 (Re-ingesting Content)

- [X] T010 [US2] Modify the `save_chunk_to_qdrant` function in `backend/main.py` to ensure idempotency.

## Phase 5: User Story 3 (Fallback to Local Markdown)

- [X] T011 [US3] Implement a fallback mechanism in `get_all_urls` in `backend/main.py` to read from the local `/docs` directory if the sitemap is unavailable.

## Final Phase: Polish & Cross-Cutting Concerns

- [X] T012 Add error handling and logging for all pipeline stages.
- [X] T013 Write unit tests for helper functions (get_all_urls, extract_text_from_url, chunk_text, embed_text).
- [X] T014 Write integration tests for the Qdrant interaction.
- [X] T015 Create a README.md in the `backend` directory explaining how to run the pipeline.

## Dependencies

-   User Story 1 must be completed before User Story 2.
-   User Story 3 can be implemented in parallel with User Stories 1 and 2.

## Implementation Strategy

The implementation will follow an MVP-first approach. The initial focus will be on building the core ingestion pipeline (User Story 1). Once the core is in place, we will focus on idempotency and the fallback mechanism.