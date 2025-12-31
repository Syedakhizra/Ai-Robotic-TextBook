# Research: Retrieval & Pipeline Validation

**Date**: 2025-12-10
**Status**: In Progress

## 1. Similarity Metric for Qdrant Search

-   **Decision**: Use Cosine Similarity as the distance metric for Qdrant vector search.
-   **Rationale**: Cosine similarity is widely used and highly effective for comparing text embeddings, especially when the magnitude of vectors is not as important as their direction (semantic similarity). This aligns with Cohere embeddings.
-   **Alternatives considered**:
    -   **Dot Product**: Also a good choice for dense embeddings, but cosine is often preferred for normalized vectors. Can be equivalent if vectors are normalized.
    -   **Euclidean Distance**: Less suitable for high-dimensional embeddings and semantic search where angular distance is often more meaningful.

## 2. Top-k Selection and Score Threshold

-   **Decision**: Implement a configurable `top_k` (e.g., 5-10) for initial retrieval and introduce an optional `score_threshold` to filter out low-confidence results.
-   **Rationale**: `top_k` allows for a configurable number of results to be passed to the LLM. A `score_threshold` helps to prevent irrelevant or noisy information from being sent to the LLM, improving response quality and reducing token usage.
-   **Alternatives considered**:
    -   **Fixed `top_k` only**: Risks including irrelevant results if `top_k` is too high, or missing relevant results if too low.
    -   **Score threshold only**: Could result in too many or too few results depending on the query and content distribution. Combining both offers flexibility.