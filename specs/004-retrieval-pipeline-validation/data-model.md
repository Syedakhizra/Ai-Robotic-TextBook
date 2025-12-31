# Data Model: Retrieval & Pipeline Validation

**Version**: 1.0
**Status**: In Progress

This document describes the data model for the Retrieval & Pipeline Validation script.

## 1. Query Request (CLI Arguments)

The script will accept the following command-line arguments:

| Argument        | Type   | Description                                     |
|-----------------|--------|-------------------------------------------------|
| `query`         | String | The natural language query from the user.       |
| `--top_k`       | Integer| The number of top relevant results to retrieve. (Optional, default 5) |
| `--score_threshold` | Float | The minimum relevance score for retrieved results. (Optional, default 0.7) |

## 2. Retrieval Result (Console Output)

The script will output the retrieval results to the console, with each result having the following structure:

| Field       | Type   | Description                                     |
|-------------|--------|-------------------------------------------------|
| `text`      | String | The full text content of the retrieved chunk.   |
| `score`     | Float  | The relevance score of the chunk to the query.  |
| `metadata`  | Object | The metadata associated with the chunk (source URL, section, heading).|