# Data Model: AI Agent with Retrieval-Augmented Capabilities

**Version**: 1.0
**Status**: In Progress

This document describes the data model for the AI agent.

## 1. Agent

The agent is the central entity in this system. It will be created and managed using the OpenAI Agents SDK.

## 2. Tool

The agent will have a single tool: `retrieval_tool`. This tool will be a Python function that takes a query string as input and returns a list of relevant text chunks from the Qdrant collection.

## 3. Conversation History

The agent will maintain a conversation history to handle follow-up questions. The history will be managed by the OpenAI Agents SDK.
