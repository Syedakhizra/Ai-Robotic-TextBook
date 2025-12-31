# Tasks for: Integrate Chatbot UI into Docusaurus Site

**Branch**: `007-chatbot-ui-integration` | **Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

This document outlines the tasks required to integrate a chatbot UI into the Docusaurus site.

## Phase 1: Setup

- [X] T001 Create `src/components/ChatbotWidget/` directory.
- [X] T002 Create `src/components/ChatbotWidget/ChatbotWidget.tsx`.
- [X] T003 Create `src/components/ChatbotWidget/ChatbotWidget.module.css`.
- [X] T004 Create `src/components/ChatbotWidget/ChatMessage.tsx`.

## Phase 2: Foundational

- [X] T005 Implement basic structure and styling for `ChatMessage.tsx` (display message text, sender).
- [X] T006 Implement basic structure for `ChatbotWidget.tsx` (input field, send button, message display area).
- [X] T007 Implement basic styling for `ChatbotWidget.module.css` (collapsible behavior, responsive design).

## Phase 3: User Story 1 (User interacts with the chatbot)

- [X] T008 [US1] Integrate `ChatbotWidget` into `src/theme/Layout/index.tsx` to display it on all pages.
- [X] T009 [US1] Implement state management in `ChatbotWidget.tsx` for `conversation history` and `user input`.
- [X] T010 [US1] Implement functionality to send user input from `ChatbotWidget.tsx` to the FastAPI backend's `/chat` endpoint using `fetch`.
- [X] T011 [US1] Implement functionality to display responses from the FastAPI backend in `ChatbotWidget.tsx`.

## Phase 4: User Story 2 (Multi-turn conversation)

- [X] T012 [US2] Enhance `ChatbotWidget.tsx` to maintain and display full conversation history.

## Phase 5: User Story 3 (Chatbot is non-intrusive)

- [X] T013 [US3] Implement collapsible UI for `ChatbotWidget.tsx` (e.g., a floating button to open/close the chatbot).

## Final Phase: Polish & Cross-Cutting Concerns

- [X] T014 Implement a loading indicator in `ChatbotWidget.tsx` while waiting for the FastAPI response.
- [X] T015 Ensure accessibility (ARIA attributes) for the chatbot UI in `ChatbotWidget.tsx`.
- [X] T016 Update documentation for the new chatbot UI in the Docusaurus site, if necessary.

## Dependencies

-   Phase 1 and 2 tasks are prerequisites for Phase 3.
-   T010 and T011 depend on the FastAPI backend being operational.
-   T012 builds upon the conversation history functionality from T009.
-   T013 builds upon the basic styling from T007.

## Implementation Strategy

The implementation will follow a component-driven approach, starting with basic UI elements and progressively adding functionality. The core communication with the backend will be established early, followed by multi-turn conversation and UI refinements.
