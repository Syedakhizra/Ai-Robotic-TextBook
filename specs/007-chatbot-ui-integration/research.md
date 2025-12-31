# Research: Integrate Chatbot UI into Docusaurus Site

**Date**: 2025-12-10
**Status**: In Progress

## 1. Chatbot UI Component Strategy

-   **Decision**: Create a custom React component for the chatbot UI. This component will be mounted as a collapsible widget on all Docusaurus pages.
-   **Rationale**: Docusaurus is built on React, making a custom React component the most natural and performant way to integrate new UI. A collapsible widget ensures the chatbot is accessible but non-intrusive.
-   **Alternatives considered**:
    -   **iframe**: Can be used to embed external web content, but limits direct interaction with Docusaurus styling and context.
    -   **Pure JavaScript/HTML**: Less idiomatic for Docusaurus and harder to maintain with React components.

## 2. Global Component Integration in Docusaurus

-   **Decision**: Use Docusaurus's `src/theme/Layout/index.js` (or `index.tsx`) to render the Chatbot UI component on all pages. This allows for a persistent UI element across the site.
-   **Rationale**: Docusaurus's theming system provides a way to inject components into the global layout, ensuring the chatbot is present on every page without modifying individual markdown files.
-   **Alternatives considered**:
    -   **Swizzling individual page components**: Too granular and would require modifications for every new page.

## 3. Communication with FastAPI Backend

-   **Decision**: Use `fetch` API (or `axios` if preferred) within the React component to send user queries to the FastAPI `/chat` endpoint and receive responses.
-   **Rationale**: Standard web API communication is reliable and well-supported by all modern browsers.
-   **Alternatives considered**:
    -   **WebSockets**: Overkill for initial query/response, but could be considered for future real-time interaction features.

## 4. Conversation History Management

-   **Decision**: Implement conversation history directly within the React component's state (using `useState` and `useEffect` hooks) for multi-turn conversations.
-   **Rationale**: For client-side-only history, React state management is appropriate and straightforward.
-   **Alternatives considered**:
    -   **External state management library (e.g., Redux, Zustand)**: Overkill for a single chatbot component's state, unless the history needs to be shared across many components or persisted beyond browser sessions.

## 5. Styling and Responsiveness

-   **Decision**: Use CSS Modules for component-specific styling and media queries for responsiveness. Leverage Docusaurus's theming variables where possible.
-   **Rationale**: CSS Modules provide scoped styles, preventing conflicts. Media queries ensure a good user experience on various devices.
-   **Alternatives considered**:
    -   **Inline styles**: Harder to maintain and less performant.
    -   **Styled Components/Emotion**: More powerful but adds another dependency layer.
