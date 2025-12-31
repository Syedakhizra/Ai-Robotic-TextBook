# Quickstart: Frontend UI/UX Refinement for Docusaurus Site

This document outlines how to view and verify the UI/UX refinements on the Docusaurus site.

## Prerequisites

-   A functional Docusaurus site setup.
-   (Optional, for chatbot testing) A running FastAPI backend with the `/chat` endpoint exposed.

## 1. Setup

1.  **Navigate to the project root directory**:
    ```bash
    cd <project-root>
    ```

2.  **Ensure Docusaurus dependencies are installed**:
    ```bash
    npm install
    ```

## 2. Running the Docusaurus Site

1.  **Start the Docusaurus development server**:
    ```bash
    npm start
    ```

2.  **Open your browser** to `http://localhost:3000`.

## 3. Verifying UI/UX Changes

**For Chatbot UI (Light Mode):**
-   Switch your Docusaurus site to Light Mode (usually via a toggle in the navbar).
-   Open the chatbot widget.
-   Verify that chat bubbles, input field, and buttons have clear contrast and are easily readable.

**For Hero Section Responsiveness:**
-   Resize your browser window to simulate mobile and tablet viewports.
-   Verify that text scales appropriately, elements align correctly, and CTA buttons stack and space well.

**For Landing Page Visual Polish:**
-   Observe the overall look and feel of the landing page.
-   Check typography, spacing between sections, and visual balance.

**For Dark Mode Compatibility:**
-   Switch your Docusaurus site to Dark Mode.
-   Verify that all changes still look good and maintain their intended aesthetic.
