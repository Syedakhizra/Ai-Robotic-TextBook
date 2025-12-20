# Physical AI & Humanoid Robotics Textbook + Integrated RAG Chatbot Constitution

## Core Principles

-   **AI-Spec–Driven Authorship**: All content creation is driven by detailed specifications, ensuring accurate, verifiable robotics and AI content.
-   **Seamless Integration**: The project prioritizes smooth integration between the book's static content (Docusaurus) and its interactive AI systems (RAG Chatbot).
-   **Modern Aesthetics & Usability**: A commitment to clear, simple, and visually appealing writing and UI, aligned with modern documentation and high-tech product design.
-   **Technical Correctness & Reproducibility**: All generated content, whether text or code, must maintain the highest standards of technical accuracy and reproducibility on specified hardware.

## Standards for Book Content

-   **Length**: The total word count must be between 60,000 and 80,000 words, aligned with the finalized module structure.
-   **Terminology**: All terminology must be consistent and follow established standards from ROS 2, Gazebo, NVIDIA Isaac, and the broader embodied AI literature.
-   **Citations**: IEEE-style citations are mandatory. At least 30% of sources must be academic papers and at least 30% must be official documentation.
-   **Code Examples**: All code must be runnable on the recommended hardware (workstation, Jetson) and software (Ubuntu 22.04, ROS 2 Humble/Iron).

## New Standards for RAG Chatbot System

-   **Grounded Responses**: The chatbot MUST answer questions ONLY from the book’s content or user-selected text. Hallucinations must be minimized through deterministic, grounded answers.
-   **Tech Stack**: The backend will use FastAPI, with OpenAI Agents/ChatKit SDKs for AI logic. Data will be stored in Neon Serverless Postgres (for metadata) and Qdrant Cloud Free Tier (for vector search).
-   **RAG Pipeline**: A full end-to-end RAG pipeline must be implemented: ingestion → chunking → embeddings → vector search → synthesis.
-   **Functionality**: The chatbot must support multi-turn conversation, document selection for context, and context highlighting in its responses.

## Deployment Requirements

-   **Docusaurus Site**: The static textbook site will be deployed on GitHub Pages.
-   **Chatbot UI**: The chatbot UI will be integrated as a custom React widget embedded within the Docusaurus site.
-   **Backend Hosting**: The FastAPI backend will be hosted on a free-tier-friendly service (e.g., Railway, Vercel, Render).
-   **Security**: Environment variables and secrets must be stored securely and MUST NOT be committed to the repository.

## Design Guidelines

-   **Aesthetic**: The UI must maintain a futuristic, clean, and robotic aesthetic with a consistent color palette.
-   **Animations**: The landing page will use GSAP for smooth animations, and the chatbot widget will feature smooth micro-interactions.
-   **Typography**: Sidebar font size will be optimized for readability, while the main module content remains untouched.
-   **Imagery**: High-quality AI/robot-themed images will be used throughout the site.

## Constraints

-   **No Breaking Changes**: This project must not introduce breaking changes to the existing book modules' content or structure.
-   **Scoped Chatbot Context**: The chatbot must not access any information outside the book's content unless explicitly provided by the user.
-   **Build Compatibility**: All build steps must remain compatible with Docusaurus and Vercel deployment workflows.

## Success Criteria

**Book**:
-   A fully published, visually polished Docusaurus site.
-   A functional and aesthetically pleasing theme, logo, animations, and responsive layout.

**Chatbot**:
-   A RAG chatbot is successfully embedded and functional on the site.
-   The chatbot can answer questions from the entire book with high accuracy and from selected text only.
-   All chatbot pipelines and architecture are documented within the project repository.

**Version**: 2.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
