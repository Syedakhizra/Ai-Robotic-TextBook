# Architecture Sketch: AI Robotics Textbook + Docusaurus Repository

## Overall System Architecture

The AI Robotics Textbook will be delivered as a static website built using Docusaurus, hosted on GitHub Pages. The repository structure is designed for modularity, maintainability, and easy content generation and updates.

```mermaid
graph TD
    subgraph Authoring Workflow
        A[Markdown Content + Code Examples] --> B(Authoring Tools: VS Code, Git)
        B --> C{Version Control: Git Repository}
    end

    subgraph Docusaurus Site Generation
        C --> D[Docs Directory: docs/]
        C --> E[Examples Directory: examples/]
        C --> F[Assets Directory: assets/]
        D -- Docusaurus Content --> G[Docusaurus Build System]
        E -- Code Execution/Verification --> G
        F -- Diagrams/Images --> G
        G -- Static HTML, CSS, JS --> H[Generated Site: build/]
    end

    subgraph Deployment & Access
        H --> I[GitHub Pages Hosting]
        I --> J[End User: Web Browser]
    end

    subgraph Metadata & Planning
        K[Specification: specs/001-ai-robotics-book/spec.md] --> L[Implementation Plan: specs/001-ai-robotics-book/plan.md]
        L --> C
        L -- Research Findings --> M[Research: specs/001-ai-robotics-book/research.md]
        L -- Data Model --> N[Data Model: specs/001-ai-robotics-book/data-model.md]
        K --> M
    end

    subgraph Environment & Tooling
        O[Ubuntu 22.04 LTS]
        P[ROS 2 Humble/Iron]
        Q[Gazebo / Unity 2022.3 LTS]
        R[NVIDIA Isaac Sim 5.1.0+]
        S[Jetson Orin Nano/NX]
        O --- P & Q & R & S
    end

    C -- Config --> T[docusaurus.config.js]
    C -- Navigation --> U[sidebars.js]
    C -- Quality Checks --> V[checklists/requirements.md]
    C -- Scripts --> W[.specify/scripts/]
```

## Component Breakdown

### 1. Content Modules (`docs/`)
Each core module (ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action) and appendices will reside in its own subdirectory within `docs/`. Each module will have an `index.md` and subsequent `.md` files for sections.

### 2. Code Examples (`examples/`)
Runnable code examples will be stored in a parallel directory structure to `docs/`. Each example will be a self-contained Python script (primarily `rclpy`) designed to be executed on the specified hardware/software stack.

### 3. Assets (`assets/`)
This directory will contain all static assets like images, diagrams (generated from Mermaid syntax), and other media.

### 4. Docusaurus Configuration
*   `docusaurus.config.js`: Main configuration for the site, including title, tagline, plugins, and themes.
*   `sidebars.js`: Defines the navigation structure for the documentation, ensuring a logical learning progression.

### 5. Specifications and Planning (`specs/`)
Dedicated directory for all project management artifacts: `spec.md`, `plan.md`, `research.md`, `data-model.md`, and checklists.

### 6. Environment & Tooling
The development and execution environment is standardized on Ubuntu 22.04 with ROS 2 Humble/Iron. Simulation environments include Gazebo, Unity 2022.3 LTS, and NVIDIA Isaac Sim 5.1.0+. Jetson Orin Nano/NX will serve as the deployment target for physical robot control examples.
