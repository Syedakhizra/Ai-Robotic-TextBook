# Data Model: AI Robotics Textbook

## Entities

### Textbook
The primary deliverable, composed of modules, examples, diagrams, appendices.

-   **Attributes**:
    -   `title`: The title of the textbook.
    -   `modules`: A collection of Module entities.
    -   `appendices`: A collection of Appendix entities.

### Module
A self-contained learning unit with objectives, prerequisites, sections, code examples, mini-labs, and quizzes.

-   **Attributes**:
    -   `title`: The title of the module.
    -   `objectives`: A list of learning objectives.
    -   `prerequisites`: A list of prerequisites for the module.
    -   `sections`: A collection of Section entities.
    -   `examples`: A collection of Code Example entities.
    -   `mini_lab`: The mini-lab for the module.
    -   `quiz`: The quiz for the module.

### Section
A section within a module.

-   **Attributes**:
    -   `title`: The title of the section.
    -   `content`: The Markdown content of the section.

### Code Example
Runnable code snippets demonstrating concepts.

-   **Attributes**:
    -   `title`: The title of the code example.
    -   `description`: A description of the code example.
    -   `code`: The code snippet.
    -   `run_instructions`: Instructions on how to run the code.

### Diagram
Visual representations (Mermaid syntax).

-   **Attributes**:
    -   `title`: The title of the diagram.
    -   `syntax`: The Mermaid syntax for the diagram.

### Appendix
An appendix to the textbook.

-   **Attributes**:
    -   `title`: The title of the appendix.
    -   `content`: The Markdown content of the appendix.
