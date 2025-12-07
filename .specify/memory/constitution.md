<!--
---
Sync Impact Report:
Version change: 0.0.0 → 1.0.0
Modified principles:
  - PRINCIPLE_1_NAME → I. High Technical Accuracy
  - PRINCIPLE_2_NAME → II. Clear, Modular Writing
  - PRINCIPLE_3_NAME → III. Practical, Reproducible Examples
  - PRINCIPLE_4_NAME → IV. Embodied Intelligence Focus
  - PRINCIPLE_5_NAME → V. AI-Native Authoring
Added sections:
  - Key Standards
  - Book Constraints
Removed sections:
  - PRINCIPLE_6_NAME
Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
Follow-up TODOs: none
---
-->
# Physical AI & Humanoid Robotics Textbook (Docusaurus) Constitution

## Core Principles

### I. High Technical Accuracy
High technical accuracy in robotics, AI, ROS 2, Gazebo, Unity, and NVIDIA Isaac. All technical claims must be verifiable and correct.

### II. Clear, Modular Writing
Writing must be clear, concise, and structured in a modular fashion suitable for senior computer science and AI students. Complex topics should be broken down into understandable components.

### III. Practical, Reproducible Examples
All examples must be practical and reproducible on specified hardware and software (ROS 2, Jetson Orin, Isaac Sim). This ensures students can follow along and verify outcomes.

### IV. Embodied Intelligence Focus
The central theme is the connection between AI and physical robot control. Content must always emphasize how software and algorithms translate to action in the physical world.

### V. AI-Native Authoring
Content must be authored with AI systems in mind. This includes structured headings, clean formatting, and the use of tables, diagrams, and explicit code blocks to facilitate machine readability, personalization, and translation. Hallucinated or ungrounded content is strictly forbidden.

## Key Standards

- **Fact Verification:** All facts must be verified against IEEE standards, official vendor documentation, or other authoritative sources.
- **Source Requirements:** A minimum of 60% of sources must be technical, including research papers, official documentation, and established robotics standards (e.g., ROS/Isaac).
- **Terminology:** Terminology must be consistent and align with current robotics industry standards.
- **Platform Compliance:** All examples must be tested and runnable on Ubuntu 22.04, ROS 2 Humble/Iron, and designated Jetson hardware.

## Book Constraints

- **Length:** The total word count must be between 60,000 and 80,000 words.
- **References:** A minimum of 30 technical references are required.
- **Core Modules:** The book must contain four core modules: ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action models.
- **Structure:** Must include sections for hardware requirements, lab setup, a weekly plan, assessments, and a capstone project.
- **Delivery Format:** The final output must be a Docusaurus website with a clean site structure and sidebar navigation.

## Governance

Success is measured by delivering a technically correct, well-organized book where all examples are executable on real or simulated hardware. The final product must be a fully deployed Docusaurus site that is ready for RAG/chatbot integration. Amendments to this constitution require review and documentation to ensure these goals are not compromised.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06