# Research: Docusaurus Landing Page UI Rebuild

**Date**: 2025-12-10
**Status**: In Progress

## 1. Color Palette

-   **Decision**: A dark theme with a primary accent color of electric blue/cyan, and a secondary accent color of a warmer tone like orange or purple for contrast.
-   **Rationale**: This combination evokes a futuristic, "AI" feel, while the warm accent color provides visual interest and helps to highlight key information.
-   **Alternatives considered**:
    -   A purely monochromatic dark theme (rejected as too plain).
    -   A light theme (rejected as less "AI/robotics" themed).

## 2. Animation Library

-   **Decision**: Framer Motion.
-   **Rationale**: Framer Motion is a popular and well-documented animation library for React. It is lightweight, easy to use, and has excellent Docusaurus integration examples available.
-   **Alternatives considered**:
    -   **GSAP**: More powerful, but also more complex. Overkill for the subtle animations required for this project.
    -   **CSS-only animations**: While performant, they can be more difficult to orchestrate for complex scroll-based animations.

## 3. Image Strategy

-   **Decision**: Optimize images for the web (e.g., using WebP format) and store them locally in the `assets/img` directory.
-   **Rationale**: Storing images locally ensures they are always available and avoids reliance on external services. Optimizing them for the web is crucial for performance.
-   **Alternatives considered**:
    -   **Hotlinking**: Rejected as it can lead to broken images if the source changes and is generally bad practice.
    -   **Using a CDN**: Overkill for this project, as the number of images will be relatively small.

## 4. Navbar Logo Handling

-   **Decision**: Use an SVG logo and style it with CSS to ensure it is visible in both light and dark modes. Docusaurus provides a `themeConfig.navbar.logo` option that allows specifying different logos for light and dark modes.
-   **Rationale**: This is the standard and recommended way to handle logos in Docusaurus. It is simple, effective, and ensures the logo always looks good.
-   **Alternatives considered**:
    -   Using a PNG with a transparent background (rejected as it can be difficult to make it look good on both light and dark backgrounds).
    -   Using JavaScript to swap the logo (rejected as overly complex).
