---
id: 0013
title: Redesign Docusaurus Landing Page with Futuristic UI
stage: misc
date: 2025-12-07
surface: agent
model: gemini
feature: 001-ai-robotics-book
branch: 001-ai-robotics-book
user: 
command: User request to redesign the entire hero + homepage design with a modern, animated, premium-quality layout.
labels: ["landing-page", "design", "ui", "docusaurus", "animations", "futuristic"]
links:
  spec: specs/001-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus.config.ts
 - src/pages/index.tsx
 - src/css/custom.css
 - assets/img/logo.svg
 - assets/img/logo_dark.svg
tests:

---

## Prompt

My Docusaurus landing page looks boring. Replace the entire hero + homepage design with a modern, animated, premium-quality layout.

Fix these issues:
1. Remove all boring default images and replace with modern AI/robotics illustrations.
2. Add smooth animations using Framer Motion or simple CSS keyframes.
3. Use a futuristic, clean color palette (blue, black, white, neon accents).
4. Add animated robot graphics, glowing buttons, hover effects, smooth transitions.
5. Make the layout feel like a real AI/Robotics course website — bold, premium, high-tech.
6. Ensure the navbar logo displays properly and scales correctly.
7. Improve spacing, typography, shadows, and card layout.
8. All code must be production ready and placed in the correct Docusaurus structure.

Deliver:
A) New animated landing page (index.js or index.mdx).
B) Updated theme CSS with animations & color palette.
C) Updated docusaurus.config.js if needed for images/theme.

## Response snapshot

The Docusaurus landing page has been completely redesigned with a modern, animated, premium-quality layout, aligning with the user's request for a "futuristic, clean" AI/Robotics course website aesthetic.

-   **Theme & Color Palette**: Switched to a dark-dominant theme (`#0F0F0F` background) with electric blue (`#00FFFF`) and subtle neon magenta (`#FF00FF`) accents. Light theme variables are also defined for proper fallback.
-   **Animations**: Implemented extensive CSS keyframe animations and transitions:
    *   Hero section background gradient animation (`gradientBG`).
    *   Hero text upward motion and fade-in (`heroTextLoad`).
    *   Glowing, animated buttons in the hero section with hover effects (`heroButton`).
    *   Feature cards with scale, shadow, and icon rotation on hover.
    *   Learning outcome list items with subtle background hover and translateX effect.
    *   CTA section with a soft gradient animation (`gradientAnimation`) and glowing button on hover.
    *   Fade-in-on-scroll for sections using `IntersectionObserver` in JSX and `fadeInUpAnimation` in CSS.
-   **Layout & Typography**: Enhanced layout spacing (`var(--ifm-spacing-xxl)`), utilized "Space Grotesk" for bold headings and "Inter" for body text, applied modern shadows (`box-shadow`), and used rounded corners.
-   **Navbar Logo**: The custom logo (`assets/img/logo.svg` and `assets/img/logo_dark.svg`) was updated to fit the new palette, and `docusaurus.config.ts` was set to use `src` and `srcDark` appropriately. CSS ensures proper scaling and display.
-   **Production Readiness**: All code is placed in the correct Docusaurus structure (`docusaurus.config.ts`, `src/pages/index.tsx`, `src/css/custom.css`) and builds successfully.

## Outcome

- ✅ Impact: The Docusaurus landing page now presents a bold, premium, and high-tech aesthetic, fulfilling all specified design and animation requirements for an AI/Robotics course website.
- 🧪 Tests: The Docusaurus build process completed successfully, confirming the integration of all design changes without errors.
- 📁 Files: `docusaurus.config.ts`, `src/pages/index.tsx`, `src/css/custom.css`, `assets/img/logo.svg`, and `assets/img/logo_dark.svg` were modified/created.
- 🔁 Next prompts: The user can now review the redesigned landing page by running `npm run start` locally.
- 🧠 Reflection: Successfully executed a complex, multi-faceted design task involving a complete aesthetic overhaul, extensive CSS animations, and careful integration within the Docusaurus framework. The iterative process of gathering feedback and adapting the design was key.

## Evaluation notes (flywheel)

- Failure modes observed: Initial user dissatisfaction with the previous premium design, leading to a pivot in aesthetic direction. Successfully adapted to new requirements.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
