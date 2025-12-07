# Tasks: Modern Landing Page UI Redesign

**Input**: Design documents from `specs/001-modern-landing-page-ui/`
**Prerequisites**: plan.md, spec.md

**Tests**: This project implicitly requires visual testing for responsiveness, animation smoothness, and overall aesthetic.

**Organization**: Tasks are grouped by component and phase to ensure a structured implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Install GSAP library: `npm install gsap` in root.
- [X] T002 [P] Configure `docusaurus.config.ts` for dark theme as default.
- [X] T003 [P] Update `src/css/custom.css` with the new futuristic dark color palette and typography settings (Space Grotesk, Inter).
- [X] T004 [P] Update `assets/img/logo.svg` and `assets/img/logo_dark.svg` to fit the new futuristic theme.
- [X] T005 [P] Source and add at least 5 high-quality AI/robot themed images to `assets/img/features/`.
- [X] T006 [P] Update `src/pages/index.tsx` to include basic GSAP setup and React hooks for animations.

---

## Phase 2: Core Components Redesign

**Purpose**: Implement the new design for the core components of the landing page.

### User Story 2: Clear Navigation and Branding

- [X] T007 [P] [US2] Update `src/css/custom.css` with new navbar styling (dark theme, responsive behavior).
- [X] T008 [P] [US2] Implement CSS to ensure the navbar logo scales correctly without distortion and is visible on light/dark themes.
- [X] T009 [P] [US2] Implement CSS for smaller, cleaner sidebar typography.
- [X] T010 [US2] Manually test navbar and sidebar responsiveness on various screen sizes.

### User Story 1: Engaging First Impression

- [X] T011 [P] [US1] Implement new Hero section layout in `src/pages/index.tsx`.
- [X] T012 [P] [US1] Implement CSS for Hero section's futuristic typography, spacing, and background in `src/css/custom.css`.
- [X] T013 [P] [US1] Implement GSAP animation for hero text (slide-up/fade-in on load) in `src/pages/index.tsx`.
- [X] T014 [P] [US1] Implement CSS for glowing button effect and smooth hover animations in `src/css/custom.css`.

### User Story 3: Visually Rich Content Presentation

- [X] T015 [P] [US3] Redesign Key Features section layout in `src/pages/index.tsx` as a grid of cards.
- [X] T016 [P] [US3] Implement CSS for futuristic feature card styling (shadows, spacing) in `src/css/custom.css`.
- [X] T017 [P] [US3] Implement GSAP animations for feature cards (fade-in, slide-up, zoom on scroll) in `src/pages/index.tsx`.
- [X] T018 [P] [US3] Replace default images in Key Features section with the new AI/robot images.

---

## Phase 3: Final Touches & Polish

**Purpose**: Apply final styling, animations, and perform quality checks.

- [X] T019 [P] Apply final styling to all sections for consistent spacing, typography, and color usage in `src/css/custom.css`.
- [X] T020 [P] Implement smooth scroll-to-section transitions.
- [X] T021 [P] Implement CTA section soft gradient animation in `src/css/custom.css`.
- [X] T022 [P] Final review of mobile and tablet responsiveness across all components.
- [X] T023 [P] Perform visual checks for animation smoothness and performance (FPS check).
- [X] T024 [P] Validate accessibility (contrast, readable fonts).
- [X] T025 [P] Final code cleanup and removal of unused CSS/JSX.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** should be completed before Phase 2.
- Within **Phase 2 (Core Components Redesign)**, sub-phases for each user story can be worked on in parallel.
- **Phase 3 (Final Touches)** depends on the completion of Phase 2.

### Within Each Component

- Layout (JSX) should be implemented before styling (CSS).
- Styling should be implemented before animations (GSAP).
- Testing should be performed after each component is complete.

---

## Implementation Strategy

### Incremental Delivery (Recommended)

1.  Complete Phase 1: Setup.
2.  Implement User Story 2: Navbar and Sidebar styling.
3.  Implement User Story 1: Hero section redesign and animations.
4.  Implement User Story 3: Key Features section redesign and animations.
5.  Complete Phase 3: Final polish and quality checks.
6.  Regularly build and preview the Docusaurus site locally (`npm run start`) to catch issues early.

### Parallel Team Strategy

-   **Developer A**: Focus on `docusaurus.config.ts` and `src/css/custom.css` for theme, color palette, and global styling.
-   **Developer B**: Focus on `src/pages/index.tsx` for layout and GSAP animation integration.
-   **Designer/QA**: Focus on sourcing images and performing visual/responsive testing.

---

## Notes

- Tasks are designed to be specific and actionable.
- Clear file paths are provided for all tasks.
- GSAP is the required animation library.
- The Module Overview section should not be modified.
