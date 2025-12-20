# Tasks for: Docusaurus Landing Page UI Rebuild

**Branch**: `002-recreate-landing-page-ui` | **Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

This document outlines the tasks required to rebuild the Docusaurus landing page UI.

## Phase 1: Setup

- [X] T001 Install Framer Motion dependency
- [X] T002 Create component files for landing page sections:
    - `src/components/HeroSection.tsx`
    - `src/components/FeaturesSection.tsx`
    - `src/components/ModulesSection.tsx`
    - `src/components/CtaSection.tsx`

## Phase 2: Foundational

- [X] T003 Update `docusaurus.config.ts` to ensure the navbar logo is visible and correct in both light and dark modes.
- [X] T004 Update `src/css/custom.css` with the new color palette and typography styles.

## Phase 3: User Story 1 (First-Time Visitor)

- [X] T005 [US1] Implement the `HeroSection` component in `src/components/HeroSection.tsx`.
- [X] T006 [US1] Implement the `CtaSection` component in `src/components/CtaSection.tsx`.
- [X] T007 [US1] Integrate the `HeroSection` and `CtaSection` components into `src/pages/index.tsx`.

## Phase 4: User Story 2 (Returning Visitor)

- [X] T008 [US2] Implement the `FeaturesSection` component in `src/components/FeaturesSection.tsx`.
- [X] T009 [US2] Implement the `ModulesSection` component in `src/components/ModulesSection.tsx`.
- [X] T010 [US2] Integrate the `FeaturesSection` and `ModulesSection` components into `src/pages/index.tsx`.

## Phase 5: User Story 3 (Mobile User)

- [X] T011 [US3] Ensure all landing page components are fully responsive across desktop, tablet, and mobile devices.

## Final Phase: Polish & Cross-Cutting Concerns

- [X] T012 Add subtle animations to the landing page sections using Framer Motion.
- [X] T013 Optimize all images used on the landing page for web performance.
- [X] T014 Perform a final visual QA of the landing page in both light and dark modes to ensure consistency and quality.

## Dependencies

-   User Story 1 can be implemented in parallel with User Story 2.
-   User Story 3 depends on the completion of User Stories 1 and 2.
-   The Final Phase depends on the completion of all user stories.

## Parallel Execution

-   The `HeroSection` and `CtaSection` can be developed in parallel.
-   The `FeaturesSection` and `ModulesSection` can be developed in parallel.

## Implementation Strategy

The implementation will follow an MVP-first approach. The initial focus will be on building the core structure and functionality of the landing page (User Stories 1 and 2). Once the core is in place, we will focus on responsive design and polish.
