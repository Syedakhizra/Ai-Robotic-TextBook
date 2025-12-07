# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `specs/001-ai-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/ (N/A for this project)

**Tests**: This project implicitly requires code testing and content validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume Docusaurus project structure from `plan.md`.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Initialize Docusaurus project structure with `npx create-docusaurus@latest website classic --typescript` in root.
- [X] T002 Configure `docusaurus.config.js` with project metadata (title, tagline, baseUrl, GitHub Pages settings).
- [X] T003 Configure `sidebars.js` for initial empty module structure and appendices.
- [X] T004 Create base `docs/` directory and `docs/README.md`.
- [X] T005 Create `examples/` directory for code examples.
- [X] T006 Create `assets/` directory and `assets/diagrams/` for visual assets.
- [X] T007 [P] Create `specs/001-ai-robotics-book/architecture-sketch.md` from plan.
- [X] T008 [P] Create `specs/001-ai-robotics-book/module-structure.md` from plan.
- [X] T009 [P] Create `specs/001-ai-robotics-book/quickstart.md` from plan.
- [X] T010 [P] Create `specs/001-ai-robotics-book/research.md` (research plan) from plan.
- [X] T011 [P] Create `specs/001-ai-robotics-book/data-model.md` from plan.
- [X] T012 [P] Configure linting and formatting tools for Markdown and Python code.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure and content that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T013 Document hardware requirements in `docs/appendices/hardware.md`.
- [X] T014 Write initial install guides for Ubuntu/ROS2 in `docs/appendices/install-guides.md`.
- [X] T015 Define glossary of key terms in `docs/appendices/glossary.md`.
- [X] T016 Write initial Preface/Motivation content in `docs/preface.md`.
- [X] T017 Set up automated content clarity checks using Docusaurus lint rules.
- [X] T018 Define process for code verification on Ubuntu + ROS2 and simulation reproducibility tests.
- [X] T019 Establish process for accuracy checks against official docs + academic papers.
- [X] T020 Implement citation format enforcement (IEEE) guidelines.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learning ROS 2 Foundations (Priority: P1) 🎯 MVP

**Goal**: Students understand core ROS 2 concepts and can execute basic `rclpy` examples.

**Independent Test**: Student successfully executes all provided ROS 2 examples in `examples/module-1/` and passes the Module 1 quiz.

### Implementation for User Story 1

- [X] T021 [P] [US1] Create `docs/module-1/index.md` for Module 1 overview and objectives.
- [X] T022 [P] [US1] Write "Introduction to ROS 2" section in `docs/module-1/introduction.md`.
- [X] T023 [P] [US1] Write "ROS 2 Concepts: Nodes, Topics, Publishers, Subscribers" section in `docs/module-1/concepts.md`.
- [X] T024 [P] [US1] Develop "Simple Publisher/Subscriber" `rclpy` example in `examples/module-1/simple_pub_sub.py`.
- [X] T025 [US1] Verify "Simple Publisher/Subscriber" example on Ubuntu 22.04 + ROS 2 Humble/Iron.
- [X] T026 [P] [US1] Write "ROS 2 Services and Actions" section in `docs/module-1/services_actions.md`.
- [X] T027 [P] [US1] Develop "ROS 2 Service client/server" example in `examples/module-1/simple_service.py`.
- [X] T028 [US1] Verify "ROS 2 Service client/server" example.
- [X] T029 [P] [US1] Write "Packages and Workspaces" section in `docs/module-1/packages_workspaces.md`.
- [X] T030 [P] [US1] Write "Launch Files" section in `docs/module-1/launch_files.md`.
- [X] T031 [P] [US1] Write "URDF Basics" section in `docs/module-1/urdf_basics.md`.
- [X] T032 [P] [US1] Develop "Displaying a simple URDF model" example in `examples/module-1/display_urdf.py`.
- [X] T033 [US1] Verify "Displaying a simple URDF model" example.
- [X] T034 [P] [US1] Create Mini-Lab 1: "Build a multi-node ROS 2 system" in `docs/module-1/mini_lab_1.md`.
- [X] T035 [P] [US1] Create Quiz 1: "ROS 2 Foundations assessment" in `docs/module-1/quiz_1.md`.
- [X] T036 [US1] Integrate Module 1 into `sidebars.js`.
- [X] T037 [US1] Fact-check all technical claims and citations in Module 1 against verified sources.

**Checkpoint**: User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Building Digital Twin Simulation (Priority: P1)

**Goal**: Students set up Gazebo/Unity digital twins and simulate sensors.

**Independent Test**: Student successfully sets up a Gazebo/Unity environment and simulates basic sensor data, correctly converting URDF to SDF.

### Implementation for User Story 2

- [X] T038 [P] [US2] Create `docs/module-2/index.md` for Module 2 overview and objectives.
- [X] T039 [P] [US2] Write "Introduction to Robotics Simulation" section in `docs/module-2/introduction.md`.
- [X] T040 [P] [US2] Write "Gazebo Setup and Physics" section in `docs/module-2/gazebo_setup.md`.
- [X] T041 [P] [US2] Develop "Simulating a simple pendulum in Gazebo" example in `examples/module-2/simple_pendulum.sdf`.
- [X] T042 [US2] Verify "Simulating a simple pendulum in Gazebo" example.
- [X] T043 [P] [US2] Write "Simulating Sensors" section (LiDAR, Depth, IMU) in `docs/module-2/simulating_sensors.md`.
- [X] T044 [P] [US2] Develop "Reading simulated LiDAR data in ROS 2" example in `examples/module-2/read_lidar.py`.
- [X] T045 [US2] Verify "Reading simulated LiDAR data in ROS 2" example.
- [X] T046 [P] [US2] Write "URDF and SDF Pipeline" section in `docs/module-2/urdf_sdf_pipeline.md`.
- [X] T047 [P] [US2] Write "Unity for HRI" section in `docs/module-2/unity_hri.md`.
- [X] T048 [P] [US2] Develop "Basic Unity scene with a robot and user input" example in `examples/module-2/unity_robot_input.unitypackage`.
- [X] T049 [US2] Verify "Basic Unity scene with a robot and user input" example.
- [X] T050 [P] [US2] Create Mini-Lab 2: "Simulate a mobile robot with a depth camera" in `docs/module-2/mini_lab_2.md`.
- [X] T051 [P] [US2] Create Quiz 2: "Digital Twin Simulation assessment" in `docs/module-2/quiz_2.md`.
- [X] T052 [US2] Integrate Module 2 into `sidebars.js`.
- [X] T053 [US2] Fact-check all technical claims and citations in Module 2.

**Checkpoint**: User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Exploring NVIDIA Isaac AI Stack (Priority: P2)

**Goal**: Students utilize Isaac Sim for advanced AI robotics, including perception, SLAM, and Jetson deployment.

**Independent Test**: Student successfully runs Isaac Sim examples, processes synthetic data, and deploys a basic application to a Jetson device.

### Implementation for User Story 3

- [X] T054 [P] [US3] Create `docs/module-3/index.md` for Module 3 overview and objectives.
- [X] T055 [P] [US3] Write "Introduction to NVIDIA Isaac Sim" section in `docs/module-3/isaac_sim_intro.md`.
- [X] T056 [P] [US3] Write "USD Workflow: Universal Scene Description" section in `docs/module-3/usd_workflow.md`.
- [X] T057 [P] [US3] Write "Isaac ROS Perception: Accelerated Perception Modules" section in `docs/module-3/isaac_ros_perception.md`.
- [X] T058 [P] [US3] Develop "Running Isaac ROS NITROS Stereo Depth" example in `examples/module-3/nitros_stereo_depth.py`.
- [X] T059 [US3] Verify "Running Isaac ROS NITROS Stereo Depth" example.
- [X] T060 [P] [US3] Write "Isaac ROS SLAM: Simultaneous Localization and Mapping" section in `docs/module-3/isaac_ros_slam.md`.
- [X] T061 [P] [US3] Write "Synthetic Data Generation" section in `docs/module-3/synthetic_data.md`.
- [X] T062 [P] [US3] Develop "Domain randomization for object pose estimation" example in `examples/module-3/domain_randomization.py`.
- [X] T063 [US3] Verify "Domain randomization for object pose estimation" example.
- [X] T064 [P] [US3] Write "Jetson Deployment Basics" section in `docs/module-3/jetson_deployment.md`.
- [X] T065 [P] [US3] Develop "Deploying a simple ROS 2 node to Jetson Orin" example in `examples/module-3/jetson_ros2_deploy.py`.
- [X] T066 [US3] Verify "Deploying a simple ROS 2 node to Jetson Orin" example.
- [X] T067 [P] [US3] Create Mini-Lab 3: "Generate synthetic data and use it to train a simple object detector" in `docs/module-3/mini_lab_3.md`.
- [X] T068 [P] [US3] Create Quiz 3: "NVIDIA Isaac AI Stack assessment" in `docs/module-3/quiz_3.md`.
- [X] T069 [US3] Integrate Module 3 into `sidebars.js`.
- [X] T070 [US3] Fact-check all technical claims and citations in Module 3.

---

## Phase 6: User Story 4 - Implementing Vision-Language-Action (Priority: P2)

**Goal**: Students integrate vision-language models for robot control and implement safety rules.

**Independent Test**: Student implements a basic voice-to-command system that translates to a robot action in simulation.

### Implementation for User Story 4

- [X] T071 [P] [US4] Create `docs/module-4/index.md` for Module 4 overview and objectives.
- [X] T072 [P] [US4] Write "Introduction to VLM in Robotics" section in `docs/module-4/vlm_intro.md`.
- [X] T073 [P] [US4] Write "Voice-to-Text with Whisper" section in `docs/module-4/whisper.md`.
- [X] T074 [P] [US4] Write "LLMs for Robot Task Planning" section in `docs/module-4/llm_planning.md`.
- [X] T075 [P] [US4] Develop "Simple LLM command parsing" example in `examples/module-4/llm_command_parser.py`.
- [X] T076 [US4] Verify "Simple LLM command parsing" example.
- [ ] T077 [P] [US4] Write "Perception Pipeline" section in `docs/module-4/perception_pipeline.md`.
- [ ] T078 [P] [US4] Write "Motion Planning" section in `docs/module-4/motion_planning.md`.
- [X] T079 [P] [US4] Develop "Integrating perception output with motion planner" example in `examples/module-4/perception_motion_integration.py`.
- [X] T080 [US4] Verify "Integrating perception output with motion planner" example.
- [X] T081 [P] [US4] Write "ROS Integration for VLA" section in `docs/module-4/ros_vla_integration.md`.
- [X] T082 [P] [US4] Develop "End-to-end voice command to robot action" example in `examples/module-4/voice_to_action.py`.
- [X] T083 [US4] Verify "End-to-end voice command to robot action" example.
- [X] T084 [P] [US4] Write "Safety Rules and Human-Robot Collaboration" section in `docs/module-4/safety_hrc.md`.
- [X] T085 [P] [US4] Create Mini-Lab 4: "Build a voice-controlled robotic arm" in `docs/module-4/mini_lab_4.md`.
- [X] T086 [P] [US4] Create Quiz 4: "Vision-Language-Action assessment" in `docs/module-4/quiz_4.md`.
- [X] T087 [US4] Integrate Module 4 into `sidebars.js`.
- [X] T088 [US4] Fact-check all technical claims and citations in Module 4.

---

## Phase 7: User Story 5 - Completing Autonomous Humanoid Capstone (Priority: P1)

**Goal**: Students integrate all learned modules into a capstone project for an autonomous humanoid robot.

**Independent Test**: Student successfully completes the capstone project, demonstrating a functional autonomous humanoid pipeline that passes the evaluation rubric.

### Implementation for User Story 5

- [X] T089 [P] [US5] Create `docs/capstone/index.md` for Capstone Project overview and objectives.
- [X] T090 [P] [US5] Write "Project Overview" section in `docs/capstone/project_overview.md`.
- [X] T091 [P] [US5] Write "System Design" section (integrating all modules) in `docs/capstone/system_design.md`.
- [X] T092 [P] [US5] Write "Implementation Guide" section in `docs/capstone/implementation_guide.md`.
- [X] T093 [P] [US5] Write "Evaluation Rubric" section in `docs/capstone/evaluation_rubric.md`.
- [X] T094 [US5] Integrate Capstone Project into `sidebars.js`.
- [X] T095 [US5] Fact-check all technical claims and citations in Capstone.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T096 [P] Review and refine overall `sidebars.js` for optimal navigation flow.
- [X] T097 [P] Review and refine `docusaurus.config.js` for final deployment settings.
- [X] T098 [P] Perform final comprehensive content review for consistency and clarity across all modules.
- [X] T099 [P] Run Docusaurus build and resolve any build warnings/errors.
- [X] T100 [P] Conduct final GitHub Pages deployment test.
- [X] T101 [P] Coordinate with team for Urdu translation readiness.
- [X] T102 [P] Coordinate with team for RAG chatbot integration readiness.
- [X] T103 [P] Final check of all code examples for reproducibility and adherence to standards.
- [X] T104 [P] Consolidate and format all references into `docs/appendices/references.md`.
- [X] T105 [P] Verify total textbook length (60,000–80,000 words).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P1 → P1 → P2 → P2)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - ROS 2 Foundations)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P1 - Digital Twin Simulation)**: Can start after Foundational (Phase 2) - Depends on US1 for ROS 2 basics.
- **User Story 3 (P2 - NVIDIA Isaac AI Stack)**: Can start after Foundational (Phase 2) - Depends on US1 for ROS 2 basics.
- **User Story 4 (P2 - Vision-Language-Action)**: Can start after Foundational (Phase 2) - Depends on US1 for ROS 2 basics, and potentially US2/US3 for simulation contexts.
- **User Story 5 (P1 - Capstone Project)**: Can start after Foundational (Phase 2) - Depends on US1, US2, US3, US4 for full integration.

### Within Each User Story

- Content writing (sections) can often be parallelized.
- Code example development and verification must happen sequentially.
- Quizzes and Mini-Labs should be developed after content and examples.
- Fact-checking should be continuous or as a final step for the module.

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel.
- All Foundational tasks marked [P] can run in parallel.
- Once Foundational phase completes, User Stories 1, 2, 3, and 4 can theoretically be worked on in parallel by different team members, but with cross-dependencies noted above. User Story 5 (Capstone) will likely be sequential.
- Within each User Story, content sections and code example development (before verification) offer parallel opportunities.

---

## Implementation Strategy

### Incremental Delivery (Recommended)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1 (ROS 2 Foundations)
4.  **STOP and VALIDATE**: Test User Story 1 independently. Build and preview Docusaurus site locally.
5.  Proceed with User Story 2, 3, 4, then 5 in a similar iterative fashion.
6.  Regularly build and preview the Docusaurus site to catch integration issues early.
7.  Final Phase: Polish & Cross-Cutting Concerns after all core modules are complete.

### Parallel Team Strategy (If multiple authors/developers)

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Author A: User Story 1 (ROS 2 Foundations)
    -   Author B: User Story 2 (Digital Twin Simulation)
    -   Author C: User Story 3 (NVIDIA Isaac AI Stack)
    -   Author D: User Story 4 (Vision-Language-Action)
    -   Developer E: Focus on code example development and verification across modules.
3.  Capstone (User Story 5) is then a collaborative integration phase.
4.  Polish phase is also collaborative.

---

## Notes

- Tasks are designed to be specific enough for an LLM to execute.
- Clear file paths are provided for all content and code creation tasks.
- Continuous fact-checking and adherence to IEEE citation standards are embedded throughout the process.
- Coordination for Urdu translation and RAG chatbot integration will occur in the final Polish phase.
