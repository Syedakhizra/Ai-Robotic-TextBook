import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // But you can create a sidebar manually
  bookSidebar: [
    {
      type: 'category',
      label: 'Preface / Motivation',
      link: { type: 'doc', id: 'preface' }, // Assuming preface.md will be directly under docs
      items: [],
    },
    {
      type: 'category',
      label: 'Module 1 - ROS 2 Foundations',
      link: { type: 'doc', id: 'module-1/index' }, // Assuming index.md inside module-1 folder
      items: [
        'module-1/introduction',
        'module-1/concepts',
        'module-1/services_actions',
        'module-1/packages_workspaces',
        'module-1/launch_files',
        'module-1/urdf_basics',
        'module-1/mini_lab_1',
        'module-1/quiz_1',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - Gazebo / Unity Digital Twin',
      link: { type: 'doc', id: 'module-2/index' },
      items: [
        'module-2/introduction',
        'module-2/gazebo_setup',
        'module-2/simulating_sensors',
        'module-2/urdf_sdf_pipeline',
        'module-2/unity_hri',
        'module-2/mini_lab_2',
        'module-2/quiz_2',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - NVIDIA Isaac AI Stack',
      link: { type: 'doc', id: 'module-3/index' },
      items: [
        'module-3/isaac_sim_intro',
        'module-3/usd_workflow',
        'module-3/isaac_ros_perception',
        'module-3/isaac_ros_slam',
        'module-3/synthetic_data',
        'module-3/jetson_deployment',
        'module-3/mini_lab_3',
        'module-3/quiz_3',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action',
      link: { type: 'doc', id: 'module-4/index' },
      items: [
        'module-4/vlm_intro',
        'module-4/whisper',
        'module-4/llm_planning',
        'module-4/perception_pipeline',
        'module-4/motion_planning',
        'module-4/ros_vla_integration',
        'module-4/safety_hrc',
        'module-4/mini_lab_4',
        'module-4/quiz_4',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project - Autonomous Humanoid',
      link: { type: 'doc', id: 'capstone/index' },
      items: [
        'capstone/project_overview',
        'capstone/system_design',
        'capstone/implementation_guide',
        'capstone/evaluation_rubric',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      link: { type: 'doc', id: 'appendices/hardware' }, // Link to first appendix item
      items: [
        'appendices/hardware',
        'appendices/install-guides',
        'appendices/glossary',
        'appendices/references', // This will be created in T104
      ],
    },
  ],
};

module.exports = sidebars;
