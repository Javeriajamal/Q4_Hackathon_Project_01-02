// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1-ros-2/chapter-1-introduction-to-ros2',
        'module-1-ros-2/chapter-2-python-agents-ros2',
        'module-1-ros-2/chapter-3-urdf-humanoid-robots',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2-digital-twin-simulation/chapter-1-introduction-to-digital-twins-and-physics-simulation',
        'module-2-digital-twin-simulation/chapter-2-robotics-simulation-with-gazebo',
        'module-2-digital-twin-simulation/chapter-3-high-fidelity-simulation-and-interaction-with-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3:  The AI-Robot Brain (NVIDIA Isaac™)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3-isaac-ai-brain/condensed-chapter-1-simulation',
        'module-3-isaac-ai-brain/condensed-chapter-2-perception',
        'module-3-isaac-ai-brain/condensed-chapter-3-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Robotics',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4-vla-robotics/chapter-1-vla-foundations',
        'module-4-vla-robotics/chapter-2-voice-cognitive-planning',
        'module-4-vla-robotics/chapter-3-capstone-autonomous-humanoid',
      ],
    },
  ],
};
  

export default sidebars;
