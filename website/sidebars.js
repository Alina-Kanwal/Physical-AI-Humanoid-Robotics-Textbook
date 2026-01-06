// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Module 0: Fundamentals of Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'doc',
          id: 'module-0-fundamentals/index',
        },
        {
          type: 'doc',
          id: 'module-0-fundamentals/introduction-to-physical-ai',
        },
        {
          type: 'doc',
          id: 'module-0-fundamentals/humanoid-robot-design',
        },
        {
          type: 'doc',
          id: 'module-0-fundamentals/physics-simulation',
        },
        {
          type: 'doc',
          id: 'module-0-fundamentals/sensor-integration',
        },
        {
          type: 'doc',
          id: 'module-0-fundamentals/actuator-systems',
        },
        {
          type: 'doc',
          id: 'module-0-fundamentals/locomotion-balance',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - Robotic Nervous System',
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/index',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/introduction-to-ros2',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/nodes-topics-services',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/urdf-modeling',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/rclpy-examples',
        },
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 5: AI Brain Systems for Humanoid Robotics',
      items: [
        {
          type: 'doc',
          id: 'module-5-ai-brain-humanoid/index',
        },
        {
          type: 'doc',
          id: 'module-5-ai-brain-humanoid/cognitive-architectures',
        },
        {
          type: 'doc',
          id: 'module-5-ai-brain-humanoid/embodied-ai',
        },
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;