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
  ],
};

export default sidebars;