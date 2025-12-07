// sidebars.ts

/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * You can create as many sidebars as you want.
 */

const sidebars = {
  mySidebar: [
    'intro', // Intro page

    // Module 1
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/chapter1-ros-core-concepts',
        'module1/chapter2-python-agents-with-rclpy',
        'module1/chapter3-humanoid-urdf-basics',
      ],
    },

    // Module 2
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/chapter1-gazebo-physics-env',
        'module2/chapter2-unity-rendering-humanoids',
        'module2/chapter3-sensor-simulation',
      ],
    },

    // Module 3
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/chapter1-isaac-sim-basics',
        'module3/chapter2-isaac-ros-vslam-nav',
        'module3/chapter3-nav2-humanoid-planning',
      ],
    },

    // Module 4
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/chapter1-voice-command-rec',
        'module4/chapter2-llm-cognitive-planning',
        'module4/chapter3-autonomous-humanoid-capstone',
      ],
    },
  ],
};

export default sidebars;
