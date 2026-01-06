# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Overview

This quickstart guide provides a fast path to getting started with the Physical AI & Humanoid Robotics book. It covers the essential setup and first steps needed to begin learning Physical AI concepts using ROS 2, Gazebo, Unity, and NVIDIA Isaac.

## Prerequisites

Before starting with the book, you'll need to have the following installed:

1. **Git** - Version control system
2. **Python 3.8+** - For ROS 2 and related tools
3. **Node.js 16+** - For Docusaurus development
4. **Docker** - For isolated development environments (recommended)
5. **Basic command line knowledge** - Comfortable with terminal/bash

## Setup Steps

### 1. Clone the Repository

```bash
git clone https://github.com/[your-repo]/physical-ai-book.git
cd physical-ai-book
```

### 2. Install Docusaurus Dependencies

```bash
cd website
npm install
```

### 3. Verify Docusaurus Setup

```bash
npm run start
```

This should start the local development server and open the book in your browser.

### 4. Install ROS 2 (Humble Hawksbill)

Follow the official installation guide for your operating system:
- [Ubuntu/Debian](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Windows](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
- [macOS](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)

### 5. Install Simulation Tools

#### Gazebo Garden
```bash
# Ubuntu
sudo apt install gazebo
```

#### Unity Hub (for Unity development)
- Download from [Unity Hub](https://unity.com/download)
- Install Unity 2022.3 LTS version

### 6. NVIDIA Isaac Setup (Optional for Advanced Users)

For NVIDIA Isaac components, you'll need:
- NVIDIA GPU with CUDA support
- CUDA toolkit installed
- Isaac ROS packages (install via ROS 2 package manager)

## First Steps

### 1. Read the Introduction
Start with the book's introduction to understand the Physical AI concepts and learning approach.

### 2. Complete Module 1 - ROS 2 Basics
- Read Chapter 1.1: Introduction to ROS 2
- Follow the examples and create your first ROS 2 node
- Complete the exercises at the end of the chapter

### 3. Set Up Your First Simulation
- Follow the Gazebo setup instructions in Module 2
- Launch your first robot simulation
- Experiment with different robot configurations

## Development Workflow

### Writing and Previewing Content
1. Edit Markdown files in the `docs/` directory
2. Run `npm run start` in the `website/` directory
3. Preview changes in your browser (auto-refresh enabled)

### Building for Production
```bash
cd website
npm run build
```

### Deploying to GitHub Pages
```bash
cd website
npm run deploy
```

## Troubleshooting Common Issues

### Docusaurus Build Issues
- Ensure Node.js version is 16 or higher
- Clear npm cache: `npm cache clean --force`
- Delete node_modules and reinstall: `rm -rf node_modules && npm install`

### ROS 2 Environment Setup
- Source the ROS 2 setup script: `source /opt/ros/humble/setup.bash`
- Check ROS 2 installation: `ros2 --version`

### Simulation Problems
- Ensure proper graphics drivers are installed for Gazebo
- Check that required plugins are available
- Verify robot models are properly configured

## Next Steps

After completing the quickstart:
1. Work through Module 1 (ROS 2) in sequence
2. Set up your simulation environment
3. Begin with simple robot control examples
4. Progress to more complex autonomous behaviors
5. Complete the capstone project integrating all modules

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Docusaurus Documentation](https://docusaurus.io/docs)
- [NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)