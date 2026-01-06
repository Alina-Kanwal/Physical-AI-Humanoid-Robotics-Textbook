# Introduction to ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the core concepts of ROS 2 architecture
- Understand the differences between ROS 1 and ROS 2
- Describe the DDS-based communication layer
- Identify the key components of a ROS 2 system

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and provide:

- **Real-time support**: Critical for many robotic applications
- **Deterministic behavior**: Predictable timing and execution
- **Security**: Built-in security features for safe robot operation
- **Commercial product support**: Ready for production environments

## Architecture Overview

ROS 2 uses a distributed system architecture based on the Data Distribution Service (DDS) standard. This provides:

- **Peer-to-peer communication**: Nodes communicate directly without a central master
- **Language independence**: Support for multiple programming languages
- **Platform independence**: Runs on various operating systems
- **Quality of Service (QoS)**: Configurable communication policies

### Core Components

1. **Nodes**: Processes that perform computation
2. **Topics**: Named buses over which nodes exchange messages
3. **Services**: Synchronous request/response communication
4. **Actions**: Asynchronous goal-oriented communication
5. **Parameters**: Configuration values shared among nodes

## DDS and Communication Layer

The Data Distribution Service (DDS) is the middleware that enables ROS 2's communication. DDS provides:

- **Publisher/Subscriber model**: For topic-based communication
- **Client/Server model**: For service-based communication
- **Discovery**: Automatic detection of nodes on the network
- **Quality of Service**: Configurable policies for reliability, durability, etc.

## Key Improvements Over ROS 1

- **No central master**: Eliminates single point of failure
- **Multi-machine support**: Better out-of-the-box networking
- **Security**: Authentication, encryption, and access control
- **Real-time support**: Deterministic execution capabilities
- **Official Windows and macOS support**: Broader platform compatibility

## Setting Up ROS 2

ROS 2 Humble Hawksbill is the recommended Long Term Support (LTS) version for this book. Installation involves:

1. Setting up the appropriate OS (Ubuntu 22.04, Windows 10/11, or macOS)
2. Adding the ROS 2 repository to your package manager
3. Installing the desktop package with all common tools
4. Sourcing the ROS 2 environment setup script

## Summary

ROS 2 provides a robust foundation for developing complex robotic systems with improved security, real-time capabilities, and production readiness compared to ROS 1. Its DDS-based architecture enables scalable and reliable communication between robot components.

## Exercises

1. Install ROS 2 Humble Hawksbill on your development machine
2. Source the ROS 2 environment and verify the installation
3. Run the basic talker/listener demo to test communication
4. Use `ros2 doctor` to check your ROS 2 installation
5. Explore the different ROS 2 distributions and understand their support timelines

## Resources

- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)