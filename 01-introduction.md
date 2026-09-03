# 1. Introduction to Taproot

Welcome to **Taproot**! This tutorial series will take you from a complete beginner to understanding how to use the Taproot control library to program RoboMaster robots effectively.

## What is Taproot?

Taproot is a comprehensive, friendly control library and framework tailored specifically for [RoboMaster](https://www.robomaster.com/) robotics competitions. If you've ever programmed an FRC (FIRST Robotics Competition) robot using WPILib, you'll feel right at home with Taproot. 

At a high level, Taproot provides:
1. **Hardware Drivers**: Pre-built drivers for RoboMaster components (like the C620, GM3508, GM3510 motors), the DR16 Receiver, and the RoboMaster Referee system.
2. **Command-based Framework**: An architecture inspired by FIRST's WPILib that lets you structure your robot code logically with `Subsystem`s and `Command`s.
3. **Algorithms & Helpers**: Tools like PID controllers, setpoint managers, jam detection, and filtering/smoothing logic.
4. **Code Generation**: A modular architecture using `modm` and `lbuild` to configure and generate exactly the code you need for your target board.

## The Foundation: `modm` and `lbuild`

Taproot is built on top of [modm](https://modm.io/), a C++ library generator for barebone embedded systems. Because different robots use different hardware (e.g., Development Board Type A vs. Type C, different pins for different peripherals), you don't just compile Taproot directly. 

Instead, you use a tool called **`lbuild`**. You provide a configuration file (typically `project.xml`), and `lbuild` generates a customized version of the Taproot library that is configured specifically for your hardware setup. 

## The Command-Based Architecture

A core philosophy in Taproot is the **Command-based architecture**. Instead of writing one massive `while` loop that checks every sensor and runs every motor, you break your robot down into logical pieces:

- **Subsystems**: Representations of the physical mechanisms on your robot (e.g., Chassis, Turret, Snail Launcher). A subsystem provides the API to interact with the hardware, but it *does not* decide when to do so.
- **Commands**: State machines that dictate *what* a subsystem should do. (e.g., "Aim Turret", "Drive Forward", "Spin Up Launcher"). Commands require one or more Subsystems to run.
- **CommandScheduler**: The brain of the operation. It ensures only one Command is using a Subsystem at a time, preventing conflicts (e.g., stopping the robot from trying to aim the turret manually and automatically at the same time).

## Where do I start?

Taproot itself is just a collection of templates. To actually write robot code, you need a **User Project**. You can either:
1. Clone the [taproot-template-project](https://gitlab.com/aruw/controls/taproot-template-project) to start a completely new robot codebase.
2. Use the built-in `test-project` folder inside the Taproot repository. This folder serves as a basic example and is used to run Taproot's internal unit tests.

In the next tutorial, we'll dive into how to set up your project and initialize your first set of drivers!

---
**Next Step:** [2. Getting Started](02-getting-started.md)
