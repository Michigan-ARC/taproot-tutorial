# 3. Command-Based Architecture

Taproot uses a Command-based architecture to organize code. This pattern separates the *hardware logic* (Subsystems) from the *behavioral logic* (Commands). In this chapter, we'll build a full, compilable example of a `ShooterSubsystem` and a `SpinUpCommand`.

## 1. Subsystems

A `Subsystem` encapsulates the hardware interactions for a specific mechanism. It provides an API for `Command`s to use, but a subsystem *never* decides when to move on its own.

Let's create a `ShooterSubsystem`. 

### `ShooterSubsystem.hpp`
```cpp
#ifndef SHOOTER_SUBSYSTEM_HPP_
#define SHOOTER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

class ShooterSubsystem : public tap::control::Subsystem 
{
public:
    // Pass the global tap::Drivers pointer to the constructor
    ShooterSubsystem(tap::Drivers* drivers);

    // Called automatically by the CommandScheduler every loop
    void refresh() override;

    // Getters for Commands to read state
    float getLeftMotorRpm() const;
    float getRightMotorRpm() const;

    // Setters for Commands to control behavior
    void setTargetRpm(float rpm);
    
    // Safety setter to turn motors off
    void stop();

private:
    // Define our two hardware motors
    tap::motor::DjiMotor leftFlywheel;
    tap::motor::DjiMotor rightFlywheel;
    
    // Internal state
    float targetRpm;
};

#endif // SHOOTER_SUBSYSTEM_HPP_
```

### `ShooterSubsystem.cpp`
```cpp
#include "ShooterSubsystem.hpp"

ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers) : 
    tap::control::Subsystem(drivers),
    // Initialize hardware drivers here (e.g., M3508 motors on CAN 1, IDs 1 and 2)
    leftFlywheel(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 1, "Left Flywheel"),
    rightFlywheel(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 2, "Right Flywheel"),
    targetRpm(0.0f)
{
    // You MUST call initialize() on DJI motors
    leftFlywheel.initialize();
    rightFlywheel.initialize();
}

void ShooterSubsystem::refresh() 
{
    // Note: In Chapter 4 and 8, we will replace this simplistic logic with a real PID controller.
    // For now, we are just illustrating the Subsystem structure.
    
    if (targetRpm > 0.0f) {
        // Send an arbitrary fixed current to spin up
        leftFlywheel.setDesiredOutput(1000); 
        rightFlywheel.setDesiredOutput(-1000); // Opposite direction
    } else {
        leftFlywheel.setDesiredOutput(0);
        rightFlywheel.setDesiredOutput(0);
    }
}

float ShooterSubsystem::getLeftMotorRpm() const {
    return leftFlywheel.getShaftRPM();
}

float ShooterSubsystem::getRightMotorRpm() const {
    return rightFlywheel.getShaftRPM();
}

void ShooterSubsystem::setTargetRpm(float rpm) {
    this->targetRpm = rpm;
}

void ShooterSubsystem::stop() {
    this->targetRpm = 0.0f;
}
```

## 2. Commands

A `Command` defines *what* a Subsystem should do over time. Let's create a `SpinUpCommand` that speeds up the shooter to a specific RPM.

### `SpinUpCommand.hpp`
```cpp
#ifndef SPIN_UP_COMMAND_HPP_
#define SPIN_UP_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "ShooterSubsystem.hpp"

class SpinUpCommand : public tap::control::Command 
{
public:
    SpinUpCommand(ShooterSubsystem* subsystem, float desiredRpm);

    void initialize() override;
    void execute() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    ShooterSubsystem* subsystem;
    float desiredRpm;
};

#endif // SPIN_UP_COMMAND_HPP_
```

### `SpinUpCommand.cpp`
```cpp
#include "SpinUpCommand.hpp"
#include <cmath>

SpinUpCommand::SpinUpCommand(ShooterSubsystem* subsystem, float desiredRpm) : 
    subsystem(subsystem), 
    desiredRpm(desiredRpm)
{
    // THIS IS CRITICAL: Tell the scheduler this command requires this subsystem.
    // If you forget this, the CommandScheduler won't resolve conflicts!
    this->addSubsystemRequirement(subsystem);
}

// Called ONCE when the command is scheduled
void SpinUpCommand::initialize() 
{
    subsystem->setTargetRpm(desiredRpm);
}

// Called REPEATEDLY as long as the command is scheduled
void SpinUpCommand::execute() 
{
    // The target RPM is already set. We might use this function 
    // to log data, flash LEDs as it spins up, or adjust targets dynamically.
}

// Called REPEATEDLY to check if the command is done
bool SpinUpCommand::isFinished() const 
{
    // Check if both motors are within 100 RPM of the target
    bool leftReady = std::abs(subsystem->getLeftMotorRpm() - desiredRpm) < 100.0f;
    bool rightReady = std::abs(subsystem->getRightMotorRpm() - (-desiredRpm)) < 100.0f;
    
    return leftReady && rightReady;
}

// Called ONCE when the command ends
void SpinUpCommand::end(bool interrupted) 
{
    // If interrupted (e.g., a "StopShooterCommand" was scheduled), ensure we stop safely.
    if (interrupted) {
        subsystem->stop();
    }
}
```

## 3. The CommandScheduler

The `CommandScheduler` resolves conflicts. If you try to schedule a `StopShooterCommand` while `SpinUpCommand` is running, the scheduler will automatically call `SpinUpCommand::end(true)` and then start the `StopShooterCommand`.

Here is a simplified `main.cpp` showing how they connect:

```cpp
#include "tap/drivers.hpp"
#include "ShooterSubsystem.hpp"
#include "SpinUpCommand.hpp"

tap::Drivers drivers;

// Initialize our subsystem and command globally
ShooterSubsystem shooterSubsystem(&drivers);
SpinUpCommand spinUpCmd(&shooterSubsystem, 5000.0f); // 5000 RPM

int main() 
{
    drivers.initialize();

    // 1. Register the subsystem with the scheduler
    drivers.commandScheduler.registerSubsystem(&shooterSubsystem);
    
    // 2. Add the command to start running it
    drivers.commandScheduler.addCommand(&spinUpCmd);

    while (true)
    {
        // 3. This does the heavy lifting:
        // - Calls shooterSubsystem.refresh()
        // - Calls spinUpCmd.execute()
        // - Checks spinUpCmd.isFinished(). If true, calls end(false) and removes it.
        drivers.commandScheduler.run();
    }
    return 0; 
}
```

In the next chapter, we will replace the dummy logic inside `ShooterSubsystem::refresh()` with real Hardware Abstractions and PID controllers.

---
**Next Step:** [4. Hardware Abstractions](04-hardware-abstractions.md)
