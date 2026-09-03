# 4. Hardware Abstractions

Taproot provides wrappers for commonly used RoboMaster hardware, making it easy to interface with motors, sensors, and buses without dealing with raw registers or low-level HAL calls.

## DJI Motors

The most common actuators on RoboMaster robots are the DJI C620 (used with M3508 motors), C610 (M2006), and GM6020 motors. Taproot encapsulates their CAN protocol in the `DjiMotor` class.

Let's upgrade the `ShooterSubsystem` from Chapter 3 to use a real PID controller to govern the DJI Motors, rather than just sending a flat output current.

### Upgrading the Shooter Subsystem

First, we need to add PID controllers to our class. Taproot provides a `SmoothPid` algorithm we will explore deeply in Chapter 8, but for now, just know that it calculates the required current to reach a target RPM.

**In `ShooterSubsystem.hpp`**, add the PID controller members:
```cpp
#include "tap/algorithms/smooth_pid.hpp"
// ...
private:
    tap::motor::DjiMotor leftFlywheel;
    tap::motor::DjiMotor rightFlywheel;
    float targetRpm;
    
    // One PID controller for each motor
    tap::algorithms::SmoothPid leftPid;
    tap::algorithms::SmoothPid rightPid;
    
    // We need a timer to know how much time (dt) has passed between refresh() calls
    uint32_t prevTimeMs;
```

**In `ShooterSubsystem.cpp`**, configure the PID constants and update the `refresh()` function:
```cpp
#include "tap/architecture/clock.hpp" // For getTimeMilliseconds()

ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers) : 
    tap::control::Subsystem(drivers),
    leftFlywheel(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 1, "Left Flywheel"),
    rightFlywheel(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 2, "Right Flywheel"),
    targetRpm(0.0f),
    // Initialize PID config: kP, kI, kD, maxICumulative, maxOutput
    leftPid({0.5f, 0.01f, 0.0f, 1000.0f, 16384.0f}), 
    rightPid({0.5f, 0.01f, 0.0f, 1000.0f, 16384.0f}),
    prevTimeMs(0)
{
    leftFlywheel.initialize();
    rightFlywheel.initialize();
}

void ShooterSubsystem::refresh() 
{
    // 1. Calculate time delta (dt)
    uint32_t currTimeMs = tap::arch::clock::getTimeMilliseconds();
    float dt = (currTimeMs - prevTimeMs) / 1000.0f; // Convert ms to seconds
    prevTimeMs = currTimeMs;
    
    // Prevent huge dt jumps on startup
    if (dt > 0.1f) dt = 0.01f; 

    // 2. Read the actual state from the hardware
    float leftActualRpm = leftFlywheel.getShaftRPM();
    float rightActualRpm = rightFlywheel.getShaftRPM();
    
    // 3. Run the PID controllers to calculate required output current
    // Note: The right motor needs to spin in the opposite direction!
    float leftOutput = leftPid.runController(targetRpm - leftActualRpm, 0.0f, dt);
    float rightOutput = rightPid.runController((-targetRpm) - rightActualRpm, 0.0f, dt);
    
    // 4. Send the desired output to the hardware abstractions
    if (targetRpm > 0.0f) {
        leftFlywheel.setDesiredOutput(leftOutput); 
        rightFlywheel.setDesiredOutput(rightOutput);
    } else {
        leftFlywheel.setDesiredOutput(0);
        rightFlywheel.setDesiredOutput(0);
        
        // Reset PID accumulators when stopped
        leftPid.reset();
        rightPid.reset();
    }
}
```

**Note:** The `DjiMotor` class abstracts the CAN transmission. Calling `setDesiredOutput()` does not immediately send a CAN message. Instead, Taproot's internal CAN handlers periodically batch the outputs of all `DjiMotor` instances and send them efficiently over the bus.

## General Purpose I/O (GPIO)

Sometimes you need to control a laser, an LED, or read a limit switch. The pins available depend on your `project.xml` configuration (e.g., `taproot:board:digital_out_pins`).

Taproot uses `modm`'s GPIO architecture. 

For LEDs:
```cpp
// Turn on the green LED
drivers.leds.set(tap::gpio::Leds::LedPin::Green, true); 
```

For custom digital pins configured in `project.xml` (like a `Laser` pin):
```cpp
// The pin must be configured in project.xml: 
// <option name="taproot:board:digital_out_pins">Laser</option>

#include "tap/architecture/board_pins.hpp"

// Set the laser high
tap::gpio::Laser::setOutput(); 
// Set the laser low
tap::gpio::Laser::resetOutput();
```

In the next section, we'll look at how to trigger your fully-functional Shooter subsystem commands using the remote controller.

---
**Next Step:** [5. Input and Mapping](05-input-and-mapping.md)
