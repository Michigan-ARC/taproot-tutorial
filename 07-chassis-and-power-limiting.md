# 7. Chassis and Power Limiting

The Chassis is usually the most complex subsystem on a RoboMaster robot. It involves complex kinematics (e.g., Mecanum wheels) and requires strict adherence to the Referee System's power limits to avoid taking penalty damage.

Taproot provides built-in utilities to make this easier, specifically the `PowerLimiter`.

## Defining a Mecanum Chassis Subsystem

A Mecanum chassis allows a robot to move in any direction without turning (holonomic drive). You typically have four M3508 motors.

Here is an example of how you might structure a simple Mecanum Chassis Subsystem using Taproot:

### `ChassisSubsystem.hpp`
```cpp
#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/chassis/power_limiter.hpp"

class ChassisSubsystem : public tap::control::Subsystem 
{
public:
    ChassisSubsystem(tap::Drivers* drivers);

    void refresh() override;

    // A command calls this to drive the robot.
    // x = forward/back, y = left/right, r = rotation
    void setDesiredVelocity(float x, float y, float r);

private:
    // 4 Motors
    tap::motor::DjiMotor lfMotor; // Left Front
    tap::motor::DjiMotor rfMotor; // Right Front
    tap::motor::DjiMotor lbMotor; // Left Back
    tap::motor::DjiMotor rbMotor; // Right Back
    
    // 4 PID Controllers for velocity
    tap::algorithms::SmoothPid lfPid, rfPid, lbPid, rbPid;
    
    // The Taproot Power Limiter
    tap::control::chassis::PowerLimiter powerLimiter;
    
    // Target speeds for each wheel
    float lfTargetRpm, rfTargetRpm, lbTargetRpm, rbTargetRpm;
    
    uint32_t prevTimeMs;
};

#endif
```

### Power Limiting in the Refresh Loop

To configure the `PowerLimiter`, you must pass it the global drivers (for access to the `RefSerial`), a current sensor, and your energy buffer thresholds.

In RoboMaster, you have a maximum power limit (e.g., 60W) and an "Energy Buffer" (e.g., 60 Joules). You can temporarily exceed 60W, draining the buffer. If the buffer hits 0, you take damage. The `PowerLimiter` smoothly scales down motor outputs as the buffer drops.

### `ChassisSubsystem.cpp`
```cpp
#include "ChassisSubsystem.hpp"
#include "tap/architecture/clock.hpp"

ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers) : 
    tap::control::Subsystem(drivers),
    lfMotor(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 1, "LF"),
    rfMotor(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 2, "RF"),
    lbMotor(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 3, "LB"),
    rbMotor(drivers, tap::motor::MOTOR_CONSTANT_M3508_C620, tap::can::CanBus::CAN_BUS1, 4, "RB"),
    // PID Configs omitted for brevity
    lfPid({0.5, 0, 0, 0, 16384}), rfPid({0.5, 0, 0, 0, 16384}),
    lbPid({0.5, 0, 0, 0, 16384}), rbPid({0.5, 0, 0, 0, 16384}),
    // Initialize Power Limiter
    // Args: drivers, current_sensor, voltage_sensor, starting_buffer, limit_threshold, crit_threshold
    // (Assuming drivers->currentSensor exists and is configured)
    powerLimiter(drivers, &drivers->currentSensor, &drivers->voltageSensor, 60.0f, 40.0f, 10.0f),
    prevTimeMs(0)
{
    lfMotor.initialize();
    rfMotor.initialize();
    lbMotor.initialize();
    rbMotor.initialize();
}

void ChassisSubsystem::setDesiredVelocity(float x, float y, float r) {
    // Basic mecanum kinematics to convert chassis velocities to wheel RPMs
    lfTargetRpm = x + y + r;
    rfTargetRpm = -x + y + r;
    lbTargetRpm = x - y + r;
    rbTargetRpm = -x - y + r;
}

void ChassisSubsystem::refresh() 
{
    uint32_t currTimeMs = tap::arch::clock::getTimeMilliseconds();
    float dt = (currTimeMs - prevTimeMs) / 1000.0f;
    prevTimeMs = currTimeMs;
    if (dt > 0.1f) dt = 0.01f;

    // 1. Calculate desired motor outputs using PID
    float lfOutput = lfPid.runController(lfTargetRpm - lfMotor.getShaftRPM(), 0.0f, dt);
    float rfOutput = rfPid.runController(rfTargetRpm - rfMotor.getShaftRPM(), 0.0f, dt);
    float lbOutput = lbPid.runController(lbTargetRpm - lbMotor.getShaftRPM(), 0.0f, dt);
    float rbOutput = rbPid.runController(rbTargetRpm - rbMotor.getShaftRPM(), 0.0f, dt);
    
    // 2. Set the outputs on the motors (but don't send to CAN yet!)
    lfMotor.setDesiredOutput(lfOutput);
    rfMotor.setDesiredOutput(rfOutput);
    lbMotor.setDesiredOutput(lbOutput);
    rbMotor.setDesiredOutput(rbOutput);
    
    // 3. Apply the Power Limiter
    // The limiter reads the referee system to check our actual power draw vs our limit.
    // It returns a fraction [0.0, 1.0]. If we are draining our buffer too fast, 
    // it will return something like 0.8, scaling down all motors by 20%.
    float powerFrac = powerLimiter.getPowerLimitRatio();
    
    // 4. Update the final outputs with the scaled fraction
    lfMotor.setDesiredOutput(powerFrac * lfMotor.getOutputDesired());
    rfMotor.setDesiredOutput(powerFrac * rfMotor.getOutputDesired());
    lbMotor.setDesiredOutput(powerFrac * lbMotor.getOutputDesired());
    rbMotor.setDesiredOutput(powerFrac * rbMotor.getOutputDesired());
}
```

The `PowerLimiter` is critical. It does all the complex integration of the referee system's `getRobotData().chassisPowerLimit` and your physical current sensors, ensuring you drive as aggressively as possible without taking damage.

---
**Next Step:** [8. Algorithms and Control](08-algorithms-and-control.md)
