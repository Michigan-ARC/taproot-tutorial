# 8. Algorithms and Control

Robotics control requires a lot of math. Taproot provides highly optimized, well-tested algorithms so you don't have to rewrite them.

## 1. `SmoothPid`

You've seen `SmoothPid` in previous chapters. It is Taproot's flagship PID controller. Unlike a standard PID, `SmoothPid` includes integrated Kalman filters for the proportional and derivative terms.

This means if your sensor (e.g., an M3508 encoder) is noisy, the derivative term (which is notorious for exploding with noise) will be smoothed out cleanly.

### Configuration
You configure it using a `SmoothPidConfig` struct:

```cpp
#include "tap/algorithms/smooth_pid.hpp"

tap::algorithms::SmoothPidConfig myConfig = {
    .kp = 1.5f,
    .ki = 0.05f,
    .kd = 0.1f,
    .maxICumulative = 1000.0f, // Prevents integral windup
    .maxOutput = 16384.0f,     // Max output for a DJI motor
    
    // The Kalman filter Q and R matrices for the Derivative term.
    // Higher R = more smoothing but more delay.
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 10.0f, 
    
    // Kalman matrices for the Proportional term
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f, // No proportional smoothing by default
    
    .errDeadzone = 5.0f, // If error is between -5 and 5, it acts as 0 error
    .errorDerivativeFloor = 0.0f
};

tap::algorithms::SmoothPid myPid(myConfig);
```

### Usage
Run it in a loop, passing the difference between target and actual:
```cpp
// 0.0f is passed for the derivative because SmoothPid calculates 
// the derivative internally using the error and dt!
float output = myPid.runController(targetAngle - actualAngle, 0.0f, dt);
```

## 2. Math Utilities and Angle Wrapping

When controlling a Turret, you deal with angles. If the turret is at 350 degrees, and the target is 10 degrees, a naive controller will calculate an error of `10 - 350 = -340` degrees, causing the turret to spin almost a full circle backwards.

Instead, the error should be `+20` degrees. Taproot provides `tap::algorithms::MathUserUtils` and `WrappedFloat` to solve this.

```cpp
#include "tap/algorithms/math_user_utils.hpp"
#include <cmath>

// Assuming angles are in radians:
float targetAngle = M_PI / 4.0f;
float actualAngle = 7.0f * M_PI / 4.0f;

// This calculates the shortest distance between two angles in radians [-PI, PI]
float error = tap::algorithms::MathUserUtils::limitAngleError(targetAngle - actualAngle);

// You can now safely pass this to a PID controller
float output = turretPid.runController(error, 0.0f, dt);
```

There are also limit functions to easily clamp values:
```cpp
// Ensures targetRpm never exceeds [-10000, 10000]
targetRpm = tap::algorithms::MathUserUtils::limitValue(targetRpm, 10000.0f);
```

## 3. Ramping and Smoothing Targets

If you instantly tell a Chassis to go from 0 to full speed, the motors will draw massive current and possibly trigger a power limit penalty. You want to *ramp* the target.

Taproot provides `tap::algorithms::Ramp`.

```cpp
#include "tap/algorithms/ramp.hpp"

// Ramp up at 1000 units per second, ramp down at 2000 units per second
tap::algorithms::Ramp speedRamp(1000.0f, 2000.0f);

void ChassisSubsystem::refresh() 
{
    float dt = getDt();
    
    // Updates the internal ramped value, approaching targetRpm by the max allowed step
    speedRamp.update(targetRpm, dt);
    
    // Get the smooth target
    float smoothTargetRpm = speedRamp.getValue();
    
    // Use the smooth target in your PID instead of the raw target
    float output = lfPid.runController(smoothTargetRpm - actualRpm, 0.0f, dt);
}
```

## Conclusion

By combining the **Command-based Architecture**, the **`DjiMotor` hardware abstractions**, the **`PowerLimiter`**, and **`SmoothPid`**, you can build a highly resilient, competitive RoboMaster robot. 

Use these tutorial files as a reference as you build out your `project.xml` and start bringing your Subsystems and Commands to life. Good luck!
