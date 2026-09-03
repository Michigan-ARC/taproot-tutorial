# 6. Advanced Features

Now that you understand the core of Taproot—Subsystems, Commands, the CommandScheduler, Hardware Abstractions, and the CommandMapper—you have everything you need to make a robot move. 

However, Taproot also provides several advanced features specifically tailored for the RoboMaster competition.

## 1. The Referee System

The RoboMaster referee system enforces the rules of the game. It tells your robot how much health it has, how much power it is allowed to consume, and when the game starts or ends.

Taproot provides a `RefSerial` driver (usually available as `drivers.refSerial`) that parses the referee system data packets.

For example, to read the robot's current health or the current game state:

```cpp
// Check if the referee system is connected and sending valid data
if (drivers.refSerial.isConnected())
{
    // Read the current robot ID (e.g. Red Hero, Blue Standard 3, etc.)
    tap::communication::serial::RefSerial::RobotId id = drivers.refSerial.getRobotData().robotId;
    
    // Read current HP
    uint16_t hp = drivers.refSerial.getRobotData().currentHp;
    
    // Check if the game is active
    bool isGameRunning = drivers.refSerial.getGameState().gameProgress == 
        tap::communication::serial::RefSerial::GameState::IN_PROGRESS;
}
```

This is vital for commands that should only run when the game is active, or logic that limits firing when the barrel is too hot.

## 2. Power Limiting

RoboMaster has strict power limits. If you exceed the power limit, the referee system will deduct your HP. Taproot includes closed-loop power limiting algorithms that can throttle your motors dynamically.

While setting this up requires more configuration than a basic tutorial covers, the core idea is that the `ChassisSubsystem` will often use a `PowerLimiter` object. The `PowerLimiter` reads the current power consumption from the `RefSerial` and dynamically scales down the `setDesiredOutput()` calls to your `DjiMotor` objects to ensure you stay right at the limit without going over.

## 3. Algorithms

Taproot includes a library of algorithms under `tap::algorithms`:
- **PID Controllers**: Robust implementations with anti-windup.
- **Ramping and Smoothing**: To prevent sudden jerks in movement.
- **Math Helpers**: Functions to smoothly wrap angles (e.g., ensuring a turret takes the shortest path between 359 degrees and 1 degree, which is 2 degrees, not 358 degrees backwards).

## 4. Error Handling and Aggregation

When something goes wrong (e.g., a motor unplugs, the referee system disconnects, or a Command fails to schedule), Taproot adds an error to the global `ErrorHandler`.

If you have a terminal enabled (via the OLED display or a UART terminal), Taproot will display these errors in a human-readable format, making debugging on the field much faster. You can also add your own custom errors to the aggregator if a subsystem detects a mechanical jam.

## Next Steps and Beyond

Congratulations! You have completed the Taproot beginner's tutorial. 

To go deeper, explore the following resources:
- The [generated API documentation (Doxygen)](https://aruw.gitlab.io/controls/taproot/) to see all the available classes and functions.
- The `aruw-mcb` repository to see how a full, competition-ready robot codebase uses Taproot.
- The `taproot-examples` repository for small, self-contained examples of specific features.

Happy coding, and good luck in the competition!
