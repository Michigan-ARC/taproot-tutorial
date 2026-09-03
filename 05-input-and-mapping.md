# 5. Input and Mapping

You have subsystems that interact with hardware, and commands that define behavior. But how do you actually trigger those commands during a match? Taproot uses the `CommandMapper` to link physical inputs (like the DR16 receiver) to `Command`s.

## The DR16 Receiver and `RemoteMapState`

The DJI DR16 receiver communicates with the robot over UART. Taproot provides a driver for this, accessible via `drivers.commandMapper`. However, instead of reading raw joystick values inside your Commands, you define *mappings*.

Taproot's `RemoteMapState` describes a specific configuration of switches, buttons, or joysticks on the remote.

```cpp
#include "tap/control/remote_map_state.hpp"

// A state where the left switch is UP and the right switch is DOWN
tap::control::RemoteMapState myState(
    tap::communication::serial::Remote::Switch::LEFT_SWITCH,
    tap::communication::serial::Remote::SwitchState::UP,
    tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
    tap::communication::serial::Remote::SwitchState::DOWN
);
```

You can also include keyboard keys or mouse buttons (if you are using the PC client):

```cpp
// Left switch UP, Right switch DOWN, AND the 'W' key is pressed
tap::control::RemoteMapState pcDriveForwardState(
    tap::communication::serial::Remote::Switch::LEFT_SWITCH,
    tap::communication::serial::Remote::SwitchState::UP,
    tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
    tap::communication::serial::Remote::SwitchState::DOWN,
    {tap::communication::serial::Remote::Key::W}
);
```

## Types of Mappings

Once you have a `RemoteMapState` and a `Command`, you link them.

### 1. Hold Mapping (`HoldCommandMapping`)
The command runs *as long as* the `RemoteMapState` is true. When the user lets go, the command is interrupted.

```cpp
#include "tap/control/hold_command_mapping.hpp"
// E.g., Hold mapping for driving forward
tap::control::HoldCommandMapping holdMapping(&drivers, pcDriveForwardState, &driveForwardCmd);
```

### 2. Press Mapping (`PressCommandMapping`)
The command is scheduled when the state *becomes* true (edge-triggered). It runs until its `isFinished()` returns true.

```cpp
#include "tap/control/press_command_mapping.hpp"
// E.g., Press mapping for firing a single shot
tap::control::PressCommandMapping pressMapping(&drivers, fireSingleShotState, &fireCmd);
```

### 3. Toggle Mapping (`ToggleCommandMapping`)
Pressing the buttons once schedules the command. Pressing them again interrupts the command. It's like a light switch.

```cpp
#include "tap/control/toggle_command_mapping.hpp"
// E.g., Toggle mapping for spinning up the shooter
tap::control::ToggleCommandMapping toggleMapping(&drivers, spinUpState, &spinUpCmd);
```

## Putting It All Together: A Full `main.cpp`

Let's bring together the `ShooterSubsystem` and `SpinUpCommand` we built in chapters 3 and 4, and map them to the DR16 remote using a toggle mapping. 

When the left switch is UP and the right switch is MID, flipping the left switch will turn the shooter on and off.

```cpp
#include "tap/drivers.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"

// Our custom classes from earlier chapters
#include "ShooterSubsystem.hpp"
#include "SpinUpCommand.hpp"

// 1. Initialize drivers globally
tap::Drivers drivers;

// 2. Initialize Subsystems and Commands globally
ShooterSubsystem shooterSubsystem(&drivers);
SpinUpCommand spinUpCmd(&shooterSubsystem, 5000.0f); // Target 5000 RPM

// 3. Define the remote state (Left Switch UP, Right Switch MID)
tap::control::RemoteMapState spinUpState(
    tap::communication::serial::Remote::Switch::LEFT_SWITCH,
    tap::communication::serial::Remote::SwitchState::UP,
    tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
    tap::communication::serial::Remote::SwitchState::MID
);

// 4. Create the mapping (Toggle: first press turns ON, second press turns OFF)
tap::control::ToggleCommandMapping spinUpMapping(&drivers, spinUpState, &spinUpCmd);

int main() 
{
    // Initialize hardware drivers
    drivers.initialize();

    // Register Subsystem
    drivers.commandScheduler.registerSubsystem(&shooterSubsystem);
    
    // Register the mapping with the CommandMapper
    drivers.commandMapper.addMap(&spinUpMapping);

    while (true)
    {
        // 1. Process new UART data from the remote
        drivers.remote.read();
        
        // 2. The command mapper checks the remote state against all registered maps.
        // If `spinUpState` just became true, it tells the scheduler to add `spinUpCmd`.
        // If it was already running and became true again, it tells the scheduler to remove it.
        drivers.commandMapper.run();
        
        // 3. The scheduler calls execute() on active commands and refresh() on subsystems.
        drivers.commandScheduler.run();
    }
    return 0; 
}
```

This separation of concerns makes Taproot incredibly scalable. You can add a Chassis, a Turret, and a Sentry system all to the same `main.cpp` without any overlapping logic or messy `if` statements!

---
**Next Step:** [6. Advanced Features](06-advanced-features.md)
