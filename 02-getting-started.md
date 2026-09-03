# 2. Getting Started

In this section, we'll cover how to get a basic Taproot project set up and built. We'll explore the `test-project` included in the repository, which is a great starting point to understand the project structure.

## Prerequisites

Before writing code, ensure you have the necessary system setup:
1. A Linux, macOS, or Windows computer.
2. `pip3` and `pipenv` installed for managing Python dependencies (used by `lbuild` and `scons`).
3. An ARM GCC toolchain and OpenOCD (for flashing the board).
4. A RoboMaster Development Board (Type A or Type C).

If you haven't installed these, refer to the [Taproot Wiki setup guides](https://gitlab.com/aruw/controls/taproot/-/wikis/home).

## Project Structure

A typical Taproot-based user project looks like this:

```text
my-robot-project/
├── project.xml      # Configuration file for lbuild
├── SConstruct       # Build system configuration
├── src/
│   ├── main.cpp     # Entry point
│   └── ...          # Your Subsystems and Commands
└── taproot/         # Taproot repository (usually a git submodule)
```

## Configuring with `project.xml`

Open `taproot/test-project/project.xml`. This file tells `lbuild` what features of Taproot you want to include and how they map to your hardware. 

For example, you can specify your board type and map hardware pins:

```xml
<options>
    <!-- Specify the board type -->
    <option name="taproot:dev_board">rm-dev-board-a</option>
    
    <!-- Map UART ports for specific functions -->
    <option name="taproot:communication:serial:terminal_serial:uart_port">Uart3</option>
    <option name="taproot:communication:serial:ref_serial:uart_port">Uart6</option>
    
    <!-- Define general purpose pins -->
    <option name="taproot:board:digital_out_pins">E,F,G,H,Laser</option>
</options>

<modules>
    <!-- Include only the modules you need -->
    <module>taproot:core</module>
    <module>taproot:communication:gpio:leds</module>
    <!-- ... -->
</modules>
```

When you run `lbuild build`, it reads this XML file and generates customized source code inside your build folder (specifically generating files like `drivers.hpp` which give you access to your configured hardware).

## Generating and Building

To generate the code and build the project, open a terminal in your project directory (e.g., `test-project/`) and run:

1. **Enter the virtual environment**: 
   ```bash
   pipenv shell
   ```
2. **Generate the Taproot/modm code**: 
   ```bash
   lbuild build
   ```
3. **Compile the code**: 
   ```bash
   scons build
   ```

If you want to compile and immediately flash it to a board connected via an ST-Link/J-Link probe, use:
```bash
scons run
```

## The `main.cpp` and `tap::Drivers`

Open `test-project/src/main.cpp`. It's very simple by default, but in a real project, this is where everything comes together.

The single most important object in Taproot is the **`tap::Drivers`** class. It acts as a central hub for all your hardware abstractions and your `CommandScheduler`. 

When `lbuild` runs, it generates `src/tap/drivers.hpp` based on your `project.xml`. You include this header in your `main.cpp`:

```cpp
#include "tap/drivers.hpp"

// Your main drivers instance
tap::Drivers drivers;

int main() 
{
    // 1. Initialize the board and drivers
    drivers.initialize();

    // 2. Setup your Subsystems and Commands here
    // (We will cover this in the next sections)

    // 3. The main control loop
    while (true)
    {
        // Update the CommandScheduler, which runs all your active commands
        drivers.commandScheduler.run();
    }
    
    return 0; 
}
```

The `drivers` object gives you access to things like `drivers.commandScheduler` (to schedule your commands) and hardware interfaces depending on what modules you enabled in `project.xml`.

---
**Next Step:** [3. Command-Based Architecture](03-command-based-architecture.md)
