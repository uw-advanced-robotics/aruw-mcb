[![pipeline status](https://gitlab.com/aruw/controls/aruw-mcb/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/aruw-mcb/-/commits/develop)
[![coverage report](https://gitlab.com/aruw/controls/aruw-mcb/badges/develop/coverage.svg)](https://gitlab.com/aruw/controls/aruw-mcb/-/commits/develop)

# aruw-mcb

ARUW's "Main Control Board" (MCB) code for the RoboMaster competition.

The MCB is a [RoboMaster Development Board Type A](https://store.dji.com/product/rm-development-board-type-a)
which directly operates all major systems on our robots. Among these are drive, turret, launcher
wheels, and human input.

Software we use:
- `modm`, a C++-native HAL
- The GCC compiler
- OpenOCD to deploy and debug
- VSCode, an editor
- [Taproot](https://gitlab.com/aruw/controls/taproot), a RoboMaster controls framework

__In addition to this readme, check out 
[our GitLab wiki](https://gitlab.com/aruw/controls/aruw-mcb/-/wikis/home) and
[generated documentation](https://aruw.gitlab.io/controls/aruw-mcb/)!__

## Contacting

If you have any questions please contact us at robomstr@uw.edu.

## Licensing

aruw-mcb is covered under the GPL-3.0-or-later with the following exceptions:
- `/modm` and `/aruw-mcb-project/modm` are licensed under MPL 2.0 by the modm project. We _are not_
  the license holder for these files. See `/modm/LICENSE` for license information.
- `aruw-mcb-project/src/taproot/algorithms/MahonyAHRS.h` and
  `/aruw-mcb-project/src/taproot/algorithms/MahonyAHRS.cpp` are licensed under the GPL by SOH
  Madgwick. The repo containing this code can be found
  [here](https://github.com/uw-advanced-robotics/MahonyAHRS).

## New user guide

### Setting up a development environment

If you want the easiest setup experience and **_do not_ require deploying code to hardware**,
consider developing within the provided [Docker container](https://gitlab.com/aruw/controls/taproot/-/wikis/Docker-Container-Setup).

Otherwise, follow the guide appropriate for your operating system.
- Linux
  - Debian: https://gitlab.com/aruw/controls/taproot/-/wikis/Debian-Linux-Setup
  - Fedora: https://gitlab.com/aruw/controls/taproot/-/wikis/Fedora-Linux-Setup
  - Other: follow one of the above guides, substituting your distribution's package names in place
    of Debian or Fedora packages.
- macOS: https://gitlab.com/aruw/controls/taproot/-/wikis/macOS-Setup
- Windows: https://gitlab.com/aruw/controls/taproot/-/wikis/Windows-Setup

Then, install pipenv with `pip3 install pipenv`.

### Getting started with this repo

_Make sure you have followed the above setup instructions._

Run the following to clone this repository:

```
git clone --recursive https://gitlab.com/aruw/controls/aruw-mcb.git
```

If you use the Docker container, or have already cloned the repository yourself, you should instead
run:

```
git submodule update --init --recursive
```

Then go into the `aruw-mcb/aruw-mcb-project` directory and set up the build tools:

```
cd aruw-mcb/aruw-mcb-project/
pipenv install
```

The following will then activate the virtualenv in the project directory and run some builds to confirm functionality:

```
pipenv shell
# Build for hardware
scons build
# Run automated tests
scons run-tests
```

### VSCode extensions that we use
Go to the VSCode extension marketplace and install "Auto Snippets" by andreasxp. This will help with creating new files in our codebase.

### Returning to the development environment

**You will need to run `pipenv shell` from this directory _every time_ you open a new terminal,
before using `scons` or `lbuild`.**


## Workflow guide

### Branch naming conventions

- When you create a new branch, always branch off of `develop` (not `master`)
- Names should follow the format `FirstL/{Issue Number}/short-description`
- Example: `RyanT/0/linter-integration`

### Getting around VSCode

Microsoft provides a [helpful website](https://code.visualstudio.com/docs/getstarted/tips-and-tricks) that has a number of helpful shortcuts for getting around VSCode. There are many shortcuts that make programming faster. It is much appreciated when someone asks for help and can quickly navigate through the codebase while we work through a code bug.

### How to build code and program the MCB

_If you would like to use the terminal instead, see the section "Building and running via the terminal" below._

1. Make sure you have VSCode opened in the folder `aruw-mcb` (**not `aruw-mcb-project`**)
2. Connect an ST-Link to the MCB and your computer.
3. In VSCode, open the Command Palette (<kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>P</kbd>)
4. Find `Tasks: Run Task`. You should see the options below
<br><br>
    <img src=https://gitlab.com/aruw/controls/aruw-mcb/uploads/2ffb02c86387916c2c49ac3548151b38/image.png height="200px" />

### How to debug using an ST-Link

1. Open the folder `aruw-mcb` in VSCode. Hit the debug tab on the left side or type <kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>D</kbd>.
2. Hit the green play arrow on the left top of the screen.
3. See [this page](https://gitlab.com/aruw/controls/taproot/-/wikis/Debugging-With-STLink) for more information about using the ST-Link for programming the MCB and debugging.
<br>
<img src=https://gitlab.com/aruw/controls/aruw-mcb/uploads/1f62ea310a20ee76092fe18de83d14a7/image.png height="400px" />

### How to debug using a J-Link

See the [wiki](https://gitlab.com/aruw/controls/taproot/-/wikis/Debugging-With-JLink) for an explanation on the difference between an ST-Link and J-Link and a step-by-step procedure on how to use the J-Link.

### How to select robot type

With the root directory opened in VSCode, type <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>. Type "ARUW: Select Robot Type" and hit enter. A dropdown menu should appear. Select the robot type from the dropdown.

### How to select an appropriate VSCode C/C++ configuration

This codebase has a number of different build targets (see [this wiki
page](https://gitlab.com/aruw/controls/taproot/-/wikis/Build-Targets-Overview) for more
information). Because the build setup is different for the test, sim, and MCB environments, while
working on a particular portion of code you may select an appropriate profile that provides optimal
[intellisense](https://code.visualstudio.com/docs/editor/intellisense). To select a configuration,
in VSCode, type <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>, then type `C/C++: Select a Configuration`
and hit enter. A dropdown menu will appear where you may choose either the "Test", "Sim", or "MCB"
configuration.

### Upgrading Taproot

The Taproot project recommends that user projects occasionally upgrade the version of
Taproot that they depend on. The guide for doing so is
[here](https://gitlab.com/aruw/controls/taproot/-/wikis/Upgrading-a-Taproot-project). 

## Working with modm

### What is modm?

We use an embedded library generator called modm in our codebase. It will eventually be important that you understand how modm works. For now, you can just think about it as handling lower level IO on our MCB. You should read [modm's homepage](https://modm.io/) so you have a general idea of what it does.

### Modm examples

The modm website provides a great number of examples that can be very useful when interacting with modm's hardware architecture layer for the first time. The examples are located on modm's website [here](https://modm.io/#examples).

### Adding new dependencies on modm modules (advanced)

Look up the fully-qualified name of the module from the [modm website](https://modm.io/reference/module/modm-architecture/).
The name will look like `:platform:gpio`. Open `aruw-mcb-project/project.xml` and add an entry to the dependencies section like the following:

```xml
<module>modm:platform:gpio</module>
```

Now open the terminal and run `lbuild build`.

## Building and running via the terminal

The below commands require that your working directory is `aruw-mcb/aruw-mcb-project` (where the `SConstruct` and `project.xml` files are).

- `lbuild build`: Re-generates our copy of modm according to the modules specified in `project.xml`. Note that there is a _separate_ instance used for the unit tests, which can be build by runnint the same command from within the `sim-modm` subdirectory.
- `scons build`: Builds the firmware image for the hardware target. Creates a "release" folder located in `build/hardware/` which contains the final `.elf` file as well as the intermediate object files (`.o`).
- `scons build-tests`: Builds a program which hosts our unit tests. This executable can be run on your host computer (only supported on Linux) and prints results for each unit test run.
- `scons run`: Builds as with `scons build` and then programs the board.
- `scons run-tests`: Builds and runs the unit test program.
- `scons size`: Prints statistics on program size and (statically-)allocated memory. Note that the reported available heap space is an upper bound, and this tool has no way of knowing about the real size of dynamic allocations.

Note that all `scons` commands have optional `profile` and `target` options; the former controls whether performance and size optimizations are applied to the output, and the latter specifies which robot to build for. The default is to build in release mode for the Soldier.

```
Usage: scons <target> [profile=<debug|release>] [robot=TARGET_<ROBOT_TYPE>] [profiling=<true|false>]
    "<target>" is one of:
        - "build": build all code for the hardware platform.
        - "run": build all code for the hardware platform, and deploy it to the board via a connected ST-Link.
        - "build-tests": build core code and tests for the current host platform.
        - "run-tests": build core code and tests for the current host platform, and execute them locally with the test runner.
        - "build-sim": build all code for the simulated environment, for the current host platform.
        - "run-sim": build all code for the simulated environment, for the current host platform, and execute the simulator locally.
    "TARGET_<ROBOT_TYPE>" is an optional argument that can override whatever robot type has been specified in robot_type.hpp.
        - <ROBOT_TYPE> must be one of the following:
            - STANDARD_WOODY, STANDARD_ELSA, STANDARD_SPIDER, DRONE, ENGINEER, SENTRY_BEEHIVE, HERO_CYCLONE
```
