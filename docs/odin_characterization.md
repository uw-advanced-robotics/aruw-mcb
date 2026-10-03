# Odin characterization (sentry)

Branch `odin-characterization`. **Do not take this branch to competition:** on the sentry it
replaces auto-nav + auto-aim on one remote switch state.

It lets an off-board controller, closed on motion capture, drive the sentry's chassis through the
vision coprocessor (project otto's `odin-characterization` mode). Odin is mounted on turret major,
so turret major is held fixed relative to the chassis at the same time.

## What changed

| file | change |
|---|---|
| `src/aruwsrc/communication/serial/vision_coprocessor.{hpp,cpp}` | new RX message `CV_MESSAGE_TYPE_CHASSIS_VELOCITY = 20` → `ChassisVelocityCommand {uint32 sequenceNum; float vx, vy, wz;}` (packed, 16 bytes; m/s, m/s, rad/s, chassis frame), plus `getLastChassisVelocityCommand()` and `getLastChassisVelocityCommandTimeMs()`. Wrong-length or non-finite messages are dropped. |
| `src/aruwsrc/control/chassis/external_velocity_command.{hpp,cpp}` | `ExternalVelocityCommand`: converts the command to wheel-RPM units with the same conversion as `ChassisAutoNavController` (`60 / (2π r_wheel)`; rotation × `r_wheelbase`), clamps to 2.5 m/s and 6 rad/s, and zeroes the chassis if no command for 150 ms, CV offline, or a chassis motor is offline. Never reads odometry. |
| `src/aruwsrc/robot/sentry/turret/turret_major_chassis_hold_command.{hpp,cpp}` | `TurretMajorChassisHoldCommand`: runs `turretMajorChassisYawController` at a fixed chassis-frame yaw (`turretMajor::YAW_MOTOR_CONFIG.startAngle`). The angle is encoder-referenced, so it is the same every time the command starts. |
| `src/aruwsrc/robot/sentry/sentry_control.cpp` | instantiates both; **left UP / right MID** now runs `{externalVelocityCommand, turretMajorChassisHoldCommand}` (was `{autoNavCommand, turretCVCommand}`). |

## Using it

1. Flash: `scons run robot=SENTRY_ACHLYS` from `aruw-mcb-project/`.
2. Run project otto's `poetry run odin-characterization` on the Jetson instead of otto's main loop.
3. Remote: **left UP / right MID** enables external velocity control. Any other switch state
   returns to the normal commands; flip out of it to stop the robot.

## Debugging

* **Chassis doesn't move but otto's status line shows ACTIVE commands:** check the switch state.
  Then check `isCvOnline()`: the bridge sends 100 messages/s, so CV should be online. Check
  `allMotorsOnline()` as well.
* **Watch the command arrive** (Ozone or a debugger): set a breakpoint or watch on
  `VisionCoprocessor::lastChassisVelocityCommand` / `lastChassisVelocityCommandTimeMs`.
* **Wrong direction or scale:** the off-board script identifies the command → velocity gains
  from mocap, so sign or scale errors are compensated there. Its `jog` step prints how the robot
  actually moved for +vx, +vy and +wz.
* **Turret major drifts:** the hold uses `turretMajor::chassisFrameController::YAW_PID_CONFIG`; tune
  there if it oscillates while the chassis spins.

## Compile status

Checked with a hosted build (`scons -k build-sim robot=SENTRY_ACHLYS`, GCC 13):
`vision_coprocessor.o`, `external_velocity_command.o`, `turret_major_chassis_hold_command.o` and
`vision_coprocessor_tests.o` compile. The sentry sim build has a pre-existing error
(`Board::SpiNss` in `sentry_drivers.hpp`), and `inter_robot_transmitter.cpp` doesn't build under
GCC 13. Neither is touched by this branch, and no other errors are reported in `sentry_control.cpp`.
The hardware build (`scons build`) has not been run.
