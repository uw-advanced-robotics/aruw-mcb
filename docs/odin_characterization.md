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

## Debugging the Jetson link in Ozone

Add the global **`odinLinkDebug`** to a watch window and set its refresh to live (e.g. 10 Hz).
Jetson → MCB arrives on **USART3** (RX pin PD9); MCB → Jetson leaves on **USART2** (TX pin PD5);
both run at 1 Mbaud on the sentry.

| field | meaning | healthy |
|---|---|---|
| `txLoops` | `sendMessage()` calls (MCB → Jetson loop) | climbs ~500/s |
| `rxBufferNonEmptySamples` | times raw bytes were waiting on USART3 | climbs while otto runs |
| `rxBufferMax` | most bytes waiting at once | > 0 |
| `rxErrorFlags` | sticky USART3 errors: `0x08` overrun, `0x04` noise, `0x02` framing, `0x01` parity | 0 |
| `rxFrames` | complete frames with good CRCs (any type) | climbs ~100/s with odin mode |
| `rxLastType` / `rxLastTimeMs` | last frame's type (2 auto-aim, 13 auto-nav, 20 chassis velocity) and MCB time | 20 / recent |
| `rxChassisVelocity` / `...Rejected` | accepted / rejected type-20 messages | climbs / 0 |
| `lastSequence`, `lastVx`, `lastVy`, `lastWz` | last accepted velocity command | follows the script |
| `cvOnline` | a valid frame within the last second | true |

Reading the result:
* `rxBufferNonEmptySamples` stays 0: no bytes reach PD9, so check the wire from the Jetson TX
  to PD9 and the ground.
* bytes arrive but `rxFrames` stays 0, and `rxErrorFlags` has `0x02`: there's a baud mismatch or
  noise. Without errors, the bytes aren't valid frames (wrong sender).
* `rxFrames` climbs: the Jetson → MCB direction works. If the Jetson still hears nothing, the
  problem is the other wire, PD5 → Jetson RX.

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
