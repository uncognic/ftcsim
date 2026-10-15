# FTC Sim

A desktop simulator for FIRST Tech Challenge robots that runs **real FTC Java code**.

The simulator models the hardware (motors, gearboxes, encoders, the hub's motor controller,
the IMU, the battery) and puts the robot in a 3D rigid-body world. Everything above the
hardware is your code: drive math, field-centric control, PID, odometry. If it works here, it
should work on the robot, and the other way around.

```
ftc-sim/
  sim/      Rust. The simulator.
    crates/sim-core   headless physics, hub firmware, runner protocol (+ sim-headless binary)
    crates/sim-app    the desktop app: 3D view and Driver Station controls (Bevy)
  robot/    Java (Gradle). The robot side.
    sdk/       the FTC SDK API (com.qualcomm.*, org.firstinspires.*), backed by the simulator
    runner/    plays the Robot Controller app: finds OpModes, runs them, talks to the sim
    TeamCode/  your OpModes, same package and layout as a Robot Controller project
```

## Running it

You need Rust (stable) and JDK 21 or newer. Gradle comes with the wrapper.

Terminal 1, the simulator:

```
cd sim
cargo run --release -p sim-app
```

Terminal 2, the robot:

```
cd robot
gradlew :runner:run
```

In the simulator window: **Tab** picks an OpMode, **Enter** runs INIT and then START,
**Backspace** stops. The keyboard is gamepad 1 (WASD left stick, arrows right stick); a real
controller works too. **F1/F2/F3** switch cameras, **R** puts the robot back at its start, **P** pauses.

Edit or add OpModes in `robot/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, then
restart the runner. The simulator can stay open.

### Headless, faster than real time

```
cd sim
cargo run --release -p sim-core --bin sim-headless -- --opmode "Encoder Square" --seconds 30
```

then start the runner as above. A 30 second autonomous finishes in well under a second, and
the same code produces the same run, so this is what tests and CI should use.

## How it fits together

The simulator and the runner talk over a local TCP socket in lockstep, much like WPILib's
simulation splits robot code from the physics: the simulator steps physics, sends sensor state,
and waits for the runner's motor commands.

On a real robot each hardware call is a round trip to the hub that takes a couple of
milliseconds, and that is what sets your loop time. The runner charges every hardware call
against a per-tick budget and parks the OpMode thread when it runs out, so loop times are
realistic and a loop that makes too many hardware calls is slow here too.

Things that are real on purpose:

- Left-side drive motors are mounted mirrored and need `REVERSE`.
- `RUN_USING_ENCODER` and `RUN_TO_POSITION` are closed loops in the hub firmware (simulator
  side), not in the SDK. Switching to `RUN_TO_POSITION` before `setTargetPosition()` throws.
- Wheels slip. Push a wall and the encoders keep counting while the robot does not move.
- Mecanum rollers only grip across their axis. Strafing is slower than driving forward.
- Battery voltage sags under load, and motors get slower when it does.
- BRAKE is back-EMF braking from the motor model; FLOAT coasts.

## Status

Early. Implemented: mecanum drivetrain with goBILDA 5203 motors, IMU, battery voltage sensor,
gamepads, telemetry, `LinearOpMode` and iterative `OpMode`, a field with walls, and balls you
can push around. Not yet: servos, other sensors, mechanisms (lifts, intakes, arms), a robot
builder, game-specific fields, and the IMU orientation check (the simulator currently trusts
whatever orientation you pass to `imu.initialize`).

## Tests

```
cd sim && cargo test
cd robot && gradlew build
```
