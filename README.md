# FTC Sim

A desktop simulator for FIRST Tech Challenge robots that runs **real FTC Java code**.

The simulator models the hardware (motors, gearboxes, encoders, the hub's motor controller,
the IMU, the battery) and puts the robot in a 3D rigid-body world. Everything above the
hardware is your code: drive math, field-centric control, PID, odometry. If it works here, it
should work on the robot, and the other way around.

```
ftcsim/
  Cargo.toml  Rust workspace (open the repo root in VS Code and rust-analyzer finds it)
  sim/      Rust. The simulator.
    crates/sim-core   headless physics, hub firmware, runner protocol (+ sim-headless binary)
    crates/sim-app    the desktop app: 3D view and Driver Station controls (Bevy)
  robots/   robot configurations (TOML): hardware names, motor types, geometry
  robot/    Java (Gradle). The robot side.
    sdk/       the FTC SDK API (com.qualcomm.*, org.firstinspires.*), backed by the simulator
    runner/    plays the Robot Controller app: finds OpModes, runs them, talks to the sim
    TeamCode/  your OpModes, same package and layout as a Robot Controller project
```

## Running it

You need Rust (stable) and JDK 21 or newer. Gradle comes with the wrapper.

Terminal 1, the simulator (from the repo root):

```
cargo run --release -p sim-app -- --robot robots/strafer.toml
```

`--robot` picks the robot configuration (without it, a built-in strafer is used) and
`--start x,y,heading` sets where the robot starts, in inches and degrees on the FTC field
(origin at the center, +y away from the red wall).

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

### Running your team's code

Point the runner at your Robot Controller project (the folder with `TeamCode/` in it):

```
cd robot
gradlew :runner:run -Pteamcode=C:\path\to\your-team-repo
```

Your `TeamCode` sources are compiled as they are, at Java 8 like the Android build; nothing in
your repo is changed. Hardware names must match the simulated robot's configuration, just like
on a real robot: a name the simulator doesn't have fails with the SDK's usual "Unable to find a
hardware device" error.

Supported SDK surface so far: motors (including PIDF and RUN_TO_POSITION), servos and CR
servos, IMU, voltage sensor, gamepads with `...WasPressed()`, telemetry, `ElapsedTime`, the
goBILDA Pinpoint driver and the I2C device API it is built on (your own I2C drivers work too),
webcams, and VisionPortal/AprilTag (no detections yet).

## Robot configurations

A file in `robots/` is the simulator's version of the Robot Controller's hardware configuration,
plus the physical facts the simulator needs. See `robots/strafer.toml` for a small one and
`robots/20761.toml` for a full competition robot. In short:

```toml
name = "My robot"
mass_kg = 14.0
length_in = 17.0
width_in = 17.0
imu = "imu"

[drivetrain]
kind = "mecanum"                 # or "tank"
wheel_diameter_mm = 104
wheelbase_in = 12.0              # front axle to back axle
track_width_in = 13.5            # left wheel to right wheel
front_left = "fl"                # motor names
front_right = "fr"
back_left = "bl"
back_right = "br"
mirrored_side = "left"           # whose motors need REVERSE in code: "left", "right" or "none"

[[motors]]
name = "shooter"
type = "gobilda-6000"            # gobilda-117/223/312/435/1150/6000
hub = "Expansion Hub 2"          # default "Control Hub"
port = 0
load = { flywheel_mass_kg = 0.15, flywheel_diameter_mm = 96, friction_nm = 0.01 }
# or load = { inertia_kg_m2 = 0.0002, friction_nm = 0.05, gear_ratio = 2.0 }

[[servos]]
name = "claw"
kind = "servo"                   # "servo", "cr-servo" or "rgb-indicator"
port = 0

[[pinpoints]]
name = "odo"
x_pod_left_mm = 35               # where the pods really are on the robot
y_pod_forward_mm = 170
pods = "4-bar"                   # or "swingarm", or ticks_per_mm = ...
x_pod_reversed = true            # how the pods are really wired

[[webcams]]
name = "Webcam 1"
```

Motors that are not on the drivetrain spin their `load`, so a flywheel under
`RUN_USING_ENCODER` spins up and settles the way the real one does with your PIDF gains. The
Pinpoint is emulated at its I2C registers from the pods' real motion, so wrong offsets or
encoder directions in your code give wrong odometry, as on the robot.

### Headless, faster than real time

```
cargo run --release -p sim-core --bin sim-headless -- --opmode "Encoder Square" --robot robots/strafer.toml --seconds 30
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

- Drive motors on one side are mounted mirrored and need `REVERSE` (which side is in the config).
- `RUN_USING_ENCODER` and `RUN_TO_POSITION` are closed loops in the hub firmware (simulator
  side), not in the SDK. Switching to `RUN_TO_POSITION` before `setTargetPosition()` throws.
- Wheels slip. Push a wall and the encoders keep counting while the robot does not move.
- Mecanum rollers only grip across their axis. Strafing is slower than driving forward.
- Battery voltage sags under load, and motors get slower when it does.
- BRAKE is back-EMF braking from the motor model; FLOAT coasts.
- Velocity PIDF gains are in the hub's units (32767 = full power), so tuned numbers carry over.
- Time passes while a loop spins, even one that never touches hardware.
- A loop that ignores `opModeIsActive()` keeps running after STOP; hardware calls from a
  stopped OpMode fail, and one that still does not exit is reported as stuck in stop().

## Status

Early. Implemented: mecanum and tank drivetrains with goBILDA 5203 motors, mechanism motors
with flywheel/roller loads, servos, IMU, Pinpoint odometry, battery voltage, gamepads,
telemetry, `LinearOpMode` and iterative `OpMode`, robot configuration files, a field with walls,
and balls you can push around.

Not yet: mechanisms that touch game pieces (intakes, shooters launching balls), the DECODE
field, simulated camera detections, other sensors (distance, color, touch), a UI beyond the
keyboard Driver Station, and the IMU orientation check (the simulator trusts whatever
orientation you pass to `imu.initialize`).

## Tests

```
cargo test
cd robot && gradlew build
```
