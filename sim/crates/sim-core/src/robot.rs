//! Robot description: what hardware exists and where it is mounted.
//! This plays the role of the Robot Controller's hardware configuration file.

use crate::motor::MotorSpec;
use crate::units::INCH;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WheelKind {
    /// Rollers at 45°. `rollers_right` means that spinning the wheel forward pushes the robot
    /// forward and to the right (front-left and back-right wheels in a standard X layout).
    Mecanum { rollers_right: bool },
    /// Grips in both directions.
    Traction,
    /// Rollers at 90°: drives forward/back, rolls freely sideways.
    Omni,
}

/// What a non-drivetrain motor turns: a flywheel, roller, spool... modelled as inertia plus friction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Load {
    /// kg·m² at the load shaft.
    pub inertia: f64,
    /// Friction torque at the load shaft, N·m.
    pub friction: f64,
    /// Motor turns per load turn (2.0 means the load spins at half the motor's speed).
    pub ratio: f64,
}

impl Default for Load {
    fn default() -> Self {
        Self { inertia: 0.0, friction: 0.0, ratio: 1.0 }
    }
}

#[derive(Clone, Debug)]
pub struct MotorConfig {
    /// Name used with `hardwareMap.get(...)`.
    pub name: String,
    /// "Control Hub" or an Expansion Hub's name.
    pub hub: String,
    pub port: u8,
    pub spec: MotorSpec,
    /// Ignored for drivetrain motors, whose load is the robot.
    pub load: Load,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServoKind {
    /// Holds an angle across its range; position 0..1 maps to the ends of the range.
    Positional,
    /// Continuous rotation: the SDK's power -1..1 sets speed.
    Continuous,
    /// goBILDA RGB Indicator Light: the servo position selects a color.
    RgbIndicator,
}

#[derive(Clone, Debug)]
pub struct ServoConfig {
    pub name: String,
    pub hub: String,
    pub port: u8,
    pub kind: ServoKind,
    /// Full travel in degrees for positional servos.
    pub range_deg: f64,
    /// No-load speed in degrees per second.
    pub speed_deg_per_s: f64,
}

#[derive(Clone, Debug)]
pub struct WheelConfig {
    /// Index into [`RobotConfig::motors`].
    pub motor: usize,
    /// Mount point in the robot frame (+X forward, +Y up, +Z right), meters.
    pub position: [f32; 3],
    pub radius: f32,
    pub mass: f32,
    pub kind: WheelKind,
    /// +1 if positive raw motor power drives this wheel forward, -1 if the motor is mounted
    /// mirrored (the usual case for left-side drive motors).
    pub mount_sign: f64,
    pub friction: f64,
}

#[derive(Clone, Debug)]
pub struct RobotConfig {
    pub name: String,
    pub length: f32,
    pub width: f32,
    pub frame_height: f32,
    pub mass: f32,
    pub motors: Vec<MotorConfig>,
    pub wheels: Vec<WheelConfig>,
    pub servos: Vec<ServoConfig>,
    pub imu_name: String,
}

impl RobotConfig {
    /// A typical goBILDA strafer: 312 RPM motors on 104 mm mecanum wheels, left side mirrored.
    pub fn default_mecanum() -> Self {
        let motor = |name: &str, port| MotorConfig { name: name.into(), hub: "Control Hub".into(), port, spec: MotorSpec::GOBILDA_312, load: Load::default() };
        let (fx, sz) = (5.9 * INCH, 6.5 * INCH);
        let wheel = |motor, x, z, rollers_right, mount_sign| WheelConfig {
            motor,
            position: [x, 0.0, z],
            radius: 0.052,
            mass: 0.2,
            kind: WheelKind::Mecanum { rollers_right },
            mount_sign,
            friction: 0.9,
        };
        Self {
            name: "Strafer".into(),
            length: 17.0 * INCH,
            width: 16.5 * INCH,
            frame_height: 2.4 * INCH,
            mass: 12.0,
            motors: vec![motor("frontLeft", 0), motor("frontRight", 1), motor("backLeft", 2), motor("backRight", 3)],
            wheels: vec![
                wheel(0, fx, -sz, true, -1.0),
                wheel(1, fx, sz, false, 1.0),
                wheel(2, -fx, -sz, false, -1.0),
                wheel(3, -fx, sz, true, 1.0),
            ],
            servos: Vec::new(),
            imu_name: "imu".into(),
        }
    }
}
