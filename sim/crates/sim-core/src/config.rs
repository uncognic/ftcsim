//! Robot configuration files (TOML).
//!
//! A config file plays the role of the Robot Controller's hardware configuration: it names every
//! device the OpMode can look up, and adds what the simulator needs to know physically (motor
//! models, wheel geometry, mass). Lengths are in inches and millimeters as FTC parts are sold.
//!
//! ```toml
//! name = "Strafer"
//! mass_kg = 12.0
//! length_in = 17.0
//! width_in = 16.5
//! imu = "imu"
//!
//! [drivetrain]
//! kind = "mecanum"
//! wheel_diameter_mm = 104
//! wheelbase_in = 11.8
//! track_width_in = 13.0
//! front_left = "frontLeft"
//! front_right = "frontRight"
//! back_left = "backLeft"
//! back_right = "backRight"
//!
//! [[motors]]
//! name = "frontLeft"
//! type = "gobilda-312"
//! port = 0
//! ```

use crate::motor::MotorSpec;
use crate::robot::{MotorConfig, RobotConfig, WheelConfig, WheelKind};
use crate::units::INCH;
use serde::Deserialize;
use std::collections::HashSet;
use std::path::Path;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    name: String,
    mass_kg: f32,
    length_in: f32,
    width_in: f32,
    #[serde(default = "default_frame_height")]
    frame_height_in: f32,
    #[serde(default = "default_imu")]
    imu: String,
    drivetrain: Drivetrain,
    #[serde(default)]
    motors: Vec<Motor>,
}

fn default_frame_height() -> f32 {
    2.4
}

fn default_imu() -> String {
    "imu".into()
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Drivetrain {
    kind: DriveKind,
    wheel_diameter_mm: f32,
    /// Front axle to back axle.
    wheelbase_in: f32,
    /// Left wheel center to right wheel center.
    track_width_in: f32,
    front_left: String,
    front_right: String,
    back_left: String,
    back_right: String,
    /// Left motors face the other way, so positive power drives them backward (the usual build).
    #[serde(default = "yes")]
    left_motors_mirrored: bool,
    #[serde(default = "default_friction")]
    wheel_friction: f64,
}

fn yes() -> bool {
    true
}

fn default_friction() -> f64 {
    0.9
}

#[derive(Deserialize, Clone, Copy, PartialEq)]
#[serde(rename_all = "lowercase")]
enum DriveKind {
    Mecanum,
    Tank,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Motor {
    name: String,
    #[serde(rename = "type")]
    kind: String,
    #[serde(default = "default_hub")]
    hub: String,
    port: u8,
}

fn default_hub() -> String {
    "Control Hub".into()
}

impl RobotConfig {
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        Self::from_toml(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn from_toml(text: &str) -> Result<Self, String> {
        let file: File = toml::from_str(text).map_err(|e| e.to_string())?;

        let mut ports = HashSet::new();
        let mut motors = Vec::new();
        for m in &file.motors {
            let spec = MotorSpec::by_name(&m.kind)
                .ok_or_else(|| format!("motor \"{}\": unknown type \"{}\" (known: {})", m.name, m.kind, MotorSpec::names().join(", ")))?;
            if m.port > 3 {
                return Err(format!("motor \"{}\": port {} does not exist (hubs have motor ports 0-3)", m.name, m.port));
            }
            if !ports.insert((m.hub.clone(), m.port)) {
                return Err(format!("motor \"{}\": {} port {} is already used", m.name, m.hub, m.port));
            }
            if motors.iter().any(|x: &MotorConfig| x.name == m.name) {
                return Err(format!("two devices are named \"{}\"", m.name));
            }
            motors.push(MotorConfig { name: m.name.clone(), hub: m.hub.clone(), port: m.port, spec });
        }

        let d = &file.drivetrain;
        let find = |name: &str| {
            motors.iter().position(|m| m.name == name).ok_or_else(|| format!("drivetrain uses motor \"{name}\", which is not in [[motors]]"))
        };
        let (fx, sz) = (d.wheelbase_in * INCH / 2.0, d.track_width_in * INCH / 2.0);
        let radius = d.wheel_diameter_mm / 2000.0;
        let left_sign = if d.left_motors_mirrored { -1.0 } else { 1.0 };
        let wheel = |motor, x: f32, z: f32, rollers_right, mount_sign| WheelConfig {
            motor,
            position: [x, 0.0, z],
            radius,
            mass: 0.2,
            kind: match d.kind {
                DriveKind::Mecanum => WheelKind::Mecanum { rollers_right },
                DriveKind::Tank => WheelKind::Traction,
            },
            mount_sign,
            friction: d.wheel_friction,
        };
        let wheels = vec![
            wheel(find(&d.front_left)?, fx, -sz, true, left_sign),
            wheel(find(&d.front_right)?, fx, sz, false, 1.0),
            wheel(find(&d.back_left)?, -fx, -sz, false, left_sign),
            wheel(find(&d.back_right)?, -fx, sz, true, 1.0),
        ];

        Ok(Self {
            name: file.name,
            length: file.length_in * INCH,
            width: file.width_in * INCH,
            frame_height: file.frame_height_in * INCH,
            mass: file.mass_kg,
            motors,
            wheels,
            imu_name: file.imu,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STRAFER: &str = include_str!("../../../../robots/strafer.toml");

    #[test]
    fn strafer_file_matches_builtin_default() {
        let file = RobotConfig::from_toml(STRAFER).unwrap();
        let builtin = RobotConfig::default_mecanum();
        assert_eq!(file.motors.len(), builtin.motors.len());
        for (a, b) in file.wheels.iter().zip(&builtin.wheels) {
            assert_eq!(a.motor, b.motor);
            assert_eq!(a.mount_sign, b.mount_sign);
            assert_eq!(a.kind, b.kind);
            for k in 0..3 {
                assert!((a.position[k] - b.position[k]).abs() < 1e-4, "{:?} vs {:?}", a.position, b.position);
            }
        }
    }

    #[test]
    fn rejects_unknown_motor_and_reused_port() {
        let bad_type = STRAFER.replacen("gobilda-312", "gobilda-999", 1);
        assert!(RobotConfig::from_toml(&bad_type).unwrap_err().contains("unknown type"));
        let reused = STRAFER.replacen("port = 1", "port = 0", 1);
        assert!(RobotConfig::from_toml(&reused).unwrap_err().contains("already used"));
    }
}
