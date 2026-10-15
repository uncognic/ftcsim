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
use crate::robot::{Load, MotorConfig, PinpointConfig, RobotConfig, ServoConfig, ServoKind, WheelConfig, WheelKind};
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
    #[serde(default)]
    servos: Vec<Servo>,
    #[serde(default)]
    pinpoints: Vec<Pinpoint>,
    #[serde(default)]
    webcams: Vec<Webcam>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Webcam {
    name: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Pinpoint {
    name: String,
    #[serde(default = "default_hub")]
    hub: String,
    #[serde(default)]
    i2c_bus: u8,
    x_pod_left_mm: f64,
    y_pod_forward_mm: f64,
    /// "4-bar" or "swingarm" (goBILDA pods), or give ticks_per_mm.
    pods: Option<String>,
    ticks_per_mm: Option<f64>,
    #[serde(default)]
    x_pod_reversed: bool,
    #[serde(default)]
    y_pod_reversed: bool,
}

/// goBILDA's published resolutions.
const FOUR_BAR_POD_TICKS_PER_MM: f64 = 19.89436789;
const SWINGARM_POD_TICKS_PER_MM: f64 = 13.26291192;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Servo {
    name: String,
    #[serde(default = "default_servo_kind")]
    kind: String,
    #[serde(default = "default_hub")]
    hub: String,
    port: u8,
    /// Defaults match a goBILDA 2000-series torque servo: 300° travel, 0.25 s per 60°.
    #[serde(default = "default_servo_range")]
    range_deg: f64,
    #[serde(default = "default_servo_speed")]
    speed_deg_per_s: f64,
}

fn default_servo_kind() -> String {
    "servo".into()
}

fn default_servo_range() -> f64 {
    300.0
}

fn default_servo_speed() -> f64 {
    240.0
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
    /// Which side's motors face the other way, so positive power drives those wheels backward:
    /// "left" (the usual build, so the left motors get REVERSE in code), "right", or "none".
    #[serde(default = "default_mirrored_side")]
    mirrored_side: String,
    #[serde(default = "default_friction")]
    wheel_friction: f64,
}

fn default_mirrored_side() -> String {
    "left".into()
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
    /// What a mechanism motor turns. Omit for drivetrain motors.
    load: Option<LoadFile>,
}

/// Either give `inertia_kg_m2` directly, or a flywheel's mass and diameter (treated as a solid disc).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LoadFile {
    inertia_kg_m2: Option<f64>,
    flywheel_mass_kg: Option<f64>,
    flywheel_diameter_mm: Option<f64>,
    #[serde(default)]
    friction_nm: f64,
    #[serde(default = "one")]
    gear_ratio: f64,
}

fn one() -> f64 {
    1.0
}

impl LoadFile {
    fn to_load(&self, motor: &str) -> Result<Load, String> {
        let inertia = match (self.inertia_kg_m2, self.flywheel_mass_kg, self.flywheel_diameter_mm) {
            (Some(i), None, None) => i,
            (None, Some(m), Some(d)) => 0.5 * m * (d / 2000.0).powi(2),
            (None, None, None) => 0.0,
            _ => return Err(format!("motor \"{motor}\": give either inertia_kg_m2 or flywheel_mass_kg with flywheel_diameter_mm")),
        };
        if self.gear_ratio <= 0.0 {
            return Err(format!("motor \"{motor}\": gear_ratio must be positive"));
        }
        Ok(Load { inertia, friction: self.friction_nm, ratio: self.gear_ratio })
    }
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

        let mut names = HashSet::new();
        let all_names = file.motors.iter().map(|m| &m.name)
            .chain(file.servos.iter().map(|s| &s.name))
            .chain(file.pinpoints.iter().map(|p| &p.name))
            .chain(file.webcams.iter().map(|w| &w.name))
            .chain(std::iter::once(&file.imu));
        for name in all_names {
            if !names.insert(name.as_str()) {
                return Err(format!("two devices are named \"{name}\""));
            }
        }

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
            let load = match &m.load {
                Some(l) => l.to_load(&m.name)?,
                None => Load::default(),
            };
            motors.push(MotorConfig { name: m.name.clone(), hub: m.hub.clone(), port: m.port, spec, load });
        }

        let mut servo_ports = HashSet::new();
        let mut servos = Vec::new();
        for s in &file.servos {
            let kind = match s.kind.as_str() {
                "servo" => ServoKind::Positional,
                "cr-servo" => ServoKind::Continuous,
                "rgb-indicator" => ServoKind::RgbIndicator,
                k => return Err(format!("servo \"{}\": unknown kind \"{k}\" (known: servo, cr-servo, rgb-indicator)", s.name)),
            };
            if s.port > 5 {
                return Err(format!("servo \"{}\": port {} does not exist (hubs have servo ports 0-5)", s.name, s.port));
            }
            if !servo_ports.insert((s.hub.clone(), s.port)) {
                return Err(format!("servo \"{}\": {} servo port {} is already used", s.name, s.hub, s.port));
            }
            servos.push(ServoConfig {
                name: s.name.clone(),
                hub: s.hub.clone(),
                port: s.port,
                kind,
                range_deg: s.range_deg,
                speed_deg_per_s: s.speed_deg_per_s,
            });
        }

        let mut pinpoints = Vec::new();
        for p in &file.pinpoints {
            let ticks_per_mm = match (p.pods.as_deref(), p.ticks_per_mm) {
                (Some("4-bar"), None) => FOUR_BAR_POD_TICKS_PER_MM,
                (Some("swingarm"), None) => SWINGARM_POD_TICKS_PER_MM,
                (None, Some(t)) if t > 0.0 => t,
                _ => return Err(format!("pinpoint \"{}\": give pods = \"4-bar\" or \"swingarm\", or a positive ticks_per_mm", p.name)),
            };
            pinpoints.push(PinpointConfig {
                name: p.name.clone(),
                hub: p.hub.clone(),
                i2c_bus: p.i2c_bus,
                x_pod_left_mm: p.x_pod_left_mm,
                y_pod_forward_mm: p.y_pod_forward_mm,
                ticks_per_mm,
                x_pod_reversed: p.x_pod_reversed,
                y_pod_reversed: p.y_pod_reversed,
            });
        }

        let d = &file.drivetrain;
        let find = |name: &str| {
            motors.iter().position(|m| m.name == name).ok_or_else(|| format!("drivetrain uses motor \"{name}\", which is not in [[motors]]"))
        };
        let (fx, sz) = (d.wheelbase_in * INCH / 2.0, d.track_width_in * INCH / 2.0);
        let radius = d.wheel_diameter_mm / 2000.0;
        let (left_sign, right_sign) = match d.mirrored_side.as_str() {
            "left" => (-1.0, 1.0),
            "right" => (1.0, -1.0),
            "none" => (1.0, 1.0),
            other => return Err(format!("drivetrain: mirrored_side must be \"left\", \"right\" or \"none\", not \"{other}\"")),
        };
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
            wheel(find(&d.front_right)?, fx, sz, false, right_sign),
            wheel(find(&d.back_left)?, -fx, -sz, false, left_sign),
            wheel(find(&d.back_right)?, -fx, sz, true, right_sign),
        ];

        Ok(Self {
            name: file.name,
            length: file.length_in * INCH,
            width: file.width_in * INCH,
            frame_height: file.frame_height_in * INCH,
            mass: file.mass_kg,
            motors,
            wheels,
            servos,
            pinpoints,
            webcams: file.webcams.into_iter().map(|w| w.name).collect(),
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
    fn mirrored_side_sets_which_motors_run_backward() {
        let right = STRAFER.replace("kind = \"mecanum\"", "kind = \"mecanum\"
mirrored_side = \"right\"");
        let signs: Vec<f64> = RobotConfig::from_toml(&right).unwrap().wheels.iter().map(|w| w.mount_sign).collect();
        assert_eq!(signs, vec![1.0, -1.0, 1.0, -1.0]);
    }

    #[test]
    fn rejects_duplicate_device_names_across_kinds() {
        let dup = format!("{STRAFER}\n[[servos]]\nname = \"odo\"\nport = 0\n");
        assert!(RobotConfig::from_toml(&dup).unwrap_err().contains("two devices are named \"odo\""));
    }

    #[test]
    fn rejects_unknown_motor_and_reused_port() {
        let bad_type = STRAFER.replacen("gobilda-312", "gobilda-999", 1);
        assert!(RobotConfig::from_toml(&bad_type).unwrap_err().contains("unknown type"));
        let reused = STRAFER.replacen("port = 1", "port = 0", 1);
        assert!(RobotConfig::from_toml(&reused).unwrap_err().contains("already used"));
    }
}
