//! Headless simulation core.
//!
//! Everything physical lives here so it can be tested and fast-forwarded without a window:
//! the field, rigid bodies (via Rapier), DC motor and battery models, wheel/tire contact,
//! the REV hub motor-controller firmware, and the wire protocol spoken with the Java runner.
//!
//! Coordinates: physics is SI and Y-up. A robot's local frame is +X forward, +Y up, +Z right.
//! The FTC field frame (x right, y away from the red wall) maps to physics as
//! `phys.x = field.x`, `phys.z = -field.y`.

pub mod gamepad;
pub mod hub;
pub mod motor;
pub mod robot;
pub mod units;
pub mod world;

pub use world::{Pose2d, SimWorld};
