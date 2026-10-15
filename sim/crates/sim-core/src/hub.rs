//! Emulation of the REV hub's motor-channel firmware.
//!
//! On a real robot, RUN_USING_ENCODER and RUN_TO_POSITION are closed-loop controllers running
//! on the hub, not in the SDK. So they live on the sim side too: the Java runner only sends
//! what the SDK would send over the wire (mode, power, target, zero-power behavior).

use crate::motor::MotorSpec;
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum RunMode {
    #[default]
    WithoutEncoder = 0,
    UsingEncoder = 1,
    ToPosition = 2,
    StopAndReset = 3,
}

impl RunMode {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::UsingEncoder,
            2 => Self::ToPosition,
            3 => Self::StopAndReset,
            _ => Self::WithoutEncoder,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum ZeroPower {
    #[default]
    Brake = 0,
    Float = 1,
}

impl ZeroPower {
    pub fn from_u8(v: u8) -> Self {
        if v == 1 { Self::Float } else { Self::Brake }
    }
}

/// What the SDK has asked of one motor port. Power and target are in the motor's raw
/// direction (the SDK applies `Direction.REVERSE` before sending).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorCommand {
    pub mode: RunMode,
    pub zero_power: ZeroPower,
    pub power: f64,
    pub target: i32,
    /// Velocity loop gains `[p, i, d, f]` in hub units (see [`MotorChannel`]).
    pub velocity_pidf: [f64; 4],
    /// RUN_TO_POSITION gain: ticks/s of target velocity per tick of position error.
    pub position_p: f64,
}

impl Default for MotorCommand {
    fn default() -> Self {
        Self {
            mode: RunMode::default(),
            zero_power: ZeroPower::default(),
            power: 0.0,
            target: 0,
            velocity_pidf: DEFAULT_VELOCITY_PIDF,
            position_p: DEFAULT_POSITION_P,
        }
    }
}

/// Electrical output of a channel for one step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Drive {
    Volts(f64),
    /// H-bridge off: windings open, no torque.
    Float,
}

/// The hub's power-on gains.
pub const DEFAULT_VELOCITY_PIDF: [f64; 4] = [10.0, 3.0, 0.0, 0.0];
pub const DEFAULT_POSITION_P: f64 = 10.0;
/// Hub controller output that means full power.
const FULL_SCALE: f64 = 32767.0;
/// The firmware integrates and differentiates per millisecond. This is our approximation of the
/// hub's units, chosen so the published defaults and tuning rules (F = 32767 / max velocity,
/// P = 0.1 F, I = 0.1 P) give a well-behaved loop.
const LOOP_TIME_UNIT: f64 = 0.001;

#[derive(Clone, Debug, Default)]
pub struct MotorChannel {
    pub cmd: MotorCommand,
    zero_angle: f64,
    integral: f64,
    last_error: f64,
}

impl MotorChannel {
    pub fn ticks(&self, shaft_angle: f64, spec: &MotorSpec) -> i32 {
        ((shaft_angle - self.zero_angle) / TAU * spec.ticks_per_rev).round() as i32
    }

    pub fn update(&mut self, shaft_angle: f64, shaft_omega: f64, spec: &MotorSpec, battery: f64, dt: f64) -> Drive {
        let max = spec.max_ticks_per_sec();
        let velocity = shaft_omega / TAU * spec.ticks_per_rev;
        let power = self.cmd.power.clamp(-1.0, 1.0);
        let duty = match self.cmd.mode {
            RunMode::StopAndReset => {
                self.zero_angle = shaft_angle;
                0.0
            }
            _ if power == 0.0 => 0.0,
            RunMode::WithoutEncoder => power,
            RunMode::UsingEncoder => self.velocity_loop(power * max, velocity, dt),
            RunMode::ToPosition => {
                let error = (self.cmd.target - self.ticks(shaft_angle, spec)) as f64;
                let limit = power.abs() * max;
                self.velocity_loop((error * self.cmd.position_p).clamp(-limit, limit), velocity, dt)
            }
        }
        .clamp(-1.0, 1.0);

        if duty == 0.0 {
            self.integral = 0.0;
            self.last_error = 0.0;
            match self.cmd.zero_power {
                ZeroPower::Brake => Drive::Volts(0.0),
                ZeroPower::Float => Drive::Float,
            }
        } else {
            Drive::Volts(duty * battery)
        }
    }

    /// Velocity PIDF in hub units: errors in ticks/s, output on a 32767 = full power scale.
    fn velocity_loop(&mut self, target: f64, velocity: f64, dt: f64) -> f64 {
        let [p, i, d, f] = self.cmd.velocity_pidf;
        let error = target - velocity;
        let units = dt / LOOP_TIME_UNIT;
        if i != 0.0 {
            // Anti-windup: the integral alone can never ask for more than full power.
            let limit = FULL_SCALE / i.abs();
            self.integral = (self.integral + error * units).clamp(-limit, limit);
        }
        let derivative = (error - self.last_error) / units;
        self.last_error = error;
        (p * error + i * self.integral + d * derivative + f * target) / FULL_SCALE
    }
}
