//! Brushed DC gearmotor model, parameterised at the output shaft from datasheet numbers.
//!
//! Linear motor model: `torque = stall * V / V_nominal - (stall / free_speed) * omega`.
//! At zero applied volts this gives back-EMF braking, which is exactly what BRAKE mode
//! does on a real hub (it shorts the windings).

use std::f64::consts::PI;

const KG_CM: f64 = 0.098_066_5; // N·m per kg·cm (goBILDA publishes torque in kg·cm)
const ROTOR_INERTIA: f64 = 1.5e-6; // kg·m², approximate 5203-series rotor inertia at the motor

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSpec {
    pub name: &'static str,
    pub nominal_voltage: f64,
    pub free_speed_rpm: f64,
    /// N·m at the output shaft.
    pub stall_torque: f64,
    pub stall_current: f64,
    pub free_current: f64,
    /// Encoder counts per output-shaft revolution, as the SDK reports them.
    pub ticks_per_rev: f64,
    /// Rotor inertia reflected through the gearbox to the output shaft, kg·m².
    pub reflected_inertia: f64,
}

impl MotorSpec {
    const fn gobilda_5203(name: &'static str, rpm: f64, stall_kg_cm: f64, tpr: f64, ratio: f64) -> Self {
        Self {
            name,
            nominal_voltage: 12.0,
            free_speed_rpm: rpm,
            stall_torque: stall_kg_cm * KG_CM,
            stall_current: 9.2,
            free_current: 0.25,
            ticks_per_rev: tpr,
            reflected_inertia: ROTOR_INERTIA * ratio * ratio,
        }
    }

    pub const GOBILDA_117: Self = Self::gobilda_5203("goBILDA 5203 117 RPM", 117.0, 68.4, 1425.1, 50.9);
    pub const GOBILDA_223: Self = Self::gobilda_5203("goBILDA 5203 223 RPM", 223.0, 38.0, 751.8, 26.9);
    pub const GOBILDA_312: Self = Self::gobilda_5203("goBILDA 5203 312 RPM", 312.0, 24.3, 537.7, 19.2);
    pub const GOBILDA_435: Self = Self::gobilda_5203("goBILDA 5203 435 RPM", 435.0, 18.7, 384.5, 13.7);
    pub const GOBILDA_1150: Self = Self::gobilda_5203("goBILDA 5203 1150 RPM", 1150.0, 7.9, 145.1, 5.2);

    pub const ALL: [Self; 5] = [Self::GOBILDA_117, Self::GOBILDA_223, Self::GOBILDA_312, Self::GOBILDA_435, Self::GOBILDA_1150];

    /// Config-file names, e.g. "gobilda-312".
    const NAMES: [(&'static str, Self); 5] = [
        ("gobilda-117", Self::GOBILDA_117),
        ("gobilda-223", Self::GOBILDA_223),
        ("gobilda-312", Self::GOBILDA_312),
        ("gobilda-435", Self::GOBILDA_435),
        ("gobilda-1150", Self::GOBILDA_1150),
    ];

    pub fn by_name(name: &str) -> Option<Self> {
        Self::NAMES.iter().find(|(n, _)| *n == name).map(|(_, s)| *s)
    }

    pub fn names() -> Vec<&'static str> {
        Self::NAMES.iter().map(|(n, _)| *n).collect()
    }

    /// Free speed at nominal voltage, rad/s at the output.
    pub fn free_speed(&self) -> f64 {
        self.free_speed_rpm * 2.0 * PI / 60.0
    }

    /// Returns `(a, b)` such that output torque is `a - b * omega`.
    pub fn torque_coeffs(&self, volts: f64) -> (f64, f64) {
        (self.stall_torque * volts / self.nominal_voltage, self.stall_torque / self.free_speed())
    }

    /// Winding current for an applied voltage and output speed (signed, amps).
    pub fn current(&self, volts: f64, omega: f64) -> f64 {
        let resistance = self.nominal_voltage / self.stall_current;
        let back_emf = self.nominal_voltage * omega / self.free_speed();
        (volts - back_emf) / resistance
    }

    /// Encoder ticks per second at free speed and nominal voltage.
    pub fn max_ticks_per_sec(&self) -> f64 {
        self.free_speed_rpm / 60.0 * self.ticks_per_rev
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_speed_has_zero_torque_and_stall_has_full_torque() {
        let m = MotorSpec::GOBILDA_312;
        let (a, b) = m.torque_coeffs(12.0);
        assert!((a - b * m.free_speed()).abs() < 1e-9);
        assert!((a - m.stall_torque).abs() < 1e-9);
        assert!((m.current(12.0, 0.0) - m.stall_current).abs() < 1e-9);
    }
}
