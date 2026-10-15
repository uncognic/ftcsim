//! The simulated world: field, game pieces, and one robot, stepped at a fixed rate.
//!
//! Wheels are not separate rigid bodies. Each wheel is a raycast suspension point with its own
//! spin state, a motor driving it through the hub firmware, and a tire contact model:
//!
//! * normal force from a stiff spring-damper (so the robot can tip, bounce, and ride over things)
//! * mecanum wheels only transmit force across their roller axis; along it the rollers spin freely
//! * friction saturates at `mu * N`, so wheels slip when pushing a wall or accelerating too hard
//!
//! The wheel spin and the tire force are solved together implicitly each step, which keeps the
//! stiff wheel/ground coupling stable at a 400 Hz physics rate.

use crate::gamepad::GamepadState;
use crate::hub::{Drive, MotorChannel};
use crate::robot::{RobotConfig, WheelKind};
use crate::units::*;
use rapier3d::prelude::*;

pub const PHYSICS_HZ: f64 = 400.0;

pub type Iso = Isometry<Real>;

pub const SUSPENSION_REST: f32 = 0.015;
const SUSPENSION_K: f32 = 20_000.0;
const SUSPENSION_C: f32 = 600.0;
/// Height of the box used to compute chassis inertia.
const EFFECTIVE_HEIGHT: f32 = 0.25;
/// Slip speed (m/s) at which tire friction reaches its full value.
const SLIP_SCALE: f64 = 0.05;
/// Fraction of the normal force that free-spinning mecanum rollers resist with.
const ROLLER_DRAG: f64 = 0.03;
const BEARING_DRAG: f64 = 2e-4;
/// Rolling resistance as a fraction of wheel load (mecanum rollers on foam tiles are lossy).
const ROLLING_RESISTANCE: f64 = 0.04;
/// Wheel speed (rad/s) over which Coulomb-style losses ramp in, to avoid chatter at rest.
const LOSS_SMOOTHING: f64 = 0.5;

/// Pose on the FTC field: x right, y away from the red wall (meters), heading CCW from +x (rad).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2d {
    pub x: f32,
    pub y: f32,
    pub heading: f32,
}

impl Pose2d {
    pub fn new(x: f32, y: f32, heading: f32) -> Self {
        Self { x, y, heading }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WheelState {
    /// rad/s, positive drives the robot forward.
    pub omega: f64,
    pub angle: f64,
    pub compression: f32,
    pub load: f32,
    /// Tire force along the wheel's drive direction, N.
    pub drive_force: f32,
    pub slipping: bool,
}

#[derive(Clone, Debug, Default)]
pub struct MotorState {
    pub channel: MotorChannel,
    pub shaft_angle: f64,
    pub shaft_omega: f64,
    pub volts: f64,
    pub current: f64,
}

/// A servo's output shaft. Positional servos report position 0..1 of their range; continuous
/// servos accumulate angle. The SDK gets no feedback from either, as on a real robot.
#[derive(Clone, Copy, Debug, Default)]
pub struct ServoState {
    /// What the SDK last sent: position 0..1 or continuous power -1..1.
    pub command: f64,
    /// False until the first command: real hubs send no PWM until a servo is set.
    pub enabled: bool,
    pub position: f64,
    pub angle_deg: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Battery {
    pub open_circuit: f64,
    pub internal_resistance: f64,
    pub voltage: f64,
    pub current: f64,
}

impl Default for Battery {
    fn default() -> Self {
        // A charged 12 V NiMH FTC pack, including wiring and connector resistance.
        Self { open_circuit: 13.2, internal_resistance: 0.09, voltage: 13.2, current: 0.0 }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ImuReading {
    /// Radians. Yaw is CCW-positive about up, pitch is nose-up, roll is right-side-down.
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    /// Robot-frame rates, rad/s, in FTC IMU axes: x right, y forward, z up.
    pub rate_x: f32,
    pub rate_y: f32,
    pub rate_z: f32,
}

pub struct GamePiece {
    pub body: RigidBodyHandle,
    pub radius: f32,
    pub color: [f32; 3],
}

struct Physics {
    pipeline: PhysicsPipeline,
    params: IntegrationParameters,
    islands: IslandManager,
    broad: DefaultBroadPhase,
    narrow: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    query: QueryPipeline,
}

impl Physics {
    fn new(dt: f32) -> Self {
        let params = IntegrationParameters { dt, ..Default::default() };
        Self {
            pipeline: PhysicsPipeline::new(),
            params,
            islands: IslandManager::new(),
            broad: DefaultBroadPhase::new(),
            narrow: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            query: QueryPipeline::new(),
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            &vector![0.0, -GRAVITY, 0.0],
            &self.params,
            &mut self.islands,
            &mut self.broad,
            &mut self.narrow,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            Some(&mut self.query),
            &(),
            &(),
        );
    }
}

pub struct SimWorld {
    pub time: f64,
    pub config: RobotConfig,
    pub wheels: Vec<WheelState>,
    pub motors: Vec<MotorState>,
    pub servos: Vec<ServoState>,
    pub battery: Battery,
    pub gamepads: [GamepadState; 2],
    pub pieces: Vec<GamePiece>,
    chassis: RigidBodyHandle,
    physics: Physics,
}

impl SimWorld {
    pub fn new(config: RobotConfig, start: Pose2d) -> Self {
        let mut physics = Physics::new((1.0 / PHYSICS_HZ) as f32);
        build_field(&mut physics);
        let chassis = spawn_chassis(&mut physics, &config, start);
        let mut world = Self {
            time: 0.0,
            wheels: vec![WheelState::default(); config.wheels.len()],
            motors: vec![MotorState::default(); config.motors.len()],
            servos: vec![ServoState { position: 0.5, ..Default::default() }; config.servos.len()],
            config,
            battery: Battery::default(),
            gamepads: [GamepadState::default(); 2],
            pieces: Vec::new(),
            chassis,
            physics,
        };
        world.spawn_balls();
        world.physics.query.update(&world.physics.colliders);
        world
    }

    pub fn dt(&self) -> f64 {
        self.physics.params.dt as f64
    }

    /// Default start: against the red wall, facing away from it.
    pub fn default_start() -> Pose2d {
        Pose2d::new(-1.5 * TILE, -FIELD_SIZE / 2.0 + 9.0 * INCH, std::f32::consts::FRAC_PI_2)
    }

    /// Put the robot back at `pose`, at rest. Encoders keep their counts, like a real robot
    /// that is picked up and moved.
    pub fn place_robot(&mut self, pose: Pose2d) {
        let body = &mut self.physics.bodies[self.chassis];
        body.set_position(chassis_isometry(pose), true);
        body.set_linvel(vector![0.0, 0.0, 0.0], true);
        body.set_angvel(vector![0.0, 0.0, 0.0], true);
        for w in &mut self.wheels {
            w.omega = 0.0;
        }
    }

    pub fn chassis_position(&self) -> Isometry<Real> {
        *self.physics.bodies[self.chassis].position()
    }

    pub fn piece_position(&self, piece: &GamePiece) -> Isometry<Real> {
        *self.physics.bodies[piece.body].position()
    }

    pub fn pose(&self) -> Pose2d {
        let iso = self.chassis_position();
        let fwd = iso.rotation * vector![1.0, 0.0, 0.0];
        Pose2d::new(iso.translation.x, -iso.translation.z, (-fwd.z).atan2(fwd.x))
    }

    pub fn encoder(&self, motor: usize) -> (i32, f64) {
        let m = &self.motors[motor];
        let spec = &self.config.motors[motor].spec;
        (m.channel.ticks(m.shaft_angle, spec), m.shaft_omega / std::f64::consts::TAU * spec.ticks_per_rev)
    }

    pub fn imu(&self) -> ImuReading {
        let body = &self.physics.bodies[self.chassis];
        let rot = body.position().rotation;
        let fwd = rot * vector![1.0, 0.0, 0.0];
        let right = rot * vector![0.0, 0.0, 1.0];
        let local_rate = rot.inverse() * body.angvel();
        ImuReading {
            yaw: (-fwd.z).atan2(fwd.x),
            pitch: fwd.y.clamp(-1.0, 1.0).asin(),
            roll: (-right.y).clamp(-1.0, 1.0).asin(),
            rate_x: local_rate.z,
            rate_y: local_rate.x,
            rate_z: local_rate.y,
        }
    }

    pub fn step(&mut self) {
        let dt = self.dt();
        let battery = self.battery.voltage;
        let drives: Vec<Drive> = self
            .motors
            .iter_mut()
            .zip(&self.config.motors)
            .map(|(m, cfg)| m.channel.update(m.shaft_angle, m.shaft_omega, &cfg.spec, battery, dt))
            .collect();

        let forces = self.wheel_forces(&drives, dt);
        let body = &mut self.physics.bodies[self.chassis];
        // Rapier keeps user forces AND torques until reset; add_force_at_point writes both.
        body.reset_forces(true);
        body.reset_torques(true);
        for (force, point) in forces {
            body.add_force_at_point(force, point, true);
        }

        // Motors follow their wheels; battery sags with the total draw.
        let mut draw = 0.0;
        for (i, m) in self.motors.iter_mut().enumerate() {
            let spec = &self.config.motors[i].spec;
            if let Some((w, cfg)) = self.wheels.iter().zip(&self.config.wheels).find(|(_, c)| c.motor == i) {
                m.shaft_omega = cfg.mount_sign * w.omega;
                m.shaft_angle = cfg.mount_sign * w.angle;
            } else {
                step_mechanism(m, spec, &self.config.motors[i].load, drives[i], dt);
            }
            match drives[i] {
                Drive::Volts(v) => {
                    m.volts = v;
                    m.current = spec.current(v, m.shaft_omega);
                    draw += (m.current * v / battery.max(1.0)).max(0.0);
                }
                Drive::Float => {
                    m.volts = 0.0;
                    m.current = 0.0;
                }
            }
        }
        self.battery.current = draw;
        self.battery.voltage = (self.battery.open_circuit - self.battery.internal_resistance * draw).max(5.0);

        for (s, cfg) in self.servos.iter_mut().zip(&self.config.servos) {
            step_servo(s, cfg, dt);
        }

        self.physics.step();
        self.time += dt;
    }

    /// Solves each wheel's spin together with its tire force; returns forces to apply to the chassis.
    fn wheel_forces(&mut self, drives: &[Drive], dt: f64) -> Vec<(Vector<Real>, Point<Real>)> {
        let p = &self.physics;
        let body = &p.bodies[self.chassis];
        let iso = *body.position();
        let com = *body.center_of_mass();
        let (linvel, angvel) = (*body.linvel(), *body.angvel());
        let filter = QueryFilter::default().exclude_rigid_body(self.chassis);
        let down = iso.rotation * vector![0.0, -1.0, 0.0];
        let mut out = Vec::with_capacity(self.wheels.len());

        for (ws, wc) in self.wheels.iter_mut().zip(&self.config.wheels) {
            let mount = iso * point![wc.position[0], wc.position[1], wc.position[2]];
            let reach = SUSPENSION_REST + wc.radius;
            let ray = Ray::new(mount, down);
            let hit = p.query.cast_ray_and_get_normal(&p.bodies, &p.colliders, &ray, reach, true, filter);

            let spec = &self.config.motors[wc.motor].spec;
            let (a, b) = match drives[wc.motor] {
                Drive::Volts(v) => spec.torque_coeffs(v),
                Drive::Float => (0.0, 0.0),
            };
            // Gearbox friction, inferred from the datasheet's no-load current.
            let gearbox_loss = spec.stall_torque * spec.free_current / spec.stall_current;
            let spin = (ws.omega / LOSS_SMOOTHING).tanh();
            // Wheel-side torque = a_w - b_w * omega.
            let a_w = wc.mount_sign * a - gearbox_loss * spin;
            let b_w = b + BEARING_DRAG;
            let r = wc.radius as f64;
            let inertia = 0.5 * wc.mass as f64 * r * r + spec.reflected_inertia;

            let Some((_, hit)) = hit else {
                ws.compression = 0.0;
                ws.load = 0.0;
                ws.slipping = false;
                ws.omega = (inertia * ws.omega + dt * a_w) / (inertia + dt * b_w);
                ws.angle += ws.omega * dt;
                continue;
            };

            let compression = reach - hit.time_of_impact;
            let normal = hit.normal;
            let contact = mount + down * hit.time_of_impact;
            // Compression rate from the body's actual velocity, not a finite difference
            // (a one-step lag in the damper is enough to make the pitch mode unstable).
            let rate = -(linvel + angvel.cross(&(mount - com))).dot(&normal);
            let load = (SUSPENSION_K * compression + SUSPENSION_C * rate).max(0.0);
            ws.compression = compression;
            ws.load = load;

            let in_plane = |v: Vector<Real>| v - normal * v.dot(&normal);
            let fwd = in_plane(iso.rotation * vector![1.0, 0.0, 0.0]).normalize();
            let right = in_plane(iso.rotation * vector![0.0, 0.0, 1.0]).normalize();
            let v = in_plane(linvel + angvel.cross(&(contact - com)));

            let mu = wc.friction;
            let n_force = load as f64;
            let a_w = a_w - ROLLING_RESISTANCE * n_force * r * spin;
            let max_friction = mu * n_force;
            let k = max_friction / SLIP_SCALE;

            // Drive direction: the direction this wheel can push. Free direction: rolls freely.
            let (drive_dir, free_dir, lateral_grip) = match wc.kind {
                WheelKind::Mecanum { rollers_right } => {
                    let d = (fwd + right * if rollers_right { 1.0 } else { -1.0 }).normalize();
                    (d, normal.cross(&d).normalize(), false)
                }
                WheelKind::Omni => (fwd, right, false),
                WheelKind::Traction => (fwd, right, true),
            };
            let along = fwd.dot(&drive_dir) as f64; // how much wheel spin projects onto the drive direction
            let v_drive = v.dot(&drive_dir) as f64;

            // Implicit solve of wheel spin with linearised (unsaturated) tire force.
            let mut omega = (inertia * ws.omega + dt * (a_w + r * along * k * v_drive))
                / (inertia + dt * (b_w + r * r * along * along * k));
            let mut f_drive = -k * (v_drive - r * omega * along);
            let mut f_free = if lateral_grip {
                -k * v.dot(&free_dir) as f64
            } else {
                (-(v.dot(&free_dir) as f64) * k).clamp(-ROLLER_DRAG * n_force, ROLLER_DRAG * n_force)
            };
            let total = f_drive.hypot(if lateral_grip { f_free } else { 0.0 });
            ws.slipping = total > max_friction;
            if ws.slipping {
                let scale = max_friction / total;
                f_drive *= scale;
                if lateral_grip {
                    f_free *= scale;
                }
                omega = (inertia * ws.omega + dt * (a_w - r * along * f_drive)) / (inertia + dt * b_w);
            }
            ws.omega = omega;
            ws.angle += omega * dt;
            ws.drive_force = f_drive as f32;

            let force = drive_dir * f_drive as f32 + free_dir * f_free as f32 + normal * load;
            out.push((force, contact));
        }
        out
    }

    fn spawn_balls(&mut self) {
        const RADIUS: f32 = 2.5 * INCH;
        let purple = [0.45, 0.25, 0.75];
        let green = [0.2, 0.7, 0.35];
        let mut spots = Vec::new();
        for row in 0..3 {
            for col in 0..3 {
                spots.push((-0.5 * TILE + col as f32 * 7.0 * INCH, 0.5 * TILE + row as f32 * 7.0 * INCH));
            }
        }
        spots.extend([(1.5 * TILE, -1.0 * TILE), (1.7 * TILE, -1.2 * TILE), (-0.2 * TILE, -1.2 * TILE)]);
        for (i, (x, y)) in spots.into_iter().enumerate() {
            let body = self.physics.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(vector![x, RADIUS, -y])
                    .linear_damping(0.3)
                    .angular_damping(1.2)
                    .ccd_enabled(true),
            );
            self.physics.colliders.insert_with_parent(
                ColliderBuilder::ball(RADIUS).mass(0.075).restitution(0.45).friction(0.7),
                body,
                &mut self.physics.bodies,
            );
            self.pieces.push(GamePiece { body, radius: RADIUS, color: if i % 3 == 0 { green } else { purple } });
        }
    }
}

fn step_servo(s: &mut ServoState, cfg: &crate::robot::ServoConfig, dt: f64) {
    use crate::robot::ServoKind;
    if !s.enabled {
        return;
    }
    match cfg.kind {
        ServoKind::Positional => {
            let step = cfg.speed_deg_per_s / cfg.range_deg * dt;
            s.position += (s.command.clamp(0.0, 1.0) - s.position).clamp(-step, step);
            s.angle_deg = (s.position - 0.5) * cfg.range_deg;
        }
        ServoKind::Continuous => s.angle_deg += s.command.clamp(-1.0, 1.0) * cfg.speed_deg_per_s * dt,
        // A light, not a shaft: the color changes as soon as the pulse width does.
        ServoKind::RgbIndicator => s.position = s.command.clamp(0.0, 1.0),
    }
}

/// A motor turning its own load (flywheel, roller, spool): rotor and load inertia, motor torque,
/// gearbox and load friction, solved implicitly in the shaft speed.
fn step_mechanism(m: &mut MotorState, spec: &crate::motor::MotorSpec, load: &crate::robot::Load, drive: Drive, dt: f64) {
    let (a, b) = match drive {
        Drive::Volts(v) => spec.torque_coeffs(v),
        Drive::Float => (0.0, 0.0),
    };
    let ratio = load.ratio;
    let inertia = spec.reflected_inertia + load.inertia / (ratio * ratio);
    let gearbox_loss = spec.stall_torque * spec.free_current / spec.stall_current;
    let friction = (gearbox_loss + load.friction / ratio) * (m.shaft_omega / LOSS_SMOOTHING).tanh();
    m.shaft_omega = (inertia * m.shaft_omega + dt * (a - friction)) / (inertia + dt * (b + BEARING_DRAG));
    m.shaft_angle += m.shaft_omega * dt;
}

fn chassis_isometry(pose: Pose2d) -> Isometry<Real> {
    Isometry::new(vector![pose.x, 0.052 + SUSPENSION_REST, -pose.y], vector![0.0, pose.heading, 0.0])
}

fn build_field(p: &mut Physics) {
    let half = FIELD_SIZE / 2.0;
    p.colliders.insert(ColliderBuilder::cuboid(half + 1.0, 0.05, half + 1.0).translation(vector![0.0, -0.05, 0.0]).friction(1.0));
    let t = 0.025;
    let h = WALL_HEIGHT / 2.0;
    for (x, z, hx, hz) in [
        (half + t, 0.0, t, half + 2.0 * t),
        (-half - t, 0.0, t, half + 2.0 * t),
        (0.0, half + t, half + 2.0 * t, t),
        (0.0, -half - t, half + 2.0 * t, t),
    ] {
        p.colliders.insert(ColliderBuilder::cuboid(hx, h, hz).translation(vector![x, h, z]).friction(0.3).restitution(0.2));
    }
}

fn spawn_chassis(p: &mut Physics, cfg: &RobotConfig, start: Pose2d) -> RigidBodyHandle {
    let handle = p.bodies.insert(
        RigidBodyBuilder::dynamic().position(chassis_isometry(start)).can_sleep(false).ccd_enabled(true),
    );
    // Mass is spread through the robot's height (mechanisms, hubs, battery), not just the
    // frame plate, so inertia uses an effective height rather than the collider's.
    let (l, w, h, m) = (cfg.length, cfg.width, EFFECTIVE_HEIGHT, cfg.mass);
    let inertia = vector![m * (w * w + h * h) / 12.0, m * (l * l + w * w) / 12.0, m * (l * l + h * h) / 12.0];
    p.colliders.insert_with_parent(
        ColliderBuilder::cuboid(cfg.length / 2.0, cfg.frame_height / 2.0, cfg.width / 2.0)
            .mass_properties(MassProperties::new(point![0.0, 0.02, 0.0], m, inertia))
            .friction(0.3)
            .restitution(0.1),
        handle,
        &mut p.bodies,
    );
    handle
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hub::{MotorCommand, RunMode};

    fn world() -> SimWorld {
        let mut w = SimWorld::new(RobotConfig::default_mecanum(), Pose2d::new(0.0, -1.2, std::f32::consts::FRAC_PI_2));
        w.pieces.clear(); // keep tests independent of game-piece layout
        w
    }

    /// Raw powers per port (fl, fr, bl, br). Left motors are mirrored, like on the real robot.
    fn set_raw(w: &mut SimWorld, p: [f64; 4]) {
        for (m, power) in w.motors.iter_mut().zip(p) {
            m.channel.cmd = MotorCommand { power, ..Default::default() };
        }
    }

    fn run(w: &mut SimWorld, seconds: f64) {
        for _ in 0..(seconds * PHYSICS_HZ) as usize {
            w.step();
        }
    }

    #[test]
    fn settles_on_its_wheels() {
        let mut w = world();
        run(&mut w, 1.0);
        let load: f32 = w.wheels.iter().map(|s| s.load).sum();
        assert!((load - 12.0 * GRAVITY).abs() < 2.0, "total wheel load {load}");
        assert!(w.pose().y > -1.21 && w.pose().y < -1.19, "robot drifted at rest: {:?}", w.pose());
    }

    #[test]
    fn mirrored_left_motors_need_reversing() {
        let mut w = world();
        run(&mut w, 0.3);
        set_raw(&mut w, [0.5, 0.5, 0.5, 0.5]); // all raw +: left wheels go backward, robot spins
        run(&mut w, 0.5);
        let p = w.pose();
        assert!((p.heading - std::f32::consts::FRAC_PI_2).abs() > 0.5, "should rotate in place: {p:?}");
    }

    #[test]
    fn drives_forward_near_free_speed() {
        let mut w = world();
        run(&mut w, 0.3);
        let y0 = w.pose().y;
        set_raw(&mut w, [-1.0, 1.0, -1.0, 1.0]);
        let mut min_battery = f64::MAX;
        for _ in 0..PHYSICS_HZ as usize {
            w.step();
            min_battery = min_battery.min(w.battery.voltage);
        }
        let free = MotorSpec312::free_surface_speed();
        let dy = w.pose().y - y0;
        assert!(dy > 0.6 * free && dy < free, "moved {dy} m in 1 s, free speed {free} m/s");
        assert!((w.pose().heading - std::f32::consts::FRAC_PI_2).abs() < 0.05);
        assert!(min_battery < 12.0, "battery should sag while accelerating, min {min_battery:.2} V");
        assert!(w.battery.current > 2.0, "cruising still draws current, got {:.2} A", w.battery.current);
    }

    #[test]
    fn mecanum_strafes_right() {
        let mut w = world();
        run(&mut w, 0.3);
        let x0 = w.pose().x;
        // strafe right: fl+, fr-, bl-, br+ in wheel terms; raw left = -wheel
        set_raw(&mut w, [-1.0, -1.0, 1.0, 1.0]);
        run(&mut w, 1.0);
        let p = w.pose();
        assert!(p.x - x0 > 0.4, "strafe distance {}", p.x - x0);
        assert!((p.y + 1.2).abs() < 0.1, "strafe should not drive forward: {p:?}");
    }

    #[test]
    fn wall_stops_robot_and_wheels_slip() {
        let mut w = world();
        run(&mut w, 0.3);
        set_raw(&mut w, [1.0, -1.0, 1.0, -1.0]); // backward into the red wall
        run(&mut w, 3.0);
        let back_edge = w.pose().y - w.config.length / 2.0;
        assert!(back_edge > -FIELD_SIZE / 2.0 - 0.01, "drove through the wall: {back_edge}");
        assert!(w.wheels.iter().any(|s| s.slipping));
        let (ticks, _) = w.encoder(0);
        assert!(ticks.abs() > 2000, "encoders keep counting while wheels slip: {ticks}");
    }

    #[test]
    fn run_to_position_reaches_target() {
        let mut w = world();
        run(&mut w, 0.3);
        for m in &mut w.motors {
            m.channel.cmd = MotorCommand { mode: RunMode::StopAndReset, ..Default::default() };
        }
        w.step();
        // Raw targets: left motors are mirrored so a forward move is negative ticks on the left.
        for (i, m) in w.motors.iter_mut().enumerate() {
            let target = if i % 2 == 0 { -1000 } else { 1000 };
            m.channel.cmd = MotorCommand { mode: RunMode::ToPosition, power: 0.6, target, ..Default::default() };
        }
        run(&mut w, 3.0);
        for i in 0..4 {
            let (t, _) = w.encoder(i);
            assert!((t.abs() - 1000).abs() <= 10, "motor {i} at {t}");
        }
    }

    #[test]
    fn run_using_encoder_holds_half_speed_with_default_gains() {
        let mut w = world();
        run(&mut w, 0.3);
        for (i, m) in w.motors.iter_mut().enumerate() {
            let power = if i % 2 == 0 { -0.5 } else { 0.5 };
            m.channel.cmd = MotorCommand { mode: RunMode::UsingEncoder, power, ..Default::default() };
        }
        run(&mut w, 2.0);
        let max = crate::motor::MotorSpec::GOBILDA_312.max_ticks_per_sec();
        for i in 0..4 {
            let (_, v) = w.encoder(i);
            assert!((v.abs() / max - 0.5).abs() < 0.05, "motor {i} at {:.0} ticks/s, wanted {:.0}", v.abs(), max * 0.5);
        }
    }

    #[test]
    fn servo_slews_at_rated_speed_and_waits_for_first_command() {
        let mut text = include_str!("../../../../robots/strafer.toml").to_string();
        text.push_str("\n[[servos]]\nname = \"claw\"\nport = 0\n");
        let mut w = SimWorld::new(RobotConfig::from_toml(&text).unwrap(), Pose2d::new(0.0, -1.2, 1.57));
        run(&mut w, 0.2);
        assert_eq!(w.servos[0].position, 0.5, "no PWM before the first setPosition");
        w.servos[0].command = 1.0;
        w.servos[0].enabled = true;
        run(&mut w, 0.25); // 240°/s over a 300° range: 0.2 of the range in 0.25 s
        assert!((w.servos[0].position - 0.7).abs() < 0.01, "at {}", w.servos[0].position);
        run(&mut w, 1.0);
        assert!((w.servos[0].position - 1.0).abs() < 1e-9);
    }

    #[test]
    fn flywheel_reaches_target_velocity_with_team_gains() {
        // A shooter like team 20761's: bare 6000 RPM motor on a ~0.2 kg, 96 mm flywheel,
        // tuned with P=340 I=55 D=0 F=2.9 in hub units, asking for 1250 ticks/s.
        let mut text = include_str!("../../../../robots/strafer.toml").to_string();
        text.push_str("\n[[motors]]\nname = \"shooter\"\ntype = \"gobilda-6000\"\nhub = \"Expansion Hub 2\"\nport = 0\nload = { flywheel_mass_kg = 0.2, flywheel_diameter_mm = 96, friction_nm = 0.01 }\n");
        let config = RobotConfig::from_toml(&text).unwrap();
        let mut w = SimWorld::new(config, Pose2d::new(0.0, -1.2, std::f32::consts::FRAC_PI_2));
        let max = crate::motor::MotorSpec::GOBILDA_6000.max_ticks_per_sec();
        w.motors[4].channel.cmd = MotorCommand {
            mode: RunMode::UsingEncoder,
            power: 1250.0 / max,
            velocity_pidf: [340.0, 55.0, 0.0, 2.9],
            ..Default::default()
        };
        let mut peak: f64 = 0.0;
        for _ in 0..(1.5 * PHYSICS_HZ) as usize {
            w.step();
            peak = peak.max(w.encoder(4).1);
        }
        let (_, v) = w.encoder(4);
        assert!((v - 1250.0).abs() < 25.0, "flywheel at {v:.0} ticks/s after 1.5 s");
        assert!(peak < 1250.0 * 1.25, "overshoot to {peak:.0} ticks/s");
        assert!(w.pose().y > -1.21 && w.pose().y < -1.19, "a mechanism motor must not move the robot");
    }

    struct MotorSpec312;
    impl MotorSpec312 {
        fn free_surface_speed() -> f32 {
            (crate::motor::MotorSpec::GOBILDA_312.free_speed() * 0.052) as f32
        }
    }
}


