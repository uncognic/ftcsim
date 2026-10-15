//! Binary protocol between the simulator and the Java runner (`org.ftcsim.runner.Protocol`).
//!
//! Framing: `u32 length` (big-endian, counts the type byte and body), `u8 type`, body.
//! Strings are `u16 length` + UTF-8. All numbers are big-endian, matching `java.io.DataOutput`.
//!
//! Lockstep: the sim sends `State(seq)` and waits for the runner's `Command(seq)` before
//! stepping on. `Control` messages are sent just before a `State`.

use crate::gamepad::GamepadState;
use crate::hub::{MotorCommand, RunMode, ZeroPower};
use std::io::{self, Read, Write};

pub const PROTOCOL_VERSION: u16 = 3;

mod tag {
    pub const HELLO: u8 = 1;
    pub const COMMAND: u8 = 2;
    pub const CONFIG: u8 = 10;
    pub const STATE: u8 = 11;
    pub const CONTROL: u8 = 12;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flavor {
    Autonomous,
    TeleOp,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OpModeInfo {
    pub name: String,
    pub flavor: Flavor,
    pub group: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OpModeState {
    #[default]
    Idle,
    Init,
    Running,
    Stopped,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Command {
    pub seq: u32,
    pub state: OpModeState,
    /// Non-empty when the OpMode failed (uncaught exception, stuck in stop, ...).
    pub error: String,
    pub motors: Vec<MotorCommand>,
    /// Per servo, in CONFIG order: `(position 0..1 or CR power -1..1, PWM enabled)`.
    pub servos: Vec<(f64, bool)>,
    /// `Some` when the OpMode called `telemetry.update()` since the last command.
    pub telemetry: Option<Vec<String>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeviceKind {
    Motor = 0,
    Imu = 1,
    Servo = 2,
    CrServo = 3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    pub kind: DeviceKind,
    pub name: String,
    pub port: u8,
    /// Motors only: encoder counts per output revolution and free speed, so the SDK can
    /// convert units the way it does with a configured motor type.
    pub ticks_per_rev: f32,
    pub max_rpm: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorReading {
    pub position: i32,
    pub velocity: f32,
    pub current: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct State {
    pub seq: u32,
    pub time: f64,
    pub battery: f32,
    pub motors: Vec<MotorReading>,
    /// yaw, pitch, roll (rad), then x/y/z rates (rad/s) in FTC robot axes.
    pub imu: [f32; 6],
    pub gamepads: [GamepadState; 2],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Control {
    Init(u16),
    Start,
    Stop,
}

#[derive(Clone, Debug, PartialEq)]
pub enum FromRunner {
    Hello { version: u16, opmodes: Vec<OpModeInfo> },
    Command(Command),
}

// ---------- encoding ----------

#[derive(Default)]
struct Enc(Vec<u8>);

impl Enc {
    fn u8(&mut self, v: u8) { self.0.push(v); }
    fn u16(&mut self, v: u16) { self.0.extend(v.to_be_bytes()); }
    fn u32(&mut self, v: u32) { self.0.extend(v.to_be_bytes()); }
    fn i32(&mut self, v: i32) { self.0.extend(v.to_be_bytes()); }
    fn f32(&mut self, v: f32) { self.0.extend(v.to_be_bytes()); }
    fn f64(&mut self, v: f64) { self.0.extend(v.to_be_bytes()); }
    fn str(&mut self, s: &str) {
        let b = s.as_bytes();
        let n = b.len().min(u16::MAX as usize);
        self.u16(n as u16);
        self.0.extend(&b[..n]);
    }
    fn gamepad(&mut self, g: &GamepadState) {
        for v in [g.left_stick_x, g.left_stick_y, g.right_stick_x, g.right_stick_y, g.left_trigger, g.right_trigger] {
            self.f32(v);
        }
        self.u32(g.buttons);
    }
}

struct Dec<'a>(&'a [u8]);

impl<'a> Dec<'a> {
    fn take(&mut self, n: usize) -> io::Result<&'a [u8]> {
        if self.0.len() < n {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "truncated message"));
        }
        let (a, b) = self.0.split_at(n);
        self.0 = b;
        Ok(a)
    }
    fn u8(&mut self) -> io::Result<u8> { Ok(self.take(1)?[0]) }
    fn u16(&mut self) -> io::Result<u16> { Ok(u16::from_be_bytes(self.take(2)?.try_into().unwrap())) }
    fn u32(&mut self) -> io::Result<u32> { Ok(u32::from_be_bytes(self.take(4)?.try_into().unwrap())) }
    fn i32(&mut self) -> io::Result<i32> { Ok(i32::from_be_bytes(self.take(4)?.try_into().unwrap())) }
    fn f32(&mut self) -> io::Result<f32> { Ok(f32::from_be_bytes(self.take(4)?.try_into().unwrap())) }
    fn str(&mut self) -> io::Result<String> {
        let n = self.u16()? as usize;
        String::from_utf8(self.take(n)?.to_vec()).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }
}

fn write_frame(w: &mut impl Write, tag: u8, body: Enc) -> io::Result<()> {
    let mut frame = Vec::with_capacity(body.0.len() + 5);
    frame.extend(((body.0.len() + 1) as u32).to_be_bytes());
    frame.push(tag);
    frame.extend(body.0);
    w.write_all(&frame)?;
    w.flush()
}

pub fn write_config(w: &mut impl Write, robot_name: &str, devices: &[DeviceInfo]) -> io::Result<()> {
    let mut e = Enc::default();
    e.u16(PROTOCOL_VERSION);
    e.str(robot_name);
    e.u8(devices.len() as u8);
    for d in devices {
        e.u8(d.kind as u8);
        e.str(&d.name);
        e.u8(d.port);
        e.f32(d.ticks_per_rev);
        e.f32(d.max_rpm);
    }
    write_frame(w, tag::CONFIG, e)
}

pub fn write_state(w: &mut impl Write, s: &State) -> io::Result<()> {
    let mut e = Enc::default();
    e.u32(s.seq);
    e.f64(s.time);
    e.f32(s.battery);
    e.u8(s.motors.len() as u8);
    for m in &s.motors {
        e.i32(m.position);
        e.f32(m.velocity);
        e.f32(m.current);
    }
    for v in s.imu {
        e.f32(v);
    }
    for g in &s.gamepads {
        e.gamepad(g);
    }
    write_frame(w, tag::STATE, e)
}

pub fn write_control(w: &mut impl Write, c: Control) -> io::Result<()> {
    let mut e = Enc::default();
    match c {
        Control::Init(i) => { e.u8(1); e.u16(i); }
        Control::Start => { e.u8(2); e.u16(0); }
        Control::Stop => { e.u8(3); e.u16(0); }
    }
    write_frame(w, tag::CONTROL, e)
}

pub fn read_from_runner(r: &mut impl Read) -> io::Result<FromRunner> {
    let mut len = [0u8; 4];
    r.read_exact(&mut len)?;
    let len = u32::from_be_bytes(len) as usize;
    if len == 0 || len > 1 << 20 {
        return Err(io::Error::new(io::ErrorKind::InvalidData, format!("bad frame length {len}")));
    }
    let mut buf = vec![0u8; len];
    r.read_exact(&mut buf)?;
    let mut d = Dec(&buf[1..]);
    match buf[0] {
        tag::HELLO => {
            let version = d.u16()?;
            let n = d.u16()?;
            let mut opmodes = Vec::with_capacity(n as usize);
            for _ in 0..n {
                let name = d.str()?;
                let flavor = if d.u8()? == 0 { Flavor::Autonomous } else { Flavor::TeleOp };
                let group = d.str()?;
                opmodes.push(OpModeInfo { name, flavor, group });
            }
            Ok(FromRunner::Hello { version, opmodes })
        }
        tag::COMMAND => {
            let seq = d.u32()?;
            let state = match d.u8()? {
                1 => OpModeState::Init,
                2 => OpModeState::Running,
                3 => OpModeState::Stopped,
                _ => OpModeState::Idle,
            };
            let error = d.str()?;
            let n = d.u8()?;
            let mut motors = Vec::with_capacity(n as usize);
            for _ in 0..n {
                let mode = RunMode::from_u8(d.u8()?);
                let zero_power = ZeroPower::from_u8(d.u8()?);
                let power = d.f32()? as f64;
                let target = d.i32()?;
                let velocity_pidf = [d.f32()? as f64, d.f32()? as f64, d.f32()? as f64, d.f32()? as f64];
                let position_p = d.f32()? as f64;
                motors.push(MotorCommand { mode, zero_power, power, target, velocity_pidf, position_p });
            }
            let n = d.u8()?;
            let mut servos = Vec::with_capacity(n as usize);
            for _ in 0..n {
                let value = d.f32()? as f64;
                servos.push((value, d.u8()? != 0));
            }
            let telemetry = if d.u8()? == 1 {
                let n = d.u16()?;
                Some((0..n).map(|_| d.str()).collect::<io::Result<Vec<_>>>()?)
            } else {
                None
            };
            Ok(FromRunner::Command(Command { seq, state, error, motors, servos, telemetry }))
        }
        t => Err(io::Error::new(io::ErrorKind::InvalidData, format!("unknown message type {t}"))),
    }
}
