//! TCP link to the Java runner, plus the glue that turns world state into protocol messages.

use crate::proto::{self, Command, Control, DeviceInfo, DeviceKind, FromRunner, MotorReading, OpModeInfo, State};
use crate::world::SimWorld;
use std::io::{self, BufReader, BufWriter};
use std::net::{TcpListener, TcpStream};
use std::time::Duration;

pub const DEFAULT_PORT: u16 = 7171;
/// Physics steps per runner exchange: 400 Hz physics, 200 Hz hardware bus.
pub const STEPS_PER_EXCHANGE: u32 = 2;

struct Conn {
    reader: BufReader<TcpStream>,
    writer: BufWriter<TcpStream>,
}

pub struct Link {
    listener: TcpListener,
    conn: Option<Conn>,
    pending: Vec<Control>,
    seq: u32,
    pub opmodes: Vec<OpModeInfo>,
    pub last: Command,
    /// Latest telemetry the OpMode published.
    pub telemetry: Vec<String>,
    pub status: String,
    reply_timeout: Duration,
}

impl Link {
    pub fn bind(port: u16) -> io::Result<Self> {
        let listener = TcpListener::bind(("127.0.0.1", port))?;
        listener.set_nonblocking(true)?;
        Ok(Self {
            listener,
            conn: None,
            pending: Vec::new(),
            seq: 0,
            opmodes: Vec::new(),
            last: Command::default(),
            telemetry: Vec::new(),
            status: format!("Waiting for the robot runner on port {port}"),
            reply_timeout: Duration::from_secs(5),
        })
    }

    /// How long to wait for the runner each exchange before declaring it gone.
    pub fn set_reply_timeout(&mut self, timeout: Duration) {
        self.reply_timeout = timeout;
    }

    pub fn connected(&self) -> bool {
        self.conn.is_some()
    }

    /// Accept a waiting runner, if any. Returns true when a new runner connected.
    pub fn poll_accept(&mut self, world: &SimWorld) -> bool {
        if self.conn.is_some() {
            return false;
        }
        let Ok((stream, _)) = self.listener.accept() else { return false };
        match self.handshake(stream, world) {
            Ok(conn) => {
                self.conn = Some(conn);
                self.status = format!("Runner connected, {} OpModes", self.opmodes.len());
                true
            }
            Err(e) => {
                self.status = format!("Runner handshake failed: {e}");
                false
            }
        }
    }

    fn handshake(&mut self, stream: TcpStream, world: &SimWorld) -> io::Result<Conn> {
        stream.set_nonblocking(false)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        let mut reader = BufReader::new(stream.try_clone()?);
        let mut writer = BufWriter::new(stream);
        let FromRunner::Hello { version, opmodes } = proto::read_from_runner(&mut reader)? else {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "expected HELLO"));
        };
        if version != proto::PROTOCOL_VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("runner speaks protocol v{version}, sim speaks v{}", proto::PROTOCOL_VERSION),
            ));
        }
        let mut devices: Vec<DeviceInfo> = world
            .config
            .motors
            .iter()
            .map(|m| DeviceInfo {
                kind: DeviceKind::Motor,
                name: m.name.clone(),
                port: m.port,
                ticks_per_rev: m.spec.ticks_per_rev as f32,
                max_rpm: m.spec.free_speed_rpm as f32,
            })
            .collect();
        devices.push(DeviceInfo { kind: DeviceKind::Imu, name: world.config.imu_name.clone(), port: 0, ticks_per_rev: 0.0, max_rpm: 0.0 });
        proto::write_config(&mut writer, &world.config.name, &devices)?;
        self.opmodes = opmodes;
        self.last = Command::default();
        self.telemetry.clear();
        Ok(Conn { reader, writer })
    }

    pub fn send(&mut self, control: Control) {
        self.pending.push(control);
    }

    /// One lockstep exchange: send controls and the current state, wait for the command,
    /// and apply motor commands to the world. Disconnects (and zeroes motors) on failure.
    pub fn exchange(&mut self, world: &mut SimWorld) {
        let Some(conn) = self.conn.as_mut() else {
            self.pending.clear();
            return;
        };
        self.seq = self.seq.wrapping_add(1);
        let state = snapshot(world, self.seq);
        let result = (|| -> io::Result<Command> {
            for c in self.pending.drain(..) {
                proto::write_control(&mut conn.writer, c)?;
            }
            proto::write_state(&mut conn.writer, &state)?;
            conn.reader.get_ref().set_read_timeout(Some(self.reply_timeout))?;
            loop {
                if let FromRunner::Command(cmd) = proto::read_from_runner(&mut conn.reader)? {
                    if cmd.seq == self.seq {
                        return Ok(cmd);
                    }
                }
            }
        })();
        match result {
            Ok(cmd) => {
                for (m, c) in world.motors.iter_mut().zip(&cmd.motors) {
                    m.channel.cmd = *c;
                }
                if let Some(t) = &cmd.telemetry {
                    self.telemetry = t.clone();
                }
                self.last = cmd;
            }
            Err(e) => {
                self.conn = None;
                self.status = format!("Runner disconnected: {e}");
                self.last = Command::default();
                for m in &mut world.motors {
                    m.channel.cmd = Default::default();
                }
            }
        }
    }
}

fn snapshot(world: &SimWorld, seq: u32) -> State {
    let imu = world.imu();
    State {
        seq,
        time: world.time,
        battery: world.battery.voltage as f32,
        motors: (0..world.motors.len())
            .map(|i| {
                let (position, velocity) = world.encoder(i);
                MotorReading { position, velocity: velocity as f32, current: world.motors[i].current.abs() as f32 }
            })
            .collect(),
        imu: [imu.yaw, imu.pitch, imu.roll, imu.rate_x, imu.rate_y, imu.rate_z],
        gamepads: world.gamepads,
    }
}
