//! Run an OpMode with no window, as fast as the CPU allows, and print what the robot did.
//!
//! ```text
//! sim-headless --opmode "Encoder Square" [--robot robots/strafer.toml] [--seconds 30] [--port 7171]
//! ```
//! Start the Java runner separately (`gradlew :runner:run` in `robot/`); it connects here.

use sim_core::link::{Link, DEFAULT_PORT, STEPS_PER_EXCHANGE};
use sim_core::proto::{Control, OpModeState};
use sim_core::robot::RobotConfig;
use sim_core::world::PHYSICS_HZ;
use sim_core::SimWorld;
use std::time::{Duration, Instant};

fn main() {
    let mut opmode = None;
    let mut seconds = 30.0;
    let mut port = DEFAULT_PORT;
    let mut robot = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--opmode" => opmode = args.next(),
            "--seconds" => seconds = args.next().and_then(|s| s.parse().ok()).unwrap_or(seconds),
            "--port" => port = args.next().and_then(|s| s.parse().ok()).unwrap_or(port),
            "--robot" => robot = args.next(),
            _ => {
                eprintln!("usage: sim-headless --opmode NAME [--robot FILE] [--seconds S] [--port P]");
                std::process::exit(2);
            }
        }
    }

    let config = match robot {
        Some(path) => RobotConfig::load(std::path::Path::new(&path)).unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(1);
        }),
        None => RobotConfig::default_mecanum(),
    };
    let mut world = SimWorld::new(config, SimWorld::default_start());
    let mut link = Link::bind(port).unwrap_or_else(|e| {
        eprintln!("cannot listen on port {port}: {e}");
        std::process::exit(1);
    });
    eprintln!("{}", link.status);
    let deadline = Instant::now() + Duration::from_secs(120);
    while !link.poll_accept(&world) {
        if Instant::now() > deadline {
            eprintln!("no runner connected within 120 s");
            std::process::exit(1);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    eprintln!("{}", link.status);
    for (i, m) in link.opmodes.iter().enumerate() {
        eprintln!("  [{i}] {} ({:?})", m.name, m.flavor);
    }

    let Some(name) = opmode else { return };
    let Some(index) = link.opmodes.iter().position(|m| m.name == name) else {
        eprintln!("no OpMode named {name:?}");
        std::process::exit(1);
    };

    let exchanges_per_sec = (PHYSICS_HZ / STEPS_PER_EXCHANGE as f64) as usize;
    let tick = |world: &mut SimWorld, link: &mut Link| {
        for _ in 0..STEPS_PER_EXCHANGE {
            world.step();
        }
        link.exchange(world);
    };

    link.send(Control::Init(index as u16));
    for _ in 0..exchanges_per_sec / 2 {
        tick(&mut world, &mut link);
    }
    link.send(Control::Start);
    let started = Instant::now();
    let start_time = world.time;
    let mut next_report = 0.0;
    while world.time - start_time < seconds {
        tick(&mut world, &mut link);
        let t = world.time - start_time;
        if t >= next_report {
            let p = world.pose();
            println!(
                "t={t:5.2}s  x={:7.2}in y={:7.2}in heading={:7.2}°  battery={:5.2}V  {:?}",
                p.x / 0.0254,
                p.y / 0.0254,
                p.heading.to_degrees(),
                world.battery.voltage,
                link.last.state
            );
            next_report += 1.0;
        }
        if !link.connected() {
            eprintln!("{}", link.status);
            break;
        }
        if link.last.state == OpModeState::Stopped && t > 0.1 {
            break;
        }
    }
    link.send(Control::Stop);
    tick(&mut world, &mut link);

    let p = world.pose();
    println!(
        "final: x={:.2}in y={:.2}in heading={:.2}° after {:.2}s sim time ({:.2}s wall)",
        p.x / 0.0254,
        p.y / 0.0254,
        p.heading.to_degrees(),
        world.time - start_time,
        started.elapsed().as_secs_f64()
    );
    if !link.last.error.is_empty() {
        println!("error: {}", link.last.error);
    }
    println!("telemetry:");
    for line in &link.telemetry {
        println!("  {line}");
    }
}
