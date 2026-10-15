//! FTC Sim desktop app: 3D view of the field, a keyboard Driver Station, and the runner link.
//!
//! Physics runs in `FixedUpdate` at [`PHYSICS_HZ`]; every [`STEPS_PER_EXCHANGE`] steps the
//! world is exchanged with the Java runner. Rendering only reads the world.

use bevy::prelude::*;
use sim_core::gamepad::{buttons, GamepadState};
use sim_core::link::{Link, DEFAULT_PORT, STEPS_PER_EXCHANGE};
use sim_core::proto::{Control, Flavor, OpModeState};
use sim_core::robot::{RobotConfig, WheelKind};
use sim_core::units::*;
use sim_core::world::{PHYSICS_HZ, SUSPENSION_REST};
use sim_core::{Pose2d, SimWorld};
use std::f32::consts::{FRAC_PI_2, PI};

const AUTO_SECONDS: f64 = 30.0;

#[derive(Resource)]
struct Sim {
    world: SimWorld,
    link: Link,
    steps: u32,
    paused: bool,
    /// Where R puts the robot back.
    start: Pose2d,
}

#[derive(Resource, Default)]
struct DriverStation {
    selected: usize,
    started_at: Option<f64>,
    stop_sent: bool,
}

#[derive(Resource, Default, Clone, Copy, PartialEq)]
enum CameraView {
    #[default]
    Audience,
    Top,
    Chase,
}

#[derive(Component)]
struct Chassis;
#[derive(Component)]
struct WheelVisual(usize);
#[derive(Component)]
struct PieceVisual(usize);
#[derive(Component)]
struct MainCamera;
#[derive(Component)]
struct DsText;
#[derive(Component)]
struct TelemetryText;
#[derive(Component)]
struct StatusText;

fn main() {
    // ftc-sim [--robot robots/20761.toml] [--start x,y,heading]
    let mut args = std::env::args().skip(1);
    let mut config = RobotConfig::default_mecanum();
    let mut start = SimWorld::default_start();
    while let Some(arg) = args.next() {
        let value = args.next().unwrap_or_default();
        let result = match arg.as_str() {
            "--robot" => RobotConfig::load(std::path::Path::new(&value)).map(|c| config = c),
            "--start" => Pose2d::parse_inches_degrees(&value).map(|p| start = p),
            _ => Err("usage: ftc-sim [--robot FILE] [--start x,y,heading]".into()),
        };
        if let Err(e) = result {
            eprintln!("{e}");
            std::process::exit(2);
        }
    }
    let world = SimWorld::new(config, start);
    let link = match Link::bind(DEFAULT_PORT) {
        Ok(link) => link,
        Err(e) => {
            eprintln!("Cannot listen on port {DEFAULT_PORT}: {e}. Is another simulator already running?");
            std::process::exit(1);
        }
    };

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "FTC Sim".into(), resolution: (1440.0, 900.0).into(), ..default() }),
            ..default()
        }))
        .insert_resource(Time::<Fixed>::from_hz(PHYSICS_HZ))
        .insert_resource(ClearColor(Color::srgb(0.07, 0.08, 0.09)))
        .insert_resource(AmbientLight { color: Color::WHITE, brightness: 350.0, ..default() })
        .insert_resource(Sim { world, link, steps: 0, paused: false, start })
        .init_resource::<DriverStation>()
        .init_resource::<CameraView>()
        .add_systems(Startup, (setup_scene, setup_hud))
        .add_systems(FixedUpdate, physics_tick)
        .add_systems(Update, (read_gamepads, driver_station_keys, sync_visuals, update_camera, update_hud).chain())
        .run();
}

// ---------------------------------------------------------------- simulation

fn physics_tick(mut sim: ResMut<Sim>, mut ds: ResMut<DriverStation>) {
    let sim = &mut *sim;
    if sim.link.poll_accept(&sim.world) {
        *ds = DriverStation::default();
    }
    if sim.paused {
        return;
    }
    sim.world.step();
    sim.steps += 1;
    if sim.steps % STEPS_PER_EXCHANGE != 0 {
        return;
    }
    sim.link.exchange(&mut sim.world);

    // The Driver Station ends autonomous after 30 s.
    let is_auto = sim.link.opmodes.get(ds.selected).is_some_and(|m| m.flavor == Flavor::Autonomous);
    if let Some(t0) = ds.started_at {
        if is_auto && !ds.stop_sent && sim.link.last.state == OpModeState::Running && sim.world.time - t0 >= AUTO_SECONDS {
            sim.link.send(Control::Stop);
            ds.stop_sent = true;
        }
    }
}

fn read_gamepads(keys: Res<ButtonInput<KeyCode>>, pads: Query<&Gamepad>, mut sim: ResMut<Sim>) {
    let axis = |pos: KeyCode, neg: KeyCode| keys.pressed(pos) as i32 as f32 - keys.pressed(neg) as i32 as f32;
    let mut g = GamepadState {
        left_stick_x: axis(KeyCode::KeyD, KeyCode::KeyA),
        left_stick_y: axis(KeyCode::KeyS, KeyCode::KeyW),
        right_stick_x: axis(KeyCode::ArrowRight, KeyCode::ArrowLeft),
        right_stick_y: axis(KeyCode::ArrowDown, KeyCode::ArrowUp),
        left_trigger: keys.pressed(KeyCode::KeyZ) as i32 as f32,
        right_trigger: keys.pressed(KeyCode::KeyC) as i32 as f32,
        buttons: 0,
    };
    let key_buttons = [
        (KeyCode::Space, buttons::A),
        (KeyCode::KeyB, buttons::B),
        (KeyCode::KeyX, buttons::X),
        (KeyCode::KeyY, buttons::Y),
        (KeyCode::KeyQ, buttons::LEFT_BUMPER),
        (KeyCode::KeyE, buttons::RIGHT_BUMPER),
        (KeyCode::KeyI, buttons::DPAD_UP),
        (KeyCode::KeyK, buttons::DPAD_DOWN),
        (KeyCode::KeyJ, buttons::DPAD_LEFT),
        (KeyCode::KeyL, buttons::DPAD_RIGHT),
        (KeyCode::KeyO, buttons::START),
        (KeyCode::KeyV, buttons::BACK),
    ];
    for (key, bit) in key_buttons {
        if keys.pressed(key) {
            g.buttons |= bit;
        }
    }

    if let Some(pad) = pads.iter().next() {
        let pick = |kb: f32, pad: f32| if pad.abs() > 0.05 { pad } else { kb };
        let (l, r) = (pad.left_stick(), pad.right_stick());
        g.left_stick_x = pick(g.left_stick_x, l.x);
        g.left_stick_y = pick(g.left_stick_y, -l.y);
        g.right_stick_x = pick(g.right_stick_x, r.x);
        g.right_stick_y = pick(g.right_stick_y, -r.y);
        g.left_trigger = g.left_trigger.max(pad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0));
        g.right_trigger = g.right_trigger.max(pad.get(GamepadButton::RightTrigger2).unwrap_or(0.0));
        let pad_buttons = [
            (GamepadButton::South, buttons::A),
            (GamepadButton::East, buttons::B),
            (GamepadButton::West, buttons::X),
            (GamepadButton::North, buttons::Y),
            (GamepadButton::LeftTrigger, buttons::LEFT_BUMPER),
            (GamepadButton::RightTrigger, buttons::RIGHT_BUMPER),
            (GamepadButton::DPadUp, buttons::DPAD_UP),
            (GamepadButton::DPadDown, buttons::DPAD_DOWN),
            (GamepadButton::DPadLeft, buttons::DPAD_LEFT),
            (GamepadButton::DPadRight, buttons::DPAD_RIGHT),
            (GamepadButton::Start, buttons::START),
            (GamepadButton::Select, buttons::BACK),
            (GamepadButton::Mode, buttons::GUIDE),
            (GamepadButton::LeftThumb, buttons::LEFT_STICK_BUTTON),
            (GamepadButton::RightThumb, buttons::RIGHT_STICK_BUTTON),
        ];
        for (button, bit) in pad_buttons {
            if pad.pressed(button) {
                g.buttons |= bit;
            }
        }
    }
    sim.world.gamepads[0] = g;
}

fn driver_station_keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut sim: ResMut<Sim>,
    mut ds: ResMut<DriverStation>,
    mut view: ResMut<CameraView>,
) {
    let sim = &mut *sim;
    let state = sim.link.last.state;
    let busy = matches!(state, OpModeState::Init | OpModeState::Running);
    let count = sim.link.opmodes.len();

    if keys.just_pressed(KeyCode::Tab) && !busy && count > 0 {
        let step = if keys.pressed(KeyCode::ShiftLeft) { count - 1 } else { 1 };
        ds.selected = (ds.selected + step) % count;
    }
    if keys.just_pressed(KeyCode::Enter) && sim.link.connected() && count > 0 {
        match state {
            OpModeState::Idle | OpModeState::Stopped => {
                sim.link.send(Control::Init(ds.selected as u16));
                ds.started_at = None;
                ds.stop_sent = false;
            }
            OpModeState::Init => {
                sim.link.send(Control::Start);
                ds.started_at = Some(sim.world.time);
            }
            OpModeState::Running => {}
        }
    }
    if keys.just_pressed(KeyCode::Backspace) && busy {
        sim.link.send(Control::Stop);
    }
    if keys.just_pressed(KeyCode::KeyR) && !busy {
        let start = sim.start;
        sim.world.place_robot(start);
    }
    if keys.just_pressed(KeyCode::KeyP) {
        sim.paused = !sim.paused;
    }
    for (key, v) in [(KeyCode::F1, CameraView::Audience), (KeyCode::F2, CameraView::Top), (KeyCode::F3, CameraView::Chase)] {
        if keys.just_pressed(key) {
            *view = v;
        }
    }
}

// ---------------------------------------------------------------- scene

fn setup_scene(
    mut commands: Commands,
    sim: Res<Sim>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let panel = materials.add(StandardMaterial {
        base_color: Color::srgba(0.75, 0.82, 0.9, 0.25),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 0.1,
        ..default()
    });
    let mut mat = |color: Color, rough: f32, metal: f32| {
        materials.add(StandardMaterial { base_color: color, perceptual_roughness: rough, metallic: metal, ..default() })
    };

    commands.spawn((Camera3d::default(), MainCamera, Transform::from_xyz(0.0, 3.0, 3.6).looking_at(Vec3::ZERO, Vec3::Y)));
    commands.spawn((
        DirectionalLight { illuminance: 9000.0, shadows_enabled: true, ..default() },
        Transform::from_xyz(2.0, 6.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Floor: 36 foam tiles with visible seams over a dark underlay.
    let underlay = mat(Color::srgb(0.12, 0.13, 0.14), 1.0, 0.0);
    commands.spawn((Mesh3d(meshes.add(Cuboid::new(FIELD_SIZE + 1.0, 0.01, FIELD_SIZE + 1.0))), MeshMaterial3d(underlay), Transform::from_xyz(0.0, -0.006, 0.0)));
    let tile_mesh = meshes.add(Cuboid::new(TILE - 0.004, 0.004, TILE - 0.004));
    let tile_a = mat(Color::srgb(0.47, 0.49, 0.51), 0.95, 0.0);
    let tile_b = mat(Color::srgb(0.45, 0.47, 0.49), 0.95, 0.0);
    for i in 0..6 {
        for j in 0..6 {
            let x = -FIELD_SIZE / 2.0 + TILE * (i as f32 + 0.5);
            let z = -FIELD_SIZE / 2.0 + TILE * (j as f32 + 0.5);
            let m = if (i + j) % 2 == 0 { tile_a.clone() } else { tile_b.clone() };
            commands.spawn((Mesh3d(tile_mesh.clone()), MeshMaterial3d(m), Transform::from_xyz(x, -0.002, z)));
        }
    }

    // Perimeter: polycarbonate panels with an aluminium top rail.
    let rail = mat(Color::srgb(0.72, 0.73, 0.75), 0.4, 0.8);
    let half = FIELD_SIZE / 2.0;
    for (x, z, sx, sz) in [
        (half + 0.012, 0.0, 0.024, FIELD_SIZE),
        (-half - 0.012, 0.0, 0.024, FIELD_SIZE),
        (0.0, half + 0.012, FIELD_SIZE + 0.048, 0.024),
        (0.0, -half - 0.012, FIELD_SIZE + 0.048, 0.024),
    ] {
        commands.spawn((Mesh3d(meshes.add(Cuboid::new(sx, WALL_HEIGHT, sz))), MeshMaterial3d(panel.clone()), Transform::from_xyz(x, WALL_HEIGHT / 2.0, z)));
        commands.spawn((Mesh3d(meshes.add(Cuboid::new(sx + 0.01, 0.025, sz + 0.01))), MeshMaterial3d(rail.clone()), Transform::from_xyz(x, WALL_HEIGHT, z)));
    }
    // Alliance stripes along the red (near) and blue (far) walls.
    let red = mat(Color::srgb(0.8, 0.15, 0.12), 0.6, 0.0);
    let blue = mat(Color::srgb(0.12, 0.3, 0.85), 0.6, 0.0);
    commands.spawn((Mesh3d(meshes.add(Cuboid::new(FIELD_SIZE, 0.05, 0.03))), MeshMaterial3d(red), Transform::from_xyz(0.0, 0.025, half + 0.04)));
    commands.spawn((Mesh3d(meshes.add(Cuboid::new(FIELD_SIZE, 0.05, 0.03))), MeshMaterial3d(blue), Transform::from_xyz(0.0, 0.025, -half - 0.04)));

    // Robot.
    let cfg = &sim.world.config;
    let frame = mat(Color::srgb(0.2, 0.22, 0.25), 0.5, 0.7);
    let plate = mat(Color::srgb(0.85, 0.65, 0.1), 0.6, 0.2);
    let tire = mat(Color::srgb(0.08, 0.08, 0.09), 0.9, 0.0);
    let hub = mat(Color::srgb(0.65, 0.66, 0.7), 0.3, 0.9);
    commands
        .spawn((Chassis, Transform::default(), Visibility::default()))
        .with_children(|robot| {
            robot.spawn((Mesh3d(meshes.add(Cuboid::new(cfg.length, cfg.frame_height, cfg.width * 0.62))), MeshMaterial3d(frame.clone()), Transform::default()));
            for side in [-1.0, 1.0] {
                robot.spawn((
                    Mesh3d(meshes.add(Cuboid::new(cfg.length, cfg.frame_height, 0.012))),
                    MeshMaterial3d(frame.clone()),
                    Transform::from_xyz(0.0, 0.0, side * (cfg.width / 2.0 - 0.006)),
                ));
            }
            robot.spawn((
                Mesh3d(meshes.add(Cuboid::new(cfg.length * 0.55, 0.006, cfg.width * 0.55))),
                MeshMaterial3d(plate.clone()),
                Transform::from_xyz(0.0, cfg.frame_height / 2.0 + 0.003, 0.0),
            ));
            // Arrow-ish marker at the front so heading is obvious from any view.
            robot.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.03, 0.01, cfg.width * 0.4))),
                MeshMaterial3d(plate.clone()),
                Transform::from_xyz(cfg.length / 2.0 - 0.02, cfg.frame_height / 2.0 + 0.005, 0.0),
            ));
            for (i, w) in cfg.wheels.iter().enumerate() {
                let width = 0.045;
                robot
                    .spawn((WheelVisual(i), Transform::from_xyz(w.position[0], w.position[1], w.position[2]), Visibility::default()))
                    .with_children(|wheel| {
                        wheel.spawn((
                            Mesh3d(meshes.add(Cylinder::new(w.radius, width))),
                            MeshMaterial3d(tire.clone()),
                            Transform::from_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                        ));
                        // Rollers (or tread blocks) so wheel spin and roller angle are visible.
                        let slant = match w.kind {
                            WheelKind::Mecanum { rollers_right } => if rollers_right { -PI / 4.0 } else { PI / 4.0 },
                            _ => 0.0,
                        };
                        for k in 0..8 {
                            let a = k as f32 * PI / 4.0;
                            wheel.spawn((
                                Mesh3d(meshes.add(Cuboid::new(0.012, 0.006, width * 1.05))),
                                MeshMaterial3d(hub.clone()),
                                Transform::from_rotation(Quat::from_rotation_z(a))
                                    * Transform::from_xyz(0.0, w.radius, 0.0)
                                    * Transform::from_rotation(Quat::from_rotation_y(slant)),
                            ));
                        }
                    });
            }
        });

    // Game pieces.
    for (i, piece) in sim.world.pieces.iter().enumerate() {
        let [r, g, b] = piece.color;
        commands.spawn((
            PieceVisual(i),
            Mesh3d(meshes.add(Sphere::new(piece.radius).mesh().uv(24, 16))),
            MeshMaterial3d(mat(Color::srgb(r, g, b), 0.55, 0.0)),
            Transform::default(),
        ));
    }
}

fn to_transform(iso: &sim_core::world::Iso) -> Transform {
    let t = iso.translation;
    let q = iso.rotation;
    Transform::from_xyz(t.x, t.y, t.z).with_rotation(Quat::from_xyzw(q.i, q.j, q.k, q.w))
}

#[allow(clippy::type_complexity)]
fn sync_visuals(
    sim: Res<Sim>,
    mut chassis: Query<&mut Transform, (With<Chassis>, Without<WheelVisual>, Without<PieceVisual>)>,
    mut wheels: Query<(&WheelVisual, &mut Transform), (Without<Chassis>, Without<PieceVisual>)>,
    mut pieces: Query<(&PieceVisual, &mut Transform), (Without<Chassis>, Without<WheelVisual>)>,
) {
    let world = &sim.world;
    for mut t in &mut chassis {
        *t = to_transform(&world.chassis_position());
    }
    for (WheelVisual(i), mut t) in &mut wheels {
        let (cfg, state) = (&world.config.wheels[*i], &world.wheels[*i]);
        let drop = SUSPENSION_REST - state.compression.min(SUSPENSION_REST + cfg.radius);
        t.translation = Vec3::new(cfg.position[0], cfg.position[1] - drop, cfg.position[2]);
        t.rotation = Quat::from_rotation_z(-(state.angle as f32));
    }
    for (PieceVisual(i), mut t) in &mut pieces {
        *t = to_transform(&world.piece_position(&world.pieces[*i]));
    }
}

fn update_camera(view: Res<CameraView>, sim: Res<Sim>, mut camera: Query<&mut Transform, With<MainCamera>>) {
    let Ok(mut cam) = camera.single_mut() else { return };
    let target = match *view {
        CameraView::Audience => Transform::from_xyz(0.0, 3.0, 3.6).looking_at(Vec3::new(0.0, 0.0, 0.2), Vec3::Y),
        CameraView::Top => Transform::from_xyz(0.0, 5.2, 0.001).looking_at(Vec3::ZERO, Vec3::NEG_Z),
        CameraView::Chase => {
            let robot = to_transform(&sim.world.chassis_position());
            let back = robot.rotation * Vec3::NEG_X;
            let mut flat = Vec3::new(back.x, 0.0, back.z).normalize_or_zero();
            if flat == Vec3::ZERO {
                flat = Vec3::Z;
            }
            Transform::from_translation(robot.translation + flat * 0.9 + Vec3::Y * 0.55).looking_at(robot.translation, Vec3::Y)
        }
    };
    *cam = target;
}

// ---------------------------------------------------------------- HUD

fn setup_hud(mut commands: Commands) {
    let panel = |commands: &mut Commands, node: Node| {
        commands.spawn((node, BackgroundColor(Color::srgba(0.05, 0.06, 0.07, 0.78)))).id()
    };
    let font = |size: f32| TextFont { font_size: size, ..default() };

    let left = panel(&mut commands, Node {
        position_type: PositionType::Absolute,
        top: Val::Px(12.0),
        left: Val::Px(12.0),
        padding: UiRect::all(Val::Px(12.0)),
        max_width: Val::Px(360.0),
        ..default()
    });
    let text = commands.spawn((DsText, Text::new(""), font(15.0), TextColor(Color::srgb(0.92, 0.93, 0.94)))).id();
    commands.entity(left).add_child(text);

    let right = panel(&mut commands, Node {
        position_type: PositionType::Absolute,
        top: Val::Px(12.0),
        right: Val::Px(12.0),
        padding: UiRect::all(Val::Px(12.0)),
        min_width: Val::Px(260.0),
        max_width: Val::Px(440.0),
        ..default()
    });
    let text = commands.spawn((TelemetryText, Text::new(""), font(14.0), TextColor(Color::srgb(0.85, 0.95, 0.85)))).id();
    commands.entity(right).add_child(text);

    let bottom = panel(&mut commands, Node {
        position_type: PositionType::Absolute,
        bottom: Val::Px(12.0),
        left: Val::Px(12.0),
        padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
        ..default()
    });
    let text = commands.spawn((StatusText, Text::new(""), font(13.0), TextColor(Color::srgb(0.75, 0.78, 0.8)))).id();
    commands.entity(bottom).add_child(text);
}

#[allow(clippy::type_complexity)]
fn update_hud(
    sim: Res<Sim>,
    ds: Res<DriverStation>,
    mut ds_text: Query<&mut Text, (With<DsText>, Without<TelemetryText>, Without<StatusText>)>,
    mut tele_text: Query<&mut Text, (With<TelemetryText>, Without<DsText>, Without<StatusText>)>,
    mut status_text: Query<&mut Text, (With<StatusText>, Without<DsText>, Without<TelemetryText>)>,
) {
    let link = &sim.link;
    let state = link.last.state;

    let mut s = String::from("DRIVER STATION\n\n");
    if !link.connected() {
        s += "Robot runner not connected.\nStart it with:\n  cd robot\n  gradlew :runner:run\n";
    } else if link.opmodes.is_empty() {
        s += "No OpModes found in TeamCode.\n";
    } else {
        for (i, m) in link.opmodes.iter().enumerate() {
            let kind = if m.flavor == Flavor::Autonomous { "AUTO" } else { "TELE" };
            s += &format!("{} {kind}  {}\n", if i == ds.selected { ">" } else { " " }, m.name);
        }
        let label = match state {
            OpModeState::Idle => "ready",
            OpModeState::Init => "INIT  (Enter to start)",
            OpModeState::Running => "RUNNING",
            OpModeState::Stopped => "stopped",
        };
        s += &format!("\nState: {label}\n");
        if let (Some(t0), OpModeState::Running) = (ds.started_at, state) {
            let elapsed = sim.world.time - t0;
            let is_auto = link.opmodes.get(ds.selected).is_some_and(|m| m.flavor == Flavor::Autonomous);
            if is_auto {
                s += &format!("Time left: {:.1} s\n", (AUTO_SECONDS - elapsed).max(0.0));
            } else {
                s += &format!("Time: {elapsed:.1} s\n");
            }
        }
        if !link.last.error.is_empty() {
            s += &format!("\nERROR: {}\n", link.last.error);
        }
    }
    s += "\nTab select   Enter init/start   Backspace stop\nR reset robot   P pause   F1/F2/F3 camera\n\nGamepad: WASD left stick, arrows right stick,\nSpace/B/X/Y buttons, Q/E bumpers, Z/C triggers,\nIJKL d-pad, O options/start";
    if let Ok(mut t) = ds_text.single_mut() {
        t.0 = s;
    }

    if let Ok(mut t) = tele_text.single_mut() {
        t.0 = if link.telemetry.is_empty() { "TELEMETRY\n\n(none)".into() } else { format!("TELEMETRY\n\n{}", link.telemetry.join("\n")) };
    }

    let world = &sim.world;
    let pose = world.pose();
    if let Ok(mut t) = status_text.single_mut() {
        t.0 = format!(
            "{}   sim {:.1} s   battery {:.2} V  {:.1} A   robot x {:.1} in  y {:.1} in  heading {:.1}°   {}",
            if sim.paused { "PAUSED" } else { "LIVE" },
            world.time,
            world.battery.voltage,
            world.battery.current,
            pose.x / INCH,
            pose.y / INCH,
            pose.heading.to_degrees(),
            link.status,
        );
    }
}
