/// Gamepad state as the SDK's `Gamepad` exposes it. Stick Y is negative when pushed forward.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GamepadState {
    pub left_stick_x: f32,
    pub left_stick_y: f32,
    pub right_stick_x: f32,
    pub right_stick_y: f32,
    pub left_trigger: f32,
    pub right_trigger: f32,
    pub buttons: u32,
}

/// Bit positions in [`GamepadState::buttons`]. Must match `org.ftcsim.runner.Protocol`.
pub mod buttons {
    pub const A: u32 = 1 << 0;
    pub const B: u32 = 1 << 1;
    pub const X: u32 = 1 << 2;
    pub const Y: u32 = 1 << 3;
    pub const LEFT_BUMPER: u32 = 1 << 4;
    pub const RIGHT_BUMPER: u32 = 1 << 5;
    pub const DPAD_UP: u32 = 1 << 6;
    pub const DPAD_DOWN: u32 = 1 << 7;
    pub const DPAD_LEFT: u32 = 1 << 8;
    pub const DPAD_RIGHT: u32 = 1 << 9;
    pub const START: u32 = 1 << 10;
    pub const BACK: u32 = 1 << 11;
    pub const GUIDE: u32 = 1 << 12;
    pub const LEFT_STICK_BUTTON: u32 = 1 << 13;
    pub const RIGHT_STICK_BUTTON: u32 = 1 << 14;
}
