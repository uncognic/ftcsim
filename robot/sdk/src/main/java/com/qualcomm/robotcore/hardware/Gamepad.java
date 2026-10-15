package com.qualcomm.robotcore.hardware;

/**
 * Gamepad state, updated by the runner every hardware cycle. Stick Y is negative when pushed
 * forward. PlayStation-style aliases (cross, circle, ...) mirror the Xbox-style fields.
 */
public class Gamepad {
    public volatile float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public volatile float left_trigger, right_trigger;
    public volatile boolean a, b, x, y;
    public volatile boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public volatile boolean left_bumper, right_bumper, left_stick_button, right_stick_button;
    public volatile boolean start, back, guide;
    public volatile boolean cross, circle, square, triangle, share, options, ps;

    public boolean atRest() {
        return left_stick_x == 0 && left_stick_y == 0 && right_stick_x == 0 && right_stick_y == 0
                && left_trigger == 0 && right_trigger == 0;
    }

    /** Copies state into this gamepad. Called by the runner, not by OpModes. */
    public void copy(Gamepad g) {
        left_stick_x = g.left_stick_x; left_stick_y = g.left_stick_y;
        right_stick_x = g.right_stick_x; right_stick_y = g.right_stick_y;
        left_trigger = g.left_trigger; right_trigger = g.right_trigger;
        a = g.a; b = g.b; x = g.x; y = g.y;
        dpad_up = g.dpad_up; dpad_down = g.dpad_down; dpad_left = g.dpad_left; dpad_right = g.dpad_right;
        left_bumper = g.left_bumper; right_bumper = g.right_bumper;
        left_stick_button = g.left_stick_button; right_stick_button = g.right_stick_button;
        start = g.start; back = g.back; guide = g.guide;
        cross = a; circle = b; square = x; triangle = y; share = back; options = start; ps = guide;
    }
}
