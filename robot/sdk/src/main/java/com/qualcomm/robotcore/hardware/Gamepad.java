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

    private static final int A = 0;
    private static final int B = 1;
    private static final int X = 2;
    private static final int Y = 3;
    private static final int DPAD_UP = 4;
    private static final int DPAD_DOWN = 5;
    private static final int DPAD_LEFT = 6;
    private static final int DPAD_RIGHT = 7;
    private static final int LEFT_BUMPER = 8;
    private static final int RIGHT_BUMPER = 9;
    private static final int LEFT_STICK_BUTTON = 10;
    private static final int RIGHT_STICK_BUTTON = 11;
    private static final int START = 12;
    private static final int BACK = 13;
    private static final int GUIDE = 14;
    private static final int CROSS = 15;
    private static final int CIRCLE = 16;
    private static final int SQUARE = 17;
    private static final int TRIANGLE = 18;
    private static final int OPTIONS = 19;
    private static final int SHARE = 20;
    private static final int PS = 21;
    private static final int BUTTONS = 22;

    /** Copies state into this gamepad and records button edges. Called by the runner, not by OpModes. */
    public synchronized void copy(Gamepad g) {
        left_stick_x = g.left_stick_x; left_stick_y = g.left_stick_y;
        right_stick_x = g.right_stick_x; right_stick_y = g.right_stick_y;
        left_trigger = g.left_trigger; right_trigger = g.right_trigger;
        a = g.a; b = g.b; x = g.x; y = g.y;
        dpad_up = g.dpad_up; dpad_down = g.dpad_down; dpad_left = g.dpad_left; dpad_right = g.dpad_right;
        left_bumper = g.left_bumper; right_bumper = g.right_bumper;
        left_stick_button = g.left_stick_button; right_stick_button = g.right_stick_button;
        start = g.start; back = g.back; guide = g.guide;
        cross = a; circle = b; square = x; triangle = y; share = back; options = start; ps = guide;
        boolean[] now = buttons();
        for (int i = 0; i < now.length; i++) {
            if (now[i] && !last[i]) pressed[i] = true;
            if (!now[i] && last[i]) released[i] = true;
        }
        last = now;
    }

    // ---- edge detection: each method reports whether the button changed since that same method was last called

    private boolean[] last = new boolean[BUTTONS];
    private final boolean[] pressed = new boolean[BUTTONS];
    private final boolean[] released = new boolean[BUTTONS];

    private boolean[] buttons() {
        return new boolean[] {a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button, start, back, guide, cross, circle, square, triangle, options, share, ps};
    }

    private synchronized boolean takePressed(int i) {
        boolean p = pressed[i];
        pressed[i] = false;
        return p;
    }

    private synchronized boolean takeReleased(int i) {
        boolean r = released[i];
        released[i] = false;
        return r;
    }

    public boolean aWasPressed() {
        return takePressed(A);
    }

    public boolean aWasReleased() {
        return takeReleased(A);
    }

    public boolean bWasPressed() {
        return takePressed(B);
    }

    public boolean bWasReleased() {
        return takeReleased(B);
    }

    public boolean xWasPressed() {
        return takePressed(X);
    }

    public boolean xWasReleased() {
        return takeReleased(X);
    }

    public boolean yWasPressed() {
        return takePressed(Y);
    }

    public boolean yWasReleased() {
        return takeReleased(Y);
    }

    public boolean dpadUpWasPressed() {
        return takePressed(DPAD_UP);
    }

    public boolean dpadUpWasReleased() {
        return takeReleased(DPAD_UP);
    }

    public boolean dpadDownWasPressed() {
        return takePressed(DPAD_DOWN);
    }

    public boolean dpadDownWasReleased() {
        return takeReleased(DPAD_DOWN);
    }

    public boolean dpadLeftWasPressed() {
        return takePressed(DPAD_LEFT);
    }

    public boolean dpadLeftWasReleased() {
        return takeReleased(DPAD_LEFT);
    }

    public boolean dpadRightWasPressed() {
        return takePressed(DPAD_RIGHT);
    }

    public boolean dpadRightWasReleased() {
        return takeReleased(DPAD_RIGHT);
    }

    public boolean leftBumperWasPressed() {
        return takePressed(LEFT_BUMPER);
    }

    public boolean leftBumperWasReleased() {
        return takeReleased(LEFT_BUMPER);
    }

    public boolean rightBumperWasPressed() {
        return takePressed(RIGHT_BUMPER);
    }

    public boolean rightBumperWasReleased() {
        return takeReleased(RIGHT_BUMPER);
    }

    public boolean leftStickButtonWasPressed() {
        return takePressed(LEFT_STICK_BUTTON);
    }

    public boolean leftStickButtonWasReleased() {
        return takeReleased(LEFT_STICK_BUTTON);
    }

    public boolean rightStickButtonWasPressed() {
        return takePressed(RIGHT_STICK_BUTTON);
    }

    public boolean rightStickButtonWasReleased() {
        return takeReleased(RIGHT_STICK_BUTTON);
    }

    public boolean startWasPressed() {
        return takePressed(START);
    }

    public boolean startWasReleased() {
        return takeReleased(START);
    }

    public boolean backWasPressed() {
        return takePressed(BACK);
    }

    public boolean backWasReleased() {
        return takeReleased(BACK);
    }

    public boolean guideWasPressed() {
        return takePressed(GUIDE);
    }

    public boolean guideWasReleased() {
        return takeReleased(GUIDE);
    }

    public boolean crossWasPressed() {
        return takePressed(CROSS);
    }

    public boolean crossWasReleased() {
        return takeReleased(CROSS);
    }

    public boolean circleWasPressed() {
        return takePressed(CIRCLE);
    }

    public boolean circleWasReleased() {
        return takeReleased(CIRCLE);
    }

    public boolean squareWasPressed() {
        return takePressed(SQUARE);
    }

    public boolean squareWasReleased() {
        return takeReleased(SQUARE);
    }

    public boolean triangleWasPressed() {
        return takePressed(TRIANGLE);
    }

    public boolean triangleWasReleased() {
        return takeReleased(TRIANGLE);
    }

    public boolean optionsWasPressed() {
        return takePressed(OPTIONS);
    }

    public boolean optionsWasReleased() {
        return takeReleased(OPTIONS);
    }

    public boolean shareWasPressed() {
        return takePressed(SHARE);
    }

    public boolean shareWasReleased() {
        return takeReleased(SHARE);
    }

    public boolean psWasPressed() {
        return takePressed(PS);
    }

    public boolean psWasReleased() {
        return takeReleased(PS);
    }
}
