package com.qualcomm.robotcore.hardware;

/** How long an I2C write waits: not at all, until it is queued in order, or until the device has it. */
public enum I2cWaitControl {
    NONE, ATOMIC, WRITTEN
}
