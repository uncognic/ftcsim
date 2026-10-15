package com.qualcomm.hardware.rev;

import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

/** How a REV Control/Expansion Hub is mounted, described by where its logo and USB port face. */
public class RevHubOrientationOnRobot implements ImuOrientationOnRobot {
    public enum LogoFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }

    public enum UsbFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }

    private final LogoFacingDirection logoFacingDirection;
    private final UsbFacingDirection usbFacingDirection;

    public RevHubOrientationOnRobot(LogoFacingDirection logoFacingDirection, UsbFacingDirection usbFacingDirection) {
        if (axis(logoFacingDirection.name()) == axis(usbFacingDirection.name())) {
            throw new IllegalArgumentException("The specified REV Hub orientation is invalid: the logo and USB port cannot face along the same axis ("
                    + logoFacingDirection + ", " + usbFacingDirection + ")");
        }
        this.logoFacingDirection = logoFacingDirection;
        this.usbFacingDirection = usbFacingDirection;
    }

    private static int axis(String direction) {
        switch (direction) {
            case "UP":
            case "DOWN":
                return 0;
            case "FORWARD":
            case "BACKWARD":
                return 1;
            default:
                return 2;
        }
    }

    public LogoFacingDirection getLogoFacingDirection() {
        return logoFacingDirection;
    }

    public UsbFacingDirection getUsbFacingDirection() {
        return usbFacingDirection;
    }
}
