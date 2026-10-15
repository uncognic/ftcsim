package com.qualcomm.hardware.lynx;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

/** An I2C device on a REV hub's I2C port. */
public interface LynxI2cDeviceSynch extends I2cDeviceSynchSimple {
    enum BusSpeed { STANDARD_100K, FAST_400K, FASTPLUS_1M, HIGHSPEED_3_4M }

    void setBusSpeed(BusSpeed speed);
}
