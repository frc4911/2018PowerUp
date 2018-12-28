package lib.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This class is a thin wrapper around the WPI_TalonSRX that reduces 
 * CAN bus / CPU overhead by skipping duplicate set commands. 
 * (By default the Talon flushes the Tx buffer on every set call).
 * 
 * Borrowed from team 254
 */

public class LazyCANTalon extends WPI_TalonSRX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyCANTalon(int deviceNumber, int controlPeriodMs) {
        this(deviceNumber);
        this.changeMotionControlFramePeriod(controlPeriodMs);
    }

    public LazyCANTalon(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void set(double value) {
        if (value != mLastSet || getControlMode() != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = getControlMode();
            super.set(value);
        }
    }
}