package lib.util.drivers;

import org.usfirst.frc.team4911.robot.Constants;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.MotorSafety;


/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. 
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the application.
 * 
 * Borrowed from team 254
 */
public class CANTalonFactory {

    public static class Configuration {
        public boolean LIMIT_SWITCH_NORMALLY_OPEN = true;
        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0;
        public double PEAK_NOMINAL = 1.0;
        public boolean ENABLE_BRAKE = true;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public int FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public double NOMINAL_CLOSED_LOOP = 1.0;
        public int REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double VOLTAGE_COMPENSATION_RAMP_RATE = 0;
        public double VOLTAGE_RAMP_RATE = 0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static WPI_TalonSRX createDefaultTalon(int id) {
        return createTalon(id, kDefaultConfiguration);
    }

    public static WPI_TalonSRX createPermanentSlaveTalon(int id, int master_id) {
        final WPI_TalonSRX  talon = createTalon(id, kSlaveConfiguration);
        talon.set(ControlMode.Follower, master_id);
        return talon;
    }

    public static WPI_TalonSRX  createTalon(int id, Configuration config) {
        WPI_TalonSRX talon = new LazyCANTalon(id, config.CONTROL_FRAME_PERIOD_MS);
        talon.set(ControlMode.PercentOutput, 0);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.setIntegralAccumulator(0, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
        talon.setIntegralAccumulator(0, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
        talon.clearMotionProfileHasUnderrun(Constants.kTimeOutMs);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(Constants.kTimeOutMs);
//        talon.configForwardLimitSwitchSource(
//        		LimitSwitchSource.FeedbackConnector, 
//        		config.LIMIT_SWITCH_NORMALLY_OPEN ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 
//        		Constants.kTimeOutMs);
        talon.configNominalOutputForward (config.NOMINAL_CLOSED_LOOP, Constants.kTimeOutMs);
        talon.configNominalOutputReverse (-config.NOMINAL_CLOSED_LOOP, Constants.kTimeOutMs);
        talon.configPeakOutputForward (config.PEAK_NOMINAL, Constants.kTimeOutMs );
        talon.configPeakOutputReverse (config.PEAK_NOMINAL, Constants.kTimeOutMs);
        // Assume this
//        talon.configReverseLimitSwitchSource(
//        		LimitSwitchSource.FeedbackConnector, 
//        		config.LIMIT_SWITCH_NORMALLY_OPEN ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 
//        		Constants.kTimeOutMs);
        talon.setNeutralMode(config.ENABLE_BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, Constants.kTimeOutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, Constants.kTimeOutMs);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);
        //talon.enableZeroSensorPositionOnForwardLimit(false);
        //talon.enableZeroSensorPositionOnIndex(false, false);
        //talon.enableZeroSensorPositionOnReverseLimit(false);
        
        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(false);
        talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, Constants.kTimeOutMs);
        talon.setExpiration(config.EXPIRATION_TIMEOUT_SECONDS);
        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, Constants.kTimeOutMs);
        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, Constants.kTimeOutMs);

        talon.selectProfileSlot(0, Constants.kTimeOutMs);
        //talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
        //talon.getSensorCollection().setAnalogPosition(0, Constants.kTimeOutMs);
        //talon.getSensorCollection().setPulseWidthPosition(0, Constants.kTimeOutMs);
        talon.setSafetyEnabled(config.SAFETY_ENABLED);
        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, Constants.kTimeOutMs);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, Constants.kTimeOutMs);
        talon.configClosedloopRamp(config.VOLTAGE_COMPENSATION_RAMP_RATE, Constants.kTimeOutMs);
        talon.configOpenloopRamp (config.VOLTAGE_RAMP_RATE, Constants.kTimeOutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, Constants.kTimeOutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, Constants.kTimeOutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, Constants.kTimeOutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, Constants.kTimeOutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, Constants.kTimeOutMs);

        return talon;
    }

}
