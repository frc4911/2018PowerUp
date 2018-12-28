package lib.util.drivers;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonUtils {

    /**
     * Run this on a fresh talon to produce good values for the defaults.
     */
    public static String getFullTalonInfo(WPI_TalonSRX talon) {
        StringBuilder sb = new StringBuilder()
                .append("getDeviceID = ").append(talon.getDeviceID()).append("\n")
                .append("getDescription = ").append(talon.getDescription()).append("\n")
                .append("getFirmwareVersion = ").append(talon.getFirmwareVersion()).append("\n")
                .append("getExpiration = ").append(talon.getExpiration()).append("\n")
                .append("getClass = ").append(talon.getClass()).append("\n")
                .append("toString = ").append(talon.toString()).append("\n")
                .append("isAlive = ").append(talon.isAlive()).append("\n")
                .append("hasResetOccurred = ").append(talon.hasResetOccurred()).append("\n")
                .append("getLastError = ").append(talon.getLastError()).append("\n")
                .append("isSafetyEnabled = ").append(talon.isSafetyEnabled()).append("\n")

                .append("get = ").append(talon.get()).append("\n")
                .append("getControlMode = ").append(talon.getControlMode()).append("\n")
                .append("getBusVoltage = ").append(talon.getBusVoltage()).append("\n")
                .append("getMotorOutputVoltage  = ").append(talon.getMotorOutputVoltage ()).append("\n")
                .append("getMotorOutputPercent  = ").append(talon.getMotorOutputPercent()).append("\n")
                .append("getOutputCurrent = ").append(talon.getOutputCurrent()).append("\n")
                .append("getTemperature = ").append(talon.getTemperature()).append("\n")

                .append("getInverted = ").append(talon.getInverted()).append("\n")
                .append("isSensorPresent = ").append(talon.getSensorCollection().getPulseWidthRiseToRiseUs() != 0).append("\n")

                .append("getSelectedSensorPosition(0) = ").append(talon.getSelectedSensorPosition(0)).append("\n")
                .append("getSelectedSensorVelocity(0) = ").append(talon.getSelectedSensorVelocity(0)).append("\n")
                .append("getSelectedSensorPosition(1) = ").append(talon.getSelectedSensorPosition(1)).append("\n")
                .append("getSelectedSensorVelocity(1) = ").append(talon.getSelectedSensorVelocity(1)).append("\n")

                // SensorCollection
                .append("getAnalogInRaw = ").append(talon.getSensorCollection().getAnalogInRaw()).append("\n")
                .append("getAnalogIn = ").append(talon.getSensorCollection().getAnalogIn()).append("\n")
                .append("getAnalogInVel = ").append(talon.getSensorCollection().getAnalogInVel()).append("\n")
                .append("getPinStateQuadA = ").append(talon.getSensorCollection().getPinStateQuadA()).append("\n")
                .append("getPinStateQuadB = ").append(talon.getSensorCollection().getPinStateQuadB()).append("\n")
                .append("getPinStateQuadIdx = ").append(talon.getSensorCollection().getPinStateQuadIdx()).append("\n")
                .append("getPulseWidthPosition = ").append(talon.getSensorCollection().getPulseWidthPosition()).append("\n")
                .append("getPulseWidthRiseToFallUs = ").append(talon.getSensorCollection().getPulseWidthRiseToFallUs()).append("\n")
				.append("getPulseWidthRiseToRiseUs = ").append(talon.getSensorCollection().getPulseWidthRiseToRiseUs()).append("\n")
				.append("getPulseWidthVelocity = ").append(talon.getSensorCollection().getPulseWidthVelocity()).append("\n")
				.append("getQuadraturePosition = ").append(talon.getSensorCollection().getQuadraturePosition()).append("\n")
				.append("getQuadratureVelocity = ").append(talon.getSensorCollection().getQuadratureVelocity()).append("\n")
                .append("isRevLimitSwitchClosed = ").append(talon.getSensorCollection().isRevLimitSwitchClosed()).append("\n")
                .append("isFwdLimitSwitchClosed = ").append(talon.getSensorCollection().isFwdLimitSwitchClosed()).append("\n")

                .append("getClosedLoopError(0) = ").append(talon.getClosedLoopError(0)).append("\n")
//                .append("getClosedLoopTarget(0) = ").append(talon.getClosedLoopTarget(0)).append("\n")
                .append("getClosedLoopError(1) = ").append(talon.getClosedLoopError(1)).append("\n")
//                .append("getClosedLoopTarget(1) = ").append(talon.getClosedLoopTarget(1)).append("\n")

                .append("getMotionProfileTopLevelBufferCount = ").append(talon.getMotionProfileTopLevelBufferCount()).append("\n")
                .append("isMotionProfileTopLevelBufferFull = ").append(talon.isMotionProfileTopLevelBufferFull()).append("\n");

                
/*                
                .append("isForwardSoftLimitEnabled = ").append(talon.isForwardSoftLimitEnabled()).append("\n")
                .append("getForwardSoftLimit = ").append(talon.getForwardSoftLimit()).append("\n")

                .append("isReverseSoftLimitEnabled = ").append(talon.isReverseSoftLimitEnabled()).append("\n")
                .append("getReverseSoftLimit = ").append(talon.getReverseSoftLimit()).append("\n")
                
                .append("isZeroSensorPosOnFwdLimitEnabled = ").append(talon.isZeroSensorPosOnFwdLimitEnabled()).append("\n")
                .append("isZeroSensorPosOnRevLimitEnabled = ").append(talon.isZeroSensorPosOnRevLimitEnabled()).append("\n")
                .append("isZeroSensorPosOnIndexEnabled = ").append(talon.isZeroSensorPosOnIndexEnabled()).append("\n")

                .append("pidGet = ").append(talon.pidGet()).append("\n")
                .append("getBrakeEnableDuringNeutral = ").append(talon.getBrakeEnableDuringNeutral()).append("\n")

                .append("GetVelocityMeasurementPeriod = ").append(talon.getVelocityMeasurementPeriod()).append("\n")
                .append("GetVelocityMeasurementWindow = ").append(talon.getVelocityMeasurementWindow()).append("\n")

                .append("getMotionMagicCruiseVelocity = ").append(talon.getMotionMagicCruiseVelocity()).append("\n")
                .append("getMotionMagicAcceleration = ").append(talon.getMotionMagicAcceleration()).append("\n")

                .append("GetNominalClosedLoopVoltage = ").append(talon.getNominalClosedLoopVoltage()).append("\n")
                // .append("createTableListener = ").append(talon.createTableListener()).append("\n")
                
                .append("getNumberOfQuadIdxRises = ").append(talon.getNumberOfQuadIdxRises()).append("\n")
                .append("isControlEnabled = ").append(talon.isControlEnabled()).append("\n")
                .append("getTable = ").append(talon.getTable()).append("\n")
                //.append("isEnabled = ").append(talon.isEnabled()).append("\n")
                .append("getSmartDashboardType = ").append(talon.getSmartDashboardType()).append("\n")
                .append("getCloseLoopRampRate = ").append(talon.getCloseLoopRampRate()).append("\n")
                // .append("getMotionMagicActTrajPosition =").append(talon.getMotionMagicActTrajPosition()).append("\n")

                .append("getPIDSourceType = ").append(talon.getPIDSourceType()).append("\n")

                .append("getF (slot 0) = ").append(talon.configGetParameter(ParamEnum.eProfileParamSlot_F, 0, 0)).append("\n")
                .append("getP (slot 0)= ").append(talon.configGetParameter(ParamEnum.eProfileParamSlot_P, 0, 0)).append("\n")
                .append("getI (slot 0)= ").append(talon.configGetParameter(ParamEnum.eProfileParamSlot_I, 0, 0)).append("\n")
                .append("getD (slot 0)= ").append(talon.configGetParameter(ParamEnum.eProfileParamSlot_D, 0, 0)).append("\n")
                .append("getIZone (slot 0)= ").append(talon.configGetParameter(ParamEnum.eProfileParamSlot_IZone, 0, 0)).append("\n")
                .append("getIntegralAccumulator(0) = ").append(talon.getIntegralAccumulator(0)).append("\n")

                // .append("getActiveTrajectoryVelocity  =").append(talon.getActiveTrajectoryVelocity ()).append("\n")
*/
                
                Faults faults = new Faults();
        		talon.getFaults(faults);
        		sb.append("Fault ForwardLimitSwitch = ").append(faults.ForwardLimitSwitch).append("\n")
        		.append("Fault ReverseLimitSwitch = ").append(faults.ReverseLimitSwitch).append("\n")
        		.append("Fault ForwardSoftLimit = ").append(faults.ForwardSoftLimit).append("\n")
        		.append("Fault ReverseSoftLimit = ").append(faults.ForwardLimitSwitch).append("\n")
        		.append("Fault HardwareFailure = ").append(faults.HardwareFailure).append("\n")
        		.append("Fault ResetDuringEn = ").append(faults.ResetDuringEn).append("\n")
        		.append("Fault SensorOverflow = ").append(faults.SensorOverflow).append("\n")
        		.append("Fault SensorOutOfPhase = ").append(faults.SensorOutOfPhase).append("\n")
        		.append("Fault HardwareESDReset = ").append(faults.HardwareESDReset).append("\n")
        		.append("Fault RemoteLossOfSignal = ").append(faults.RemoteLossOfSignal).append("\n");
        		
        		StickyFaults stickyFaults = new StickyFaults();
        		talon.getStickyFaults(stickyFaults);
        		sb.append("StickyFault ForwardLimitSwitch = ").append(stickyFaults.ForwardLimitSwitch).append("\n")
        		.append("StickyFault ReverseLimitSwitch = ").append(stickyFaults.ReverseLimitSwitch).append("\n")
        		.append("StickyFault ForwardSoftLimit = ").append(stickyFaults.ForwardSoftLimit).append("\n")
        		.append("StickyFault ReverseSoftLimit = ").append(stickyFaults.ReverseSoftLimit).append("\n")
        		.append("StickyFault ResetDuringEn = ").append(stickyFaults.ResetDuringEn).append("\n")
        		.append("StickyFault SensorOverflow = ").append(stickyFaults.SensorOverflow).append("\n")
        		.append("StickyFault SensorOutOfPhase = ").append(stickyFaults.SensorOutOfPhase).append("\n")
        		.append("StickyFault HardwareESDReset = ").append(stickyFaults.HardwareESDReset).append("\n")
        		.append("StickyFault RemoteLossOfSignal = ").append(stickyFaults.RemoteLossOfSignal).append("\n");
        		
        return sb.toString();
    }
}
