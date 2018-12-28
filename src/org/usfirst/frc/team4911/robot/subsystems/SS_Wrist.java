package org.usfirst.frc.team4911.robot.subsystems;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team4911.robot.Constants;
import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;
import org.usfirst.frc.team4911.robot.commands.C_DriveWrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.util.ReflectingCSVWriter;
import lib.util.drivers.LazyCANTalon;

/**
 *
 */
public class SS_Wrist extends Subsystem {
	
    public static class WristDebugOutput {
        public double timestamp;
        public double setPoint;
        public double voltage;
        public double current;
        public double velocity;
        public double angle;
        public ControlMode controlMode;
        public int sensorvalue;
    }

	private static enum WristSystemState {
		DEFAULT,
		AUTO_FLIPUP_COLLECTING,	// Wrist in COLLECT position
		AUTO_FLIPUP_RASING_WRIST,	// Cube collected - sensors triggered
		AUTO_FLIPUP_HOLDING,	// Holding Cube - sensors triggered
		ZERO_WRIST,	// Move wrist slowly until limit switch is engaged
//		OPEN_LOOP
	}
    
    private final ReflectingCSVWriter<WristDebugOutput> mCSVWriter;
    private WristDebugOutput mDebug = new WristDebugOutput();
	
	private static final int kPositionHoldSlotIdx = 0;
	private static final int kPositionControlSlotIdx = 1;
	
	public static double TicksPerDeg = 2927.0 / 90.0;
	private final double kp = .0017;
	private final double maxSpeed = 0.65;
	private final double minSpeed = 0.15;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private static final boolean BRAKE_MODE = true;
	private static final boolean INVERTED = true;
	private static final boolean ENC_INVERTED = false;

	public static final int QUAD_ENCODER_VALUE_AT_TOP_LIMIT_PIXEL = 4515;// 4710; //4228; -1907
	public static final int QUAD_ENCODER_VALUE_AT_TOP_LIMIT_VOXEL = 4710;//4228; -1907

	private static final Map<WristPresets, Integer> presetAngles = Collections
			.unmodifiableMap(new HashMap<WristPresets, Integer>() {
				private static final long serialVersionUID = -1823762655581500254L;
				{
					put(WristPresets.COLLECT, 45); // TODO: Voxel = 50, Pixel = 33
					put(WristPresets.PORTAL, 85); // 80
					put(WristPresets.SWITCH, 105); // TODO: Voxel = 105, Pixel = 95
					put(WristPresets.SCALE_BACKWARD, 110); // 120
					put(WristPresets.SCALE_FORWARD, -42); // -67);
					put(WristPresets.FLIP_UP, 130);
					put(WristPresets.SCALE_BACKWARD_LOW, 125);
					put(WristPresets.SCALE_BACKWARD_HIGH, 90);
				}
			});

	private final WPI_TalonSRX mTalon;

	private final DigitalInput cubeCollectorSensorRight; 
	private final DigitalInput cubeCollectorSensorLeft; 

	private boolean isHomed = false;
	private Faults toFill = new Faults();
	private boolean wristHolding = false;
	private boolean autoFlipUpActivated = false; 
	
//	private final static double kZeroWristPeriod = 2.0;
	private final static double kAutoFlipUpRaiseWristPeriod = 2.0;
	private final static double kAutoFlipUpEjectCubePeriod = 0.5;
	private final static double kDefaultWristSpeed = 0.60;
    private final static double autoFlipUpSetPoint = TicksPerDeg * getAngleForPreset(WristPresets.FLIP_UP);

    private final static double kThrottleDeadband = 0.08;
	private boolean wristFlippedUp = false; 
	private double delay = .25; 
	private double wait = 0; 
	private double mCurrentStateStartTime;
	WristSystemState mWristSystemState = WristSystemState.DEFAULT;
	
	
	public SS_Wrist() {
		cubeCollectorSensorRight = new DigitalInput(0); 
		cubeCollectorSensorLeft = new DigitalInput(1); 
		mTalon = new LazyCANTalon(RobotMap.WRIST, 10);

		mTalon.clearStickyFaults(Constants.kTimeOutMs);
		mTalon.set(ControlMode.PercentOutput, 0);
		mTalon.setInverted(INVERTED);

		mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
        boolean sensorPresent = mTalon.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
        if (!sensorPresent) {
            DriverStation.reportError("Could not detect wrist encoder", false);
        }
		
		mTalon.setSensorPhase(ENC_INVERTED);
		mTalon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
		
		mTalon.setNeutralMode(BRAKE_MODE ? NeutralMode.Brake : NeutralMode.Coast);
		
		mTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.kTimeOutMs);
		mTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.kTimeOutMs);
		mTalon.configForwardSoftLimitThreshold(0, Constants.kTimeOutMs);
		mTalon.configForwardSoftLimitEnable(false, Constants.kTimeOutMs);
		mTalon.configReverseSoftLimitThreshold(0, Constants.kTimeOutMs);
		mTalon.configReverseSoftLimitEnable(false, Constants.kTimeOutMs);

		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, Constants.kTimeOutMs);
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Constants.kTimeOutMs);
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20, Constants.kTimeOutMs);
		//mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.kTimeOutMs);
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 20, Constants.kTimeOutMs);
		
		mTalon.configNominalOutputForward(0.0, Constants.kTimeOutMs);
		mTalon.configNominalOutputReverse(0.0, Constants.kTimeOutMs);

		mTalon.configPeakOutputForward(1.0, Constants.kTimeOutMs);
		mTalon.configPeakOutputReverse(-1.0, Constants.kTimeOutMs);

		mTalon.configOpenloopRamp(0.100, Constants.kTimeOutMs);
		mTalon.configClosedloopRamp(0, Constants.kTimeOutMs);
		
		configureTalonPositionClosedLoop();
		configureTalonPositionControlClosedLoop();
		
		// Rolling log file for wrist data.
		mCSVWriter = new ReflectingCSVWriter<WristDebugOutput>("/home/lvuser/WRIST-LOGS.csv",
        		WristDebugOutput.class);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new C_DriveWrist());
	}
	
	/*
	 * Rotates the wrist.
	 */
	public synchronized void rotate(double speed) {
		//configureTalonForDefault();
//		SmartDashboard.putNumber("Wrist Rotate", speed);
		mTalon.set(ControlMode.PercentOutput, speed);		
		wristHolding = false;
		log(Timer.getFPGATimestamp(), speed);

		if (!isHomed) {
			if (getLimitSwitch()) {
				mTalon.setSelectedSensorPosition(getQuadEncoderOffset(),  Constants.kPIDLoopIdx,  Constants.kTimeOutMs);
				isHomed = true;
			}
		}
	}
	
	public synchronized void setHomed(boolean isHomed) {
		this.isHomed = isHomed;
	}

	public synchronized boolean getHomed() {
		return isHomed;
	}

	public synchronized int getEncoder() {
		return mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
	}

	public synchronized boolean getLimitSwitch() {
		mTalon.getFaults(toFill);
		return toFill.ForwardLimitSwitch;
	}
	
	public synchronized boolean moveWristToSetPoint(double setPoint, double speedSign) {
		// read and save for isFinished
		double currentEncTicks = mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);

		// ramp down about 20 degrees from goal. Note currentEncTicks can be negative.
		double speed = Math.abs(setPoint - Math.abs(currentEncTicks)) * kp;

		// limit to maxSpeed and set direction
		speed = Math.min(speed, maxSpeed);

		// make sure it is going to move
		speed = Math.copySign(Math.max(speed, minSpeed), speedSign);

		// check if target has been passed
		boolean reachedTarget = speedSign > 0 ? currentEncTicks >= setPoint : currentEncTicks <= setPoint;
		//boolean reachedTarget = Math.abs(currentEncTicks - setPoint) < (2 * TicksPerDeg);
//		double angle = (double)currentEncTicks / TicksPerDeg;
//		System.out.println("WRIST: reachedTarget=" + reachedTarget + " angle=" + angle + " speed=" + speed + " setPoint =" + setPoint + " currentEncTicks=" + currentEncTicks);

		if (reachedTarget || getLimitSwitch()) {
			holdPosition();
		} else {
			// move wrist at new speed
			mTalon.set(ControlMode.PercentOutput, speed);

			log(Timer.getFPGATimestamp(), speed);
			wristHolding = false;
		}
		
		return reachedTarget;
	}

	/*
	 * Moves the wrist to s given position
	 * 
	 *  @param setPoint position to move arm to
	 *  @param speedSign direction, positive is moving up
	 *  @maxSpeed maximum speed expressed as a positive number
	 */
	public synchronized boolean moveWristToSetPoint(double setPoint, double speedSign, double maxSpeed) {		
		double currentEncTicks = mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
		
		double rampLength = 30 * TicksPerDeg;
		double speed;
		if (speedSign > 0) {
			// Wrist is below the set point and moving up
			double limit = setPoint - rampLength;
		
			if (limit < currentEncTicks) {
				speed = Math.max(minSpeed, (double)(setPoint - currentEncTicks) / rampLength);
			} else {
				speed = maxSpeed;
			}
		} else {
			// Wrist is above the set point and moving down
			double limit = setPoint + rampLength;

			if (limit > currentEncTicks) {
				speed = -Math.max(minSpeed, (double)(currentEncTicks - setPoint) / rampLength);
			} else {
				speed = -maxSpeed;
			}
		}
		
		// Anticipate additional movement based on velocity
		double lookAhead = mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx) / 2; // positive or negative movement
		boolean reachedTarget = speedSign > 0 ? currentEncTicks >= (setPoint - lookAhead) : currentEncTicks <= (setPoint - lookAhead);
		
		// check if target has been passed
		if (reachedTarget || getLimitSwitch()) {
			holdPosition();
		} else {
			mTalon.set(ControlMode.PercentOutput, speed);
			log(Timer.getFPGATimestamp(), speed);
			wristHolding = false;
		}
		
		return reachedTarget;
	}
	
	public synchronized int getQuadEncoderOffset() {
		if (Robot.robotName.equals("voxel")) {
			return QUAD_ENCODER_VALUE_AT_TOP_LIMIT_VOXEL;
		}
		
		return QUAD_ENCODER_VALUE_AT_TOP_LIMIT_PIXEL;
	}
	
	public synchronized double getSensorPos() {
		return (double) mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
	}

	public synchronized double getAngle() {
		return (double)mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx) / TicksPerDeg;
	}

	public static synchronized int getAngleForPreset(WristPresets preset) {
		if (presetAngles.containsKey(preset)) {
			return presetAngles.get(preset).intValue();
		}

		// this is error case
		return 120;
	}

	/*
	 * Gets an indicator whether to move wrist up or down by comparing the target position against current position.
	 * 
	 * @param targetPosition the target position to move wrist to.
	 *            
	 * @return 1.0 going up, else -1.0 if going down
	 */
	public synchronized double getSpeedSign(double targetPosition) {
		return targetPosition > mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx) ? 1.0 : -1.0;
	}
	
	/*
	 * Holds the current position using closed loop pid
	 */
	public synchronized void holdPosition() {
		double setPoint = mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
		mTalon.selectProfileSlot(kPositionHoldSlotIdx, Constants.kPIDLoopIdx);
		mTalon.set(ControlMode.Position, setPoint);		
		wristHolding = true;
		log(Timer.getFPGATimestamp(), setPoint);
	}

	public void setEncPos(int ticks) {
		mTalon.setSelectedSensorPosition(ticks,  Constants.kPIDLoopIdx,  Constants.kTimeOutMs);
	}
	
	public boolean isCubeCollected() {
		return !cubeCollectorSensorLeft.get() || !cubeCollectorSensorRight.get(); 
	}

    public void writeToLog() {
        mCSVWriter.write();
    }

	public synchronized void outputToSmartDashboard() {
		double currentPosition = (double)mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
		SmartDashboard.putNumber("Wrist Encoder Quad", currentPosition);
		SmartDashboard.putNumber("Wrist Angle", currentPosition / TicksPerDeg);
		SmartDashboard.putNumber("Wrist Amps", mTalon.getOutputCurrent());
		SmartDashboard.putNumber("Wrist Volts", mTalon.getMotorOutputVoltage());
//		SmartDashboard.putNumber("Wrist Velocity", mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
//		SmartDashboard.putString("Wrist ControlMode", mTalon.getControlMode().toString());
		SmartDashboard.putBoolean("Wrist isHomed", isHomed);
//		SmartDashboard.putBoolean("Wrist Holding", wristHolding);
//		SmartDashboard.putBoolean("Wrist Cube Sensor Left", !cubeCollectorSensorLeft.get());
//		SmartDashboard.putBoolean("Wrist Cube Sensor Right", !cubeCollectorSensorRight.get());
//		SmartDashboard.putBoolean("Auto Flip-up Activated", autoFlipUpActivated);
	}

//	private double handleDeadband(double val, double deadband) {
//        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
//    }
	
	private synchronized void handleJoyStickInput(double joyStickValue) {
//		SmartDashboard.putNumber("Right Thumb Stick", joyStickValue);
//		joyStickValue = handleDeadband(joyStickValue, kThrottleDeadband);
		
		if (Math.abs(joyStickValue) < kThrottleDeadband) {
			if (!wristHolding) {
				holdPosition();
			}
		} else {
			rotate(joyStickValue * kDefaultWristSpeed);
		}
	}

	public synchronized void onLoop(double joyStickValue) {
		double timestamp = Timer.getFPGATimestamp();
		WristSystemState newState;
		
		switch (mWristSystemState) {
			case AUTO_FLIPUP_COLLECTING:
				newState = handleAutoFlipUpCollecting();
				break;	
	
			case AUTO_FLIPUP_RASING_WRIST:
				newState = handleAutoFlipUpRaisingWrist(timestamp, mCurrentStateStartTime);
				break;
				
			case AUTO_FLIPUP_HOLDING :
				newState = handleAutoFlipUpHolding(timestamp, mCurrentStateStartTime);
				break;

//			case ZERO_WRIST:
//				newState = handleZeroWrist(timestamp, mCurrentStateStartTime);
//				break;
				
			case DEFAULT:
			default:	
				newState = handleDefault();
				break;
		}

        if (newState != mWristSystemState) {
            //System.out.println("Wrist state " + mWristSystemState + " to " + newState);
            mWristSystemState = newState;
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }
        
		if (mWristSystemState != WristSystemState.AUTO_FLIPUP_RASING_WRIST) {
			handleJoyStickInput(joyStickValue);
		}
	}

	private WristSystemState handleDefault() {
		if (autoFlipUpActivated && Robot.ss_RobotArm.isAtBottom()) {
			return WristSystemState.AUTO_FLIPUP_COLLECTING;
		}

		return WristSystemState.DEFAULT;
	}

//	private WristSystemState handleZeroWrist(double now, double startStartedAt) {
//		if (!isHomed) {
//			// rotate will set encoder position.
//			rotate(0.35);
//			return WristSystemState.ZERO_WRIST;
//		}
//
//        if (now - startStartedAt > kZeroWristPeriod ) {
//			holdPosition();
//			setEncPos(getQuadEncoderOffset());
//			isHomed = true;
//        }
//
//		return WristSystemState.DEFAULT;
//	}

	
	private WristSystemState handleAutoFlipUpCollecting() {
		wristFlippedUp = false;

		if (!autoFlipUpActivated) {
			return WristSystemState.DEFAULT;
		}
		
		if (isCubeCollected() && Robot.ss_RobotArm.isAtBottom()) {
			wait = 0;
			return WristSystemState.AUTO_FLIPUP_RASING_WRIST;
		}

		return WristSystemState.AUTO_FLIPUP_COLLECTING;
	}
	
	private WristSystemState handleAutoFlipUpRaisingWrist(double now, double startStartedAt) {
		if (!autoFlipUpActivated) {
			holdPosition();
			return WristSystemState.DEFAULT;
		}

		if (!isCubeCollected()) {
			holdPosition();
			return WristSystemState.AUTO_FLIPUP_COLLECTING;
		}

		if (wristFlippedUp) {
			return WristSystemState.AUTO_FLIPUP_HOLDING;
		}

        if (now - startStartedAt > kAutoFlipUpRaiseWristPeriod) {
			holdPosition();
        	return WristSystemState.AUTO_FLIPUP_HOLDING;
        }
		
		if (wait == 0) {
			wait = now + delay;
		} else if (wait < now) {
			double speedSign = getSpeedSign(autoFlipUpSetPoint);
			wristFlippedUp = moveWristToSetPoint(autoFlipUpSetPoint, speedSign);
			if (wristFlippedUp) {
				holdPosition();
			}
		}
		
		return WristSystemState.AUTO_FLIPUP_RASING_WRIST;
	}
	
	private WristSystemState handleAutoFlipUpHolding(double now, double startStartedAt) {
		if (!autoFlipUpActivated) {
			return WristSystemState.DEFAULT;
		}

		if (isCubeCollected()) {
	        if (now - startStartedAt > kAutoFlipUpEjectCubePeriod) {
	        	return WristSystemState.AUTO_FLIPUP_COLLECTING;
	        }
	     }

		return WristSystemState.AUTO_FLIPUP_HOLDING;
	}

	private void configureTalonPositionClosedLoop() {
		mTalon.config_kF(kPositionHoldSlotIdx, 0, Constants.kTimeOutMs);
		mTalon.config_kP(kPositionHoldSlotIdx, 0.837463, Constants.kTimeOutMs);
		mTalon.config_kI(kPositionHoldSlotIdx, 0, Constants.kTimeOutMs);
		mTalon.config_kD(kPositionHoldSlotIdx, 80.324387, Constants.kTimeOutMs);
		mTalon.config_IntegralZone(kPositionHoldSlotIdx, 30, Constants.kTimeOutMs);
	}

	private void configureTalonPositionControlClosedLoop() {
		mTalon.config_kF(kPositionControlSlotIdx, 0, Constants.kTimeOutMs);
		mTalon.config_kP(kPositionControlSlotIdx, 0.837463, Constants.kTimeOutMs);
		mTalon.config_kI(kPositionControlSlotIdx, 0, Constants.kTimeOutMs);
		mTalon.config_kD(kPositionControlSlotIdx, 80.324387, Constants.kTimeOutMs);
		mTalon.config_IntegralZone(kPositionHoldSlotIdx, 30, Constants.kTimeOutMs);
		//mTalon.configAllowableClosedloopError(kPositionHoldSlotIdx, (int)TicksPerDeg, Constants.kTimeOutMs);
	}

	private synchronized void log(double timestamp, double setPoint) {
		mDebug.timestamp = timestamp;
		mDebug.setPoint = setPoint;
		mDebug.controlMode = mTalon.getControlMode();
		mDebug.current = mTalon.getOutputCurrent();
		mDebug.voltage = mTalon.getMotorOutputVoltage();
		mDebug.velocity = mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
		mDebug.sensorvalue = mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
		mDebug.angle = mDebug.sensorvalue / TicksPerDeg;
		mCSVWriter.add(mDebug);
	}

	public synchronized void setWristHolding(boolean isHolding) {
		wristHolding = isHolding;
	}

	public synchronized void toggleAutoFlipUpMode() {
		autoFlipUpActivated = !autoFlipUpActivated;
	}
}