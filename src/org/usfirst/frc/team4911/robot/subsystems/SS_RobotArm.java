package org.usfirst.frc.team4911.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.util.ReflectingCSVWriter;
import lib.util.drivers.Dart;
import lib.util.drivers.DartFactory;
import lib.util.drivers.LazyCANTalon;

import org.usfirst.frc.team4911.robot.Constants;
import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap;
import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.commands.C_MoveArm;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The arm subsystem consists of a linear actuator, DART. It's position is measured by a potentiometer. 
 * Hall effect limit switches are wired directly into a Talon SRX motor controller.
 */
public class SS_RobotArm extends Subsystem {

	private static final double kMaxSpeedForward = 1.0;
	private static final double kMaxSpeedReverse = -1.0;
   
    public static class ArmDebugOutput {
        public double timestamp;
        public double setPoint;
        public double voltage;
        public double current;
        public double velocity;
        public int sensorvalue;
    }

    private final ReflectingCSVWriter<ArmDebugOutput> mCSVWriter;
    private ArmDebugOutput mDebug = new ArmDebugOutput();
	
//	private static final double kTimeout = 2.1;

	// This ramps done using cosine.
	private double[] cosineRamp = new double[] { 
			//0.987688341, 
			0.951056516, 
			0.891006524, 
			0.809016994, 
			0.707106781, 
			0.587785252,
			0.453990500, 
			0.309016994, 
			0.309016994, 
			0.156434465,
			// 0.309016994, 
			0.086434465,
			0.0,
			0.0 };

	private final WPI_TalonSRX mTalon;

	private int targetPosition = 0;
	private int lastPosition = 0;

	private Dart dart;
	private int startRampingWhenMovingUpPosition;
	private int startRampingWhenMovingDownPosition;

	private int rampLimit;
	private double lastSpeed = 0;

	double rampLength = 125.0;
			
	/**
	 * Creates new instance of a robot arm object.
	 */
	public SS_RobotArm() {
		// mTalonLeft = CANTalonFactory.createDefaultTalon(RobotMap.ARM);
		dart = DartFactory.getDart(Robot.robotName);
		mTalon = new LazyCANTalon(RobotMap.ARM, 10);
		mTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
		mTalon.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0x00, 0x00, Constants.kTimeOutMs);
		mTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.kTimeOutMs);
		mTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.kTimeOutMs);

		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, Constants.kTimeOutMs);
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, Constants.kTimeOutMs);
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeOutMs);

		// Get analog input sensor reading faster. Default is 160ms.
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10, Constants.kTimeOutMs);
		configureTalon();
		mTalon.setNeutralMode(NeutralMode.Brake);
		mTalon.set(ControlMode.PercentOutput, 0);

		mTalon.configOpenloopRamp(0.160, Constants.kTimeOutMs);
		mTalon.configClosedloopRamp(0.160, Constants.kTimeOutMs);
		
		mTalon.configForwardSoftLimitThreshold(dart.getTop(), Constants.kTimeOutMs);
		mTalon.configForwardSoftLimitEnable(true, Constants.kTimeOutMs);
		mTalon.configReverseSoftLimitThreshold(dart.getBottom(), Constants.kTimeOutMs);
		mTalon.configReverseSoftLimitEnable(true, Constants.kTimeOutMs);
//		mTalon.configForwardSoftLimitThreshold(0, Constants.kTimeOutMs);
//		mTalon.configForwardSoftLimitEnable(false, Constants.kTimeOutMs);
//		mTalon.configReverseSoftLimitThreshold(0, Constants.kTimeOutMs);
//		mTalon.configReverseSoftLimitEnable(false, Constants.kTimeOutMs);

		// Testing 1:1 DART so limit current  draw
		mTalon.configContinuousCurrentLimit(30, Constants.kTimeOutMs);
		mTalon.configPeakCurrentLimit(50, Constants.kTimeOutMs);
		mTalon.configPeakCurrentDuration(1000, Constants.kTimeOutMs);
		mTalon.enableCurrentLimit(true);
		
		// Rolling log file for arm data.
		mCSVWriter = new ReflectingCSVWriter<ArmDebugOutput>("/home/lvuser/ARM-LOGS.csv", ArmDebugOutput.class);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new C_MoveArm());
	}

	/**
	 * Moves arm up or down using speed as a percent range [1.0, -1.0]
	 * 
	 * @param speed
	 */
	public synchronized void actuate(double speed) {
		if (mTalon.getControlMode() != ControlMode.PercentOutput) {
			configureTalon();
			// Default somewhere safe
			targetPosition = Dart.getAngleForPreset(ArmPresets.COLLECT);
		}
		setDefaultRampPositions();
		
		lastPosition = mTalon.getSensorCollection().getAnalogInRaw();
		lastSpeed = applyRamp(lastPosition, speed, cosineRamp);
		
//		SmartDashboard.putNumber("ARM Joystick", lastSpeed);
//		if (Math.abs(lastSpeed) > 0.05) {
//			System.out.println("ARM: speed=" + speed + " Adj Speed=" + lastSpeed + " position=" + lastPosition + " timestamp=" + Timer.getFPGATimestamp());
//		}
		
		mTalon.set(ControlMode.PercentOutput, lastSpeed);
		log(Timer.getFPGATimestamp(), lastSpeed);
	}

	public synchronized void actuate(double speed, int position) {
		if (mTalon.getControlMode() != ControlMode.PercentOutput) {
			configureTalon();
			// Default somewhere safe
			targetPosition = Dart.getAngleForPreset(ArmPresets.COLLECT);
		}
		setRampPositions(position);

		lastPosition = mTalon.getSensorCollection().getAnalogInRaw();
		lastSpeed = applyRamp(lastPosition, speed, cosineRamp);
		
//		SmartDashboard.putNumber("ARM Joystick", lastSpeed);
//		if (Math.abs(lastSpeed) > 0.04) {
//			System.out.println("ARM: speed=" + speed + " Adj Speed=" + lastSpeed + " position=" + lastPosition + " timestamp=" + Timer.getFPGATimestamp());
//		}

		mTalon.set(lastSpeed);
		log(Timer.getFPGATimestamp(), lastSpeed);
	}

	
	private double applyRamp(int currentPosition, double speed, double ramp[]) {
		// there's jitter in the joystick
		if (-0.025 <= speed && speed <= 0.025) {
			return 0.0;
		}

		// Apply ramp function
		// If speed is above threshold, adjust it
		int index = 0;

		double percentOnRamp = 0.01;
		int offset = 0;
		if ((speed > 0.05) && (currentPosition > startRampingWhenMovingUpPosition)) {
			offset = currentPosition - startRampingWhenMovingUpPosition;
		} else if ((speed < -0.05) && (startRampingWhenMovingDownPosition > currentPosition)) {
			offset = startRampingWhenMovingDownPosition - currentPosition;
		} else {
			return speed;
		}

		percentOnRamp = 100.0 * (offset) / rampLimit;
		index = (int) Math.ceil(percentOnRamp) / ramp.length;

		index = Math.max(0, Math.min(ramp.length-1, index));

		double newSpeed = speed;
		
		// only apply ramp if driving too fast
		if ((speed > 0) && (speed > ramp[index])) {
			newSpeed = ramp[index];
		} else if ((speed < 0) && (-speed > ramp[index])) {
			newSpeed = -ramp[index];
		}
		
		return newSpeed;
	}
	
	private synchronized void log(double timestamp, double setPoint) {
		mDebug.timestamp = timestamp;
		mDebug.setPoint = setPoint;
		mDebug.current = mTalon.getOutputCurrent();
		mDebug.voltage = mTalon.getMotorOutputVoltage();
		mDebug.velocity = mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
		mDebug.sensorvalue = mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx);
		mCSVWriter.add(mDebug);
	}
	
    public void writeToLog() {
        mCSVWriter.write();
    }
	
	/**
	 * Gets the angle for a given preset
	 * 
	 * @param preset the preset to lookup
	 * @return the angle corresponding to the preset
	 */
	public synchronized int getAngleForPreset(ArmPresets preset) {
		// Encapsulate behavior
		return Dart.getAngleForPreset(preset);
	}

	/**
	 * Moves the arm to a given angle
	 * 
	 * @param angle the angle to position arm at
	 * @param velocity the cruise velocity for Motion Magic
	 * @return true if at position, otherwise false
	 */
	public synchronized boolean moveToAngle(double angle) {
		// Valid angles & velocity?
		SmartDashboard.putNumber("Arm Set Angle", angle);

		if (angle < -52 || angle > 67) {
			//System.out.println("Invalid angle = " + angle);
			return true;
		}

		// Are we in the correct mode
		if (this.mTalon.getControlMode() != ControlMode.MotionMagic) {
			// Use MotionMagic here
			configureTalonForMotionMagicControl(dart.getVcruise());
			targetPosition = (int) dart.getPositionForAngle(angle);
			mTalon.set(ControlMode.MotionMagic, targetPosition);
			SmartDashboard.putNumber("Arm Set Position", targetPosition);
		}

		// Are we there yet?
		int currentPosition = mTalon.getSensorCollection().getAnalogInRaw();
		int error = Math.abs(targetPosition - currentPosition);
		boolean done = error <= 5;
		lastPosition = currentPosition;

//		SmartDashboard.putNumber("ARM MovetoAngle Error", error);
		return done;
	}

	/**
	 * Moves the arm to a given preset
	 * 
	 * @param preset  the preset to position arm at
	 * @param isMovingUp direction to move arm in (only used when moving to switch)
	 * @return true if at position, otherwise false
	 */
	public synchronized boolean moveToPreset(ArmPresets preset, boolean isMovingUp) {
		if (mTalon.getControlMode() != ControlMode.PercentOutput) {
			configureTalon();
		}
		
		if (Robot.CancelCommandButtonPressed) {
			mTalon.set(ControlMode.PercentOutput, 0.0);
			return true;
		}

		int currentPosition = mTalon.getSensorCollection().getAnalogInRaw();
		boolean reachedTarget = true;
		switch (preset) {
			case COLLECT:
				reachedTarget = handleMoveToCollect(currentPosition);
				break;
			case SCALE:
				reachedTarget = handleMoveToScale(currentPosition);
				break;
			case SWITCH:
				reachedTarget = handleMoveToSwitch(currentPosition, isMovingUp);
				break;
			default:
				//System.out.println("moveToPreset() received unknown ArmPreset=" + preset);
				break;
		}
		
		if (reachedTarget || Robot.CancelCommandButtonPressed) {
			mTalon.set(ControlMode.PercentOutput, 0.0);
			reachedTarget = true;
		}

		return reachedTarget;
	}
	
	private boolean handleMoveToSwitch(int currentPosition, boolean isMovingUp) {
		final int switchPoistion = dart.getSwitch();
		boolean done = false;

		if (Math.abs(mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx)) > 0.10) {
			done = isAtSwitch();
		} else {
			done = isMovingUp ? currentPosition > switchPoistion : currentPosition < switchPoistion;
		}
		
		if (!done) {
			double speed = 0.0;
			if (isMovingUp) {
				int limit = switchPoistion - 90;
				
				if (limit < currentPosition) {
					speed = Math.max(0.15, (double)(switchPoistion - currentPosition) / 90.0);
				} else {
					speed = kMaxSpeedForward;
				}
			} else {
				int limit = switchPoistion + 200;

				if (limit > currentPosition) {
					speed = -Math.max(0.20, (double)(currentPosition - switchPoistion) / 200.0);
				} else {
					speed = kMaxSpeedReverse;
				}
			}

//			System.out.println("ARM: handleMoveToSwitch() speed=" + speed + " currentPosition=" + currentPosition + " timestamp=" + Timer.getFPGATimestamp());
			mTalon.set(ControlMode.PercentOutput, speed);
		}

		return done;
	}

	private boolean handleMoveToScale(int currentPosition) {
		double speed = 0.0;
		boolean done = isAtTop();
		if (!done) {
			int limit = dart.getTop() - 135;
			if (currentPosition > limit) {
				speed = Math.max(0.17, (double)(dart.getTop() - currentPosition) / 135.0);
			} else {
				speed = kMaxSpeedForward;
			}

//			System.out.println("ARM: handleMoveToScale() speed=" + speed + " currentPosition=" + currentPosition + " timestamp=" + Timer.getFPGATimestamp());
			mTalon.set(ControlMode.PercentOutput, speed);
		}

		return done;
	}

	private boolean handleMoveToCollect(int currentPosition) {
		double speed;
		int velocity = Math.abs(mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		
		boolean done = isAtBottom();
		if (!done) {

			if (velocity > 70 && rampLength == 125.0) {
				rampLength = 200;
			}
			
			int limit = dart.getBottom() + (int)rampLength ;
			if (currentPosition < limit) {
				speed = -Math.max(0.20, (double)(currentPosition - dart.getBottom()) / rampLength);
			} else {
				speed = kMaxSpeedReverse;
			}
			
//			System.out.println("ARM: handleMoveToCollect() speed=" + speed + " currentPosition=" + currentPosition + " velocity= " + velocity+ " timestamp=" + Timer.getFPGATimestamp());
			mTalon.set(ControlMode.PercentOutput, speed);
		}

		if (done) {
			rampLength = 125.0;
		}
		
		return done;
	}


	public synchronized int getPositionForAngle(double angle) {
		return (int) dart.getPositionForAngle(angle);
	}

	public synchronized void setDart(Dart dart) {
		this.dart = dart;
		setDefaultRampPositions();
	}

	public synchronized Dart getDart() {
		return dart;
	}

	private void setDefaultRampPositions() {
		//rampLimit = (int) (2.3 * dart.getTicksPerInch());  // FOR 2:1 Ratio
		rampLimit = (int) (3.75 * dart.getTicksPerInch());	 // FOR 1:1 Ratio
		startRampingWhenMovingDownPosition = dart.getBottom() + rampLimit;
		startRampingWhenMovingUpPosition = dart.getTop() - rampLimit;
	}

	private void setRampPositions(int position) {
		rampLimit = (int) (1.75 * dart.getTicksPerInch());
		startRampingWhenMovingDownPosition = position + rampLimit;
		startRampingWhenMovingUpPosition = Math.max(dart.getBottom(), position - rampLimit);
	}

	public synchronized boolean isAtBottom() {
		int position = mTalon.getSensorCollection().getAnalogInRaw();
		if (position <= dart.getBottom()) {
			return true;
		}
		
		if ((position - dart.getBottom()) < 13) {
			return true;
		}
		
		return false;
	}

	public synchronized boolean isAtTop() {
		int position = mTalon.getSensorCollection().getAnalogInRaw();
		if (position >= dart.getTop()) {
			return true;
		}

		if ((dart.getTop() - position) < 13) {
			return true;
		}
		
		return false;
	}

	public synchronized boolean isAtSwitch() {
		return Math.abs(mTalon.getSensorCollection().getAnalogInRaw() - dart.getSwitch()) < 13;
	}

	 public synchronized double getLastSpeed() {
		 return lastSpeed;
	 }

	public synchronized int getPosition() {
		return mTalon.getSensorCollection().getAnalogInRaw();
	}
	
	public synchronized int getVelocity() { 
		return mTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx);
	}
	public synchronized double getAngle() {
		return dart.getAngleforPosition(mTalon.getSensorCollection().getAnalogInRaw());
	}

	/**
	 * Gets a preset position for the current arm position.
	 * 
	 * @return preset for given arm position or unknown.
	 */
	public synchronized ArmPresets getLastArmPreset() {
		if (isAtTop()) {
			return ArmPresets.SCALE;
		}

		if (isAtSwitch()) {
			return ArmPresets.SWITCH;
		}

		if (isAtBottom()) {
			return ArmPresets.COLLECT;
		}

		return ArmPresets.UKNOWN;
	}

	private synchronized void configureTalon() {
		mTalon.set(ControlMode.PercentOutput, 0);

		mTalon.configNominalOutputForward(0.0, Constants.kTimeOutMs);
		mTalon.configNominalOutputReverse(0.0, Constants.kTimeOutMs);

		mTalon.configPeakOutputForward(1.0, Constants.kTimeOutMs);
		mTalon.configPeakOutputReverse(-1.0, Constants.kTimeOutMs);

		mTalon.setIntegralAccumulator(0.0, Constants.kPIDLoopIdx, Constants.kTimeOutMs);
		mTalon.clearMotionProfileHasUnderrun(Constants.kTimeOutMs);
		mTalon.clearMotionProfileTrajectories();
		mTalon.clearStickyFaults(Constants.kTimeOutMs);

	}

	private synchronized void configureTalonForMotionMagicControl(int vcruise) {
		if (this.mTalon.getControlMode() != ControlMode.MotionMagic) {

			// Set relevant frame periods to be at least as fast as periodic rate
			mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.kTimeOutMs);
			mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, Constants.kTimeOutMs);

			mTalon.configNominalOutputForward(0.0, Constants.kTimeOutMs);
			mTalon.configNominalOutputReverse(0.0, Constants.kTimeOutMs);

			// TODO: Consider limiting forward peak to compensate for pneumatic lifts!
			mTalon.configPeakOutputForward(1.0, Constants.kTimeOutMs);
			mTalon.configPeakOutputReverse(-1.0, Constants.kTimeOutMs);

			// Set closed loop gains in slot 1
			mTalon.selectProfileSlot(Constants.kMotionMagicSlotIdx, Constants.kPIDLoopIdx);

			double feedForward = (1023 * this.dart.getkFAdjustment()) / vcruise;

			mTalon.config_kF(Constants.kMotionMagicSlotIdx, feedForward, Constants.kTimeOutMs);
			mTalon.config_kP(Constants.kMotionMagicSlotIdx, this.dart.getkP(), Constants.kTimeOutMs);
			mTalon.config_kI(Constants.kMotionMagicSlotIdx, this.dart.getkI(), Constants.kTimeOutMs);
			mTalon.config_kD(Constants.kMotionMagicSlotIdx, this.dart.getkD(), Constants.kTimeOutMs);
			mTalon.config_IntegralZone(Constants.kMotionMagicSlotIdx, this.dart.getkIntegralZone(), Constants.kTimeOutMs);

			// Set acceleration and vcruise velocity
			mTalon.configMotionCruiseVelocity(vcruise, Constants.kTimeOutMs);
			mTalon.configMotionAcceleration((int) (vcruise / 2), Constants.kTimeOutMs);
		}
	}

	public synchronized void outputToSmartDashboard() {
		double motorOutputPercent = mTalon.getMotorOutputPercent();

//		SmartDashboard.putString("ARM ControlMode", mTalon.getControlMode().toString());
//		SmartDashboard.putNumber("ARM SensorPosition", mTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx));
//		SmartDashboard.putNumber("ARM SensorVelocity", selectedSensorVelocity);
		SmartDashboard.putNumber("ARM MotorOutputPercent", motorOutputPercent);
		SmartDashboard.putNumber("ARM MotorOutputVoltage", mTalon.getMotorOutputVoltage());
		SmartDashboard.putNumber("ARM OutputCurrent", mTalon.getOutputCurrent());
		// SmartDashboard.putNumber("ARM AnalogIn", mTalon.getSensorCollection().getAnalogIn());
		int inRaw = mTalon.getSensorCollection().getAnalogInRaw();
		SmartDashboard.putNumber("ARM AnalogInRaw", inRaw);

		double angle = this.dart.getAngleforPosition(inRaw);
		SmartDashboard.putNumber("ARM Angle", angle);
	}
}
