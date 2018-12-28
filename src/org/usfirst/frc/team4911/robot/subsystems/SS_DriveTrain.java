package org.usfirst.frc.team4911.robot.subsystems;

import org.usfirst.frc.team4911.robot.RobotMap;
import org.usfirst.frc.team4911.robot.commands.C_DriveByJoystick;

import cyberknightsLib.DefaultMotorSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_DriveTrain extends Subsystem {
	
	public final boolean SQUARED_INPUTS = true;
	
	// driveTrainLeft
	private static final boolean LEFT_BRAKE_MODE   = true;
	private static final boolean LEFT_INVERTED     = true;
	private static final boolean LEFT_ENC_INVERTED = false;
	private static final String LEFT_NAME 		   = "driveTrainLeft";

	public final DefaultMotorSRX dm_DriveTrainLeft = new DefaultMotorSRX(RobotMap.DRIVE_TRAIN_FRONT_LEFT,
			RobotMap.DRIVE_TRAIN_REAR_LEFT, LEFT_INVERTED, LEFT_NAME);

	// driveTrainRight
	private static final boolean RIGHT_BRAKE_MODE   = true;
	private static final boolean RIGHT_INVERTED     = false; // was true
	private static final boolean RIGHT_ENC_INVERTED = false; // was true
	private static final String RIGHT_NAME          = "driveTrainRight";

	public final DefaultMotorSRX dm_DriveTrainRight = new DefaultMotorSRX(RobotMap.DRIVE_TRAIN_FRONT_RIGHT,
			RobotMap.DRIVE_TRAIN_REAR_RIGHT, RIGHT_INVERTED, RIGHT_NAME);

	//shifters
	private final DoubleSolenoid solGearShift = new DoubleSolenoid(RobotMap.SHIFTER_LEFT, RobotMap.SHIFTER_RIGHT);
	
	private boolean inHighGear = false;

	public void initDefaultCommand() {
		// driveTrainLeft
		dm_DriveTrainLeft.setBrakeMode(LEFT_BRAKE_MODE);
		dm_DriveTrainLeft.configQuadEnc(LEFT_ENC_INVERTED);

		// driveTrainRight
		dm_DriveTrainRight.setBrakeMode(RIGHT_BRAKE_MODE);
		dm_DriveTrainRight.configQuadEnc(RIGHT_ENC_INVERTED);
		
		// set current motor
//		dm_DriveTrainRight.getLeaderTalon().configPeakCurrentLimit(60, 10);
//		dm_DriveTrainRight.getLeaderTalon().configPeakCurrentDuration(200, 10);
//		dm_DriveTrainRight.getLeaderTalon().configContinuousCurrentLimit(60, 10);
//		dm_DriveTrainRight.enableCurrentLimit(true);
//		
//		dm_DriveTrainLeft.getLeaderTalon().configPeakCurrentLimit(60, 10);
//		dm_DriveTrainLeft.getLeaderTalon().configPeakCurrentDuration(200, 10);
//		dm_DriveTrainLeft.getLeaderTalon().configContinuousCurrentLimit(60, 10);
//		dm_DriveTrainLeft.enableCurrentLimit(true);

		// Set the default command for a subsystem here.
		setDefaultCommand(new C_DriveByJoystick());
	}
	
	protected double limit(double value) {
		if (value > 1.0) {
			return 1.0;
		}
		if (value < -1.0) {
			return -1.0;
		}
		return value;
	}

	protected double applyDeadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	double m_deadband = 0;
	double m_maxOutput = 1;
	public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
		xSpeed = limit(xSpeed);
		xSpeed = applyDeadband(xSpeed, m_deadband);

		zRotation = limit(zRotation);
		zRotation = applyDeadband(zRotation, m_deadband);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squaredInputs) {
			xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
			zRotation = Math.copySign(zRotation * zRotation, zRotation);
		}

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

		if (xSpeed >= 0.0) {
			// First quadrant, else second quadrant
			if (zRotation >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = xSpeed - zRotation;
			} else {
				leftMotorOutput = xSpeed + zRotation;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (zRotation >= 0.0) {
				leftMotorOutput = xSpeed + zRotation;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = xSpeed - zRotation;
			}
		}
		
		dm_DriveTrainLeft.setSpeed(limit(leftMotorOutput) * m_maxOutput);
		dm_DriveTrainRight.setSpeed(limit(rightMotorOutput) * m_maxOutput);
	}

	public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
		leftSpeed = limit(leftSpeed);
		leftSpeed = applyDeadband(leftSpeed, m_deadband);

		rightSpeed = limit(rightSpeed);
		rightSpeed = applyDeadband(rightSpeed, m_deadband);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squaredInputs) {
			leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);
			rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
		}

		dm_DriveTrainLeft.setSpeed(leftSpeed * m_maxOutput);
		dm_DriveTrainRight.setSpeed(rightSpeed * m_maxOutput);
	}
	
	public void stopDriving() {
		dm_DriveTrainLeft.setSpeed(0);
		dm_DriveTrainRight.setSpeed(0);
	}
	
	public boolean isInHighGear() {
		return inHighGear;
	}

	public void Shift(boolean shiftUp) {
		if (shiftUp) {
			solGearShift.set(DoubleSolenoid.Value.kReverse);
			SmartDashboard.putString("Gear", "High");
			inHighGear = true;
		} else {
			solGearShift.set(DoubleSolenoid.Value.kForward);
			SmartDashboard.putString("Gear", "Low");
			inHighGear = false;
		}
	}

	public void encoderValues() {
		int encoderLValue = dm_DriveTrainLeft.getSensorPos();
		int encoderRValue = dm_DriveTrainRight.getSensorPos();
//		SmartDashboard.putNumber("Encoder Right Value", encoderRValue);
//		SmartDashboard.putNumber("Encoder Left Value", encoderLValue);
	}

	public void encoderResetValues() {
		dm_DriveTrainLeft.zeroEncPos();
		dm_DriveTrainRight.zeroEncPos();
	}
	
	//PowerDistributionPanel pdp = new PowerDistributionPanel(); 

	int ccc = 0;
	public void outputToSmartDashBoard() {
		if (++ccc > 4)
			ccc = 0;
		
		switch (ccc) {
			case 0:
				SmartDashboard.putNumber("LF Drive A", dm_DriveTrainLeft.getLeaderTalon().getOutputCurrent());
			break;
			
			case 1:
				SmartDashboard.putNumber("LR Drive A", dm_DriveTrainLeft.getFollowerTalon0().getOutputCurrent());
			break;
			
			case 2:
				SmartDashboard.putNumber("RF Drive A", dm_DriveTrainLeft.getLeaderTalon().getOutputCurrent());
			break;
			
			case 3:
				SmartDashboard.putNumber("RR Drive A", dm_DriveTrainLeft.getFollowerTalon0().getOutputCurrent());
			break;
			
			case 4:
				//SmartDashboard.putNumber("pdp", pdp.getTotalCurrent());
			break;
			
		}		
	}
}
