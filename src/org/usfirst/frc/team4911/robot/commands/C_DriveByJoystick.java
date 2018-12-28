package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_DriveByJoystick extends Command {

	//TODO: comment variables
	final double nudge = 0.01;
	// start adjust = 0.5
	final double startAdjust = 0;
	
	int arrayListLength = 5;
	int lastEncoderValue = 0;
	int speedListIndex = 0;
	int driveState = 0;
	int index = 0;
	int tipCounter = 0;
	
	double speedList[] = new double[arrayListLength];
	double lastTimeStamp = 0;
	double adjust = 0;
	
	float maxRoll = 0;

	//still working on these
	double lastLeftY = 0;
	double lastRightY = 0;
	double lastLeftX = 0;
	double yAccelerationUp = 1; //0.05;
	double yAccelerationDn = 1; //0.06;
	double xAcceleration =   1; //0.03;
	
	public C_DriveByJoystick() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.ss_DriveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		for (int i = 0; i < arrayListLength; i++) {
			speedList[i] = 0;
		}
		index = 0;
		tipCounter = 0;
		maxRoll = -10000; // very small number
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double rightY = Robot.m_oi.stickRight.getY();
		rightY = limitAcceleration(rightY, lastRightY, yAccelerationDn, yAccelerationUp);
		lastRightY = rightY;
		
		switch (Robot.driveStyle) {
		case 0:
			double leftX = Robot.m_oi.stickLeft.getX();
			leftX = limitAcceleration(leftX, lastLeftX, xAcceleration, xAcceleration);
			lastLeftX = leftX;
			
			Robot.ss_DriveTrain.arcadeDrive(-rightY, leftX, false);
			break;
		case 1:
			double leftY = Robot.m_oi.stickLeft.getY();
			leftY = limitAcceleration(leftY, lastLeftY, yAccelerationDn, yAccelerationUp);
			lastLeftY = leftY;
			
			Robot.ss_DriveTrain.tankDrive(-leftY, -rightY, false);
			break;
		default:
			//System.out.println("EROORRRRRRRRRRRRRRRRRRR!!!!!!!!!!!");
			break;
		}
		
	}
	
	private double limitAcceleration(double newValue, double lastValue, double accDn, double accUp)
	{
		if(Robot.ss_RobotArm.getAngle() > 20)
		{
			if(newValue > lastValue + accUp)
			{
				newValue = lastValue + accUp;
			}
			else if (newValue < lastValue - accDn)
			{
				newValue = lastValue - accDn;
			}
		}
		
		return newValue;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.ss_DriveTrain.stopDriving();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
