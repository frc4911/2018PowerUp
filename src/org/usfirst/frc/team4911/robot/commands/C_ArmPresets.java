package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_ArmPresets extends Command {
	public static final double kArmCommandTimeout = 4.0;
	
	protected boolean done = false;
	private double endTime;
	private double timeoutPeriod = kArmCommandTimeout;
	private boolean isMovingUp = false;
	private ArmPresets preset;
	
	public C_ArmPresets(ArmPresets preset) {
		this(preset, kArmCommandTimeout);
	}

	public C_ArmPresets(ArmPresets preset, double timeoutPeriod) {
		requires(Robot.ss_RobotArm);
		this.timeoutPeriod = timeoutPeriod;
		this.preset = preset;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		switch (preset) {
			case SCALE:
				isMovingUp = true;
				break;
			case SWITCH:
				isMovingUp = Robot.ss_RobotArm.getDart().getSwitch() > Robot.ss_RobotArm.getPosition();
				break;
			case COLLECT:
			default:
				isMovingUp = false;
		}

		endTime = Timer.getFPGATimestamp() + timeoutPeriod;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!Robot.CancelCommandButtonPressed) {
			done = Robot.ss_RobotArm.moveToPreset(preset, isMovingUp);
		}
		
		if (done || Robot.CancelCommandButtonPressed) {
			Robot.ss_RobotArm.actuate(0.0);
			done = true;
		}

		//System.out.println("setPoint=" + setPoint + " currentPosition=" + currentPosition + " done= " + done);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// check timeout
		double now = Timer.getFPGATimestamp();
		boolean timedOut = now > endTime;
//		if (done) {
//			System.out.println("Arm " + preset.toString() + " elapsed time = " + (now - (endTime - timeoutPeriod)));
//		} else if (timedOut) {
//			System.out.println("Arm" + preset.toString() + " elapsed time = TIMED OUT after " + timeoutPeriod);
//		}
		
		return done || timedOut || Robot.CancelCommandButtonPressed;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.ss_RobotArm.actuate(0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		this.end();
	}
}