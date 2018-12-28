package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;
import org.usfirst.frc.team4911.robot.subsystems.SS_Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_WristPresets extends Command {

	public static final double kWristCommandTimeoutPeriod = 2.00;
	
	private double targetEncTicks = 0.0;
	private double timeoutPeriod = kWristCommandTimeoutPeriod;
	private double speedSign = 1.0; // 1 or -1
	private double endTime = 0;
	private boolean reachedTarget;
	
	public C_WristPresets(double angle, double timeoutPeriod) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.ss_Wrist);
		
		targetEncTicks = angle * SS_Wrist.TicksPerDeg;
		this.timeoutPeriod = timeoutPeriod;
		reachedTarget = false;
	}

	public C_WristPresets(double angle) {
		this(angle, kWristCommandTimeoutPeriod);
	}
	
	public C_WristPresets(WristPresets preset) {
		this(SS_Wrist.getAngleForPreset(preset), kWristCommandTimeoutPeriod);
	}

	public C_WristPresets(WristPresets preset, double timeoutPeriod) {
		this(SS_Wrist.getAngleForPreset(preset), timeoutPeriod);
	}
	
	// Called just before this Command runs the first time
	protected void initialize() {
		//System.out.println("C_WristPresets::initialize()");
		//1.0 going up, -1.0 going down
		speedSign = Robot.ss_Wrist.getSpeedSign(targetEncTicks);
		
		// set timeout just in case
		endTime = Timer.getFPGATimestamp() + timeoutPeriod;		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
//		reachedTarget = Robot.ss_Wrist.moveWristToSetPoint(targetEncTicks, speedSign);
		reachedTarget = Robot.ss_Wrist.moveWristToSetPoint(targetEncTicks, speedSign, 0.8);
		// Use Position PID in Talon
//		reachedTarget = Robot.ss_Wrist.moveWristToSetPoint(targetEncTicks);
		if (reachedTarget) {
			Robot.ss_Wrist.holdPosition();
//			System.out.println("C_WristPresets(), reachedTarget=" +  reachedTarget + " targetEncTicks=" + targetEncTicks);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// check timeout
		double now = Timer.getFPGATimestamp();
		boolean timedOut = now > endTime;
		if (reachedTarget) {
			//System.out.println("Wrist targetEncTicks " + targetEncTicks + " elapsed time = " + (now - (endTime - timeoutPeriod)) + ", homed=" + Robot.ss_Wrist.getHomed());
		} else if (timedOut) {
			//System.out.println("Wrist targetEncTicks " + targetEncTicks + " elapsed time = TIMED OUT after " + timeoutPeriod + ", homed=" + Robot.ss_Wrist.getHomed());
		}
	
		return reachedTarget || timedOut || !Robot.ss_Wrist.getHomed();
	}

	// Called once after isFinished returns true
	protected void end() {
		// stop
		Robot.ss_Wrist.rotate(0.0);
		// PID to hold at current position
		Robot.ss_Wrist.holdPosition();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
