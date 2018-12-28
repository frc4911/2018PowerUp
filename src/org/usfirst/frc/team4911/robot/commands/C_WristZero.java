package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_WristZero extends Command {
	
	private final static double speed = 0.35;
	private final static double TIMEOUT = 2.0;
	private double endTime;
	
    public C_WristZero() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.ss_Wrist);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	endTime = Timer.getFPGATimestamp() + TIMEOUT;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println("WristZero isHomed = " + Robot.ss_Wrist.getHomed());
    	if (!Robot.ss_Wrist.getHomed()) {
			Robot.ss_Wrist.rotate(speed);
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

		double now = Timer.getFPGATimestamp();
		boolean timedOut = now > endTime;
//		if (Robot.ss_Wrist.getHomed()) {
//			System.out.println("WristZero elapsed time = " + (now - (endTime - TIMEOUT)) + ", homed=" + Robot.ss_Wrist.getHomed());
//		} else if (timedOut) {
//			System.out.println("WristZero elapsed time = TIMED OUT after " + TIMEOUT + ", homed=" + Robot.ss_Wrist.getHomed());
//		}
		
    	if (Robot.ss_Wrist.getHomed() || timedOut) {
    		return true;
		}

    	return false;

//		boolean isFinished = Robot.ss_Wrist.getLimitSwitch();
//    	System.out.println("WristZero isFinished=" + isFinished);
//    	return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
		// If timed out, assume wrist has moved up as far as it goes. Perhaps limit switch is broken.
    	if (!Robot.ss_Wrist.getHomed()) {
        	//System.out.println("WristZero in end homing.");
			Robot.ss_Wrist.setEncPos(Robot.ss_Wrist.getQuadEncoderOffset());
			Robot.ss_Wrist.setHomed(true);
			Robot.ss_Wrist.holdPosition();
		}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}