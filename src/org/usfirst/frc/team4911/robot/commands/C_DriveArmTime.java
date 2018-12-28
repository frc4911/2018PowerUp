package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_DriveArmTime extends Command {
	
	double speed = 0;
	double time = 0;
	double endTime = 0;

    public C_DriveArmTime(double speed, double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.speed = speed;
    	this.time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	endTime = Timer.getFPGATimestamp() + time;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.ss_RobotArm.actuate(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ss_RobotArm.actuate(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
