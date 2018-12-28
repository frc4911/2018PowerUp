package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class C_CollectorDrive extends Command {
	
	final double OUTPUT_SPEED = 0.3;
	
	boolean intake;
	double time = 0;
	double speed = 0;
	double endTime = 0;
	
	boolean joystickSpeed;
	
    public C_CollectorDrive(boolean intake) {
        // Use requires() here to declare subsystem dependencies
    	this(intake, -1.0, Double.MAX_VALUE);
    }
    
    public C_CollectorDrive(boolean intake, double time, double speed) {
    	this.intake = intake;

    	if (time == -1)
    	{
    		time = 10000; // -1 means run forever
    	}	
    	this.time = time;
    	
    	this.speed = speed;
    	joystickSpeed = speed == Double.MAX_VALUE;
    	
        requires(Robot.ss_Collector);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	endTime = Timer.getFPGATimestamp() + time;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (joystickSpeed) {
    		if (intake)
    			speed = Robot.ss_Collector.getInSpeed();
    		else
    			speed = Robot.ss_Collector.getOutSpeed();
    			
    	}
    	double finalSpeed = intake ? speed : -speed;
    	Robot.ss_Collector.drive(finalSpeed, finalSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	// default command takes over
    	//Robot.ss_Collector.drive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//this.end();
    }
}
