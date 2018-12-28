package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class C_POVWatch extends Command {
	
	public static final double DELTA_SET = 0.05;
	
	boolean running = false;
	double speed = 0;
	Command c_Drive; 
    public C_POVWatch() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	this.running= false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	int POV_value = Robot.m_oi.operator.getPOV();
    	boolean input = true; 
    	//SmartDashboard.putNumber("POV Value", POV_value);
		SmartDashboard.putNumber("Col Speed", Robot.ss_Collector.getOutSpeed());
    	switch (POV_value){
    	case 0:
    		this.speed = .60;
    		break;
    	case 45:
    		this.speed = .50;
    		break;
    	case 90:
    		this.speed = .40; 
    		break;
    	case 135:
    		this.speed = .30;
    		break;
    	case 180:
    		this.speed = .20;
    		break;
    	case 270:
    		this.speed = .80;
    		break;
    	case 315: 
    		this.speed = .70;
    		break;
    	default:
    		input = false; 
    		break;
    	}
    	
    	if (!running && input) {
	       	running = true;
       		this.c_Drive = new C_CollectorDrive(false, -1, speed);
       		c_Drive.start();
    	}
    	
    	else if (running && !input) {
    		running = false;
    		this.c_Drive.cancel();
    	}
    }
//	James
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
