package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class C_CollectorHold extends Command {

    public C_CollectorHold() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.ss_Collector);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//SmartDashboard.putNumber("Col Speed", Robot.ss_Collector.getInSpeed());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double speed = Robot.ss_Collector.getHoldSpeed();
    	Robot.ss_Collector.drive(speed, speed*0.75);
	}	

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.ss_Collector.drive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
