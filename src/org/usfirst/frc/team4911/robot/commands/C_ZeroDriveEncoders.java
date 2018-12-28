package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_ZeroDriveEncoders extends Command {

	public static int counter = 0;
	
    private int count = 0;

    // TODO: make references instead of Robot.subsystem.getDefaultMotor()
	
    public C_ZeroDriveEncoders() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.ss_DriveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.ss_DriveTrain.dm_DriveTrainLeft.zeroEncPos();
    	Robot.ss_DriveTrain.dm_DriveTrainLeft.zeroEncPos();    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	if ((Robot.ss_DriveTrain.dm_DriveTrainLeft.getSensorPos()==0) && (Robot.ss_DriveTrain.dm_DriveTrainRight.getSensorPos()==0)){
    		return true;
    	}

    	Robot.ss_DriveTrain.dm_DriveTrainLeft.zeroEncPos();
    	Robot.ss_DriveTrain.dm_DriveTrainRight.zeroEncPos();
    	count++;
    	
    	if (count > 25){ // TODO: remove count
    		return true;
    	}
    	
   		return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
