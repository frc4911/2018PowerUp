package org.usfirst.frc.team4911.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_StopCommand extends Command {
	Command command;
	
    public C_StopCommand(Command command) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.command = command;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	command.cancel();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	Robot.ss_UpdateLog.logRunningCommands(this.getName());
//    	Robot.ss_UpdateLog.logStoppedCommands(this.getName(), command.getName()); brian removing logging
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
