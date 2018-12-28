package org.usfirst.frc.team4911.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_TriggerWhenPressed extends Command {
	Command cmd;
	Joystick gamepad;
	int axis;
	
	boolean changeOnce = true;
	boolean onOff = false;
	boolean start = false;
	boolean cancel = false;
	final double TOLERANCE = 0.8;
	
    public C_TriggerWhenPressed(Command cmd, Joystick gamepad , boolean leftTrigger) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.cmd = cmd;
    	this.gamepad = gamepad;
    	
    	if(leftTrigger) {
    		axis = 2;
    	} else {
    		axis = 3;
    	}
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	if((gamepad.getRawAxis(axis) > TOLERANCE) && changeOnce) {
//    		if(onOff) {
//    			start = true;
//    		} else {
//    			cancel = true;
//    		}
//    		
//    		changeOnce = false;
//    		onOff = (!onOff);
//    	} else if(gamepad.getRawAxis(axis) < TOLERANCE){
//    		changeOnce = true;
//    	}
//    	
//    	if(start) {
//    		cmd.start();
//    		start = false;
//    	} else if (cancel) {
//    		cmd.cancel();
//    		cancel = false;
//    	}
    	if((gamepad.getRawAxis(axis) > TOLERANCE) && changeOnce) {
    		start = true;
    		changeOnce = false;
    	} else if(gamepad.getRawAxis(axis) < TOLERANCE){
    		changeOnce = true;
    	}
    	
    	if(start) {
    		cmd.start();
    		start = false;
    	}
    }

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