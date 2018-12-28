package org.usfirst.frc.team4911.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class C_TriggerWhenPressedCancelable extends Command {

	Command cmd;
	Joystick gamepad;
	int axis;
	
	final double PRESSED_TOLERANCE = 0.8;
	final double RELEASED_TOLERANCE = 0.2;
	int state = 0;
	
    public C_TriggerWhenPressedCancelable(Command cmd, Joystick gamepad , boolean leftTrigger) {
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
    	state = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	switch (state) {
    	// no command running, looking for press
    	case 0:
    		if (gamepad.getRawAxis(axis) > PRESSED_TOLERANCE) {
    			cmd.start();
    			state = 1;
    		}
    		break;
    	case 1:
        	// command running, looking for release
    		if (gamepad.getRawAxis(axis) < RELEASED_TOLERANCE) {
    			cmd.cancel();
    			state = 0;
    		}
    		break;
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
