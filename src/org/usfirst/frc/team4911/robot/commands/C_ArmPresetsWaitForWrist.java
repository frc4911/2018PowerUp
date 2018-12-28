package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;

/**
 *
 */
public class C_ArmPresetsWaitForWrist extends C_ArmPresets {

//	private boolean movingUp;
	private double wirstAngleToWaitFor = 0.0;
	private boolean reachedTarget = false;
	
	public C_ArmPresetsWaitForWrist(ArmPresets preset, double wirstAngleToWaitFor, boolean movingUp) {
		this(preset, kArmCommandTimeout, wirstAngleToWaitFor, movingUp);
	}

	public C_ArmPresetsWaitForWrist(ArmPresets preset, double timeoutPeriod, double wirstAngleToWaitFor, boolean movingUp) {
		// Making an implicit requirement that on wrist is available but driven by another command
		super(preset, timeoutPeriod);
		this.wirstAngleToWaitFor = wirstAngleToWaitFor;
//		this.movingUp = movingUp;
	}
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	if (!reachedTarget ) {
	    	double currentAngle = Robot.ss_Wrist.getAngle();
	    	//System.out.println("MovingUp=" + movingUp + " wirstAngleToWaitFor=" + wirstAngleToWaitFor + " currentAngle=" + currentAngle );
	
	//    	if (movingUp && (Math.abs(currentAngle - wirstAngleToWaitFor) < 2)) {
	//    		return;
	//    	}
	//
	//    	if (!movingUp && (Math.abs(currentAngle - wirstAngleToWaitFor) < 2)) {
	//    		return;
	//    	}
	
	    	if (currentAngle < wirstAngleToWaitFor) {
	    		return;
	    	}
	    	
	    	reachedTarget = true;
    	}

    	super.execute();
    }
}
