package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;
import org.usfirst.frc.team4911.robot.subsystems.SS_Wrist;

/**
 * This command is intended to be called as part of a command group where arm is moving in 
 * parallel with wrist and arm direction is known. 
 */
public class C_WristPresetsWaitForArm extends C_WristPresets {

	private boolean isArmMovingUp;
	private double armPositionToWaitFor = 0.0;
	private boolean reachedTarget = false;
	
	public C_WristPresetsWaitForArm(double angle, double timeoutPeriod, double armPositionToWaitFor, boolean isArmMovingUp) {
		// Making an implicit requirement that on arm is available but driven by another command
		super(angle, timeoutPeriod);
		this.armPositionToWaitFor = armPositionToWaitFor;
		this.isArmMovingUp = isArmMovingUp;
	}

//	public C_WristPresetsWaitForArm(double angle, double armPositionToWaitFor, boolean isArmMovingUp) {
//		this(angle, kWristCommandTimeoutPeriod, armPositionToWaitFor, isArmMovingUp);
//	}
//	
	public C_WristPresetsWaitForArm(WristPresets preset, double timeoutPeriod, double armPositionToWaitFor, boolean isArmMovingUp) {
		this(SS_Wrist.getAngleForPreset(preset), timeoutPeriod, armPositionToWaitFor, isArmMovingUp);
	}

	public C_WristPresetsWaitForArm(WristPresets preset, double armPositionToWaitFor, boolean isArmMovingUp) {
		this(SS_Wrist.getAngleForPreset(preset), kWristCommandTimeoutPeriod, armPositionToWaitFor, isArmMovingUp);
	}
	
	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		if (!reachedTarget) {
			double currentArmPosition = Robot.ss_RobotArm.getPosition();
	
			if (isArmMovingUp && (currentArmPosition < armPositionToWaitFor)) {
				return;
			} else if (!isArmMovingUp && (currentArmPosition > armPositionToWaitFor)) {
				return;
			} 
			
			reachedTarget = true;
		}
		
		super.execute();
	}
}