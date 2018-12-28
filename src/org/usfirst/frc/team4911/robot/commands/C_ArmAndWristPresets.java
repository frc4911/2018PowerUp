package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class C_ArmAndWristPresets extends Command {
	private static final double kArmCommandTimeout = 4.0;
	
	private boolean done = false;
	private double timeoutPeriod = kArmCommandTimeout;

	private ArmPresets wantedArmPreset, currentArmPreset;
	private WristPresets wantedWristPreset;
	private CommandGroup commandGroup;
	private double endTime;
	ArmAndWristPresetsCommandBuilder builder;
	
	public C_ArmAndWristPresets(ArmPresets armPreset, WristPresets wristPreset) {
		this(armPreset, wristPreset, kArmCommandTimeout);
	}

	public C_ArmAndWristPresets(ArmPresets armPreset, WristPresets wristPreset, double timeoutPeriod) {
		super.setTimeout(timeoutPeriod);
		this.timeoutPeriod = timeoutPeriod;
		wantedArmPreset = armPreset;
		wantedWristPreset = wristPreset;
		builder = new ArmAndWristPresetsCommandBuilder();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.CancelCommandButtonPressed = false;
		currentArmPreset = Robot.ss_RobotArm.getLastArmPreset();
		endTime = Timer.getFPGATimestamp() + timeoutPeriod;

		// Build up the command group
		commandGroup = builder.Build(currentArmPreset, wantedArmPreset, wantedWristPreset);
		commandGroup.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.CancelCommandButtonPressed) {
			commandGroup.cancel();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		done = commandGroup.isCompleted();

		// check timeout
		double now = Timer.getFPGATimestamp();
		boolean timedOut = now > endTime;
//		if (done) {
//			System.out.println("Arm(" + wantedArmPreset.toString() + ")/wrist("+ wantedWristPreset.toString() + ") elapsed time = " + (now - (endTime - timeoutPeriod)));
//		} else if (timedOut) {
//			System.out.println("Arm(" + wantedArmPreset.toString() + ")/wrist("+ wantedWristPreset.toString() + ") = TIMED OUT after " + timeoutPeriod);
//		}

		return done || timedOut;
	}

	// Called once after isFinished returns true
	protected void end() {
		commandGroup.cancel();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		this.end();
	}
}