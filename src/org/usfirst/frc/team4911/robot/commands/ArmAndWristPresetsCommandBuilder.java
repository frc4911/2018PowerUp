package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ArmAndWristPresetsCommandBuilder {

	private static final double kSafeArmAngle = 120;
	
/*
 * Create a command group for moving arm and wrist simultaneously.
 * 
 *  @param currentArmPreset Current present for arm
 *  @param wantedArmPreset Wanted present for arm
 *  @param wantedWristPreset Wanted present for wrist
 */
	public CommandGroup Build(ArmPresets currentArmPreset, ArmPresets wantedArmPreset, WristPresets wantedWristPreset) {
		CommandGroup commandGroup;
		// Build up the command group
		switch (currentArmPreset) {
		case COLLECT:
			commandGroup = handleFromCollect(wantedArmPreset, wantedWristPreset);
			break;
		case SWITCH:
			commandGroup = handleFromSwitch(wantedArmPreset, wantedWristPreset);
			break;
		case SCALE:
			commandGroup = handleFromScale(wantedArmPreset, wantedWristPreset);
			break;
		default:
			commandGroup = handleFromUnknown(wantedArmPreset, wantedWristPreset);
		}
		
		return commandGroup;
	}

	private CommandGroup handleFromUnknown(ArmPresets wantedArmPreset, WristPresets wantedWristPreset) {
//		System.out.println("handleFromUnknown - Arm to " + wantedArmPreset.toString() + " Wrist to " + wantedWristPreset.toString());
		CommandGroup group = new CommandGroup();
		group.addSequential(new C_WristPresets(kSafeArmAngle));
		group.addSequential(new C_ArmPresets(wantedArmPreset));
		group.addSequential(new C_WristPresets(wantedWristPreset));
		return group;
	}

	private CommandGroup handleFromScale(ArmPresets wantedArmPreset, WristPresets wantedWristPreset) {
//		System.out.println("handleFromScale - Arm to " + wantedArmPreset.toString() + " Wrist to " + wantedWristPreset.toString());
		CommandGroup group = new CommandGroup();

		switch (wantedArmPreset) {
		case SWITCH:
			group.addParallel(new C_ArmPresetsWaitForWrist(wantedArmPreset, -20, false));
			group.addSequential(new C_WristPresets(kSafeArmAngle));
			group.addSequential(new C_WristPresetsWaitForArm(wantedWristPreset, Robot.ss_RobotArm.getDart().getSwitch() + 13, false));
			break;
		case COLLECT:
			// TODO: get wrist position to see if we can move in parallel
			group.addParallel(new C_ArmPresetsWaitForWrist(wantedArmPreset, -20, false));
			group.addSequential(new C_WristPresets(kSafeArmAngle));
			group.addParallel(new C_WristPresetsWaitForArm(wantedWristPreset, Robot.ss_RobotArm.getDart().getBottom() + 30, false));
			group.addSequential(new C_ArmPresets(wantedArmPreset));
			break;
		default:
			group.addSequential(new C_WristPresets(wantedWristPreset));
		}

		return group;
	}

	private CommandGroup handleFromSwitch(ArmPresets wantedArmPreset, WristPresets wantedWristPreset) {
//		System.out.println("handleFromSwitch - Arm to " + wantedArmPreset.toString() + " Wrist to " + wantedWristPreset.toString());
		CommandGroup group = new CommandGroup();
		switch (wantedArmPreset) {
		case COLLECT:
			group.addParallel(new C_ArmPresetsWaitForWrist(wantedArmPreset, kSafeArmAngle, false));
			group.addSequential(new C_WristPresets(kSafeArmAngle));
			group.addParallel(new C_WristPresetsWaitForArm(wantedWristPreset, Robot.ss_RobotArm.getDart().getBottom() + 30, false));
			group.addSequential(new C_ArmPresets(wantedArmPreset));
			break;
		case SCALE:
			group.addParallel(new C_ArmPresetsWaitForWrist(wantedArmPreset, kSafeArmAngle, true));
			group.addSequential(new C_WristPresets(kSafeArmAngle));
			group.addParallel(new C_WristPresetsWaitForArm(wantedWristPreset, 460.0, true)); // was 300
			group.addSequential(new C_ArmPresets(wantedArmPreset));
			break;
		default:
			group.addSequential(new C_WristPresets(wantedWristPreset));
		}

		return group;
	}

	private CommandGroup handleFromCollect(ArmPresets wantedArmPreset, WristPresets wantedWristPreset) {
		CommandGroup group = new CommandGroup();
//	    System.out.println("handleFromCollect - Arm to " + wantedArmPreset.toString() + " Wrist to " + wantedWristPreset.toString());
		switch (wantedArmPreset) {
		case SWITCH:
			group.addParallel(new C_WristPresets(kSafeArmAngle));
			group.addSequential(new C_ArmPresetsWaitForWrist(wantedArmPreset, 100, true));
			group.addSequential(new C_WristPresets(wantedWristPreset));
			break;
		case SCALE:
			group.addParallel(new C_ArmPresetsWaitForWrist(wantedArmPreset, kSafeArmAngle, true));
			group.addSequential(new C_WristPresets(kSafeArmAngle));
			
			// Can start moving at -15 degrees
			group.addParallel(new C_WristPresetsWaitForArm(wantedWristPreset, 460.0, true)); // was 300
			group.addSequential(new C_ArmPresets(wantedArmPreset));
			break;
		default:
			group.addSequential(new C_WristPresets(wantedWristPreset));
		}
		return group;
	}
}
