package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CG_MoveArmToCollect extends CommandGroup {

	public CG_MoveArmToCollect() {
		addSequential(new C_ArmAndWristPresets(ArmPresets.COLLECT, WristPresets.COLLECT));
	}
}
