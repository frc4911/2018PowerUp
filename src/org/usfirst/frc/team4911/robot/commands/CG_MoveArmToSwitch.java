package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CG_MoveArmToSwitch extends CommandGroup {

    public CG_MoveArmToSwitch() {
    	addSequential(new C_ArmAndWristPresets(ArmPresets.SWITCH, WristPresets.SWITCH));
    }
}
