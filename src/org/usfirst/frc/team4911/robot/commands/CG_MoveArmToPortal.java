package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CG_MoveArmToPortal extends CommandGroup {

    public CG_MoveArmToPortal() {
    	addSequential(new C_ArmAndWristPresets(ArmPresets.COLLECT, WristPresets.PORTAL));
   }
}
