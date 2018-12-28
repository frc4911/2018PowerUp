package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CG_PID extends CommandGroup {
	
	public static final int CRUISE_VELOCITY	 = 1500;
	public static final int COLLECT_VELOCITY = 750;
		
    public CG_PID() {
		addSequential(new C_ZeroDriveEncoders());
		addSequential(new C_DriveTrainPID(CRUISE_VELOCITY, 
    			(int) Math.round(Constants.ENC_TO_INCH * 60),
    			0, 0, 0, false, true, true, true, true, 10));	
		
		// arc
		addSequential(new C_DriveTrainPID(CRUISE_VELOCITY, 
    			90, true, 48, true, false, 10));
		addSequential(new C_DriveTrainPID(COLLECT_VELOCITY, 
				90, false, 0, true, false, 10));
		
		/* compensation */
		addSequential(new C_ZeroDriveEncoders());
		addSequential(new C_DriveTrainPID(CRUISE_VELOCITY, 
    			(int) Math.round(Constants.ENC_TO_INCH * 10),
    			90, true, true, true, 10));
		/*				*/
		
		addSequential(new C_DriveTrainPID(COLLECT_VELOCITY, 
    			180, false, 0, true, false, 10));

		addSequential(new C_ZeroDriveEncoders());
		addSequential(new C_DriveTrainPID(CRUISE_VELOCITY, 
    			(int) Math.round(Constants.ENC_TO_INCH * 12),
    			180, false, true, true, 10));
		
		addSequential(new C_DriveTrainPID(COLLECT_VELOCITY, 
    			90, false, 0, true, false, 10));
		
		// arc
		addSequential(new C_DriveTrainPID(CRUISE_VELOCITY, 
    			0, true, 48, false, true, 10));
		addSequential(new C_DriveTrainPID(COLLECT_VELOCITY, 
				0, false, 0, false, false, 10));
		
		addSequential(new C_ZeroDriveEncoders());
		addSequential(new C_DriveTrainPID(CRUISE_VELOCITY, 
				(int) Math.round(Constants.ENC_TO_INCH * (60 + 12 - 9)), // -9 is compensation
    			0, false, true, true, 10));
    }
}
