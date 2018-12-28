package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

public class C_CollectorDriveUntilCubeCollected extends C_CollectorDrive {

    public C_CollectorDriveUntilCubeCollected(boolean intake) { 
    	super(intake);
    }

    public C_CollectorDriveUntilCubeCollected(boolean intake, double time, double speed)  {
    	super(intake, time, speed);
    }

    @Override
    protected boolean isFinished() {
        return Robot.ss_Wrist.isCubeCollected() || super.isFinished();
    }
}