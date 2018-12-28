package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import cyberknightsLib.DefaultMotorSRX;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class C_RunPositionPID extends Command {

	public static final int THRESHOLD = 50;
	public static final int HOLD_TIME = 50;
	
	private DefaultMotorSRX dm;
	
	private boolean pidActive;
	
	private int target;
	
	private int kP;
	private int kI;
	private int kD;
	private int iZone;
	
	private int iteration = 0;
		
    public C_RunPositionPID(Subsystem subsystem, DefaultMotorSRX dm, int target,
    		int kP, int kI, int kD, int iZone) {
        // Use requires() here to declare subsystem dependencies
//        requires(subsystem);
        
        this.dm = dm;
        
        this.target = target;
        
        this.kP    = kP;
        this.kI    = kI;
        this.kD    = kD;
        this.iZone = iZone;
    }

    // Called just before this Command runs the first time
    protected void initialize() { 
    	Robot.activeDriveTrainPIDs++;
    	pidActive = true;
    	
    	iteration = 0;
    	
    	// init pid
        dm.configKF(0, 0, 0);
        dm.configKPID(0, kP, kI, kD, 0);
        dm.configIntegralZone(0, iZone, 0);
        dm.selectProfileSlot(0);
        
        // start PID
    	dm.startPositionPID(target);    	
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println(Robot.activeDriveTrainPIDs);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Math.abs(dm.getClosedLoopError()) < THRESHOLD) {
    		//System.out.println(dm.getClosedLoopError());
    		if (++iteration > HOLD_TIME && pidActive) {
    			Robot.activeDriveTrainPIDs--; // TODO: doesn't get decremented if robot is disabled early
    			pidActive = false;
    		}
    	} else {
    		iteration = 0;
    	}   
    	
    	return Robot.activeDriveTrainPIDs <= 1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	dm.stopPID();
    	
    	Robot.activeDriveTrainPIDs = 0; // if command is stopped early
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
