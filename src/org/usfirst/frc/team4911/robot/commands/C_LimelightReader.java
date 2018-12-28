package org.usfirst.frc.team4911.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class C_LimelightReader extends Command {

	NetworkTable table;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	double avgTx = 0;
	double avgTy = 0;
	double avgArea = 0;
	int counter = 0;
	
	final int size = 12;
	double txList[] = new double[size];
	double tyList[] = new double[size];
	double areaList[] = new double[size];
	
    public C_LimelightReader() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	table = NetworkTableInstance.getDefault().getTable("limelight");
    	tx = table.getEntry("tx");
    	ty = table.getEntry("ty");
    	ta = table.getEntry("ta");
    	avgTx = 0;
    	avgTy = 0;
    	avgArea = 0;
    	counter = 0;
    	
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	for(int i = 0; i < size; i++)
    	{
    		txList[i] = x;
    		tyList[i] = y;
        	areaList[i] = area;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//System.out.println(counter);    	
    	
    	double x = tx.getDouble(0);
	    double y = ty.getDouble(0);
	    double area = ta.getDouble(0);
//	    SmartDashboard.putNumber("X Offset", x);
//	    SmartDashboard.putNumber("Y Offset", y);
//	    SmartDashboard.putNumber("Area", area);
	    
	    txList[counter % txList.length] = x;
	    tyList[counter % tyList.length] = y;
	    areaList[counter % areaList.length] = area;
	    avgTx = 0;
	    avgTy = 0;
	    avgArea = 0;
	    for(int i = 0; i < txList.length; i++)
	    {
	    	avgTx += txList[i];
	    	avgTy += tyList[i];
	    	avgArea += areaList[i];
	    }
	    avgTx /= txList.length;
	    avgTy /= tyList.length;
	    avgArea /= areaList.length;
	    
//	    SmartDashboard.putNumber("Avg X Offset", avgTx);
//	    SmartDashboard.putNumber("Avg Y Offset", avgTy);
//	    SmartDashboard.putNumber("Avg Area", avgArea);
//	    SmartDashboard.putNumber("Robot Angle", Robot.ahrs.getAngle());
	    
	    counter++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
