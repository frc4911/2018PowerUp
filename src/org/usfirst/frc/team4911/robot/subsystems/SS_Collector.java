package org.usfirst.frc.team4911.robot.subsystems;

import org.usfirst.frc.team4911.robot.RobotMap;
import org.usfirst.frc.team4911.robot.commands.C_CollectorHold;

import cyberknightsLib.DefaultMotorSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_Collector extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private static final boolean LEFT_BRAKE_MODE   = true;
	private static final boolean LEFT_INVERTED     = false; //false
	private static final String  LEFT_NAME 		   = "CollectorLeft";
	
	public static final DefaultMotorSRX dm_CollectorLeft = new DefaultMotorSRX(RobotMap.COLLECTOR_LEFT,
			LEFT_INVERTED, LEFT_NAME);
	
	private static final boolean RIGHT_BRAKE_MODE   = true;
	private static final boolean RIGHT_INVERTED     = true;
	private static final String  RIGHT_NAME 		= "CollectorRight";
	
	public static final DefaultMotorSRX dm_CollectorRight = new DefaultMotorSRX(RobotMap.COLLECTOR_RIGHT,
			RIGHT_INVERTED, RIGHT_NAME);
	
	
    public void initDefaultCommand() {

    	dm_CollectorLeft.setBrakeMode(LEFT_BRAKE_MODE);
    	dm_CollectorRight.setBrakeMode(RIGHT_BRAKE_MODE);
    	
    	setDefaultCommand(new C_CollectorHold());
    	
    }
    
    final double intakeSpeed = 0.8;
    public double getInSpeed() {
    	return intakeSpeed;
    }
    
    double outSpeed = 0.35;
    public double getOutSpeed() {
    	return outSpeed;
    }
    
    public void setOutSpeed(double newSpeed) {
    	if(newSpeed > 1)
    	{
    		newSpeed = 1;
    	}
    	else if(newSpeed < 0)
    	{
    		newSpeed = 0;
    	}
    	outSpeed = newSpeed;
    	//SmartDashboard.putNumber("Col Speed", outSpeed);
    }
    
    double holdSpeed = 0.4;
    public void setHoldSpeed(double newSpeed)
    {
    	holdSpeed = newSpeed;
    }
    
    public double getHoldSpeed()
    {
    	return holdSpeed;
    }
    
    double x = 0;
	public void drive(double x, double y) {
		this.x = x;
		dm_CollectorLeft.setSpeed(x);
		dm_CollectorRight.setSpeed(y);
		
//		SmartDashboard.putNumber("Collector L Amps", dm_CollectorLeft.getLeaderTalon().getOutputCurrent());
//		SmartDashboard.putNumber("Collector R Amps", dm_CollectorRight.getLeaderTalon().getOutputCurrent());
	}
	
	public double returnSpeed()
	{
		return x;
	}
		
}
