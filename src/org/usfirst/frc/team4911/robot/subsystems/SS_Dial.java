package org.usfirst.frc.team4911.robot.subsystems;

import org.usfirst.frc.team4911.robot.RobotMap;
import org.usfirst.frc.team4911.robot.RobotMap.AutoRoutines;
import org.usfirst.frc.team4911.robot.RobotMap.FieldLocation;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_Dial extends Subsystem {

	AnalogInput dial0 = new AnalogInput(RobotMap.DIAl_POSITION); // start position
	AnalogInput dial1 = new AnalogInput(RobotMap.DIAL_PRIORITY); // priorities
	
	double dial0Detents[] = {4, 61, 455, 954, 1476, 2021, 2571, 3088, 3579, 3995, 4076};
	double dial0Ranges[] = new double[dial0Detents.length];
	double dial1Detents[] = {4, 63, 454, 929, 1440, 1979, 2546, 3082, 3584, 4000, 4077};
	double dial1Ranges[] = new double[dial1Detents.length];
	
	double error = 1000;
	
	static final int MAX_POT_VALUE = 4100;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	public SS_Dial() {
		for(int i = 0; i < dial0Ranges.length - 1; i++) {
			dial0Ranges[i] = (dial0Detents[i] + dial0Detents[i + 1]) / 2;
		}
		dial0Ranges[dial0Ranges.length - 1] = MAX_POT_VALUE;
		
		for(int i = 0; i < dial1Ranges.length - 1; i++) {
			dial1Ranges[i] = (dial1Detents[i] + dial1Detents[i + 1]) / 2;
		}
		dial1Ranges[dial1Ranges.length - 1] = MAX_POT_VALUE;
		
//		for(int i = 0; i < dial1Ranges.length; i++) {
//			System.out.println(dial1Detents[i]);
//			System.out.println("    " + dial1Ranges[i]);
//		}
	}

	public double getAnalogValue(int port) {
		switch (port) {
		case 0:
			return dial0.getValue();

		case 1:
			return dial1.getValue();

		default:
			return error;
		}
	}
	
	public int analog0Reader() {
		int dialNum = 0;
		double startDial = getAnalogValue(RobotMap.DIAl_POSITION);
		
		for(int i = 0; i < dial0Ranges.length; i++) {
			if(startDial < dial0Ranges[i]) {
				dialNum = i;
				break;
			}
		}
		return dialNum;
	}
	
	public int analog1Reader() {
		int dialNum = 0;
		double priorityDial = getAnalogValue(RobotMap.DIAL_PRIORITY);
		
		for(int i = 0; i < dial1Ranges.length; i++) {
			if(priorityDial < dial1Ranges[i]) {
				dialNum = i;
				break;
			}
		}
		return dialNum;
	}
	
	public FieldLocation startPos() {
		FieldLocation position = null;
		
		switch(analog0Reader()) {
		case 4:
			position = FieldLocation.LEFT;
			break;
		case 5:
			position = FieldLocation.CENTER;
			break;
		case 6:
			position = FieldLocation.RIGHT;
			break;
		default:
			position = FieldLocation.UNSPECIFIED;
			break;
		}
		
		return position;
	}
	
	public String priorities() {
		String priority = "L";
		
		switch(analog1Reader()) {
		case 0:
			priority = "CB,XC";
			break;
		case 1:
			priority = "CC,XC"; // scale
			break;
		case 2:
			priority = "CW,CC,XC"; // scale-switch/scale
			break;
		case 3:
			priority = "CB,X";
			break;
		case 4:
			priority = "CC,X";
			break;
		case 5:
			priority = "CW,CC,X";
			break;
		case 6:
			priority = "CB,W";
			break;
		case 7:
			priority = "CC,W"; //scale-switch
			break;
		case 8:
			priority = "CW,CC,W"; // scale-switch/scale
			break;
		case 9:
			break;
		case 10:
			priority = "W";
			break;
		default:
			break;
		}
		
		return priority;
	}

	public void outputToSmartDashboard() {
//		SmartDashboard.putNumber("Dial 0", getAnalogValue(RobotMap.DIAl_POSITION));
//		SmartDashboard.putNumber("Dial 1", getAnalogValue(RobotMap.DIAL_PRIORITY));
	}

}
