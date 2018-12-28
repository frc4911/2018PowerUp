/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4911.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	// Pixel Rio Admin password = "password"
		
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
	 * Other Data
	 * TODO: better title
	 */
	
	////////////////////////////////
	// FMS Data
	
	// Field Layout
	public static String fieldLayout; // TODO: use	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	 * Port Numbers
	 * 
	 * TODO: reorder to alphabetical
	 */
	
	////////////////////////////////
	// Driver Station Controllers
	
	// Driver
	public static final int STICK_LEFT	= 0;	
	public static final int STICK_RIGHT	= 1;
	
	// Operator
	public static final int OP_PAD		= 2;
	
	// Test
	public static final int TEST_PAD	= 3;
	
	////////////////////////////////
	// CANBus
	
	// DriveTrain
	public static final int DRIVE_TRAIN_FRONT_LEFT 	= 0;
	public static final int DRIVE_TRAIN_REAR_LEFT	= 1;

	public static final int DRIVE_TRAIN_FRONT_RIGHT = 15;
	public static final int DRIVE_TRAIN_REAR_RIGHT 	= 14;
	
	// Winch
	public static final int WINCH_FRONT_LEFT 		= 2;
	public static final int WINCH_BACK_LEFT			= 3;
	public static final int WINCH_FRONT_RIGHT		= 13;
	public static final int WINCH_BACK_RIGHT		= 12;
	
	
	// Arm
	public static final int ARM						= 9;
	
	// Wrist
	public static final int WRIST					= 6;

	// Collector
	public static final int COLLECTOR_LEFT			= 7;
	public static final int COLLECTOR_RIGHT			= 8;
	
	////////////////////////////////
	// Solenoids/Shifters
	public static final int SHIFTER_LEFT	= 0;
	public static final int SHIFTER_RIGHT	= 1;
	public static final int CLIMBER_ARM_DEPLOY = 2;
	
	////////////////////////////////
	// Analog Ports
	public static final int DIAl_POSITION = 0;
	public static final int DIAL_PRIORITY = 1;	
		
	////////////////////////////////
	// PS4 Style Joysticks
	
	// axis
	public static final int X_THUMB_STICK_LEFT = 0;
	public static final int Y_THUMB_STICK_LEFT = 1;
	public static final int X_THUMB_STICK_RIGHT = 4;
	public static final int Y_THUMB_STICK_RIGHT = 5;
	
	// triggers
	public static final int TRIGGER_LEFT = 2;
	public static final int TRIGGER_RIGHT = 3;
	
	//buttons
	public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;

	// Preset positions used by ARM and WRIST
	public static enum ArmPresets {
		UKNOWN, COLLECT, SWITCH, SCALE
	}

	public static enum WristPresets {
		COLLECT, SWITCH, PORTAL, SCALE_FORWARD, SCALE_BACKWARD, FLIP_UP, SCALE_BACKWARD_LOW, SCALE_BACKWARD_HIGH
	}
	
	public static enum FieldLocation {
		LEFT, CENTER, RIGHT, DONT_CARE, UNSPECIFIED
	}
	
	public static enum FieldPiece {
		SWITCH, SCALE, LINE
	}
	
	public static enum FieldPosition {
		DRIVER_STATION, SWITCH, ALLY, SCALE
	}

	public static enum AutoRoutines {
		CW, CC, CB, XC, X, W, L
	}
}
