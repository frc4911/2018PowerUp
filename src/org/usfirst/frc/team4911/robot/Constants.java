package org.usfirst.frc.team4911.robot;

public class Constants {

	public static final int kPIDLoopIdx 		= 0;
	public static final int kSlotIdx 			= 0;
	public static final int kMotionMagicSlotIdx = 1;
	public static final int kTimeOutMs			= 50;
	
	public static final int kRightArmMasterId = 12;
	public static final int kLeftArmMasterId  = 3;
	
	// auto timeouts
	public static final double TIMEOUT_DRIVE = 5;
	public static final double TIMEOUT_TURN  = 2.5;
	public static final double TIMEOUT_SHOOT = 1.5;
	
	// auto collector speeds
	public static final double COL_SWITCH_SPEED = 0.5;
	public static final double COL_SCALE_SPEED = 0.6;	
	public static final double COL_COLLECT = 0.7;
	
	// drive train
	public static final double ENC_TO_INCH = 241.4439432979; //233.3333333333; //244.4619925891;
	
	// measurements are in inches
	public static final double CENTER_TO_FRONT = 10.477; 
	public static final double CENTER_TO_REAR  = 15.394;
	public static final double BUMPER_WIDTH    = 3;
	public static final double ROBOT_LENGTH    = CENTER_TO_FRONT + CENTER_TO_REAR + (BUMPER_WIDTH * 2); // TODO: robot is actually 38
													// BUMPER_WIDTH * 2 for front and back bumpers
	
	public static final double WHEEL_BASE	   = 22.5;
	
	// center of rotation
	public static final double CR_RADIUS = WHEEL_BASE / 2;
	public static final double CR_BEHIND_CENTER_WHEEL = 2;
	public static final double CR_FROM_FRONT = BUMPER_WIDTH + CENTER_TO_REAR + CR_BEHIND_CENTER_WHEEL;
	public static final double CR_FROM_REAR  = BUMPER_WIDTH + CENTER_TO_REAR - CR_BEHIND_CENTER_WHEEL;
	
	// game element dimension (in inches)
	public static final double CUBE_WIDTH = 13;
	public static final double CUB_HEIGTH = 11;
	
	public static final double DRIVER_TO_NEAR_SWITCH = 140;
	public static final double DRIVER_TO_FAR_SWITCH = 196;
	public static final double SWITCH_WIDTH = DRIVER_TO_FAR_SWITCH - DRIVER_TO_NEAR_SWITCH;
	
	public static final double DRIVER_TO_SCALE = 300;
	
	public static final double EXCHANGE_LENGTH = 36;
}
