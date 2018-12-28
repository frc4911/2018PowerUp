package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Constants;
import org.usfirst.frc.team4911.robot.Robot;

import cyberknightsLib.PID_Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//System.out.println(Robot.ss_DriveTrain.dm_DriveTrainRight.getLeaderTalon().getSensorCollection().getPulseWidthRiseToRiseUs());

// TODO: add enums
public class C_DriveTrainPID extends Command {
	
	public static final int COUNTER_THRESHOLD = 10;
	
	// adjustedVel PID
	public static final double kP = 30; // was 40
	public static final double kI = 90; // was 150
	public static final double kD = 4.5; // was 6.5 
	public static final double kIZone = 5;
	
	public static final int ANGLE_ARRAY_LENGTH = 10;
	
	public static final double MAJOR_ANGLE = 1.5; // angle window acceptable to be done
	public static final double MINOR_ANGLE = 0.3; // amount of change of angle to be considered near targetAngle // was 0.1
	
	private static int instanceIndex = 0;
		
	private PID_Velocity pid_DriveTrainLeft;
	private PID_Velocity pid_DriveTrainRight;
	
	// arc wheel ratio
	private double leftRightRatio;
	private boolean ratioIsInfinity;
	
	// driving control
	private int drivingMode;
	private boolean isTurning;
	private boolean driveForward;
	
	private boolean rampUp;
	private boolean rampDown;
	
	// reset NavX angle
	private boolean resetNavX;
	
	// counters
	private int iterationCounter;
	private int angleHoldCounter;

	// target
	private int targetVel;
	private int targetPos;
	private double targetAngle;
	
	// angle
	private double[] angleArray;
	private double startAngle;
	private double avgAngle;
	private double lastAvgAngle;
	
	// error
	private double currError;
	private double lastError;
	private double deltaError;
	private double totalError;
	
	// time
	private double currTime; 
	private double lastTime;
	private double deltaTime;
	private double endTime;
	private double timeout;

	/**
	 * Full control
	 * <br>
	 * <br>
	 * <code>drivingMode</code>: 
	 * 	0 = adjust dragging side target velocity
	 * 	1 = drive robot in an arc 
	 * 	2 = no functionality, throws an exception
	 * 	3 = adjust both side velocities at the same time
	 * <br>
	 * <code>resetRefAngle</code> resets the AHRS and sets resetRefAngle to the
	 * reset angle of the AHRS.
	 * 
	 * @param targetVel
	 * @param targetPos
	 * @param targetAngle
	 * @param drivingMode
	 * @param isTurning
	 * @param driveForward
	 * @param rampDown
	 * @param resetNavX
	 * @param timeout
	 */
    public C_DriveTrainPID(int targetVel, int targetPos, double targetAngle, double turningRadius, int drivingMode, 
    					   boolean isTurning, boolean driveForward, boolean rampUp, boolean rampDown, boolean resetNavX, double timeout) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.ss_DriveTrain);
       	
    	construct(targetVel, targetPos, targetAngle, turningRadius, drivingMode, isTurning, driveForward, rampUp, rampDown, resetNavX, timeout);
    }
    
	/**
	 * Turning Mode
	 * 
	 * @param targetVel
	 * @param targetAngle
	 * @param drivingMode
	 * @param timeout
	 */
    public C_DriveTrainPID(int targetVel, double targetAngle, boolean arc, double turningRadius, 
    					   boolean driveForward, boolean rampUp, double timeout) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.ss_DriveTrain);
    	
    	int drivingMode = (arc) ? 1 : 3;
    	
    	construct(targetVel, 0, targetAngle, turningRadius, drivingMode, true, driveForward, rampUp, false, false, timeout);
    }
    
	/**
	 * Driving Mode
	 * 
	 * @param targetVel
	 * @param targetPos
	 * @param targetAngle
	 * @param driveForward
	 * @param rampDown
	 * @param timeout
	 */
    public C_DriveTrainPID(int targetVel, int targetPos, double targetAngle, 
    					   boolean driveForward, boolean rampUp, boolean rampDown, double timeout) {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.ss_DriveTrain);
    	
    	construct(targetVel, targetPos, targetAngle, 0.0, 0, false, driveForward, rampUp, rampDown, false, timeout);
    }
    
    private void construct(int targetVel, int targetPos, double targetAngle, double turningRadius, int drivingMode, 
    					   boolean isTurning, boolean driveForward, boolean rampUp, boolean rampDown, boolean resetNavX, double timeout) {
    	// initialize variables for direction of robot
    	this.targetVel = (driveForward) ? targetVel : -targetVel;
    	this.targetPos = (driveForward) ? targetPos : -targetPos;
    	
    	this.targetAngle = targetAngle;
    	
    	// turningMode
    	if (isTurning) {
    		this.targetPos = (int) Math.round(targetAngle);
    	}    	
    	
    	this.drivingMode = drivingMode;
    	this.isTurning = isTurning;
    	this.driveForward = driveForward;
    	this.rampDown = rampDown;
    	this.resetNavX = resetNavX;
    	    	
    	this.timeout = timeout;
    	
    	// TODO: only calculate if driving in arc
		calcLeftRightRatio(turningRadius);
    }

    // Called just before this Command runs the first time
    protected void initialize() {    	 
    	Robot.ss_DriveTrain.dm_DriveTrainLeft.setBrakeMode(true);
    	Robot.ss_DriveTrain.dm_DriveTrainRight.setBrakeMode(true);
    	
    	// reset counters
    	iterationCounter = 0;
    	angleHoldCounter = 0;
    	
    	endTime = Timer.getFPGATimestamp() + timeout;
    	    	
    	// zero ahrs
    	if (resetNavX) {
			Robot.ahrs.zeroYaw();
	    	for(int i = 0; i < 10; i++) {
	    		if (Math.abs(Robot.ahrs.getAngle()) <= 1) {
	    			break;
	    		} else {
	    			Timer.delay(0.01);
	    		}
	    	}
    	}
    	
    	// initialize angle
    	startAngle = Robot.ahrs.getAngle();
    	angleArray = new double[ANGLE_ARRAY_LENGTH];
    	for (int i = 0; i < ANGLE_ARRAY_LENGTH; i++) {
    		angleArray[i] = startAngle;
    	}
    	lastAvgAngle = startAngle;

    	// TODO: optional reset enc and position pid for rolling into next pid
    	 
    	// reset error
    	lastError = 0;
    	totalError = 0;
    	
    	// reset time
    	currTime = Timer.getFPGATimestamp();
    	lastTime = currTime;
    	
    	// initialize pid's
    	pid_DriveTrainLeft = new PID_Velocity(Robot.ss_DriveTrain.dm_DriveTrainLeft, targetPos, 
    			isTurning, rampUp, rampDown,
    			0.32, 1.0, 0.0, 0.0, 0); // 0.32, 1.0, 0.0, 0.0, 0 at 1500 u/s
    	pid_DriveTrainRight = new PID_Velocity(Robot.ss_DriveTrain.dm_DriveTrainRight, targetPos, 
    			isTurning, rampUp, rampDown,
    			0.32, 1.0, 0.0, 0.0, 0); // 0.335, 1.0, 0.0, 0.0, 0 at 1500 u/s
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {;    	
    	DriveTrainVelocities dtVel = new DriveTrainVelocities(targetVel, targetVel);
		
		if (isTurning) {
			dtVel.leftVel = 0;
			dtVel.rightVel = 0;
		}
		
    	// average angles
    	avgAngle = calcAverageAngle();
    	
    	// TODO: updating the error and adjustedVel is not necessary for drivingMode 1
    	// update errors
    	updateError();
    	
    	// calculate velocity based on angle
    	int adjustedVel = calcVelDiff();
		
		// apply adjusted velocity
		applyVelAdjustment(dtVel, adjustedVel);
		
    	// update pid
    	pid_DriveTrainLeft.runLoop(dtVel.leftVel);
    	pid_DriveTrainRight.runLoop(dtVel.rightVel);
    	    	    	
    	//SmartDashboard.putNumber("Auto avgAngle", avgAngle);    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {    	
    	if (iterationCounter++ > COUNTER_THRESHOLD) { // counter is so turning pid can get moving and not drop out early
    		// drive to distance
    		if (!isTurning) {
        		int encPos = Robot.ss_DriveTrain.dm_DriveTrainLeft.getSensorPos();
        		
    			if ((driveForward && encPos >= targetPos) || (!driveForward && encPos <= targetPos)) {
    				printEndCondition("distance stop");
    	    		return true;
    			}
	    		
	    	// drive to angle
	    	} else if (isTurning && (avgAngle < targetPos + MAJOR_ANGLE && avgAngle > targetPos - MAJOR_ANGLE)) {
	    		if (avgAngle < lastAvgAngle + MINOR_ANGLE && avgAngle > lastAvgAngle - MINOR_ANGLE) { 
    				angleHoldCounter++;
    			} else { 
    				angleHoldCounter = 0;
    			}
    			lastAvgAngle = avgAngle;
    			
    			if (angleHoldCounter > 3) {
    				printEndCondition("angle stop");
	    			return true;
    			}
    			
    		// drive to angle in arc
	    	} else if (drivingMode == 1) {
	    		if (targetAngle > startAngle) {
	    			if (avgAngle > targetAngle) {
	    				printEndCondition("arc stop");
	    				return true;
	    			}
	    		} else {
	    			if (avgAngle < targetAngle) {
	    				printEndCondition("arc stop");
	    				return true;
	    			}
	    		}
	    	} 
    		// stop if idle
    		// TODO: stop if encoder vel is within threshold for time
//	    	} else if (dm_Main.getSensorVel() == 0) {
//	    		System.out.println(dm_Main.getDescription() + ": idle stop");
//    			return true;
//    		}
    	} 
    	
    	if (currTime > endTime) {
			printEndCondition("timed out");
    	}
        return currTime > endTime;
    }
    
    // Called once after isFinished returns true
    protected void end() {
    	pid_DriveTrainLeft.interrupt();
    	pid_DriveTrainRight.interrupt(); 
    	
    	Robot.activeDriveTrainPIDs = 0;
    	
    	instanceIndex++;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//System.out.println(this.getName() + " interrupted");
    	end();
    }
    
    private void calcLeftRightRatio(double turningRadius) {
    	double halfWheelBase = .5*Constants.WHEEL_BASE;
		if (turningRadius == -halfWheelBase) {
			leftRightRatio = Double.NaN;  // will check for this later
			ratioIsInfinity = true;
		} else {
			leftRightRatio = (turningRadius - halfWheelBase)/
				             (turningRadius + halfWheelBase);
			ratioIsInfinity = false;
		}
    }
    
    private double calcAverageAngle() {
       	angleArray[iterationCounter % ANGLE_ARRAY_LENGTH] = Robot.ahrs.getAngle();
    	double avgAngle = 0;
    	for (int i = 0; i < ANGLE_ARRAY_LENGTH; i++) {
    		avgAngle += angleArray[i];
    	}
    	return avgAngle / ANGLE_ARRAY_LENGTH;
    }
    
    private void updateError() {
    	currError  = targetAngle - avgAngle;
    	deltaError = currError - lastError;
    	
    	currTime  = Timer.getFPGATimestamp();
    	deltaTime = currTime - lastTime;
    	    	
    	// iZone
    	if ((Math.abs(currError) > kIZone)
    			|| (currError > 0 && lastError <= 0)
    			|| (currError < 0 && lastError >= 0)) { 
	    	totalError = 0;
    	} else {
        	totalError += currError * deltaTime;
    	}
    	
    	lastError  = currError;
    	lastTime  = currTime;
    }
    
    private int calcVelDiff() {
    	int adjustedVel = (int) Math.round(pid(kP, kI, kD, currError, totalError, deltaError, deltaTime));
    	return capVel(adjustedVel, targetVel);
    }
    
    private double pid(double kP, double kI, double kD, double currError, double totalError, double deltaError, double deltaTime) {
    	return (kP * currError) + (kI * totalError) + (kD * (deltaError / deltaTime));
    }
    
    private int capVel(int currVel, int targetVel) {
    	if (Math.abs(currVel) > Math.abs(targetVel)) {
    		currVel = (int) Math.copySign(targetVel, currVel);
    	}
    	return currVel;
    }
    
    private void applyVelAdjustment(DriveTrainVelocities dtVel, double adjustedVel) {
    	switch (drivingMode) {
		case (0):
			if (avgAngle > targetAngle) { // re ordered then statements
				dtVel.leftVel += adjustedVel;
			} else {
				dtVel.rightVel -= adjustedVel;
			}
			break;
		case (1):
			applyArcVelocity(dtVel);
			break;			
		case (2):
			throw new IllegalStateException("DrivingMode " + drivingMode + " currently has to functionality");
		case (3):
			dtVel.leftVel += adjustedVel;
			dtVel.rightVel -= adjustedVel;
			break;
		default:
			throw new IllegalArgumentException("invalid drivingMode " + drivingMode);
    	}
    }
    
    private void applyArcVelocity(DriveTrainVelocities dtVel) {
    	if (ratioIsInfinity) {
			// process like normal
    		dtVel.leftVel = 0;
    		dtVel.rightVel = targetVel;
//	    	velocityLeft = applyRamp(velocityLeft,lastVelocityLeft, ACCELERATION);
//	    	velocityRight = applyRamp(velocityRight,lastVelocityRight, ACCELERATION);
		} else {
			if (leftRightRatio == 0) {
				dtVel.leftVel = targetVel;
				dtVel.rightVel = 0;
//		    	velocityLeft = applyRamp(velocityLeft,lastVelocityLeft, ACCELERATION);
//		    	velocityRight = applyRamp(velocityRight,lastVelocityRight, ACCELERATION*Math.abs(leftRightRatio));
		    	
			} else if (Math.abs(leftRightRatio) > 1) {
				dtVel.leftVel = (int) Math.round((targetVel / leftRightRatio));// * 1.166);
				dtVel.rightVel = targetVel;	
//		    	velocityLeft = applyRamp(velocityLeft,lastVelocityLeft, ACCELERATION/Math.abs(leftRightRatio));
//		    	velocityRight = applyRamp(velocityRight,lastVelocityRight, ACCELERATION);
			} else {
				dtVel.leftVel = targetVel;
				dtVel.rightVel = (int) Math.round((targetVel * leftRightRatio)); //* 0.98);
//		    	velocityLeft = applyRamp(velocityLeft,lastVelocityLeft, ACCELERATION);
//		    	velocityRight = applyRamp(velocityRight,lastVelocityRight, ACCELERATION*Math.abs(leftRightRatio));
			}
		}
    }
    
    private void printEndCondition(String endCondition) {
    	final String PID = "Auto: ";
    	//System.out.println(PID + instanceIndex + ": " + endCondition);
    }
    
    public static void resetPIDIndex() {
    	instanceIndex = 0;
    }
}

class DriveTrainVelocities {
	public int leftVel, rightVel;
	
	public DriveTrainVelocities(int leftVel, int rightVel) {
		this.leftVel = leftVel;
		this.rightVel = rightVel;
	}
}
