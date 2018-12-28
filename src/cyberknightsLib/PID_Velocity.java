package cyberknightsLib;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class PID_Velocity {
		
	// TODO: reduce number of field
	
	public final static int ACCELERATION = 75;
	
	// adjustedVel PID
	public static final double kP_POSITION 	   = 0.2;
	public static final double kI_POSITION 	   = 2;
	public static final double kD_POSITION 	   = 0;
	public static final double kIZone_POSITION = 1000;
	
	public static final int COUNTER_THRESHOLD = 10;
	public static final int HOLD_TIME = 50;
	
	public static final double MAJOR_ANGLE = 1.5;
	public static final double MINOR_ANGLE = 0.3; // was 0.1
			
	private DefaultMotorSRX dm;
		
	private int targetVel;
	private int targetPos;
		
	private boolean isTurning;
	
	private boolean rampUp;
	private boolean rampDown;
	
	private int lastVel = 0;
		
	private double currError = 0;
	private double lastError = 0;
	private double deltaError = 0;
	private double totalError = 0;
	
	private double currTime = 0;
	private double lastTime = 0;
	private double deltaTime = 0;
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
	 * Command Variables
	 */
	
	private boolean isFinished = false;
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*
	 * Constructors
	 */
	
    public PID_Velocity(DefaultMotorSRX dm, int targetPos, boolean isTurning, boolean rampUp, boolean rampDown,
    		double kF, double kP, double kI, double kD, int iZone) {
        this.dm = dm;
        
        this.targetPos = targetPos;
        
        this.isTurning = isTurning;
        this.rampDown = rampDown;
        
        initialize(kF, kP, kI, kD, iZone);
    }
    
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*
     * Command
     */

    // Called just before this Command runs the first time
    private void initialize(double kF, double kP, double kI, double kD, int iZone) {     	    	    	
    	// init pid
    	dm.configKF(0, kF, 0);
        dm.configKPID(0, kP, kI, kD, 0);
    	dm.configIntegralZone(0, iZone, 0);
        dm.selectProfileSlot(0); 
    }

    // Called repeatedly when this Command is scheduled to run
    private void execute() {
    	int currVel = targetVel;
    	
    	// cap for position
    	if (!isTurning && rampDown) {
        	updateError(Robot.ss_DriveTrain.dm_DriveTrainLeft.getSensorPos(), kIZone_POSITION);
    		int maxVel = (int) Math.round(pid(kP_POSITION, kI_POSITION, kD_POSITION, currError, totalError, deltaError, deltaTime));
    		currVel = capVel(targetVel, maxVel);
    	}
    	   
    	if (rampUp) {
    		currVel = applyRamp(currVel, lastVel, ACCELERATION); // TODO: might not work
    	}
    	
		dm.startVelocityPID(currVel); // TODO: only adjust as necessary
				
		lastVel = currVel;
//		printValues();
    }

    // Make this return true when this Command no longer needs to run execute()
    private boolean isFinished() {    	
    	return false;
    }

    // Called once after isFinished returns true
    private void end() {
    	dm.stopPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    private void interrupted() {
    	end();
    }
    
    private void updateError(double currPos, double iZone) {
    	currError = targetPos - currPos;
    	deltaError = currError - lastError;
    	
    	currTime  = Timer.getFPGATimestamp();
    	deltaTime = currTime - lastTime;
    	
    	totalError += currError * deltaTime;
    	
    	// iZone
    	if ((Math.abs(currError) > iZone)
    			|| (currError > 0 && lastError <= 0)
    			|| (currError < 0 && lastError >= 0)) { 
	    	totalError = 0;
    	}
    	
    	lastError  = currError;
    	lastTime  = currTime;
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
    
    private int applyRamp(int desiredVelocity, int lastVelocity, int acc) {
//    	double diff = Math.abs(desiredVelocity - lastVelocity);
//    	
//    	if (diff > acc) {
//    		if (desiredVelocity > lastVelocity)
//    			desiredVelocity = lastVelocity+acc;
//    		else
//    			desiredVelocity = lastVelocity-acc;
//    	}
    	
    	if (desiredVelocity > lastVelocity)
			desiredVelocity = lastVelocity+acc;
    	
    	return desiredVelocity;
    }
    
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*
     * Command Manipulation
     */
    
    public boolean runLoop(int targetVel) {
    	if (!isFinished) {
    		this.targetVel = targetVel;
    		
    		execute();
    		
    		isFinished = isFinished();
    		
    		if (isFinished) {
    			end();
    		}
    	}
    	return isFinished;
    }
    
    public void interrupt() {
    	interrupted();
    	isFinished = true;
    }
}
