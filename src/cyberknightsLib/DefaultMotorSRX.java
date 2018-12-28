package cyberknightsLib;

import org.usfirst.frc.team4911.robot.Constants;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import lib.util.drivers.LazyCANTalon;

/**
 * Added:
 * <br> - <code>configClosedloopRamp(double seconds)</code>
 * <br>	- <code>setEncPos</code>
 * <br> - <code>LazyCANTalon</code> functionality from 254 Cheesy Poofs for <code>setSpeed(double speed)</code>
 * 
 * <br><br>Changed:
 * <br> - <code>getFollowerTalon0()</code> throws a <code>NullPointerException</code> if
 * 			the talon has not been initialized
 * <br> - <code>getFollowerTalon1()</code> throws a <code>NullPointerException</code> if
 *			the talon has not been initialized
 * 
 * @version 1.4
 * @author caleb 
 *  
 */
public class DefaultMotorSRX extends DefaultMotor {

	// talons
	private WPI_TalonSRX l_Talon;
	private WPI_TalonSRX f_Talon0;
	private WPI_TalonSRX f_Talon1;
	
    private double mLastSet = Double.NaN;
    private ControlMode mLastControlMode = null;
		
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	 * Constructors
	 * 
	 * TODO: soft limits
	 * TODO: three (3) talon soft limits constructor
	 * TODO: Refactor talons into an array for easier access and control
	 * TODO: pass in talon timout number
	 */
		
	/**
	 * Single Talon
	 * 
	 * @param l_TalonPort
	 * @param isInverted
	 * @param description
	 */
	public DefaultMotorSRX(int l_TalonPort, boolean isInverted, String description) {		
		construct(l_TalonPort, isInverted, false, description);
	}
	
	/**
	 * Talon Pair
	 * 
	 * @param l_TalonPort
	 * @param f_TalonPort0
	 * @param isInverted
	 * @param description
	 */
	public DefaultMotorSRX(int l_TalonPort, int f_TalonPort0, boolean isInverted, String description) {
		this.f_Talon0 = new LazyCANTalon(f_TalonPort0);
		
		motorPair = true;
		
		this.f_Talon0.set(ControlMode.Follower, l_TalonPort);
		
		this.f_Talon0.setInverted(isInverted);
		
		construct(l_TalonPort, isInverted, false, description);
	}
	
	/**
	 * Talon triplet
	 * 
	 * @param l_TalonPort
	 * @param f_TalonPort0
	 * @param f_TalonPort1
	 * @param isInverted
	 * @param description
	 */
	public DefaultMotorSRX(int l_TalonPort, int f_TalonPort0, int f_TalonPort1, boolean isInverted, String description) {
		this.f_Talon0 = new LazyCANTalon(f_TalonPort0,10);
		this.f_Talon1 = new LazyCANTalon(f_TalonPort1,10);
		
		motorTriplet = true;
		
		this.f_Talon0.set(ControlMode.Follower, l_TalonPort);
		this.f_Talon1.set(ControlMode.Follower, l_TalonPort);
		
		this.f_Talon0.setInverted(isInverted);
		this.f_Talon1.setInverted(isInverted);
		
		construct(l_TalonPort, isInverted, false, description);
	}
	
	/**
	 * Single Talon with limits <br>
	 * <code>Deprecated</code> because soft limits are not yet
	 * supported.
	 * 
	 * @param l_TalonPort
	 * @param isInverted
	 * @param upLimit
	 * @param lowLimit
	 * @param description
	 */
	@Deprecated
	public DefaultMotorSRX(int l_TalonPort, boolean isInverted, double upLimit, double lowLimit, String description) {
//		this.upLimit = upLimit;
//		this.lowLimit = lowLimit;
		this.description = description;
		
		motorPair = false;
		
		construct(l_TalonPort, isInverted, true, description);
	}
	
	/**
	 * Talon pair with limits <br>
	 * <code>Deprecated</code> because soft limits are not yet
	 * supported.
	 * 
	 * @param l_TalonPort
	 * @param f_TalonPort0
	 * @param isInverted
	 * @param upLimit
	 * @param lowLimit
	 * @param description
	 */
	@Deprecated
	public DefaultMotorSRX(int l_TalonPort, int f_TalonPort0, boolean isInverted, double upLimit, double lowLimit, String description) {
		this.f_Talon0 = new WPI_TalonSRX(f_TalonPort0);
//		this.upLimit = upLimit;
//		this.lowLimit = lowLimit;
		
		motorPair = true;
		
		this.f_Talon0.set(ControlMode.Follower, l_TalonPort);

		this.f_Talon0.setInverted(isInverted);

		
		construct(l_TalonPort, isInverted, true, description);
	}
	
	/**
	 * Used by the constructors to initialize Talons
	 * 
	 * @param l_TalonPort
	 * @param isInverted
	 * @param limited
	 * @param description
	 */
	private void construct(int l_TalonPort, boolean isInverted, boolean limited, String description) {
		this.l_Talon = new WPI_TalonSRX(l_TalonPort);
//		this.motorInverted = inverted;
		this.description = description;
		
		this.l_Talon.setInverted(isInverted);
		
//		if(limited) {
//			setSoftLimits(l_Talon);
//			enableSoftLimits(l_Talon, true);
//		}
//		else {
//			enableSoftLimits(l_Talon, false);
//			if (m_Pair){
//				enableSoftLimits(f_Talon, false);				
//			}
//		}
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	* Moving and stopping
	*/
	
	
	/**
	 * Set <code>speed</code> for the motors ranging from +1 to -1. 
	 * Only updates <code>l_Talon</code> speed if the current speed set
	 * as well as the current <code>ControlMode</code> doesn't match
	 * that of the previous set to this call.
	 * <br>Functionality adopted from team 254 Cheesy Poof's 
	 * <code>LazyCANTalon</code>.
	 * 
	 * @param speed
	 */
	@Override
	public void setSpeed(double speed) {
		ControlMode mode = l_Talon.getControlMode(); // TODO: get control mode may deplete purpose
		if (speed != mLastSet || mode != mLastControlMode) {
			if ((Math.abs(speed) > speedLimit) && limitedSpeed) {
				speed = speedLimit * Math.signum(speed);
			}
			
            mLastSet = speed;
            mLastControlMode = mode;
            l_Talon.set(speed);
        }
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	* Closedloop
	*/
	
	/**
	 * Changes <code>l_Talon</code>'s <code>ControlMode</code> to
	 * <code>ControlMode.Position</code>.
	 * 
	 * @param target
	 */
	public void startPositionPID(int target) {
		l_Talon.set(ControlMode.Position, target);
	}
	
	/**
	 * Changes <code>l_Talon</code>'s <code>ControlMode</code> to
	 * <code>ControlMode.Velocity</code>.
	 * 
	 * @param target
	 */
	public void startVelocityPID(int target) {
		l_Talon.set(ControlMode.Velocity, target);
	}
	
	/**
	 * Resets <code>l_Talon</code>'s control mode back to
	 * <code>ControlMode.PercentOutput</code>.
	 */
	public void stopPID() {
		l_Talon.set(ControlMode.PercentOutput, 0);
	}
	
	////////////////////////////////
	// Set FPID values
	
	public void selectProfileSlot(int kSlotIndex) {
		l_Talon.selectProfileSlot(kSlotIndex, 0);
	}
	
	/**
	 * Configure <code>PID</code> for one slot index all in one method
	 * 
	 * @param kSlotIndex
	 * @param kP
	 * @param kI
	 * @param kD
	 * @param kTimeoutMS
	 */
	public void configKPID(int kSlotIndex, double kP, double kI, double kD, int kTimeoutMS) {
		l_Talon.config_kP(kSlotIndex, kP, kTimeoutMS);
		l_Talon.config_kI(kSlotIndex, kI, kTimeoutMS);
		l_Talon.config_kD(kSlotIndex, kD, kTimeoutMS);
	}
	
	/**
	 * Configure <code>kF</code>
	 * 
	 * @param kSlotIndex
	 * @param kF
	 * @param kTimeoutMS
	 */
	public void configKF(int kSlotIndex, double kF, int kTimeoutMS) {
		l_Talon.config_kF(kSlotIndex, kF, kTimeoutMS);
	}
	
	/**
	 * Configure <code>kP</code>
	 * 
	 * @param kSlotIndex
	 * @param kP
	 * @param kTimeoutMS
	 */
	public void configKP(int kSlotIndex, double kP, int kTimeoutMS) {
		l_Talon.config_kP(kSlotIndex, kP, kTimeoutMS);
	}
	
	/**
	 * Configure <code>kI</code>
	 * 
	 * @param kSlotIndex
	 * @param kI
	 * @param kTimeoutMS
	 */
	public void configKI(int kSlotIndex, double kI, int kTimeoutMS) {
		l_Talon.config_kI(kSlotIndex, kI, kTimeoutMS);
	}
	
	/**
	 * Configure <code>kD</code>
	 * 
	 * @param kSlotIndex
	 * @param kD
	 * @param kTimeoutMS
	 */
	public void configKD(int kSlotIndex, double kD, int kTimeoutMS) {
		l_Talon.config_kD(kSlotIndex, kD, kTimeoutMS);
	}
	
	/**
	 * Configure <code>kIZone</code>
	 * 
	 * @param kSlotIndex
	 * @param kIZone
	 * @param kTimeoutMS
	 */
	public void configIntegralZone(int kSlotIndex, int kIZone, int kTimeoutMS) {
		l_Talon.config_IntegralZone(kSlotIndex, kIZone, kTimeoutMS);
	}
	
	////////////////////////////////
	// Config Closedloop
	
	public void configClosedloopRamp(double seconds) {
		l_Talon.configClosedloopRamp(seconds, 0);
	}
	
	////////////////////////////////
	// Get Closedloop Values
	
	/**
	 * Get the error of <code>l_Talon</code>'s closed loop.
	 * 
	 * @return closedLoopError
	 */
	public int getClosedLoopError() {
		return l_Talon.getClosedLoopError(0);
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	 * Sensors
	 */
	
	////////////////////////////////
	// General Sensor Methods
	
	/**
	 * If the a sensor is reversed to the direction to which the motor
	 * spins, then set <code>inverted</code> to <code>true</code>.
	 * 
	 * @param inverted
	 */
	public void reverseSensor(boolean inverted) {
		l_Talon.setSensorPhase(inverted);
	}
	
	////////////////////////////////
	// Potentiometer
	public void configPot() {
		l_Talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.kTimeOutMs); /* PIDLoop=0, timeoutMs=0 */
		/* eFeedbackNotContinuous = 1, subValue/ordinal/timeoutMs = 0 */
		l_Talon.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
	}
	
	////////////////////////////////
	// Encoder
	
	/**
	 * Configure a quadrature encoder connected to <code>l_Talon</code>. <br>
	 * If the encoder is reversed to the direction to which the motor
	 * spins, then set <code>inverted</code> to <code>true</code>. <br>
	 * Then set the encoder position to 0.
	 * 
	 * @param inverted
	 */
	public void configQuadEnc(boolean inverted) {
		l_Talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeOutMs); /* PIDLoop=0,timeoutMs=0 */
		
		reverseSensor(inverted);
		zeroEncPos();
	}

	/**
	 * Reassigns the encoder position of <code>l_Talon</code> back to 0.
	 */
	@Override
	public void zeroEncPos() {
		/* sensorIndex/position/timeoutMS = 0*/
		l_Talon.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs); // TODO: change timeout
	}
	
	// TODO: document
	public void setEncPos(int ticks) {
		l_Talon.setSelectedSensorPosition(ticks,  0,  Constants.kTimeOutMs);
	}
	
	////////////////////////////////
	// Get Sensor Stats
	
	/**
	 * Get the sensor position of an sensor connected to
	 * <code>l_Talon</code>.
	 */
	@Override
	public int getSensorPos() {
		return l_Talon.getSelectedSensorPosition(0);
	}
	
	/**
	 * Get the sensor position of an sensor connected to
	 * <code>l_Talon</code>.
	 */
	public int getPWMSensorPos() {
		return l_Talon.getSensorCollection().getPulseWidthPosition(); 
	}
	
	
	/**
	 * Get the sensor velocity of an sensor connected to
	 * <code>l_Talon</code>.
	 */
	public int getSensorVel() {
		return l_Talon.getSelectedSensorVelocity(0);
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	* Single Talon dependent methods
	*/
	
	@Override
	public double getTalonSpeed() {
		return l_Talon.get();
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	 * Talon pair supported methods
	 * 
	 * TODO: import methods from DefaultMotor2017
	 * 		It may be better to not have these methods and just pass in a talon.
	 */		
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	 * Talon triplet supported methods
	 * 
	 * TODO: document methods
	 */
	
	public void inverted(boolean isInverted) {
		l_Talon.setInverted(isInverted);
		if (motorPair || motorTriplet)
			f_Talon0.setInverted(isInverted);
		if (motorTriplet)
			f_Talon0.setInverted(isInverted);
	}
	
	public void enableCurrentLimit(boolean enable) {
		l_Talon.enableCurrentLimit(enable);
		if (motorPair || motorTriplet)
			f_Talon0.enableCurrentLimit(enable);
		if (motorTriplet)
			f_Talon0.enableCurrentLimit(enable);
	}
	
	public void configPeakCurrent(int amps) {
		l_Talon.configPeakCurrentLimit(amps, 0);
		if (motorPair || motorTriplet)
			f_Talon0.configPeakCurrentLimit(amps, 0);
		if (motorTriplet)
			f_Talon0.configPeakCurrentLimit(amps, 0);
	}
	
	public void setBrakeMode(boolean brake) {
		NeutralMode mode;
		
		if (brake)
			mode = NeutralMode.Brake;
		else
			mode = NeutralMode.Coast;
		
		l_Talon.setNeutralMode(mode);
		if (motorPair || motorTriplet)
			f_Talon0.setNeutralMode(mode);
		if (motorTriplet)
			f_Talon0.setNeutralMode(mode);
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	* Get the Talon references
	* 
	* TODO: custom exception if null talon
	*/
	
	/**
	 * Get reference to <code>l_Talon</code>
	 * 
	 * @return l_Talon
	 */
	public WPI_TalonSRX getLeaderTalon() {
		return l_Talon;
	}
	
	/**
	 * Get reference to <code>f_Talon0</code>
	 * 
	 * @return f_Talon0
	 */
	public WPI_TalonSRX getFollowerTalon0() {
		if (motorPair || motorTriplet)
			return f_Talon0;
		else
			throw new NullPointerException("f_Talon0 may not have been initialized");
	}
	
	/**
	 * Get reference to <code>f_Talon1</code>
	 * 
	 * @return f_Talon1
	 */
	public WPI_TalonSRX getFollowerTalon1() {
		if (motorTriplet)
			return f_Talon1;
		else 
			throw new NullPointerException("f_Talon1 may not have been initialized");
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	*	* //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	/*
	* DefaultMotorSRX and Object Methods
	* 
	* TODO: override object.equals()
	* TODO: test equals()
	*/
	
	/**
	 * Compares DefaulMotor <code>description</code>s and <code>WPI_TalonSRX</code> 
	 * device ID numbers. Returns true if they all match.
	 * 
	 * @param dm
	 * @return 
	 */
	public boolean equals(DefaultMotorSRX dm) {
		boolean motorName 	= this.getDescription().equals(dm.getDescription());
		
		// early out if the names don't match
		if (!motorName)
			return false;
		
		boolean leaderID	= matchTalons(this.getLeaderTalon(), dm.getLeaderTalon());
		boolean follower0ID = matchTalons(this.getFollowerTalon0(), dm.getFollowerTalon0());
		boolean follower1ID = matchTalons(this.getFollowerTalon1(), dm.getFollowerTalon1());
		
		return leaderID && follower0ID && follower1ID;
	}
	
	/**
	 * Check if both <code>WPI_TalonSRX</code>s are either equal to
	 * <code>null</code> or if they have the same device ID.
	 * 
	 * @param thisTalon
	 * @param theirTalon
	 * @return
	 */
	private boolean matchTalons(WPI_TalonSRX thisTalon, WPI_TalonSRX theirTalon) {
		if (thisTalon == null && theirTalon == null) {
			return true;
		} else if (thisTalon != null && theirTalon != null) {
			return thisTalon.getDeviceID() == theirTalon.getDeviceID();
		} else {
			return false;
		}
	}
}
