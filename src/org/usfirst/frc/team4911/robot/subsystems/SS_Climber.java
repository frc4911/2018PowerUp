package org.usfirst.frc.team4911.robot.subsystems;

import org.usfirst.frc.team4911.robot.Constants;
import org.usfirst.frc.team4911.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.util.drivers.LazyCANTalon;

/**
 * The climber subsystem consists of a winch and deployment of arm to to latch hook on bar.  
 * The winch is driven by CIM motors driving and the arm by a air piston controlled by a single solenoid.
 */
public class SS_Climber extends Subsystem {

	private final WPI_TalonSRX mTalonLeader, mTalonFollower;
	private final Solenoid solDeploy = new Solenoid(RobotMap.CLIMBER_ARM_DEPLOY);

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public SS_Climber() {
		mTalonLeader = new LazyCANTalon(RobotMap.WINCH_FRONT_LEFT, 10);
		mTalonLeader.set(ControlMode.PercentOutput, 0);
		mTalonLeader.configNominalOutputForward(0.0, Constants.kTimeOutMs);
		mTalonLeader.configNominalOutputReverse(0.0, Constants.kTimeOutMs);
		mTalonLeader.configPeakOutputForward(1.0, Constants.kTimeOutMs);
		mTalonLeader.configPeakOutputReverse(-1.0, Constants.kTimeOutMs);
		mTalonLeader.setNeutralMode(NeutralMode.Brake);

		mTalonFollower = new LazyCANTalon(RobotMap.WINCH_BACK_LEFT, 10);
		mTalonFollower.set(ControlMode.Follower, RobotMap.WINCH_FRONT_LEFT);
		mTalonFollower.configNominalOutputForward(0.0, Constants.kTimeOutMs);
		mTalonFollower.configNominalOutputReverse(0.0, Constants.kTimeOutMs);
		mTalonFollower.configPeakOutputForward(1.0, Constants.kTimeOutMs);
		mTalonFollower.configPeakOutputReverse(-1.0, Constants.kTimeOutMs);
		mTalonFollower.setNeutralMode(NeutralMode.Brake);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new C_DriveWinch());
	}

	public void climb() {
		mTalonLeader.set(1); //0.7
	}

	public void stopClimb() {
		mTalonLeader.set(0.0);
	}
	
	public void deploy(boolean toggle) {
		solDeploy.set(toggle);
	}

	public synchronized void outputToSmartDashboard() {
//		SmartDashboard.putNumber("CLIMBER Leader MotorOutputPercent", mTalonLeader.getMotorOutputPercent());
//		SmartDashboard.putNumber("CLIMBER Leader MotorOutputVoltage", mTalonLeader.getMotorOutputVoltage());
//		SmartDashboard.putNumber("CLIMBER Leader OutputCurrent", mTalonLeader.getOutputCurrent());
//		SmartDashboard.putNumber("CLIMBER Follower MotorOutputPercent", mTalonFollower.getMotorOutputPercent());
//		SmartDashboard.putNumber("CLIMBER Follower MotorOutputVoltage",mTalonFollower.getMotorOutputVoltage());
//		SmartDashboard.putNumber("CLIMBER Follower OutputCurrent", mTalonFollower.getOutputCurrent());
		SmartDashboard.putString("CLIMBER Solenoid ", solDeploy.get() ? "On" : "Off");
	}
}
