/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4911.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.util.ConnectionMonitor;
import lib.util.CrashTracker;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import org.usfirst.frc.team4911.robot.commands.C_AutoModesAll;
import org.usfirst.frc.team4911.robot.commands.C_POVWatch;
import org.usfirst.frc.team4911.robot.subsystems.SS_Collector;
import org.usfirst.frc.team4911.robot.subsystems.SS_Dial;
import org.usfirst.frc.team4911.robot.subsystems.SS_DriveTrain;
import org.usfirst.frc.team4911.robot.subsystems.SS_RobotArm;
import org.usfirst.frc.team4911.robot.subsystems.SS_Climber;
import org.usfirst.frc.team4911.robot.subsystems.SS_Wrist;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final String robotName = readRobotName();
	
	public static SS_DriveTrain ss_DriveTrain = new SS_DriveTrain();
	public static SS_Collector ss_Collector = new SS_Collector();	
	public static SS_Wrist ss_Wrist = new SS_Wrist();
	public static SS_RobotArm ss_RobotArm = new SS_RobotArm(); 	
	public static SS_Climber ss_Climber = new SS_Climber();	
	public static SS_Dial ss_Dial = new SS_Dial();	
	public static C_POVWatch c_POVWatch = null;
	public static OI m_oi;

	//tank and arcade drive mode
	public static int driveStyle = 0;
	public static int maxDriveStyles = 2;
	public static int activeDriveTrainPIDs = 0;
	
	public static AHRS ahrs;
	public static boolean CancelCommandButtonPressed = false;
	
	public DriverStation ds = DriverStation.getInstance();
	private ConnectionMonitor connectionMonitor;
	
	Command m_autonomousCommand;

	public Robot() {
        CrashTracker.logRobotConstruction();
	}
	
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {			
	        CrashTracker.logRobotInit();
	        connectionMonitor = new ConnectionMonitor();
			
			SmartDashboard.putString("Robot", robotName);
			m_oi = new OI();
			c_POVWatch = new C_POVWatch();

			UsbCamera cameraFront = CameraServer.getInstance().startAutomaticCapture("Front", "/dev/video0");
			cameraFront.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 30);
			
//			UsbCamera cameraArm = CameraServer.getInstance().startAutomaticCapture("Arm", "/dev/video1");
//			cameraArm.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 20);
			
			try {
				/* Communicate w/navX-MXP via the MXP SPI Bus. */
				/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
				/*
				 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
				 * details.
				 */
				ahrs = new AHRS(Port.kMXP, (byte) 200);
				ahrs.reset();
			} catch (RuntimeException ex) {
				DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true); 
			}
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
		
            Robot.ss_Wrist.setWristHolding(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		allPeriodic();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            //System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

			ss_Wrist.setHomed(false);
			m_autonomousCommand = new C_AutoModesAll();
	
			if (m_autonomousCommand != null) {
				m_autonomousCommand.start();  // uncomment when ready
			}
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		allPeriodic();
	}

	@Override
	public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // This makes sure that the autonomous stops running when
			// teleop starts running. If you want the autonomous to
			// continue until interrupted by another command, remove
			// this line or comment it out.
			if (m_autonomousCommand != null) {
				m_autonomousCommand.cancel();
			}

			c_POVWatch.start();
			m_oi.c_MoveToScaleBackward.start();
			//m_oi.c_MoveToScaleForward.start();
			m_oi.c_MoveToScaleBackwardLow.start();
			
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
        try {
        	Scheduler.getInstance().run();
        	updateDashboard();
            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}
	
	public void updateDashboard() {
//		ss_DriveTrain.outputToSmartDashBoard();
		ss_RobotArm.outputToSmartDashboard();
		ss_Wrist.outputToSmartDashboard();
		ss_Dial.outputToSmartDashboard();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		allPeriodic();
	}

    /**
     * Helper function that is called in all periodic functions
     */
	private void allPeriodic() {
		ss_RobotArm.writeToLog();
		ss_Wrist.writeToLog();
		connectionMonitor.onLoop(Timer.getFPGATimestamp());
	}
	
	private static String readRobotName() {
		Scanner scn = null;
		String name = "voxel";
		try {
			File f = new File("/usr/local/frc/robotname.txt");
			
			if (f.exists()) {
				scn = new Scanner(f);
				name = scn.nextLine();
			}
			// TODO:  Throw - need to add appropriate file on the roborio and possible create dart classes
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} finally {
			if (scn != null) {
				scn.close();
				scn = null;
			}
		}
		
		return name;
	}
}
