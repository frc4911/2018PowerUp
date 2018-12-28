package org.usfirst.frc.team4911.robot;

import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;
import org.usfirst.frc.team4911.robot.commands.CG_MoveArmToCollect;
import org.usfirst.frc.team4911.robot.commands.CG_MoveArmToPortal;
import org.usfirst.frc.team4911.robot.commands.CG_MoveArmToScaleBackwards;
import org.usfirst.frc.team4911.robot.commands.CG_MoveArmToScaleBackwardsLow;
import org.usfirst.frc.team4911.robot.commands.CG_MoveArmToScaleForward;
import org.usfirst.frc.team4911.robot.commands.CG_MoveArmToSwitch;
import org.usfirst.frc.team4911.robot.commands.CG_PID;
import org.usfirst.frc.team4911.robot.commands.C_CancelPresetCommands;
import org.usfirst.frc.team4911.robot.commands.C_ClimberDeployArm;
import org.usfirst.frc.team4911.robot.commands.C_ClimberDriveWinch;
import org.usfirst.frc.team4911.robot.commands.C_CollectorDrive;
import org.usfirst.frc.team4911.robot.commands.C_CollectorHoldIsOn;
import org.usfirst.frc.team4911.robot.commands.C_DriveShiftDown;
import org.usfirst.frc.team4911.robot.commands.C_DriveShiftUp;
import org.usfirst.frc.team4911.robot.commands.C_DriveStyleChooser;
import org.usfirst.frc.team4911.robot.commands.C_EncoderReset;
import org.usfirst.frc.team4911.robot.commands.C_LimelightReader;
import org.usfirst.frc.team4911.robot.commands.C_StopCommand;
import org.usfirst.frc.team4911.robot.commands.C_TriggerWhenPressed;
import org.usfirst.frc.team4911.robot.commands.C_WristPresets;
import org.usfirst.frc.team4911.robot.commands.C_WristZero;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	public final Joystick stickLeft = new Joystick(RobotMap.STICK_LEFT);
	public final Joystick stickRight = new Joystick(RobotMap.STICK_RIGHT);
	public final Joystick operator = new Joystick(RobotMap.OP_PAD);
	
	// driver
	Button leftBtn2 = new JoystickButton(stickLeft, 2);
	Button leftBtn4 = new JoystickButton(stickLeft, 4); // temp
	Button leftBtn16 = new JoystickButton(stickLeft, 16); // temp
	
	Button rightBtn1 = new JoystickButton(stickRight, 1);
	Button rightBtn2 = new JoystickButton(stickRight, 2);
	Button rightBtn3 = new JoystickButton(stickRight, 3); // temp
	Button rightBtn4 = new JoystickButton(stickRight, 4); // temp
	Button rightBtn5 = new JoystickButton(stickRight, 5);
	Button rightBtn11 = new JoystickButton(stickRight, 11);
	Button rightBtn12 = new JoystickButton(stickRight, 12);
	Button rightBtn13 = new JoystickButton(stickRight, 13);
	Button rightBtn14 = new JoystickButton(stickRight, 14);
	Button rightBtn15 = new JoystickButton(stickRight, 15);
	//Button rightBtn16 = new JoystickButton(stickRight, 16);
	
	// operator
	Button opBtnB = new JoystickButton(operator, RobotMap.BUTTON_B);
	Button opBtnX = new JoystickButton(operator, RobotMap.BUTTON_X);
	Button opLeftBumper = new JoystickButton(operator, 5);
	Button opRightBumper = new JoystickButton(operator, 6);
	Button opBtnStart = new JoystickButton(operator, RobotMap.BUTTON_START);
	Button opBtnBack = new JoystickButton(operator, RobotMap.BUTTON_BACK);
	Button opBtnA = new JoystickButton(operator, 1 );
	Button opBtnY = new JoystickButton(operator, 4);
	Button opBtn9 = new JoystickButton(operator, 9);
	
	//commands
	public Command c_MoveToCollect;
	public Command c_MoveToScaleBackward;
	public Command c_MoveToScaleForward;
	public Command c_MoveToScaleBackwardLow;
	
	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	public OI() {

//		leftBtn2.whenPressed(new CG_PID());
		leftBtn4.whenPressed(new C_CollectorDrive(false, 2, Robot.ss_Collector.getInSpeed()));
		Command cmd = new CG_PID();
		leftBtn16.whenPressed(cmd);
		leftBtn16.whenReleased(new C_StopCommand(cmd));
		
		rightBtn1.whenPressed(new C_LimelightReader()); // temp
		rightBtn2.whenPressed(new C_WristPresets(0));
		rightBtn3.whenPressed(new C_CollectorHoldIsOn(true)); // temp
		rightBtn4.whenPressed(new C_CollectorHoldIsOn(false)); // temp
		rightBtn5.whenPressed(new C_WristZero());
		rightBtn12.whenPressed(new C_DriveShiftUp());
		rightBtn13.whenPressed(new C_DriveShiftDown());
		rightBtn14.whenPressed(new C_DriveStyleChooser());
		rightBtn15.whenPressed(new C_EncoderReset());
		
		this.opBtnB.whenPressed(new CG_MoveArmToScaleForward());
		this.opBtnX.whileHeld(new C_CollectorDrive(true));
		this.opBtnY.whenPressed(new CG_MoveArmToSwitch());
		this.opLeftBumper.whenPressed(new CG_MoveArmToCollect());
		c_MoveToScaleBackward = new C_TriggerWhenPressed(new CG_MoveArmToScaleBackwards() , operator, true);
//		c_MoveToScaleForward = new C_TriggerWhenPressed(new CG_MoveArmToScaleForward() , operator, false);
		c_MoveToScaleBackwardLow = new C_TriggerWhenPressed(new CG_MoveArmToScaleBackwardsLow() , operator, false);
		this.opRightBumper.whenPressed( new CG_MoveArmToPortal());
		this.opBtnA.whenPressed(new C_WristPresets(WristPresets.FLIP_UP));
//		this.opBtnA.whenPressed(new C_AutoFlipUpActivated());
		this.opBtn9.whenPressed(new C_CancelPresetCommands());
		
		// Climber Controls
		opBtnStart.whenPressed(new C_ClimberDeployArm());
		opBtnBack.whileHeld(new C_ClimberDriveWinch());
//		opBtnBack.whenReleased(new C_ClimberDriveWinch(false));
	}
}
