package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class C_DriveStyleChooser extends Command {

	public C_DriveStyleChooser() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveStyle++;
		if (Robot.driveStyle >= Robot.maxDriveStyles) {
			Robot.driveStyle = 0;
		}
		switch (Robot.driveStyle) {
		case 0:
			SmartDashboard.putString("Drive Style", "Arcade Mode");
			break;
		case 1:
			SmartDashboard.putString("Drive Style", "Tank Drive Mode");
			break;
		default:
			SmartDashboard.putString("Drive Style", "Problem: STOP!");
			break;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
