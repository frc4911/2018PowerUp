package org.usfirst.frc.team4911.robot.commands;

import org.usfirst.frc.team4911.robot.Constants;
import org.usfirst.frc.team4911.robot.Robot;
import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;
import org.usfirst.frc.team4911.robot.RobotMap.AutoRoutines;
import org.usfirst.frc.team4911.robot.RobotMap.FieldLocation;
import org.usfirst.frc.team4911.robot.RobotMap.FieldPiece;
import org.usfirst.frc.team4911.robot.RobotMap.WristPresets;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class C_AutoModesAll extends Command {
	
	public static final int TURNING_VELOCITY = 500;
	
	public static final int D_SLOW 	 = 1500;
	public static final int D_CRUISE = 1800;
	public static final int D_FAST   = 2200;
	
	public static final int S_SLOW 	 = 1200;
	public static final int S_CRUISE = 1500;
	public static final int S_FAST	 = 1800;
	
	public static final int SAFETY_ROOM = 6;
	
	public static final int SD_TIMEOUT = 50;
	private int smartDashboardTimeout = SD_TIMEOUT;
	
	public static final String autoStartStr = "auto start location";
	public static final String autoPrioritiesStr = "auto priority list";
	
	private ArmAndWristPresetsCommandBuilder pcb = new ArmAndWristPresetsCommandBuilder();
	
	private FieldLocation startLocation = null; // left, center, right
	
	private CommandGroup commandGroup = null;
	
	// new way
			
	//private Queue<AutoRoutines> prioritiesList = new LinkedList<>();
	String prioritiesList[];
	
	private String gameData;
	
	int state = 0;
	
	boolean isDone = false;
    public C_AutoModesAll() {
        // requires nothing because CG's will require all
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//System.out.println(this.getName() + " initialize");
    	
    	smartDashboardTimeout = SD_TIMEOUT;
    	
    	state = 0;
    	
    	startLocation = null;
    	prioritiesList = null;
    }

    public void autoPriorities() {
    	String priority = Robot.ss_Dial.priorities(); // this is where we will read the dial
    	prioritiesList = priority.split(",");
    	SmartDashboard.putString(autoPrioritiesStr, priority);
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	switch (state) {
    	case 0:        	
        	autoPriorities();
        	    		
        	startLocation = Robot.ss_Dial.startPos();
        	
    		if (startLocation != null) {
    			SmartDashboard.putString(autoStartStr, startLocation.toString());
    		}
    			
        	state++;
    	case 1:
    		// TODO: put safety for gameData
			gameData = DriverStation.getInstance().getGameSpecificMessage();
			SmartDashboard.putString("game data", gameData);
    		
    		// this may take multiple attempts
			if (gameData != null && gameData.length() != 3) {
				if(--smartDashboardTimeout > 0) {
					break;
				}
				gameData = "UUU"; // unspecified, unspecified, unspecified
			}
			state++;
    	case 2:
    		commandGroup = selectAuto();
    		
    		if(commandGroup == null) {
    			isDone = true;
    		}
    		else {
    			commandGroup.start();
        		//System.out.println("isRunning: " + commandGroup.isRunning());
    		}
    		
    		state++;
    		break;
    	case 3:
    		// do nothing;
    		break;
    	}	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// exit if auto settings not valid
//    	if (!gotAutoValues)
//    		return true;
    	
    	if(isDone) {
    		return true;
    	}
    	
    	// keep going until cammandGroup created
    	if (commandGroup == null)
    		return false;
    	
    	// stop when command group completed
        return commandGroup.isCompleted();
    }

    // Called once after isFinished returns true
    protected void end() {
    	//System.out.println(this.getName() + " end");
    	
    	if (commandGroup != null)
    		commandGroup.cancel();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//System.out.println(this.getName() + " interrupted");
    	end();
    }
    
    // startPosition equals left, center, right, or unspecified
    // prioritiesList has to equal at least "L"
    // gameData can equal valid or "UUU"
    private CommandGroup selectAuto() {
//    	System.out.println("Dial 0: " + Robot.ss_Dial.analog0Reader());
//    	System.out.println("Dial 1: " + Robot.ss_Dial.analog1Reader());
//    	System.out.println("Start Pos: " + startLocation);
//    	for(int i = 0; i < prioritiesList.length; i++) {
//    		System.out.println("Priority: " + prioritiesList[i]);
//    	}
//    	System.out.println("Game Data: " + gameData);
    	
    	CommandGroup group = null;    
    	if(startLocation == FieldLocation.UNSPECIFIED) {
    		//System.out.println("nothing");
    		return group;
    	}
    	
    	if (startLocation == FieldLocation.CENTER) {
    		if(gameData.compareTo("UUU") == 0) {
    			group = CG_CenterLine(); // TODO: return on this statement // TODO: create line routine from center
        		//System.out.println("CG_CenterLine()");
    		}
    		
    		if(gameData.charAt(0) == 'R') {
    			group = CG_CenterDoubleSwitch(false); //CG_ToSwitchFromCenter(false);
    			//System.out.println("CG_CenterDoubleSwitch(false)");
    		}
    		else if(gameData.charAt(0) == 'L') {
    			group = CG_CenterDoubleSwitch(true);
    			//System.out.println("CG_ToSwitchFromCenter(true)");
    		}
    		else { // TODO: is never reached
    			group = CG_CenterLine();
    			//System.out.println("CG_CenterLine");
    		}
    		
    	} else {
    		if(gameData.compareTo("UUU") == 0) {
    			group = CG_SideLine(); // TODO: return on this statement
    			//System.out.println("CG_SideLine()");
    		}
    		else {
		    	for (int i = 0; i < prioritiesList.length && group == null; i++) {
		    		AutoRoutines route = AutoRoutines.valueOf(prioritiesList[i]);
		    		
		    		switch(route) {
		    		case CW:
		    			boolean startCWLeft = startLocation == FieldLocation.LEFT && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L';
		    			boolean startCWRight = startLocation == FieldLocation.RIGHT && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R';
		    			if (startCWLeft || startCWRight) {
		    				group = CG_SideTwoCubeScaleSwitch(startCWLeft, false);
		    				//System.out.println("CG_SideTwoCubeScaleSwitch(startCWLeft, false)");
		    			}
		    			break;
		    		case CC:
		    			boolean startCCLeft = startLocation == FieldLocation.LEFT && gameData.charAt(1) == 'L';
		    			boolean startCCRight = startLocation == FieldLocation.RIGHT && gameData.charAt(1) == 'R';
		    			if (startCCLeft || startCCRight) {
		    				group = CG_SideTwoCubeScaleSwitch(startCCLeft, true);
		    				//System.out.println("CG_SideTwoCubeScaleSwitch(startCCLeft, true)");
		    			}
		    			break;
		    		case CB:
		    			boolean startCBLeft = startLocation == FieldLocation.LEFT && gameData.charAt(1) == 'L';
		    			boolean startCBRight = startLocation == FieldLocation.RIGHT && gameData.charAt(1) == 'R';
		    			if (startCBLeft || startCBRight) {
		    				group = CG_SideScaleBackboard(startCBLeft);
		    				//System.out.println("CG_SideScaleBackboard(startCBLeft)");
		    			}
		    			break;
		    		case XC:
		    			boolean startXCLeft= startLocation == FieldLocation.LEFT && gameData.charAt(1) == 'R';
		    			boolean startXCRight = startLocation == FieldLocation.RIGHT && gameData.charAt(1) == 'L';
		    			if (startXCLeft|| startXCRight) {
		    				group = CG_SideCrossFieldScale(startXCLeft);
//		    				group = CG_SideCrossFieldScaleSplines(startXCLeft);
		    				//System.out.println("CG_SideCrossFieldScale(startXCLeft)");
		    			}
		    			break;
		    		case X:
		    			boolean startXLeft= startLocation == FieldLocation.LEFT && gameData.charAt(1) == 'R';
		    			boolean startXRight = startLocation == FieldLocation.RIGHT && gameData.charAt(1) == 'L';
		    			if (startXLeft|| startXRight) {
		    				group = CG_SideCrossHalfField(startXLeft);
		    				//System.out.println("CG_SideCrossHalfField(startXLeft)");
		    			}
		    			break;
		    		case W:
		    			boolean startWLeft = startLocation == FieldLocation.LEFT && gameData.charAt(0) == 'L';
		    			boolean startWRight = startLocation == FieldLocation.RIGHT && gameData.charAt(0) == 'R';
		    			if (startWLeft || startWRight) {
		    				group = CG_SideSwitch(startWLeft);
		    				//System.out.println("CG_SideSwitch(startWLeft)");
		    			}
		    			break;
		    		default:
		    			group = CG_SideLine();
		    			//System.out.println("CG_SideLine()");
		    			break;
		    		}
		    	}
    		}
    	}
    	
    	if (group == null) {
    		group = CG_SideLine();
    	}
  
    	return group;
    }
    
    ////////////////////////////////
    // Center
    
    // TODO: curve around cubes
    private CommandGroup CG_CenterLine() {
    	CommandGroup group = new CommandGroup();
    	
		group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
		group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			0,
    			0.0, 0.0, 0, false, true, false, true, true, 0)); 
		
		group.addSequential(new C_DriveTrainPID(S_CRUISE, 40, true, 45, true, true, Constants.TIMEOUT_DRIVE));
   		group.addSequential(new C_DriveTrainPID(S_CRUISE, 0, true, -50, true, true, Constants.TIMEOUT_DRIVE));
    	
    	return group;
    }

    private CommandGroup CG_ToSwitchFromCenter(boolean switchLeft) {
    	CommandGroup group = new CommandGroup();
    	
    	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
	   	group.addSequential(new C_ZeroDriveEncoders());
	   	if (switchLeft) {
	   		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.EXCHANGE_LENGTH + Constants.CENTER_TO_REAR)),
	    			0.0, 0.0, 0, false, true, true, false, true, Constants.TIMEOUT_DRIVE)); 
	   		
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -90, false, 0.0, false, true, Constants.TIMEOUT_TURN));
			
			group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SWITCH, WristPresets.SWITCH));
		   	group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * ((48 + 12) * 2)), // TODO: get rid of magic numbers
	    			-90, true, true, true, Constants.TIMEOUT_DRIVE));
			
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0, false, 0.0, false, true, Constants.TIMEOUT_TURN));

		   	group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (46)), // TODO get rid of magic numbers
	    			0, true, true, true, Constants.TIMEOUT_DRIVE));
			
	   	} else {
	   		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.CENTER_TO_REAR)),
	    			0.0, 0.0, 0, false, true, true, false, true, Constants.TIMEOUT_DRIVE));  
			
			group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SWITCH, WristPresets.SWITCH));
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.DRIVER_TO_NEAR_SWITCH - Constants.CENTER_TO_REAR - 24 - 4)),
	    			0, true, true, true, Constants.TIMEOUT_DRIVE));										// TODO: figure out 24 magic number
			
	   	}
    	
		group.addParallel(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SWITCH_SPEED));
	   	
    	return group;
    }
    
    private CommandGroup CG_CenterDoubleSwitch(boolean switchLeft) {
    	CommandGroup group = new CommandGroup();
    	
    	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
		group.addSequential(new C_DriveTrainPID(0, 
    			0,
    			0.0, 0.0, 0, false, false, true, false, true, 0)); 
		
		// spline
   		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SWITCH, WristPresets.SWITCH));
    	if (switchLeft) {
    		group.addSequential(new C_DriveTrainPID(S_FAST, -45, true, -48, true, true, Constants.TIMEOUT_DRIVE)); // angle was -35
       		group.addSequential(new C_DriveTrainPID(S_FAST, 0, true, 48, true, true, Constants.TIMEOUT_DRIVE)); 
    	} else {
    		group.addSequential(new C_DriveTrainPID(S_CRUISE, 40, true, 50, true, true, Constants.TIMEOUT_DRIVE));
       		group.addSequential(new C_DriveTrainPID(S_CRUISE, 0, true, -50, true, true, Constants.TIMEOUT_DRIVE)); 
    	}

    	// drop cube
		group.addParallel(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SWITCH_SPEED));
		group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0, false, 0.0, true, true, Constants.TIMEOUT_TURN));
		group.addSequential(new WaitCommand(0.25)); // was 0.5
		
		// back up
		group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			(int) Math.round(Constants.ENC_TO_INCH * (25)),
    			0, false, true, true, Constants.TIMEOUT_DRIVE));
		
		// turn and approach next cube
   		group.addParallel(pcb.Build(ArmPresets.SWITCH, ArmPresets.COLLECT, WristPresets.COLLECT));
   		if (switchLeft) {
   			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 47, false, 0.0, false, true, Constants.TIMEOUT_TURN));
   		} else {
   			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -47, false, 0.0, false, true, Constants.TIMEOUT_TURN));   			
   		}
		group.addSequential(new C_ZeroDriveEncoders());
		group.addParallel(new C_CollectorDrive(true, 6, 1.0)); // custom speed
		if (switchLeft) {
			group.addSequential(new C_DriveTrainPID(700, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (27)),
	    			47, true, true, true, 3.0)); // custom timeout
		} else {
			group.addSequential(new C_DriveTrainPID(700, 
					(int) Math.round(Constants.ENC_TO_INCH * (27)),
					-47, true, true, true, 3.0)); // custom timeout
		}
		
		// backup and line up with switch
		group.addSequential(new C_ZeroDriveEncoders());
		if (switchLeft) {
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    				(int) Math.round(Constants.ENC_TO_INCH * (27)),
    				47, false, true, true, Constants.TIMEOUT_DRIVE));
		} else {
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    				(int) Math.round(Constants.ENC_TO_INCH * (27)),
    				-47, false, true, true, Constants.TIMEOUT_DRIVE));
		}
		
    	group.addParallel(pcb.Build(ArmPresets.COLLECT, ArmPresets.SWITCH, WristPresets.SWITCH));
		group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0, false, 0.0, false, true, Constants.TIMEOUT_TURN));

		// drive to switch
       	group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			(int) Math.round(Constants.ENC_TO_INCH * (23)),
    			0, true, true, true, Constants.TIMEOUT_DRIVE));
		
		// drop cube
		group.addParallel(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SWITCH_SPEED));
		group.addSequential(new WaitCommand(0.5));
    	
    	return group;
    }
    
    ////////////////////////////////
    // Side

    private CommandGroup CG_SideLine() {
    	CommandGroup group = new CommandGroup();
    	
    	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
		group.addSequential(new C_DriveShiftDown());
		group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			(int) Math.round(Constants.ENC_TO_INCH * (120 - Constants.ROBOT_LENGTH - 6 + 36)),
    			0.0, 0.0, 0, false, false, false, true, true, Constants.TIMEOUT_DRIVE)); // TODO: set posPID to true
    	
    	return group;
    }
    
    private CommandGroup CG_SideSwitch(boolean startLeft) {
    	CommandGroup group = new CommandGroup();
   	  
		group.addSequential(new C_WristZero()); 
		group.addSequential(new C_DriveShiftDown());
		
		group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_FAST, 
    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.CR_FROM_REAR)), // TODO: C_TO_R
    			0.0, 0.0, 0, false, false, true, false, true, Constants.TIMEOUT_DRIVE));
    	
		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SWITCH, WristPresets.SWITCH)); 
		group.addParallel(new C_CollectorDrive(true, 1, 0));
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.DRIVER_TO_NEAR_SWITCH + (Constants.SWITCH_WIDTH / 2) - 2 - Constants.CENTER_TO_REAR)), // TODO: "2" is a magic num
    			0, false, true, true, Constants.TIMEOUT_DRIVE));
    			
		if (startLeft) {
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -90, false, 0.0, false, true, Constants.TIMEOUT_TURN));
			
	    	group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (18)), // robot should be ~5 inches from switch
	    			-90, true, true, true, Constants.TIMEOUT_DRIVE));
			
			group.addSequential(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SWITCH_SPEED));
			
			group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (18)),
	    			-90, false, true, true, Constants.TIMEOUT_DRIVE));
			
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0, false, 0.0, false, true, Constants.TIMEOUT_TURN));
			
		} else {
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 90, false, 0.0, false, true, Constants.TIMEOUT_TURN));
			
	    	group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (18)), // robot should be ~5 inches from switch
	    			90, true, true, true, Constants.TIMEOUT_DRIVE));
			
			group.addSequential(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SWITCH_SPEED));
			
			group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (18)),
	    			0, false, true, true, Constants.TIMEOUT_DRIVE));
			
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0, false, 0.0, false, true, Constants.TIMEOUT_TURN));
		}
		
		return group;
    }
    
    private CommandGroup CG_SideScaleBackboard(boolean startLeft) {
       	CommandGroup group = new CommandGroup();

       	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
		group.addSequential(new C_DriveTrainPID(0, 
    			0,
    			0.0, 0.0, 0, false, false, true, false, true, 0)); 
       	
       	if (startLeft) {
       		group.addSequential(new C_DriveTrainPID(S_FAST, -15, true, 90, false, true, Constants.TIMEOUT_DRIVE));
       		group.addSequential(new C_DriveTrainPID(S_FAST, -5, true, -90, false, true, Constants.TIMEOUT_DRIVE));
       	} else {
			group.addSequential(new C_DriveTrainPID(S_FAST, 15, true, -90, false, true, Constants.TIMEOUT_DRIVE));
       		group.addSequential(new C_DriveTrainPID(S_FAST, 5, true, 90, false, true, Constants.TIMEOUT_DRIVE));        		
       	}
       	
		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SCALE, WristPresets.SCALE_BACKWARD_HIGH));
		group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0, false, 0.0, false, true, Constants.TIMEOUT_TURN));
		
       	group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_FAST, 
    			(int) Math.round(Constants.ENC_TO_INCH * (200)),
    			0, false, true, true, Constants.TIMEOUT_DRIVE));
		
       	if (startLeft) {
       		group.addSequential(new C_DriveTrainPID(700, 90, false, 0.0, false, true, Constants.TIMEOUT_TURN));
			
			group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_SLOW, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (28)),
	    			90, false, true, true, Constants.TIMEOUT_DRIVE));
       	} else {
			group.addSequential(new C_DriveTrainPID(700, -90, false, 0.0, false, true, Constants.TIMEOUT_TURN));
			
			group.addSequential(new C_ZeroDriveEncoders());
			group.addSequential(new C_DriveTrainPID(D_SLOW, 
	    			(int) Math.round(Constants.ENC_TO_INCH * (28)),
	    			-90, false, true, true, Constants.TIMEOUT_DRIVE));
       	}
		
		group.addSequential(new WaitCommand(1));
		group.addParallel(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SCALE_SPEED));
		
		group.addSequential(new WaitCommand(0.5));
		group.addParallel(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));
		if (startLeft) {
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 180, false, 0.0, false, true, Constants.TIMEOUT_TURN));
		} else {
			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -180, false, 0.0, false, true, Constants.TIMEOUT_TURN));
		}
       	return group;
    }
    
    private CommandGroup CG_SideTwoCubeScaleSwitch(boolean startLeft, boolean twoInScale) {
       	CommandGroup group = new CommandGroup();
    	    		
       	group.addSequential(new C_WristZero());
       	group.addSequential(new C_DriveShiftDown());
       	
	   	group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.CENTER_TO_REAR)),
    			0.0, 0.0, 0, false, false, true, false, true, Constants.TIMEOUT_DRIVE));  
		
		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SCALE, WristPresets.SCALE_BACKWARD_HIGH));
		group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    			(int) Math.round(Constants.ENC_TO_INCH * (Constants.DRIVER_TO_FAR_SWITCH + Constants.CUBE_WIDTH - Constants.CENTER_TO_REAR - 4)), // - 10
    			0, false, true, false, Constants.TIMEOUT_DRIVE));
       	
    		if (startLeft) {
    			group.addSequential(new C_DriveTrainPID(S_CRUISE, 34, true, -72, false, true, Constants.TIMEOUT_DRIVE));
    			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 30, false, 0.0, false, true, Constants.TIMEOUT_TURN));
    			
        		group.addParallel(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_COLLECT));
        		group.addSequential(new WaitCommand(0.5));
        		
        		group.addParallel(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));
        		group.addParallel(new C_CollectorDrive(true, 8, Constants.COL_SCALE_SPEED));
    			group.addSequential(new C_DriveTrainPID(800, -27, true, -40, true, true, Constants.TIMEOUT_DRIVE));
    			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -40, false, 0.0, true, true, Constants.TIMEOUT_TURN));
    			
    			group.addSequential(new WaitCommand(1));
    	    	group.addSequential(new C_ZeroDriveEncoders());
    			group.addSequential(new C_DriveTrainPID(D_SLOW,
    	    			(int) Math.round(Constants.ENC_TO_INCH * (11)),
    	    			-36, true, true, true, 2.5));
    			
    			if (twoInScale) {
	    			group.addParallel(pcb.Build(ArmPresets.COLLECT, ArmPresets.SCALE, WristPresets.SCALE_BACKWARD));
	    			group.addSequential(new C_DriveTrainPID(S_CRUISE, 25, true, -30, false, true, Constants.TIMEOUT_DRIVE));
	    			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 30, false, 0.0, false, true, Constants.TIMEOUT_TURN));
	    			
	    			group.addSequential(new C_ZeroDriveEncoders());
	    			group.addSequential(new C_DriveTrainPID(D_CRUISE,
	    	    			(int) Math.round(Constants.ENC_TO_INCH * (4)),
	    	    			30, false, true, true, 2.5));
	    			
    			} else {
    				group.addSequential(new C_ZeroDriveEncoders());
    				group.addParallel(pcb.Build(ArmPresets.COLLECT, ArmPresets.SWITCH, WristPresets.SWITCH));
        			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
        	    			(int) Math.round(Constants.ENC_TO_INCH * (14)),
        	    			-30, false, true, true, 2.5));
        			
        			// TODO: need to cross switch vertical plane, arm is too low
    				group.addSequential(new C_ZeroDriveEncoders());
        			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
        	    			(int) Math.round(Constants.ENC_TO_INCH * (20)),
        	    			-25, true, true, true, 2.5));
    			}
    			
        		group.addSequential(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, 0.5));
        		if (twoInScale) {
        			group.addSequential(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));
        		}
    		} else {
    			group.addSequential(new C_DriveTrainPID(S_CRUISE, -34, true, 72, false, true, Constants.TIMEOUT_DRIVE));        		
    			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -30, false, 0.0, false, true, Constants.TIMEOUT_TURN));
    			
        		group.addParallel(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, Constants.COL_SCALE_SPEED));
        		group.addSequential(new WaitCommand(0.5)); // 0.5
        		
        		group.addParallel(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));
        		group.addParallel(new C_CollectorDrive(true, 8, Constants.COL_SCALE_SPEED));
    			group.addSequential(new C_DriveTrainPID(S_SLOW, 27, true, 40, true, true, Constants.TIMEOUT_DRIVE)); // was 1200 and 42
    			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 50, false, 0.0, true, true, Constants.TIMEOUT_TURN));
    			
    			group.addSequential(new WaitCommand(1));
    	    	group.addSequential(new C_ZeroDriveEncoders());
    			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
    	    			(int) Math.round(Constants.ENC_TO_INCH * (14)),
    	    			36, true, true, true, 2.5));
    			
    			if (twoInScale) {
	    			group.addParallel(pcb.Build(ArmPresets.COLLECT, ArmPresets.SCALE, WristPresets.SCALE_BACKWARD));
	    			group.addSequential(new C_DriveTrainPID(S_CRUISE, -25, true, 30, false, true, Constants.TIMEOUT_DRIVE));
	    			group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, -30, false, 0.0, false, true, Constants.TIMEOUT_TURN));
    			} else {
    				group.addSequential(new C_ZeroDriveEncoders());
    				group.addParallel(pcb.Build(ArmPresets.COLLECT, ArmPresets.SWITCH, WristPresets.SWITCH));
        			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
        	    			(int) Math.round(Constants.ENC_TO_INCH * (14)),
        	    			30, false, true, true, 2.5));
        			
        			// TODO: need to cross switch vertical plane, arm is too low
    				group.addSequential(new C_ZeroDriveEncoders());
        			group.addSequential(new C_DriveTrainPID(D_CRUISE, 
        	    			(int) Math.round(Constants.ENC_TO_INCH * (20)),
        	    			25, true, true, true, 2.5));
    			}
    			
        		group.addSequential(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, 0.5));
        		if (twoInScale) {
        			group.addSequential(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));
        		}
    		}
    		    		    		    		
    		return group;
    }

    private CommandGroup CG_SideCrossFieldScale(boolean startLeft) {
    	CommandGroup group = new CommandGroup();
    	
    	final int SIDE_FACTOR = (startLeft) ? 1 : -1;

    	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
	   	group.addSequential(new C_ZeroDriveEncoders());
	   	group.addSequential(new C_DriveTrainPID(D_FAST, // TODO: may want to start slow
    			(int) Math.round(Constants.ENC_TO_INCH * (210)),
    			0.0, 0.0, 0, false, false, true, false, true, Constants.TIMEOUT_DRIVE));
	   	
	   	group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, SIDE_FACTOR * 90, false, 0.0, false, true, Constants.TIMEOUT_TURN));

		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SCALE, WristPresets.SCALE_BACKWARD_HIGH));
		group.addSequential(new C_ZeroDriveEncoders());
		
		group.addSequential(new C_DriveTrainPID(D_FAST, 
    			(int) Math.round(Constants.ENC_TO_INCH * (188)),
    			SIDE_FACTOR * 90, false, true, true, Constants.TIMEOUT_DRIVE));
		
		group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0.0, false, 0.0, false, true, Constants.TIMEOUT_TURN));

		group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(1200, 
    			(int) Math.round(Constants.ENC_TO_INCH * (34 + 10)),
    			0, false, true, true, Constants.TIMEOUT_DRIVE));
		
		group.addSequential(new WaitCommand(0.5));
		group.addSequential(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, 0.5));
		
		group.addSequential(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));

    	return group;
    }
    
    private CommandGroup CG_SideCrossFieldScaleSplines(boolean startLeft) {
    	CommandGroup group = new CommandGroup();
    	
    	final int SIDE_FACTOR = (startLeft) ? 1 : -1;

    	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
	   	group.addSequential(new C_ZeroDriveEncoders());
	   	group.addSequential(new C_DriveTrainPID(D_FAST, // TODO: may want to start slow
    			(int) Math.round(Constants.ENC_TO_INCH * (170)), // was 210
    			0.0, 0.0, 0, false, false, true, false, true, Constants.TIMEOUT_DRIVE));
	   	
	   	//group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, SIDE_FACTOR * 90, false, 0.0, false, true, Constants.TIMEOUT_TURN));
		group.addSequential(new C_DriveTrainPID(S_CRUISE, 34 * SIDE_FACTOR, true, 72 * -SIDE_FACTOR, false, true, Constants.TIMEOUT_DRIVE));
	   	
		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SCALE, WristPresets.SCALE_BACKWARD_HIGH));
		group.addSequential(new C_ZeroDriveEncoders());
		
		group.addSequential(new C_DriveTrainPID(D_FAST, 
    			(int) Math.round(Constants.ENC_TO_INCH * (188)),
    			SIDE_FACTOR * 90, false, true, true, Constants.TIMEOUT_DRIVE));
		
		//group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, 0.0, false, 0.0, false, true, Constants.TIMEOUT_TURN));
		group.addSequential(new C_DriveTrainPID(S_CRUISE, 0 * SIDE_FACTOR, true, 72 * -SIDE_FACTOR, false, true, Constants.TIMEOUT_DRIVE));
		
		group.addSequential(new C_ZeroDriveEncoders());
		group.addSequential(new C_DriveTrainPID(1200, 
    			(int) Math.round(Constants.ENC_TO_INCH * (12)), // was 34 + 10
    			0, false, true, true, Constants.TIMEOUT_DRIVE));
		
		group.addSequential(new WaitCommand(0.5));
		group.addSequential(new C_CollectorDrive(false, Constants.TIMEOUT_SHOOT, 0.5));
		
		group.addSequential(pcb.Build(ArmPresets.SCALE, ArmPresets.COLLECT, WristPresets.COLLECT));

    	return group;
    }
    
    private CommandGroup CG_SideCrossHalfField(boolean startLeft) {
    	CommandGroup group = new CommandGroup();
    	
    	final int SIDE_FACTOR = (startLeft) ? 1 : -1;

    	group.addSequential(new C_WristZero());
		group.addSequential(new C_DriveShiftDown());
		
	   	group.addSequential(new C_ZeroDriveEncoders());
	   	group.addSequential(new C_DriveTrainPID(D_FAST, // TODO: may want to start slow
    			(int) Math.round(Constants.ENC_TO_INCH * (210)),
    			0.0, 0.0, 0, false, false, true, false, true, Constants.TIMEOUT_DRIVE));
	   	
	   	group.addSequential(new C_DriveTrainPID(TURNING_VELOCITY, SIDE_FACTOR * 90, false, 0.0, false, true, Constants.TIMEOUT_TURN));

		group.addParallel(pcb.Build(Robot.ss_RobotArm.getLastArmPreset(), ArmPresets.SCALE, WristPresets.SCALE_BACKWARD_HIGH));
		group.addSequential(new C_ZeroDriveEncoders());
		
		group.addSequential(new C_DriveTrainPID(D_FAST, 
    			(int) Math.round(Constants.ENC_TO_INCH * (94)),
    			SIDE_FACTOR * 90, false, true, true, Constants.TIMEOUT_DRIVE));
		
		return group;
    }
}