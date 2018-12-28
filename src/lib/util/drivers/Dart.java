package lib.util.drivers;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team4911.robot.RobotMap.ArmPresets;

public abstract class Dart {

	private static final Map<ArmPresets, Integer> presetAngles = 
			Collections.unmodifiableMap(new HashMap<ArmPresets, Integer>() {
				private static final long serialVersionUID = -175660384470796292L;
				{
					put(ArmPresets.COLLECT, -50);
					put(ArmPresets.SWITCH, -33);
					put(ArmPresets.SCALE, 65);
				}
			});

	// Max velocity in potentiometer ticks per 100 ms
	protected int vcruise;

	// DART related potentiometer measurements
	protected int dartBottom;
	protected int dartTop;
	protected int dartSwitch;

	protected double rangeInInches;
	
	// Angle to position: position = Angle * kDart + kDartYOffset
	protected double kSlope;
	protected double yOffest;

	protected double kFAdjustment = 0.80;
	protected double kP = 0.0;
	protected double kI = 0.0;
	protected double kD = 0.0;
	protected int kIntegralZone = 0;
	protected int kClosedloopError;
	
	/**
	 * Gets the cruise velocity / 100ms expressed in potentiometer ticks
	 * 
	 * @return cruise velocity (ticks / 100 ms)
	 */
	public int getVcruise() {
		return this.vcruise;
	}	

	/**
	 * Gets the potentiometer reading for the top position
	 * 
	 * @return top in potentiometer ticks
	 */
	public int getTop() {
		return this.dartTop;
	}

	/**
	 * Gets the potentiometer reading for the bottom position
	 * 
	 * @return bottom in potentiometer ticks
	 */
	public int getBottom() {
		return this.dartBottom;
	}

	/**
	 * Gets the potentiometer reading for the bottom position
	 * 
	 * @return bottom in potentiometer ticks
	 */
	public int getSwitch() {
		return this.dartSwitch;
	}
	
	/**
	 * Gets the potentiometer range 
	 * 
	 * @return bottom in potentiometer ticks
	 */
	public int getRange() {
		return this.dartTop - this.dartBottom;
	}
	
	public double getTicksPerInch() {
		return (this.dartTop - this.dartBottom) / this.rangeInInches;
	}
	
	
	public double getkFAdjustment() {
		return this.kFAdjustment;
	}
	
	public double getkP() {
		return this.kP;
	}
	
	public double getkI() {
		return this.kI;
	}
	
	public double getkD() {
		return this.kD;
	}
	
	public int getkIntegralZone() {
		return this.kIntegralZone;
	}
	
	public int getkClosedloopError() {
		return this.kClosedloopError;
	}
	
	/**
	 * Gets the angle for a given preset
	 * 
	 * @param preset the preset to lookup
	 * @return the angle corresponding to the preset
	 */
	public static int getAngleForPreset(ArmPresets preset) {
		int angle = 0;
		if (presetAngles.containsKey(preset)) {
			angle = presetAngles.get(preset).intValue();
		}

		return Math.max(-52, Math.min(60, angle));
	}

	/**
	 * Gets the potentiometer position for a given angle
	 * 
	 * @param armPreset the preset to lookup
	 * @return the angle corresponding to the preset
	 */
	public double getPositionForAngle(double angle) {
		return Math.max(dartBottom, Math.min(dartTop, kSlope * angle + yOffest));
	}

	/**
	 * Gets the angle for a given potentiometer reading
	 * 
	 * @param armPreset the preset to lookup
	 * @return the angle corresponding to the preset
	 */
	public double getAngleforPosition(int position) {
		return (position - yOffest) / kSlope;
	}
}