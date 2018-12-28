package lib.util.drivers;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class VoxelDart extends Dart {
	private static final int kDartBottom = 93;//71; //96; //65;	// Where limit switch engages
	private static final int kDartSwitch = 209; //187;	// At -33 degrees and wrist gear should clear 20"
	private static final int kDartTop = 924; //923;	// Where limit switch engages

	private static final Map<Integer, Integer> presetAnglesToPosition = Collections
			.unmodifiableMap(new HashMap<Integer, Integer>() {
				private static final long serialVersionUID = 6148936802672794089L;
				{
					put(-50, kDartBottom);	// Position at Collect
					put(-33, kDartSwitch);	// Position at Switch
					put(67, kDartTop); 	// Position at Scale
				}
			});

	public VoxelDart() {
		super();

		this.dartBottom = kDartBottom;
		this.dartSwitch = kDartSwitch;
		this.dartTop = kDartTop;
		
		this.kSlope = 7.0584;
		this.yOffest = 435.96;
		this.rangeInInches = 11.75;

		// Motion magic constants
		this.vcruise = 42;
		this.kFAdjustment = 0.90;
		this.kP = 6.576429;
		this.kI = 0.002;
		this.kD = 65.76429;
		this.kIntegralZone = 60;
		this.kClosedloopError = 5;
	}

	/**
	 * Gets the potentiometer position for a given angle
	 * 
	 * @param preset
	 *            the preset to lookup
	 * @return the angle corresponding to the preset
	 */
	@Override
	public double getPositionForAngle(double angle) {
		if (presetAnglesToPosition.containsKey((int) angle)) {
			return presetAnglesToPosition.get((int) angle).intValue();
		}

		// (-0.0003*POWER(A7,3))-(0.0013*POWER(A7,2))+(8.85179*A7)+451.820
		double position = -0.0003 * Math.pow(angle, 3) - 0.0013 * angle * angle + 8.85179 * angle + 451.82;
		return Math.max(dartBottom, Math.min(dartTop, position));
	}

	@Override
	public double getAngleforPosition(int position) {
		// (-0.0003*POWER(A7,3))-(0.0013*POWER(A7,2))+(8.85179*A7)+451.820
		double angle = -0.0000001 * Math.pow(position, 3) - 0.0001 * position * position + 0.1725 * position - 60.424;
		return angle;
	}
}
