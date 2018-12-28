package lib.util.drivers;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class PixelDart extends Dart {

	private static final int kDartBottom = 90; // 105; // Where limit switch engages
	private static final int kDartSwitch = 215;	// At -33 degrees and wrist gear should clear 20"
	private static final int kDartTop = 922;	// Where limit switch engages
	
	private static final Map<Integer, Integer> presetAnglesToPosition = Collections
			.unmodifiableMap(new HashMap<Integer, Integer>() {
				private static final long serialVersionUID = 8066979222837770706L;

				{
					put(-52, kDartBottom);	// Position at Collect
					put(-33, kDartSwitch);	// Position at Switch
					put( 65, kDartTop);	// Position at Scale
				}
			});

	public PixelDart() {
		super();
		this.dartBottom = kDartBottom; 
		this.dartSwitch = kDartSwitch;
		this.dartTop = kDartTop;

		// 2:1 DART
//		this.dartBottom = 82; 	// Where limit switch engages
//		this.dartSwitch = 200;  // At -33 degrees and wrist gear should clear 20"
//		this.dartTop = 913;		// Where limit switch engages
		
		this.kSlope = 7.4798;
		this.yOffest = 487.41;
		this.rangeInInches = 11.5;
		
		// Motion magic constants
		this.vcruise = 48;
		this.kFAdjustment = 0.95;
		this.kP = 12.0;
		this.kI = 0.002;
		this.kD = 0.4;
		this.kIntegralZone = 12;
		this.kClosedloopError = 3;
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

		return super.getPositionForAngle(angle);
	}
}
