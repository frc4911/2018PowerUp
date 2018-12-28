package lib.util.drivers;

public class DartFactory {

	public static Dart getDart(String robotName) throws IllegalArgumentException { 
		switch (robotName) {
			case "pixel" : 
				return new PixelDart();
			case "voxel" :
				return new VoxelDart();
			default:
				throw new IllegalArgumentException ("Invalid robotName [" + robotName +"]");
		}
	}
}
