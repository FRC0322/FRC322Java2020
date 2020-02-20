package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 *
 * @author Dan Waxman
 * @author Corey Applegate
 *
 * Certain methods
 * @author Gabriel McMillan
 *
 * Modified for WPIlib 2020
 * @author Raa'Shaun Hunter
 */
public class Limelight {
	private static NetworkTableInstance table = null;

	/**
	 * Light modes for Limelight.
	 *
	 * @author Corey Applegate
	 */
	public enum LightMode {
		kpipeLine(0),	//0	use the LED Mode set in the current pipeline
		kforceOff(1),	//1	force off
		kforceBlink(2),	//2	force blink
		kforceOn(3);	//3	force on

		private static final Map<Integer, LightMode> MY_MAP = new HashMap<Integer, LightMode>();

		static {
			for (LightMode LedMode : values()) {
				MY_MAP.put(LedMode.getValue(), LedMode);
			}
		}

		private int value;

		private LightMode(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static LightMode getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}
	}

	/**
	 * Camera modes for Limelight.
	 *
	 * @author Corey Applegate
	 */
	public enum CameraMode {
		kvision(0),
		kdriver(1);

		private static final Map<Integer, CameraMode> MY_MAP = new HashMap<Integer, CameraMode>();

		static {
			for (CameraMode CameraMode : values()) {
				MY_MAP.put(CameraMode.getValue(), CameraMode);
			}
		}

		private int value;

		private CameraMode(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static CameraMode getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}
	}

	/**
	 * Stream modes for Limelight.
	 *
	 * @author Corey Applegate
	 */
	public enum StreamType {
		kStandard(0),
		kPiPMain(1),
		kPiPSecondary(2);

		private static final Map<Integer,  StreamType> MY_MAP = new HashMap<Integer,  StreamType>();

		static {
			for ( StreamType StreamType : values()) {
				MY_MAP.put( StreamType.getValue(),  StreamType);
			}
		}

		private int value;

		private StreamType(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static StreamType getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}

	}

	/**
	 * Snapshot modes for Limelight.
	 *
	 * @author Corey Applegate
	 */
	public enum Snapshot {

		kon(1),
		koff(0);

		private static final Map<Integer,  Snapshot> MY_MAP = new HashMap<Integer,  Snapshot>();

		static {
			for ( Snapshot Snapshot : values()) {
				MY_MAP.put( Snapshot.getValue(),  Snapshot);
			}
		}

		private int value;

		private Snapshot(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static Snapshot getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}
	}

	/**
	 * Advanced Target modes for Limelight.
	 *
	 * @author Corey Applegate
	 */
	public enum  Advanced_Target {

		kone(0),
		ktwo(1),
		kthree(2);

		private static final Map<Integer,  Advanced_Target> MY_MAP = new HashMap<Integer,  Advanced_Target>();

		static {
			for ( Advanced_Target Advanced_Target : values()) {
				MY_MAP.put( Advanced_Target.getValue(),  Advanced_Target);
			}
		}

		private int value;

		private Advanced_Target(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static Advanced_Target getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}

	}

	/**
	 * Advanced Crosshair modes for Limelight.
	 *
	 * @author Corey Applegate
	 */
	public enum Advanced_Crosshair {

		kone(0),
		ktwo(1);

		private static final Map<Integer,  Advanced_Crosshair> MY_MAP = new HashMap<Integer,  Advanced_Crosshair>();

		static {
			for ( Advanced_Crosshair Advanced_Crosshair : values()) {
				MY_MAP.put( Advanced_Crosshair.getValue(),  Advanced_Crosshair);
			}
		}

		private int value;

		private Advanced_Crosshair(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

		public static Advanced_Crosshair getByValue(int value) {
			return MY_MAP.get(value);
		}

		public String toString() {
			return name();
		}

	}

	/**
	 * Gets whether a target is detected by the Limelight.
	 *
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean isTarget() {
		return getValue("tv").getDouble(0) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 *
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(0.00);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 *
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.00);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 *
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.00);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 *
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.00);
	}

	/**
	 * Gets target latency (ms).
	 *
	 * @return Target latency.
	 */
	public double getTl() {
		return getValue("tl").getDouble(0.00);
	}

	/**
	 * Sets LED mode of Limelight.
	 *
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 *
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 *
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	/**
	 * getCameraTranslation() - Results of a 3D position solution
	 *
	 * @return 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)
	 * @author Gabriel McMillan
	 */
	public double getCameraTranslation() {
		return getValue("camtran").getDouble(0);
	}

	/**
	 * get() - monitor any value needed outside of currently provided.
	 *
	 * @param key to pull
	 * @return value of key
	 *
	 * @author Gabriel McMillan
	 */
	public double get(String entry) {
		return getValue(entry).getDouble(0);
	}

	/**
	 * set() - Set any value outside what is currently provided with the Limelight
	 *
	 * @return False if the table key already exists with a different type
	 * @param key to set, and value to set.
	 *
	 * @author Gabriel McMillan
	 */
	public boolean set(String entry, Double value) {
		return getValue(entry).setNumber(value);
	}

	/**
	 * getDist() - calculates approximate distance from a fixed angled limelight to
	 * the target.
	 *
	 * @param targetHeight= target height in meters, limelightHeight = height of limelight from the ground in meters, limelightAngle =
	 *           angle in degrees of the limelight on the robot.
	 * @return approx distance in meters
	 *
	 * @author Gabriel McMillan
	 */
	public double getDist(double targetHeight, double limelightHeight, double limelightAngle) {
		double a2 = getTy();
		double currentDist = (Math.abs(targetHeight - limelightHeight) / Math.tan(limelightAngle + a2));
		return currentDist;
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 *
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}

		return table.getTable("limelight").getEntry(key);
	}
}
