package coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;

/**
 * The {@link Mission} data structure represents a goal for a robot, to be reached via a given
 * path connecting two location poses. 
 * 
 * @author fpa
 *
 */
public class Mission implements Comparable<Mission> {
	private static int NUMMISSIONS = 0;
	private int robotID;
	private String path;
	private int order = NUMMISSIONS++;
	private String fromLocation = null;
	private String toLocation = null;
	private Pose fromPose = null;
	private Pose toPose = null;
	
	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * @param robotID The ID of the robot.
	 * @param path A pointer to a file containing the path to be driven.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 */
	public Mission(int robotID, String path, String fromLocation, String toLocation) {
		this(robotID, path, fromLocation, toLocation, null, null);
	}

	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param path A pointer to a file containing the path to be driven.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public Mission(int robotID, String path, String fromLocation, String toLocation, Pose fromPose, Pose toPose) {
		this.robotID = robotID;
		this.path = path;
		this.fromLocation = fromLocation;
		this.toLocation = toLocation;
		this.fromPose = fromPose;
		this.toPose = toPose;
	}
	
	@Override
	public int compareTo(Mission o) {
		return this.order-o.order;
	}
	
	/**
	 * Get the ID of the robot driving this {@link Mission}.
	 * @return The ID of the robot driving this {@link Mission}.
	 */
	public int getRobotID() {
		return this.robotID;
	}
	
	/**
	 * Get the name of the file containing the path of this {@link Mission}.
	 * @return The name of the file containing the path of this {@link Mission}.
	 */
	public String getPath() {
		return this.path;
	}
	
	/**
	 * Get the name of the source location of this {@link Mission}.
	 * @return The name of the source location of this {@link Mission}.
	 */
	public String getFromLocation() {
		return fromLocation;
	}
	
	/**
	 * Get the name of the destination location of this {@link Mission}.
	 * @return The name of the destination location of this {@link Mission}.
	 */
	public String getToLocation() {
		return toLocation;
	}

	@Override
	public String toString() {
		return "Robot" + this.getRobotID() + ": " + fromLocation + " --> " + toLocation + "(path: " + path + ")";
	}

	/**
	 * Get the {@link Pose} of source location of this {@link Mission}.
	 * @return The {@link Pose} of source location of this {@link Mission}.
	 */
	public Pose getFromPose() {
		return fromPose;
	}

	/**
	 * Set the {@link Pose} of source location of this {@link Mission}.
	 * @param fromPose The {@link Pose} of source location of this {@link Mission}.
	 */
	public void setFromPose(Pose fromPose) {
		this.fromPose = fromPose;
	}


	/**
	 * Get the {@link Pose} of destination location of this {@link Mission}.
	 * @return The {@link Pose} of destination location of this {@link Mission}.
	 */
	public Pose getToPose() {
		return toPose;
	}

	/**
	 * Get the {@link Pose} of destination location of this {@link Mission}.
	 * @param toPose The {@link Pose} of destination location of this {@link Mission}.
	 */
	public void setToPose(Pose toPose) {
		this.toPose = toPose;
	}
}
