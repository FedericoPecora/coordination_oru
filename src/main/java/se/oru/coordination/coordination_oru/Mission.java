package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.util.Missions;

/**
 * The {@link Mission} data structure represents a goal for a robot, to be reached via a given
 * path connecting two location poses. 
 * 
 * @author fpa
 *
 */

public class Mission extends TaskData implements Comparable<Mission> {
	protected static int NUMMISSIONS = 0;
	protected int robotID;
	protected PoseSteering[] path;
	protected int order = NUMMISSIONS++;
		
	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations, but where the path
	 * is not given and should be computed subsequently (before adding the mission to the {@link TrajectoryEnvelopeCoordinator}).
	 * @param robotID The ID of the robot.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public Mission(int robotID, String fromLocation, String toLocation, Pose fromPose, Pose toPose) {
		super(fromLocation, toLocation, fromPose, toPose);	
		this.robotID = robotID;
		this.path = null;
			
	}

	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param path An array of {@link PoseSteering}s representing the path to be driven.
	 */
	public Mission(int robotID, PoseSteering[] path) {
		this(robotID, path, path[0].getPose().toString(), path[path.length-1].getPose().toString(), path[0].getPose(), path[path.length-1].getPose());
	}
	
	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param fromLocation The identifier of the source location.
	 * @param path An array of {@link PoseSteering}s representing the path to be driven.
	 */
	public Mission(int robotID, String fromLocation, String toLocation, PoseSteering[] path) {
		this(robotID, path, fromLocation, toLocation, path[0].getPose(), path[path.length-1].getPose());
	}

	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param path An array of {@link PoseSteering}s representing the path to be driven.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public Mission(int robotID, PoseSteering[] path, String fromLocation, String toLocation, Pose fromPose, Pose toPose) {
		super(fromLocation, toLocation, fromPose, toPose);
		this.robotID = robotID;
		this.path = path;
	}

	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param pathFile A pointer to a file containing the path to be driven.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public Mission(int robotID, String pathFile, Pose fromPose, Pose toPose) {
		this(robotID, pathFile, null, null, fromPose, toPose);
	}
		
	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param pathFile A pointer to a file containing the path to be driven.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public Mission(int robotID, String pathFile, String fromLocation, String toLocation, Pose fromPose, Pose toPose) {
		super(fromLocation, toLocation, fromPose, toPose);
		this.robotID = robotID;
		this.path = Missions.loadPathFromFile(pathFile);
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
	 * Set the ID of the robot driving this {@link Mission}.
	 * @return The ID of the robot driving this {@link Mission}.
	 */
	public void setRobotID(int robotID) {
		this.robotID = robotID;
	}
	
	/**
	 * Get the array of {@link PoseSteering}s representing the path to be driven.
	 * @return The array of {@link PoseSteering}s representing the path to be driven.
	 */
	public PoseSteering[] getPath() {
		return this.path;
	}
	
	/**
	 * Set the path of this mission.
	 * @param path The path of this mission.
	 */
	public void setPath(PoseSteering[] path) {
		this.path = path;
	}
		
	@Override
	public String toString() {
		return "Robot" + this.getRobotID() + ": " + fromLocation + " --> " + toLocation + (path != null ? " (path length: " + path.length + ")" : "");
	}
}
