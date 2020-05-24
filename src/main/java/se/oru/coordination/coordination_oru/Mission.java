package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.util.Missions;

/**
 * The {@link Mission} data structure represents a goal for a robot, to be reached via a given
 * path connecting two location poses. 
 * 
 * @author fpa
 *
 */
public class Mission implements Comparable<Mission> {
	protected static int NUMMISSIONS = 0;
	protected int robotID;
	protected PoseSteering[] path;
	protected int order = NUMMISSIONS++;
	protected String fromLocation = null;
	protected String toLocation = null;
	protected Pose fromPose = null;
	protected Pose toPose = null;
	protected ArrayList<Pose> stoppingPoints = new ArrayList<Pose>();
	protected ArrayList<Integer> stoppingPointDurations = new ArrayList<Integer>();
	//protected HashMap<Pose,Integer> stoppingPoints = new HashMap<Pose, Integer>();
	
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
		this.robotID = robotID;
		this.path = null;
		this.fromLocation = fromLocation;
		this.toLocation = toLocation;
		this.fromPose = fromPose;
		this.toPose = toPose;		
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
	 * Make the robot stop at the nearest location to a given pose for a given duration.
	 * @param pose A pose to stop at.
	 * @param duration Stopping time in milliseconds. 
	 */
	public void setStoppingPoint(Pose pose, int duration) {
		this.stoppingPoints.add(pose);
		this.stoppingPointDurations.add(duration);
		//this.stoppingPoints.put(pose, duration);
	}
	
	/**
	 * Clear the stopping points of this {@link Mission}.
	 */
	public void clearStoppingPoints() {
		this.stoppingPoints.clear();
		this.stoppingPointDurations.clear();
	}
	
	/**
	 * Get the stopping points along this {@link Mission}'s trajectory along with their durations.
	 * @return The stopping points along this {@link Mission}'s trajectory along with their durations.
	 */
	public HashMap<Pose,Integer> getStoppingPoints() {
		HashMap<Pose,Integer> ret = new HashMap<Pose, Integer>();
		for (int i = 0; i < this.stoppingPoints.size(); i++) {
			ret.put(this.stoppingPoints.get(i), this.stoppingPointDurations.get(i));
		}
		return ret;
		//return this.stoppingPoints;
	}
	
	/**
	 * Set the destination location for this {@link Mission}.
	 * @param location The destination of this {@link Mission}.
	 */
	public void setToLocation(String location) {
		this.toLocation = location;
	}

	/**
	 * Set the source location for this {@link Mission}.
	 * @param location The source of this {@link Mission}.
	 */
	public void setFromLocation(String location) {
		this.fromLocation = location;
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
		this.robotID = robotID;
		this.path = path;
		this.fromLocation = fromLocation;
		this.toLocation = toLocation;
		this.fromPose = fromPose;
		this.toPose = toPose;
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
		this.robotID = robotID;
		this.path = Missions.loadPathFromFile(pathFile);
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
		return "Robot" + this.getRobotID() + ": " + fromLocation + " --> " + toLocation + (path != null ? " (path length: " + path.length + ")" : "");
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
