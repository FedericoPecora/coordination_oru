package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.util.Missions;

/**
 * The {@link Mission} data structure represents a goal for a robot, to be reached via a given
 * path connecting two location poses. A mission may also require the robot to yield along the path in the so called ``stopping points''. 
 * 
 * @author fpa, anmi
 *
 */

public class Mission implements Comparable<Mission> {
	protected static int NUMMISSIONS = 0;
	protected int missionID = NUMMISSIONS++;
	protected int robotID = -1;
	protected PoseSteering[] path = null;
	protected String fromLocation = null;
	protected String toLocation = null;
	protected Pose fromPose = null;
	protected Pose toPose = null;
	protected ArrayList<Pose> stoppingPoints = null;
	protected ArrayList<Long> stoppingPointDurations = null;
		
	
	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations, but where the path
	 * is not given and should be computed subsequently (before adding the mission to the {@link TrajectoryEnvelopeCoordinator}).
	 * @param robotID The ID of the robot.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 * @param stoppingPoints Poses where the robot should stop for a given duration.
	 * @param stoppingPointDurations Durations of the stopping points (in millis).
	 * @param path An array of {@link PoseSteering}s representing the path to be driven.
	 */
	public Mission(int robotID, String fromLocation, String toLocation, Pose fromPose, Pose toPose, ArrayList<Pose> stoppingPoints, ArrayList<Long> stoppingPointDurations, PoseSteering[] path) {
		this.robotID = robotID;
		this.fromLocation = fromLocation;
		this.toLocation = toLocation;
		this.fromPose = fromPose;
		this.toPose = toPose;	
		this.stoppingPoints = new ArrayList<Pose>(stoppingPoints);
		this.stoppingPointDurations = new ArrayList<Long>(stoppingPointDurations);
		this.path = path.clone();
		if (path != null && (path.length < 2 || !fromPose.equals(path[0].getPose()) || !toPose.equals(path[path.length-1].getPose()))) {
			System.out.println("Mission is not well defined. Class members have incoherent values.");
			throw new Error("Mission is not well defined. Class members have incoherent values.");
		}
	}
	
	/**
	 * Instantiates a {@link Mission} for a given robot to navigate between two locations via a given path.
	 * 
	 * @param robotID The ID of the robot.
	 * @param path An array of {@link PoseSteering}s representing the path to be driven.
	 */
	public Mission(int robotID, PoseSteering[] path) {
		this(robotID, path[0].getPose().toString(), path[path.length-1].getPose().toString(), path[0].getPose(), path[path.length-1].getPose(), new ArrayList<Pose>(), new ArrayList<Long>(), path);
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
		this(robotID, fromLocation, toLocation, fromPose, toPose, new ArrayList<Pose>(), new ArrayList<Long>(), path);
	}
	
	/**
	 * Make the robot stop at the nearest location to a given pose for a given duration.
	 * @param pose A pose to stop at.
	 * @param duration Stopping time in milliseconds. 
	 */
	public void setStoppingPoint(Pose pose, long duration) {
		this.stoppingPoints.add(pose);
		this.stoppingPointDurations.add(duration);
		//this.stoppingPoints.put(pose, duration);
	}
	
	/**
	 * Clear the stopping points of this {@link SimpleNonCooperativeTask}.
	 */
	public void clearStoppingPoints() {
		this.stoppingPoints.clear();
		this.stoppingPointDurations.clear();
	}
	
	/**
	 * Get the stopping points along this {@link SimpleNonCooperativeTask}'s trajectory along with their durations.
	 * @return The stopping points along this {@link SimpleNonCooperativeTask}'s trajectory along with their durations.
	 */
	public HashMap<Pose, Long> getStoppingPoints() {
		HashMap<Pose, Long> ret = new HashMap<Pose, Long>();
		for (int i = 0; i < this.stoppingPoints.size(); i++) {
			ret.put(this.stoppingPoints.get(i), this.stoppingPointDurations.get(i));
		}
		return ret;
		//return this.stoppingPoints;
	}
	
	/**
	 * Set the source location for this {@link SimpleNonCooperativeTask}.
	 * @param location The source of this {@link SimpleNonCooperativeTask}.
	 */
	public void setFromLocation(String location) {
		this.fromLocation = location;
	}
	
	/**
	 * Get the name of the source location of this {@link SimpleNonCooperativeTask}.
	 * @return The name of the source location of this {@link SimpleNonCooperativeTask}.
	 */
	public String getFromLocation() {
		return fromLocation;
	}
	
	/**
	 * Set the destination location for this {@link SimpleNonCooperativeTask}.
	 * @param location The destination of this {@link SimpleNonCooperativeTask}.
	 */
	public void setToLocation(String location) {
		this.toLocation = location;
	}
	
	/**
	 * Get the name of the destination location of this {@link SimpleNonCooperativeTask}.
	 * @return The name of the destination location of this {@link SimpleNonCooperativeTask}.
	 */
	public String getToLocation() {
		return toLocation;
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
	
	@Override
	public int compareTo(Mission o) {
		return this.missionID-o.missionID;
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
