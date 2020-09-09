package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.collections.comparators.ComparatorChain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

/**
 * This class provides the simplest definition of a non cooperative task.
 * Note that {@link Mission}s are paths assigned to a specific robot, 
 * while the definition of task is more generic and does not include neither the path specification,
 * nor the robot which is assigned to this specific task.
 * 
 * @author anmi
 *
 */
public class SimpleNonCooperativeTask implements Comparable<SimpleNonCooperativeTask>{
	//ID
	protected static int NUMTASKS = 0;
	protected int taskID = NUMTASKS++;
	
	//Temporal constraints (soft)
	protected long deadline = -1;
	
	//List of robot types which can perform this task.
	protected ArrayList<Integer> compatibleRobotTypes = null;
	
	//Path-related class members
	protected String fromLocation = null;
	protected String toLocation = null;
	protected Pose fromPose = null;
	protected Pose toPose = null;
	protected ArrayList<Pose> stoppingPoints = null;
	protected ArrayList<Long> stoppingPointDurations = null;
	
	//FIXME add operations at start and at stopping points.
	
	/**
	 * Instantiate the data to navigate between two locations.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 * @param deadline The expected, absolute deadline to complete this task in millis (-1 if none).
	 * @param compatibleRobotTypes List of robot types which can perform this task (all can if empty).
	 */
	public SimpleNonCooperativeTask(String fromLocation, String toLocation, Pose fromPose, Pose toPose, ArrayList<Pose> stoppingPoints, ArrayList<Long> stoppingPointsDurations, long deadline, ArrayList<Integer> compatibleRbotTypes) {
		this.fromLocation = fromLocation;
		this.toLocation = toLocation;
		this.fromPose = fromPose;
		this.toPose = toPose;
		this.stoppingPoints = new ArrayList<Pose>(stoppingPoints);
		this.stoppingPointDurations = new ArrayList<Long>(stoppingPointDurations);
		this.deadline = deadline;
		this.compatibleRobotTypes = new ArrayList<Integer>(compatibleRobotTypes);		
	}
		
	/**
	 * Instantiate the data to navigate between two locations. 
	 * Using default values for all the unspecified fields, i.e., no deadline, 
	 * no stopping points, all the robots can perform this task. 
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public SimpleNonCooperativeTask(Pose fromPose, Pose toPose) {
		this(fromPose.toString(), toPose.toString(), fromPose, toPose, new ArrayList<Pose>(), new ArrayList<Long>(), -1, new ArrayList<Integer>());		
	}
	
	/**
	 * Get the name of the source location of this {@link SimpleNonCooperativeTask}.
	 * @return The name of the source location of this {@link SimpleNonCooperativeTask}.
	 */
	public String getFromLocation() {
		return fromLocation;
	}
	
	/**
	 * Get the name of the destination location of this {@link SimpleNonCooperativeTask}.
	 * @return The name of the destination location of this {@link SimpleNonCooperativeTask}.
	 */
	public String getToLocation() {
		return toLocation;
	}
	/**
	 * Get the {@link Pose} of source location of this {@link SimpleNonCooperativeTask}.
	 * @return The {@link Pose} of source location of this {@link SimpleNonCooperativeTask}.
	 */
	public Pose getFromPose() {
		return fromPose;
	}

	/**
	 * Set the {@link Pose} of source location of this {@link SimpleNonCooperativeTask}.
	 * @param fromPose The {@link Pose} of source location of this {@link SimpleNonCooperativeTask}.
	 */
	public void setFromPose(Pose fromPose) {
		this.fromPose = fromPose;
	}


	/**
	 * Get the {@link Pose} of destination location of this {@link SimpleNonCooperativeTask}.
	 * @return The {@link Pose} of destination location of this {@link SimpleNonCooperativeTask}.
	 */
	public Pose getToPose() {
		return toPose;
	}

	/**
	 * Get the {@link Pose} of destination location of this {@link SimpleNonCooperativeTask}.
	 * @param toPose The {@link Pose} of destination location of this {@link SimpleNonCooperativeTask}.
	 */
	public void setToPose(Pose toPose) {
		this.toPose = toPose;
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
	 * Set the destination location for this {@link SimpleNonCooperativeTask}.
	 * @param location The destination of this {@link SimpleNonCooperativeTask}.
	 */
	public void setToLocation(String location) {
		this.toLocation = location;
	}

	/**
	 * Set the source location for this {@link SimpleNonCooperativeTask}.
	 * @param location The source of this {@link SimpleNonCooperativeTask}.
	 */
	public void setFromLocation(String location) {
		this.fromLocation = location;
	}
	
	/**
	 * Set the soft, expected, absolute deadline to complete this task.
	 * @param deadline The soft, expected, absolute deadline to complete this task (in millis). 
	 * -1 if no indication. 
	 */
	public void setDeadline(long deadline) {
		this.deadline = deadline;
	}
	
	/**
	 * Get the expected, absolute time to task completion.
	 */
	public double getDeadline() {
		return this.deadline;
	}
	
	/**
	 * Set the robot types of robots which may be required to perform this task.
	 * @param compatibleRobotTypes
	 */
	public void setCompatibleRobotTypes(ArrayList<Integer> compatibleRobotTypes) {
		this.compatibleRobotTypes = new ArrayList<Integer>(compatibleRobotTypes);	
	}
	
	/**
	 * Get the robot types which are compatible with this task.
	 */
	public ArrayList<Integer> getCompatibleRobotTypes() {
		return this.compatibleRobotTypes;	
	}
	
	/**
	 * Return whether the task is compatible with the given robot type.
	 * @param robotType The type of the robot.
	 */
	public boolean isCompatible(int robotType) {
		return this.compatibleRobotTypes.contains(robotType);
	}

	@Override
	public int compareTo(SimpleNonCooperativeTask o) {
		return this.taskID-o.taskID;
	}
	
}
