package se.oru.coordination.coordination_oru.taskallocation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.TaskData;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.Robot;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

public class NonCooperativeTask extends TaskData implements Comparable<NonCooperativeTask> {
	
	protected static int NUMTASK = 0;
	protected String name;
	protected int robotID = -1;
	protected HashMap<Integer, List<PoseSteering[]>> paths = null;
	protected double deadline = -1;
	protected int order = NUMTASK++;
	
	/**
 	 * Create a new {@link NonCooperativeTask} to be executed.
	 * @param name The identifier of the task.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 * @param deadline The upper bound of the expected time to complete the task.
	 * @param robotTypes Types of the robots that can execute this task.
	 */
	public NonCooperativeTask(String name, String fromLocation, String toLocation, Pose fromPose, Pose toPose, double deadline, int ... robotTypes) {
		super(fromLocation, toLocation, fromPose, toPose);
		this.name = name;
		this.deadline = deadline;
		if (robotTypes.length == 0) throw new Error("Need to specify at least one robot type!");
		this.paths = new HashMap<Integer, List<PoseSteering[]>>();
		for (int rt : robotTypes) this.paths.put(rt, new ArrayList<PoseSteering[]>());
	}
		
	/** 
	 * Return the list of paths to execute this task for a given robot type. 
	 * @param robotType The type of the robot for the returned set of paths.
	 * @return The list of paths for this task and this robot type, or an empty list if the requested type is not supported.
	 */
	public List<PoseSteering[]> getPaths(int robotType) {
		return this.paths.containsKey(robotType) ? this.paths.get(robotType) : new ArrayList<PoseSteering[]>();
	}
	
	/** Set the executable paths to perform this task for a given robot type.
	 * @param robotType The type of the robot associated to this set of paths.
	 * @return <code>false</code> whether the path cannot be added since not compatible with the supported robot types. 
	 * Otherwise, return true. 
	 */
	public boolean setPaths(int robotType, PoseSteering[] ... newPaths) {
		if (!isCompatible(robotType)) return false;
		for (int i = 0; i < newPaths.length-1; i++) {
			if (!(newPaths[i][newPaths[i].length-1].equals(newPaths[i+1][0]))) 
				throw new Error("Teletransport not supported yet!");
			this.paths.get(robotType).add(newPaths[i]);
		}
		this.paths.get(robotType).add(newPaths[newPaths.length-1]);
		return true;
	}
		
	public boolean isAssigned() {
		return this.robotID != -1;
	}

	public boolean hasDeadline() {
		return this.deadline != -1;
	}
	
	public void setDeadline(double deadline) {
		this.deadline = deadline;
	}
	
	public double getDeadline() {
		return this.deadline;
	}
	
	public String getName() {
		return this.name;
	}
	
	/**
	 * Return whether the task is compatible with the given robot type.
	 * @param robotType The type of the robot.
	 */
	boolean isCompatible(int robotType) {
		return this.paths.keySet().contains(robotType);
	}
			
	public boolean assignToRobot(Robot robot) {
		if (isCompatible(robot.getType())) {
			this.robotID = robot.getID();
			return true;
		}
		return false;
	}
	
	public Mission[] getMissions() {
		if (this.paths == null) throw new Error("No paths specified!");
		if (this.robotID == -1) throw new Error("No robot assigned!");
		Mission[] ret = new Mission[paths.size()];
		for (int i = 0; i < paths.size(); i++) {
			ret[i] = new Mission(this.robotID, paths.get(this.robotID).get(i));
			if (i == 0) ret[i].setFromLocation("Init for Robot" + this.robotID);
			else if (i ==  paths.size()-1) ret[i].setFromLocation("Target for Robot" + this.robotID);
			else ret[i].setFromLocation("Waypoint " + i + " for Robot" + this.robotID);
		}
		return ret;
	}
	
	@Override
	public int compareTo(NonCooperativeTask o) {
		return this.order-o.order;
	}
	

}
