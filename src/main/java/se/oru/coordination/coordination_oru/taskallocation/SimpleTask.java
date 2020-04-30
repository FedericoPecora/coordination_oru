package se.oru.coordination.coordination_oru.taskallocation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.Robot;

public class SimpleTask {
	
	protected int ID;
	protected Set<Integer> robotTypes = null;	
	protected Pose start;
	protected double startOperationTime = 0;
	protected Pose target;
	protected double targetOperationTime = 0;
	protected HashMap<Integer, List<PoseSteering[]>> paths = null;
	protected int assignedTo = -1;
	protected double deadline = -1;
	
	/**
 	 * Create a new {@link SimpleTask} to be executed.
	 * @param ID The task ID.
	 * @param start The starting pose of the task.
	 * @param startOperationTime The estimated time to execute operations in the start position.
	 * @param targetPose The target pose of the task.
	 * @param targetOperationTime The estimated time to execute operations in the target position.
	 * @param deadline The upper bound of the time in which the task should be executed.
	 * @param robotTypes Types of the robots that can execute this task.
	 */
	public SimpleTask(int ID, Pose start, double startOperationTime, Pose target, double targetOperationTime, double deadline, int ... robotTypes) {
		this.ID = ID;
		this.start = start;
		this.target = target;
		this.deadline = deadline;
		if (robotTypes.length == 0) throw new Error("Need to specifiy at least one robot type!");
		this.paths = new HashMap<Integer, List<PoseSteering[]>>();
		for (int rt : robotTypes) this.paths.put(rt, new ArrayList<PoseSteering[]>());
	}
	
	/**
 	 * Create a new {@link Task} to be executed.
	 * @param ID The task ID.
	 * @param start The starting pose of the task.
	 * @param targetPose The target pose of the task.
	 * @param robotTypes Types of the robots that can execute this task.
	 */
	public SimpleTask(int ID, Pose start, Pose target, int ... robotTypes) {
		this(ID,start,0,target,0,-1,robotTypes);
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
	
	public Pose getStart() {
		return this.start;		
	}
	
	public Pose getTarget() {
		return this.target;	
	}
	
	public boolean isAssigned() {
		return this.assignedTo != -1;
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
	
	public int getID() {
		return this.ID;
	}

	public double getTargetOperationTime() {
		return this.targetOperationTime;
	}
	
	public double getStartOperationTime() {
		return this.targetOperationTime;
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
			this.assignedTo = robot.getID();
			return true;
		}
		return false;
	}

	public Mission[] getMissions() {
		if (this.paths == null) throw new Error("No paths specified!");
		if (this.assignedTo == -1) throw new Error("No robot assigned!");
		Mission[] ret = new Mission[paths.size()];
		for (int i = 0; i < paths.size(); i++) {
			ret[i] = new Mission(this.assignedTo, paths.get(this.assignedTo).get(i));
			if (i == 0) ret[i].setFromLocation("Init for Robot" + this.assignedTo);
			else if (i ==  paths.size()-1) ret[i].setFromLocation("Target for Robot" + this.assignedTo);
			else ret[i].setFromLocation("Waypoint " + i + " for Robot" + this.assignedTo);
		}
		return ret;
	}

}
