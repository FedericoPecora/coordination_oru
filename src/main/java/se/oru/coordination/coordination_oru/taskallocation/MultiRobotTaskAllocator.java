package se.oru.coordination.coordination_oru.taskallocation;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.collections.comparators.ComparatorChain;
import org.metacsp.utility.logging.MetaCSPLogging;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.SimpleNonCooperativeTask;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.StringUtils;

/**
 * This class provides task allocation for a fleet of robots. An instantiatable {@link MultiRobotTaskAllocator}
 * must provide a comparator for queuing tasks. Default ordering is EDF (Earliest Deadline First) when a deadline is provided, FIFO otherwise.
 * FIXME Capire come fare questa cosa della catena di comparatori.
 * 
 * @author anmi, pf, fpa
 *
 */
public class MultiRobotTaskAllocator {
	
	public static String TITLE = "coordination_oru - Robot-agnostic online coordination for multiple robots";
	public static String COPYRIGHT = "Copyright \u00a9 2017-2020 Federico Pecora";

	//null -> public (GPL3) license
	public static String LICENSE = null;

	public static String PUBLIC_LICENSE = "This program comes with ABSOLUTELY NO WARRANTY. "
			+ "This program is free software: you can redistribute it and/or modify it under the "
			+ "terms of the GNU General Public License as published by the Free Software Foundation, "
			+ "either version 3 of the License, or (at your option) any later version. see LICENSE for details.";
	public static String PRIVATE_LICENSE = "This program comes with ABSOLUTELY NO WARRANTY. "
			+ "This program has been licensed to " + LICENSE + ". The licensee may "
			+ "redistribute it under certain conditions; see LICENSE for details.";

	//Force printing of (c) and license upon class loading
	static { printLicense(); }
	
	//Logging
	protected static Logger metaCSPLogger = MetaCSPLogging.getLogger(MultiRobotTaskAllocator.class);
	
	//A task queue (whenever a new task is posted, it is automatically added to the task queue).
	protected TreeSet<SimpleNonCooperativeTask> taskPool = null;
	protected HashMap<Integer,TreeSet<SimpleNonCooperativeTask>> singleRobotTaskPools = null;
	protected ComparatorChain comparators = null;
	
	//Mission dispatcher for each robot (where to put the output of each instance).
	
	//Coordinator (to get informations about the current paths in execution and status of the robots).
	//what's the info required?
	protected AbstractTrajectoryEnvelopeCoordinator tec = null;
	//tec.getMotionPlanner(robotID) -> gestire conflitto sulla risorsa (synchronized? plan and doPlanning yes!).
	//tec.getRobotReport(robotID) -> getLastRobotReport
	//tec.getRobotType(robotID)
	//tec.getRobotFootprint(robotID)
	//tec.getFootprintPolygon(robotID)
	//tec.getDrivingEnvelope(robotID)
	//tec.getForwardModel(robotID)
	//tec.getIdleRobots(robotID)
	//ALL OK.
	protected Missions missionsDispatchers = null;
	
	//Fleetmaster: use a local instance instead of the coordination one.
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	
	//Visualization on Rviz? (Non completely useful).
	
	//Parameters: use_scenario, weights for the B function, alpha, period of the main loop
	double interferenceWeight = 0;
	double pathLengthWeight = 1;
	double arrivalTimeWeight = 0;
	double tardinessWeight = 0;
	int maxNumberPathsPerTask = 1;
	
	//Start a periodic thread which checks for current posted goals and solves the MRTA problem at each instance.
	//flow:
	//
	
	/**
	 * Add a criterion for determining the order of tasks in the task queue. 
	 * Comparators are considered in the order in which they are added.
	 * @param c A new comparator for determining task ordering in the task queue.
	 */
	public void addComparator(Comparator<SimpleNonCooperativeTask> c) {
		this.comparators.addComparator(c);
	}
	
	public MultiRobotTaskAllocator(AbstractTrajectoryEnvelopeCoordinator tec, ComparatorChain comparators, double interferenceWeight, double pathLengthWeight, double arrivalTimeWeight, double tardinessWeight, int maxNumberPathsPerTask, 
			double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean propagateDelays, boolean debug) {
		if (tec == null) {
			metaCSPLogger.severe("Passed null coordinator.");
			throw new Error("Passed null coordinator.");
		}
		this.tec = tec;		
		
		//Initialize the task queue and its comparators.
		this.comparators = new ComparatorChain(comparators);
		if (comparators != null) this.taskPool = new TreeSet<SimpleNonCooperativeTask>(this.comparators);
		else this.taskPool = new TreeSet<SimpleNonCooperativeTask>();
		
		//Initialize all the parameters
		setInterferenceWeight(interferenceWeight);
		setInterferenceFreeWeights(pathLengthWeight, arrivalTimeWeight, tardinessWeight);
		setMaxNumberPathsPerTask(maxNumberPathsPerTask);
		
		//Instantiate the fleetmaster
		instantiateFleetMaster(origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, propagateDelays, debug);
		//TODO Capire come fare la query per F.
	}
	
	
	/**
	 * Set the weight of the interference cost in the optimization function f defined as:
	 * 		  f = (1-value) * interference-free cost + value * interference cost
	 * @param value Normalized weight of the interference cost. 
	 */
	public void setInterferenceWeight(double value) {
		if (value < 0 || value > 1) metaCSPLogger.severe("Invalid interference weight. Restoring previously assigned value: " + this.interferenceWeight + ".");
		else this.interferenceWeight = value;
		metaCSPLogger.info("Updated interference weight with value " + this.interferenceWeight + ".");
	}
	
	
	/**
	 * Set the weights of the interference-free cost defined as
	 * interference-free cost = pathLengthWeight * path length cost + arrivalTimeWeight * arrival time cost + tardinessWeight * tardiness cost.
	 * @param pathLengthWeight Normalized weight of the path length cost (the longest the path, the higher the cost).
	 * @param arrivalTimeWeight Normalized weight of the arrival time cost (XXX FIXME: add definition).
	 * @param tardinessWeight Normalized weight of the tardiness cost (XXX FIXME: add definition).
	 */
	public void setInterferenceFreeWeights(double pathLengthWeight, double arrivalTimeWeight, double tardinessWeight)  {
		if (pathLengthWeight < 0 || arrivalTimeWeight < 0 || tardinessWeight < 0 || pathLengthWeight + arrivalTimeWeight + tardinessWeight > 1)
			metaCSPLogger.severe("Invalid interference-free weights. Restoring previously assigned values: path length weight " + this.pathLengthWeight + ", arrival time weight " + this.arrivalTimeWeight + ", tardiness weight " + tardinessWeight + ".");
		else {
			this.pathLengthWeight = pathLengthWeight;
			this.arrivalTimeWeight = arrivalTimeWeight;
			this.tardinessWeight = tardinessWeight;
		}
		metaCSPLogger.info("Updated interference-free cost functions weights with values: path length " + this.pathLengthWeight + ", arrival time " + this.arrivalTimeWeight + ", tardiness " + this.tardinessWeight + ".");
	}
	
	
	/**
	 * Set the maximum number of paths for each task which are considered in the optimization problem.
	 * @param value The maximum number of paths for each task.
	 */
	public void setMaxNumberPathsPerTask(int value) {
		this.maxNumberPathsPerTask = value;
		metaCSPLogger.info("Updated the maximum number of paths for each tasks parameter to " + this.maxNumberPathsPerTask + ".");
	}
	
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * Note: this function should be called before placing the first robot.
	 * ATTENTION: If dynamic_size is <code>false</code>, then the user should check that all the paths will lay in the given area.
	 * @param origin_x The x coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta coordinate (in rads) of the lower-left pixel map (counterclockwise rotation). Many parts of the system currently ignore it.
	 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
	 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
	 * @param width Number of columns of the map (>= 1) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (>= 1) if dynamic sizing is not enabled.
	 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 * @param debug If <code>true</code>, it enables writing to screen debugging info.
	 */
	public void instantiateFleetMaster(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean propagateDelays, boolean debug) {
		this.fleetMasterInterface = new FleetMasterInterface(origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, debug);
		this.fleetMasterInterface.setDefaultFootprint(this.tec.getDefaultFootprint());
	}
	
	
	/**
	 * Add a {@link SimpleNonCooperativeTask} to the task pool.
	 * @param task The task to be added.
	 * @return <code>true</code> whether the task was correctly added.
	 */
	public boolean addTask(SimpleNonCooperativeTask task) {
		boolean ret = this.taskPool.add(task);
		if (!ret) metaCSPLogger.severe("Error. Task " + task.getID() +" was not correctly added to the task pool.");
		return ret;
	}
	
	
	/**
	 * Remove a {@link SimpleNonCooperativeTask} from the task pool.
	 * @param task The task to be removed.
	 * @return <code>true</code> whether the task was correctly removed.
	 */
	public boolean removeTask(SimpleNonCooperativeTask task) {
		boolean ret = this.taskPool.remove(task);
		if (!ret) metaCSPLogger.severe("Error. Task " + task.getID() + " was not correctly removed from the task pool.");
		return ret;
	}
	
	
	/**
	 * Add a {@link SimpleNonCooperativeTask} to a specific robot.
	 * @param robotID ID of the robot which should perform this task
	 * @param task The task to add.
	 * @return <code>true</code> whether the task was correctly added.
	 */
	public boolean addTask(int robotID, SimpleNonCooperativeTask task) {
		if (!tec.getAllRobotIDs().contains(robotID)) {
			metaCSPLogger.severe("Task " + task.getID() + "cannot be assigned to robot " + robotID + " since robotID is not valid.");
			return false;
		}
		if (!task.getCompatibleRobotTypes().contains(tec.getRobotType(robotID))) {
			metaCSPLogger.severe("Task " + task.getID() + "cannot be assigned to robot " + robotID + " since types are not compatible.");
			return false;
		}
		if (!this.singleRobotTaskPools.containsKey(robotID)) this.singleRobotTaskPools.put(robotID, new TreeSet<SimpleNonCooperativeTask>());
		boolean ret = this.singleRobotTaskPools.get(robotID).add(task);
		return ret;
	}
	
	
	/**
	 * Remove a {@link SimpleNonCooperativeTask} assigned to a specific robot.
	 * @param robotID ID of the robot.
	 * @param task The task to be removed.
	 * @return <code>true</code> whether the task was correctly removed.
	 */
	public boolean removeTask(int robotID, SimpleNonCooperativeTask task) {
		boolean ret = this.singleRobotTaskPools.containsKey(robotID) && this.singleRobotTaskPools.get(robotID).remove(task);
		if (!ret) metaCSPLogger.severe("Error. Task " + task.getID() + " was not correctly removed from the task pool.");
		return ret;
	}
	
	/**
	 * Update the deadline of a previously added task.
	 * @param task The task to be updated.
	 * @param deadline The new deadline of the task (in millis).
	 * @return <code>true</code> whether the deadline was correctly updated.
	 */
	public boolean updateDeadline(SimpleNonCooperativeTask task, long deadline) {
		for (SimpleNonCooperativeTask _task : this.taskPool) {
			if (_task.equals(task)) {
				boolean ret = this.removeTask(task);
				task.setDeadline(deadline);
				ret &= this.addTask(task);
				return ret;
			}
		}
		for (Integer robotID : this.singleRobotTaskPools.keySet()) {
			for (SimpleNonCooperativeTask _task : this.singleRobotTaskPools.get(robotID)) {
				if (_task.equals(task)) {
					boolean ret = this.removeTask(robotID, task);
					task.setDeadline(deadline);
					ret &= this.addTask(robotID, task);
					return ret;
				}
			}
		}
		return false;
	}
	
	/**
	 * Get a previously added task.
	 * @param taskID The ID of the task.
	 * @return The desired task (null if not found).
	 * ATTENTION. Use the method {@link updateDeadline} to update the deadline.
	 */
	public SimpleNonCooperativeTask getTask(int taskID) {
		for (SimpleNonCooperativeTask task : this.taskPool) 
			if (task.getID() == taskID) return task;
		for (Integer robotID : this.singleRobotTaskPools.keySet()) {
			for (SimpleNonCooperativeTask task : this.singleRobotTaskPools.get(robotID)) 
				if (task.getID() == taskID) return task;
		}
		return null;
	}
	
	
	private static void printLicense() {
		System.out.println("\n"+MultiRobotTaskAllocator.TITLE);
		System.out.println(MultiRobotTaskAllocator.COPYRIGHT+"\n");
		if (MultiRobotTaskAllocator.LICENSE != null) {
			List<String> lic = StringUtils.fitWidth(MultiRobotTaskAllocator.PRIVATE_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		else {
			List<String> lic = StringUtils.fitWidth(MultiRobotTaskAllocator.PUBLIC_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		System.out.println();
	}
		
}
