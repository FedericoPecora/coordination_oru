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
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.StringUtils;

/**
 * This class provides task allocation for a fleet of robots. An instantiatable {@link MultiRobotTaskAllocator}
 * must provide a comparator for queuing tasks. Default ordering is EDF (Earliest Deadline First) when a deadline is provided, FIFO otherwise.
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
	protected TreeSet<SimpleNonCooperativeTask> taskQueue = null;
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
	int maximumPathsPerTask = 1;
	
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
	
	public MultiRobotTaskAllocator(AbstractTrajectoryEnvelopeCoordinator tec, ComparatorChain comparators, double interferenceWeight, double pathLengthWeight, double arrivalTimeWeight, double tardinessWeight, int maximumPathsPerTask) {
		if (tec == null) {
			metaCSPLogger.severe("Passed null coordinator.");
			throw new Error("Passed null coordinator.");
		}
		this.tec = tec;		
		
		//Initialize the task queue and its comparators.
		this.comparators = new ComparatorChain(comparators);
		if (comparators != null) this.taskQueue = new TreeSet<SimpleNonCooperativeTask>(this.comparators);
		else this.taskQueue = new TreeSet<SimpleNonCooperativeTask>();
		
		//Initialize all the parameters
		setInterferenceWeight(interferenceWeight);
		setInterferenceFreeWeights(pathLengthWeight, arrivalTimeWeight, tardinessWeight);
		setMaximumPathsPerTask(maximumPathsPerTask);
		//TODO add finest/info to write to screen the values.
		
		//TODO Instantiate the fleetmaster
		//continue here.
	}
	
	/**
	 * Set the weight of the interference cost in the optimization function f defined as:
	 * 		  f = (1-value) * interference-free cost + value * interference cost
	 * @param value Normalized weight of the interference cost. 
	 */
	public void setInterferenceWeight(double value) {
		if (value < 0 || value > 1) metaCSPLogger.severe("Invalid interference weight. Restoring previously assigned value: " + this.interferenceWeight + ".");
		else this.interferenceWeight = value;
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
	}
	
	/**
	 * Set the maximum number of paths for each task which are considered in the optimization problem.
	 * @param value The maximum number of paths for each task.
	 */
	public void setMaximumPathsPerTask(int value) {
		this.maximumPathsPerTask = value;
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
