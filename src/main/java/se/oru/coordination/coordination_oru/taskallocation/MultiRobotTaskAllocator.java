package se.oru.coordination.coordination_oru.taskallocation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.apache.commons.collections.comparators.ComparatorChain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.SimpleNonCooperativeTask;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
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
	
	protected int CONTROL_PERIOD = 5000;
	protected double TEMPORAL_RESOLUTION = 1000;
	public static int EFFECTIVE_CONTROL_PERIOD = 0;
	protected Thread inference = null;
	protected volatile Boolean stopInference = new Boolean(true);
	
	//Logging
	protected static Logger metaCSPLogger = MetaCSPLogging.getLogger(MultiRobotTaskAllocator.class);
	
	//A task queue (whenever a new task is posted, it is automatically added to the task queue).
	protected TreeSet<SimpleNonCooperativeTask> taskPool = null;
	protected ComparatorChain comparators = null;
	
	//Mission dispatcher for each robot (where to put the output of each instance).
	
	//Coordinator (to get informations about the current paths in execution and status of the robots).
	protected AbstractTrajectoryEnvelopeCoordinator tec = null;
	
	//Fleetmaster: use a local instance instead of the coordination one.
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	
	//Visualization on Rviz? (Non completely useful).
	
	//Parameters: use_scenario, weights for the B function, alpha, period of the main loop
	double interferenceWeight = 0;
	double pathLengthWeight = 1;
	double arrivalTimeWeight = 0;
	double tardinessWeight = 0;
	int maxNumberPathsPerTask = 1;
		
	/**
	 * Add a criterion for determining the order of tasks in the task queue. 
	 * Comparators are considered in the order in which they are added.
	 * @param c A new comparator for determining task ordering in the task queue.
	 */
	public void addComparator(Comparator<SimpleNonCooperativeTask> c) {
		this.comparators.addComparator(c);
	}
	
	public MultiRobotTaskAllocator(int controlPeriod, int temporalResolution, AbstractTrajectoryEnvelopeCoordinator tec, ComparatorChain comparators, double interferenceWeight, double pathLengthWeight, double arrivalTimeWeight, double tardinessWeight, int maxNumberPathsPerTask, 
			double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean propagateDelays, boolean debug) {
		
		setControlPeriod(controlPeriod, temporalResolution);
		
		if (tec == null) {
			metaCSPLogger.severe("Passed null coordinator.");
			throw new Error("Passed null coordinator.");
		}
		this.tec = tec;		
		
		//Initialize the task queue and its comparators.
		if (comparators != null) {
			this.comparators = new ComparatorChain(comparators);
			this.taskPool = new TreeSet<SimpleNonCooperativeTask>(this.comparators);
		}
		else this.taskPool = new TreeSet<SimpleNonCooperativeTask>();
		
		//Initialize all the parameters
		setInterferenceWeight(interferenceWeight);
		setInterferenceFreeWeights(pathLengthWeight, arrivalTimeWeight, tardinessWeight);
		setMaxNumberPathsPerTask(maxNumberPathsPerTask);
		
		//Instantiate the fleetmaster
		instantiateFleetMaster(origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, propagateDelays, debug);
	}
	
	
	public void startMissionsDispatchers(int ... robotIDs) {
		Missions.startMissionDispatchers(this.tec, false, robotIDs);
	}
	
	public void stopMissionsDispatchers(int ... robotIDs) {
		Missions.stopMissionDispatchers(robotIDs);
	}
	
	/**
	 * Set the period of the task assignment loop.
	 * @param controlPeriod The control period of the MRTA loop (in millis).
	 * @param temporalResolution The temporal resolution at which the control period is specified (e.g., 1000 for millis).
	 */
	public void setControlPeriod(int controlPeriod, int temporalResolution) {
		if (controlPeriod <= 0 || temporalResolution <= 0) {
			metaCSPLogger.severe("Invalid control period or temporal resolution parameter. Restoring previously assigned parameters: control period " + this.CONTROL_PERIOD + " msec, temporal resolution " + this.TEMPORAL_RESOLUTION + ".");
			return;
		}
		this.CONTROL_PERIOD= controlPeriod;
		this.TEMPORAL_RESOLUTION = temporalResolution;
		metaCSPLogger.info("Updated control period and temporal resolution with values " + this.CONTROL_PERIOD + " msec and " + this.TEMPORAL_RESOLUTION + " respectively.");
	}
	
 	
	/**
	 * Set the weight of the interference cost in the optimization function f defined as:
	 * 		  f = (1-value) * interference-free cost + value * interference cost
	 * @param value Normalized weight of the interference cost. 
	 */
	public void setInterferenceWeight(double value) {
		if (value < 0 || value > 1) {
			metaCSPLogger.severe("Invalid interference weight. Restoring previously assigned value: " + this.interferenceWeight + ".");
			return;
		}
		this.interferenceWeight = value;
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
		if (pathLengthWeight < 0 || arrivalTimeWeight < 0 || tardinessWeight < 0 || pathLengthWeight + arrivalTimeWeight + tardinessWeight > 1) {
			metaCSPLogger.severe("Invalid interference-free weights. Restoring previously assigned values: path length weight " + this.pathLengthWeight + ", arrival time weight " + this.arrivalTimeWeight + ", tardiness weight " + tardinessWeight + ".");
			return;
		}
		this.pathLengthWeight = pathLengthWeight;
		this.arrivalTimeWeight = arrivalTimeWeight;
		this.tardinessWeight = tardinessWeight;
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
		synchronized(taskPool) {
			boolean ret = this.taskPool.add(task);
			if (!ret) metaCSPLogger.severe("Error. Task " + task.getID() +" was not correctly added to the task pool.");
			return ret;
		}
	}
	
	
	/**
	 * Remove a {@link SimpleNonCooperativeTask} from the task pool.
	 * @param task The task to be removed.
	 * @return <code>true</code> whether the task was correctly removed.
	 */
	public boolean removeTask(SimpleNonCooperativeTask task) {
		synchronized(taskPool) {
			boolean ret = this.taskPool.remove(task);
			if (!ret) metaCSPLogger.severe("Error. Task " + task.getID() + " was not correctly removed from the task pool.");
			return ret;
		}
	}
	
	/**
	 * Update the deadline of a previously added task.
	 * @param task The task to be updated.
	 * @param deadline The new deadline of the task (in millis).
	 * @return <code>true</code> whether the deadline was correctly updated.
	 */
	public boolean updateDeadline(SimpleNonCooperativeTask task, long deadline) {
		synchronized(taskPool) {
			for (SimpleNonCooperativeTask _task : this.taskPool) {
				if (_task.equals(task)) {
					boolean ret = this.removeTask(task);
					task.setDeadline(deadline);
					ret &= this.addTask(task);
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
	 * ATTENTION. Use the method {@link updateDeadline} to update the task deadline.
	 */
	public SimpleNonCooperativeTask getTask(int taskID) {
		synchronized(taskPool) {
			for (SimpleNonCooperativeTask task : this.taskPool) 
				if (task.getID() == taskID) return task;
		}
		return null;
	}
	
	//TODO get functions for missionPool
	
	/**
	 * Call this method to start the thread that assigns new goals at every clock tick.
	 */
	public void startInferenceCallback() {		
		if (!stopInference) {
			metaCSPLogger.info("MRTA inference thread is already started.");
			return;
		}

		//Start the thread that checks and enforces dependencies at every clock tick
		this.setupInferenceCallback();
	}
	
	/**
	 * Call this method to stop the thread that assigns new goals at every clock tick.
	 */
	public void stopInferenceCallback() {
		if (stopInference) metaCSPLogger.severe("MRTA inference thread is not live.");
		stopInference = true;
	}

	/**
	 * Call this method to check if the thread that assigns new goals at every clock tick is live.
	 */
	public boolean isStartedInferenceCallback() {
		return !stopInference;
	}
	
	
	protected void setupInferenceCallback() {
		
		//Start the trajectory envelope coordinator if not started yet.
		if (!tec.isStartedInference()) tec.startInference();
		
		//Start the main task allocation loop
		this.stopInference = false;
		this.inference = new Thread("MRTA inference") {
			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
			
			@Override
			public void run() {
				while (!stopInference) {
															
					//Sample the current robots' and tasks' status
					HashSet<Integer> idleRobots = null;			
					HashMap<Integer, TrajectoryEnvelope> allCurrentDrivingEnvelopes = new HashMap<Integer, TrajectoryEnvelope>();
					synchronized(tec) {
						//Sample the set of idle robots
						idleRobots = new HashSet<Integer>(Arrays.asList(tec.getIdleRobots()));
						
						//Sample the set of driving envelopes
						for (int robotID : tec.getAllRobotIDs()) {
							allCurrentDrivingEnvelopes.put(robotID, tec.getCurrentDrivingEnvelope(robotID));
							
							//FIXME filter out idle robots which will be assigned to some missions
							if (idleRobots.contains(robotID) && Missions.getMissions(robotID) != null && !Missions.getMissions(robotID).isEmpty()) idleRobots.remove(robotID);
						}
					}
					
					TreeSet<SimpleNonCooperativeTask> currentTaskPool = null;
					synchronized(taskPool) {
						currentTaskPool = new TreeSet<SimpleNonCooperativeTask>(taskPool); //FIXME!! Be carefull with TreeSet (we have natural ordering, so it should be fine)
					}
										
					if (!currentTaskPool.isEmpty() && idleRobots.size() > 0) {
						
						//Setup and solve the MRTA optimization problem.
						
						//1. Check if the problem is solvable (i.e., all the end positions of the current set of tasks should not intersect).
						//   This is a sufficient yet not necessary condition so that it will exists an ordering of robots which
						//   ensures all may achieve their current goal.
						//   If not, then one of the two tasks (the less critical) is delayed.
						
						//2. Compute dummy robots and tasks.
						
						//3. Evaluate all the paths to tasks and the related costs.
						
						//4. Setup the OAP.
						
						//5. Solve the OAP.
					}
						
					//Dispatch (i.e., enqueue) missions to robots according to the decided assignment.
					
					//Sleep a little...
					if (CONTROL_PERIOD > 0) {
						try { 
							Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					EFFECTIVE_CONTROL_PERIOD = (int)(Calendar.getInstance().getTimeInMillis()-threadLastUpdate);
					threadLastUpdate = Calendar.getInstance().getTimeInMillis();

				}
			}
		};
		//t.setPriority(Thread.MAX_PRIORITY);
		this.inference.start();
	}
	
	/**
	 * 
	 * @param toDelay
	 * @param allCurrentDrivingEnvelopes
	 * @param currentTaskPool
	 * @return <code>true</code> whether the current set of task is admissible
	 */
	private boolean checkAdmissibilityOfAllCurrentTasks(HashSet<SimpleNonCooperativeTask> toDelay, HashMap<Integer, TrajectoryEnvelope> allCurrentDrivingEnvelopes, TreeSet<SimpleNonCooperativeTask> currentTaskPool) {
		
		toDelay.clear();
		
		ArrayList<SimpleNonCooperativeTask> currentTaskPoolList = new ArrayList<SimpleNonCooperativeTask>(currentTaskPool);
		for (int i = 0; i < currentTaskPoolList.size(); i++) {
			//Consider the robot occupancy in the task end point in the worst case 
			//(i.e., the largest compatible footprint) 
			//Check tasks with tasks
			SimpleNonCooperativeTask task1 = currentTaskPoolList.get(i);
			for (int j = i+1; j < currentTaskPoolList.size(); j++) {
				SimpleNonCooperativeTask task2 = currentTaskPoolList.get(j);
				Pose goal1 = task1.getToPose();
				Pose goal2 = task2.getToPose();
				//Robot1 in pose (w.c. footprint)
				//Robot2 in pose (w.c. footprint)
				//If intersect, then add to toDelay and continue
				
				//If not, check the task with respect to the current driving envelopes
				
				//TODO
			}
		}
		
		return toDelay.isEmpty();
	}
	
	
	
	private int[] evaluateDummyRobotsAndTasks() {
		int[] dummyRobotsAndPaths = new int[] {0,0};
		//TODO
		
		
		return dummyRobotsAndPaths;
	}
	
	/*
	 * Evaluate the set of paths and related costs to achieve the current destinatiom 
	 * @param augmentedIdleRobotIDs The set of robots' IDs augmented with dummy robots.
	 * @param augmentedTaskIDs The set of tasks' IDs augmented with dummy task.
	 * @param allPathsToTasks Variable to retrieve all the paths to the current set of tasks.
	 * @return The matrix of interference-free costs.
	 */
	private double[][][] computeAllPathsToTasksAndTheirCosts(int[] augmentedIdleRobotIDs, int[] augmentedTaskIDs, ArrayList<PoseSteering[]> allPathsToTasks){
		
		//FIXME Controlla il codice qui sotto
		synchronized(taskPool) {
			if (augmentedIdleRobotIDs.length < Math.max(1, tec.getAllRobotIDs().size()) || augmentedTaskIDs.length < Math.max(1, taskPool.size())) {
				metaCSPLogger.severe("Error in evaluating the number of robors and tasks.");
				throw new Error("Error in evaluating the number of robors and tasks.");
			}
		}
		
		//Clear the variable storing all the paths to the current set of tasks.
		allPathsToTasks.clear();
		
		//Initialize the matrix which stores all costs
		double[][][] allPathsCosts = new double[augmentedIdleRobotIDs.length][augmentedTaskIDs.length][this.maxNumberPathsPerTask];
		
		//FIXME Handle the case of using a pre-loaded scenario

		//Conversely paths are planned online if the roadmap is not pre-loaded.
		for (int i = 0; i < augmentedIdleRobotIDs.length; i++) {
			for (int j = 0; j < augmentedTaskIDs.length; j++) {
				//Check if robot i is compatible with task j. If not, then allPathsToTasks.add(null);
				
				//Otherwise compute this.maxNumberPathsPerTask paths (if possible).
				
				//Evaluate its interference-cost and update the matrix.
			}
	
		}
		
		//Eventually add the interference cost
	
		return allPathsCosts;
	}
	
	
	private MPSolver setupOAP(int augmentedNumberOfRobots, int augmentedNumberOfTasks, Set<Integer> idleRobotIDs) {
		
		//Create the linear solver with the CBC backend.
		MPSolver solver = new MPSolver(
				"MRTA - Optimization problem", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		//Declare the set of decision variables 
		MPVariable [][][] decisionVariables = new MPVariable[augmentedNumberOfRobots][augmentedNumberOfTasks][this.maxNumberPathsPerTask];
		for (int i = 0; i < augmentedNumberOfRobots; i++) {
			 for (int j = 0; j < augmentedNumberOfTasks; j++) {
				 for(int s = 0; s < this.maxNumberPathsPerTask; s++) {
					 decisionVariables[i][j][s] = solver.makeBoolVar("x"+"["+i+","+j+","+s+"]");
				 }
				
			 }
		}
		
		//Declare the constraints of the OAP
		//(tutorials and examples at https://github.com/google/or-tools/blob/stable/ortools/linear_solver/samples/SimpleMipProgram.java 
		// and https://developers.google.com/optimization/examples). 
		//Constraints are in the form a1*x1 + ...  + an*xn <= b (linear constraint), with xi being a decision variable.
		
		//1. each robot may be assigned only to one task, i.e.,	
		//   for each i, sum_j sum_s x_{ijs} == 1
		 for (int i = 0; i < augmentedNumberOfRobots; i++) {			 
			//Define the domain [lb, ub] of the constraint
			 MPConstraint c = solver.makeConstraint(1, 1);
			 for (int j = 0; j < augmentedNumberOfTasks; j++)
				 for(int s = 0; s < this.maxNumberPathsPerTask; s++) 
					//Set the coefficient ai of the decision variable xi
					 c.setCoefficient(decisionVariables[i][j][s], 1);//
		 }
		
		//2. tasks are non-cooperative (they can be assigned to at most one robot), i.e.,
		//   for each j, sum_i sum_s x_{ijs} == 1
		 for (int j = 0; j < augmentedNumberOfTasks; j++) {
			 MPConstraint c = solver.makeConstraint(1, 1); 
			 for (int i = 0; i < augmentedNumberOfRobots; i++) {
				 for(int s = 0; s < this.maxNumberPathsPerTask; s++) 
					 c.setCoefficient(decisionVariables[i][j][s], 1); 
			 }
		 }
		 
		 ///////////////////////////////////////////////////////////
		 //			The following code needs to be checked
		 ///////////////////////////////////////////////////////////
	
		 //3. set to 0 the variables related to unfeasible assignments of robots to tasks.
		 //   A pair is unfeasible either is no path from the starting location of the robot exists or 
		 //   if the robot and task types are not compatible.
		 /*for (int robotID : idleRobotIDs) {
				int i = IDsAllRobots.indexOf(robotID);
				for (int taskID : IDsAllTasks) {
					int j = IDsAllTasks.indexOf(taskID);
					for(int s = 0; s < this.maxNumberPathsPerTask; s++) {
							 if (i < numRobot) { //i is not a dummy robot?
								 if (pathsToTargetGoal.get(robotID*augmentedNumberOfTasks*this.maxNumberPathsPerTask+taskID*this.maxNumberPathsPerTask+s) == null) {
									 MPConstraint c3 = solver.makeConstraint(0,0);
									 c3.setCoefficient(decisionVariables[i][j][s],1); 
								 }
							 }
					}
				}
		 }
		 
		//END CONSTRAINTS
		//In case of having more task than robots, the task with a closest deadline are set with a higher priority
		 if(taskPool.size() > idleRobotIDs) {
			 checkOnTaskDeadline();
			//Each task can be performed only by a robot
			 for (int j = 0; j < taskPool.size(); j++) {
				//Initialize the constraint
				 if(taskPool.get(j).isPriority()) {
					 MPConstraint c3 = solver.makeConstraint(1, 1); 
					 for (int i = 0; i < idleRobotIDs.size(); i++) {
						 for(int s = 0; s < this.maxNumberPathsPerTask; s++) {
							 //Build the constraint
							 c3.setCoefficient(decisionVariables[i][j][s], 1); 
						 } 		
					 }
				 }
			 }
		 }*/
		/////////////////////////////////////////////////
		return solver;	
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
