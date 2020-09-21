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
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotTypeInterface;
import se.oru.coordination.coordination_oru.SimpleNonCooperativeTask;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
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
							allCurrentDrivingEnvelopes.put(robotID, tec.getCurrentTrajectoryEnvelope(robotID));
							
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
						
						//1. FIXME Check if the problem is solvable (i.e., all the end positions of the current set of tasks should not intersect).
						//   This is a sufficient yet not necessary condition so that it will exists an ordering of robots which
						//   ensures all may achieve their current goal.
						//   If not, then one of the two tasks (the less critical) is delayed.
						
						//2. Evaluate all the paths to tasks and the related costs (via {@link computeAllPathsToTasksAndTheirCosts} function).
						
						//3. Setup the OAP (via {@link setupOAP} function.
						
						//4. Solve the OAP.
					}
						
					//Dispatch (i.e., enqueue) missions to robots according to the decided assignment (only if feasible).
					
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
	
	
	/*
	 * Evaluate the set of paths and related costs to achieve the current destination 
	 * @param idleRobots The set of idle robots' IDs at current time.
	 * @param currentTasksPool The current set of tasks.
	 * @param allPathsToTasks Variable to return all the paths to the current set of tasks.
	 * @return The matrix of interference-free costs.
	 */
	private double[][][] computeAllPathsToTasksAndTheirCosts(HashSet<Integer> idleRobots, TreeSet<SimpleNonCooperativeTask> currentTasksPool, ArrayList<PoseSteering[]> allPathsToTasks){
		
		//Add m-n "dummy robots" if the number of tasks m is greater than the number of idle robots n.
		TreeSet<Integer> augmentedIdleRobotIDs = new TreeSet<Integer>(idleRobots);
		for (int i = 1; i < currentTasksPool.size()-idleRobots.size(); i++)
			augmentedIdleRobotIDs.add(augmentedIdleRobotIDs.last()+1);
		
		//Initialize n-m "dummy tasks" if the number of idle robots n is greater than the number of posted tasks m.
		TreeSet<SimpleNonCooperativeTask> augmentedTaskSet = new TreeSet<SimpleNonCooperativeTask>(currentTasksPool);
		for (int i = 1; i < idleRobots.size()-currentTasksPool.size(); i++)
			augmentedTaskSet.add(new SimpleNonCooperativeTask(null, null, null, null, null, null, -1));
		
		//Clear the variable storing all the paths to the current set of tasks.
		allPathsToTasks.clear();
		
		//Initialize the matrix which stores all costs
		double[][][] allPathsCostsMatrix = new double[augmentedIdleRobotIDs.size()][augmentedTaskSet.size()][this.maxNumberPathsPerTask];
		//ArrayList<Double> allPathsCosts = new ArrayList<Double>(Arrays.asList(new Double[augmentedIdleRobotIDs.size()*augmentedTaskIDs.size()*this.maxNumberPathsPerTask]));
		HashMap<Integer, PoseSteering[]> paths = new HashMap<Integer,  PoseSteering[]>();
		
		//FIXME Handle the case of using a pre-loaded scenario

		//Conversely paths are planned online if the roadmap is not pre-loaded.
		int robotIndex = 0;
		int taskIndex = 0;
		
		for (final int robotID : augmentedIdleRobotIDs) {
			for (SimpleNonCooperativeTask task : augmentedTaskSet) {
				//Initialize the starting and the target poses of the path. 
				Pose start = null;
				Pose[] targets = null;

				//If both the robot and the task are not dummy and types are compatible, 
				//then the start is the current starting pose of the robot and targets are the sequence of stopping point plus the task ending pose.
				if (robotIndex < idleRobots.size() && taskIndex < currentTasksPool.size() && task.isCompatible(tec.getRobotType(robotID))) {
					AbstractMotionPlanner mp = null;
					synchronized(tec) {
						Pose[] ste = tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPose();
						start = ste[ste.length-1]; //FIXME Either like this or from the current robot report.
						mp = tec.getMotionPlanner(robotID);
					}
					ArrayList<Pose> targetArray = new ArrayList<Pose>(task.getStoppingPoints().keySet());
					targetArray.add(task.getToPose());
					targets = (Pose[]) targetArray.toArray();
					if (mp == null) {
						metaCSPLogger.severe("Motion planner of Robot" + robotID + " is not initialized.");
						throw new Error("Motion planner of Robot" + robotID + " is not initialized.");
					}
					else {
						//FIXME In AbstractMotionPlanner class: we need a multi-path function for planning!
						final AbstractMotionPlanner mp_ = mp;
						final Pose start_ = start;
						final Pose[] goals_ = targets;
						final int robotIndex_ = robotIndex;
						final int taskIndex_ = taskIndex;
						for (int pathIndex = 0; pathIndex < this.maxNumberPathsPerTask; pathIndex++) {
							final int pathIndex_ = pathIndex;
							final int key_ = (robotIndex*augmentedTaskSet.size()+taskIndex)*this.maxNumberPathsPerTask+pathIndex;
							//Start a thread for motion planning.
							Thread t = new Thread("MRTA planner Robot" + robotID) {
								@Override
								public void run() {
									mp_.setStart(start_);
									mp_.setGoals(goals_);
									if (mp_.plan()) paths.put(key_, mp_.getPath());
									else {
										paths.put(key_, null);
										//allPathsCosts.add(key, Double.POSITIVE_INFINITY);
									}
									allPathsCostsMatrix[robotIndex_][taskIndex_][pathIndex_] = computeInterferenceFreeCost(paths.get(key_), robotID);
									//allPathsCosts.add(key, computeInterferenceFreeCost(paths.get(key), robotID));
								}
							};
							t.start();
						}
					}
					
				}
				else {
					//FIXME Sampling here: tec.getAllRobotIDs().contains(robotID)
					if (tec.getAllRobotIDs().contains(robotID) && !task.isCompatible(tec.getRobotType(robotID))) { //the robot is real but it is not compatible with the current task.
																		 //Note: dummy robots are compatible with any task. Dummy task are compatible with any robot.
						for (int pathIndex = 0; pathIndex < this.maxNumberPathsPerTask; pathIndex++) {
							int key = (robotIndex*augmentedTaskSet.size()+taskIndex)*this.maxNumberPathsPerTask+pathIndex;
							paths.put(key, null);
							//allPathsCosts.add(key, Double.POSITIVE_INFINITY);
							//allPathsCosts.add(key, computeInterferenceFreeCost(paths.get(key), robotID));
							allPathsCostsMatrix[robotIndex][taskIndex][pathIndex] = computeInterferenceFreeCost(paths.get(key), robotID);
						}
					}
					else {
						if (taskIndex > currentTasksPool.size()) {
							if (robotIndex > idleRobots.size()) {
								metaCSPLogger.severe("Error. Found pair (robot, task) where both the robot and the task are dummy.");
								throw new Error("Error. Found pair (robot, task) where both the robot and the task are dummy.");
							}
							
							//The task is dummy, that is, it will require the robot to stay in its current position.
							//Note: all robots are compatible with dummy tasks by definition.
							PoseSteering[] path = null;
							synchronized(tec) {
								path = new PoseSteering[] {tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPoseSteering()[tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPoseSteering().length-1]};
							}
							task.setFromPose(new Pose(path[path.length].getX(),path[path.length].getY(),path[path.length].getTheta()));
							task.setFromLocation(task.getFromPose().toString());
							task.setToPose(new Pose(path[path.length].getX(),path[path.length].getY(),path[path.length].getTheta()));
							task.setToLocation(task.getToPose().toString());
							for (int pathIndex = 0; pathIndex < this.maxNumberPathsPerTask; pathIndex++) {
								int key = (robotIndex*augmentedTaskSet.size()+taskIndex)*this.maxNumberPathsPerTask+pathIndex;
								paths.put(key, path);
								//allPathsCosts.add(key, 0.0);
								//allPathsCosts.add(key, computeInterferenceFreeCost(paths.get(key), robotID));
								allPathsCostsMatrix[robotIndex][taskIndex][pathIndex] = computeInterferenceFreeCost(paths.get(key), robotID);
							}							
						}
						else  { //The task is real, robot and task are compatible but the robot is dummy.
							
							//Dummy robots are already located in the task ending pose.
							for (int pathIndex = 0; pathIndex < this.maxNumberPathsPerTask; pathIndex++) {
								int key = (robotIndex*augmentedTaskSet.size()+taskIndex)*this.maxNumberPathsPerTask+pathIndex;
								paths.put(key, new PoseSteering[] {new PoseSteering(task.getToPose(), 0)});
								//allPathsCosts.add(key, Double.POSITIVE_INFINITY);
								//allPathsCosts.add(key, computeInterferenceFreeCost(paths.get(key), robotID));
								allPathsCostsMatrix[robotIndex][taskIndex][pathIndex] = computeInterferenceFreeCost(paths.get(key), robotID);
							}
						}
					}
				}
				taskIndex++;
				taskIndex = taskIndex % augmentedTaskSet.size();
			}
			robotIndex++;
			robotIndex = robotIndex % augmentedIdleRobotIDs.size();
	
		}
		
		//Eventually add the interference cost?
	
		return allPathsCostsMatrix;
	}
	
	//TODO write down + fixme maxvalue and infinity
	private double computeInterferenceFreeCost(PoseSteering[] path, int robotID) {
		if (path == null) return Double.MAX_VALUE;
		PoseSteering[] lastPose = null;
		synchronized(tec) {
			if (!tec.getAllRobotIDs().contains(robotID)) return Double.MAX_VALUE; //the robot is dummy. Thus, by definition, it can be assigned to any task with infinite costs. 
			lastPose = new PoseSteering[] {tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPoseSteering()
					[tec.getCurrentTrajectoryEnvelope(robotID).getTrajectory().getPoseSteering().length-1]}; //FIXME: prenderla da robot report?
		}
		if (path.length == 1 && lastPose.equals(path[path.length-1])) return 0.;
		double cost = 0;
		return cost;
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
