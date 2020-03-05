package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.TreeSet;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.sun.jna.NativeLong;

import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.CumulatedIndexedDelaysList;

public abstract class TimedTrajectoryEnvelopeCoordinator extends TrajectoryEnvelopeCoordinator {
		
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	protected boolean propagateDelays = false;
	
	public TimedTrajectoryEnvelopeCoordinator(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
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
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
		this.propagateDelays = propagateDelays;
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time
	 * while minimizing the computational requirements (bounding box are used to set the size of each path-image).
	 * Note: this function should be called before placing the first robot.
	 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
	 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 */
	public void instantiateFleetMaster(double resolution, boolean propagateDelays) {
		this.fleetMasterInterface = new FleetMasterInterface(0., 0., 0., resolution, 100, 100, true, false);
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
		this.propagateDelays = propagateDelays;
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * @param mapYAMLFile file to the global map of the environment (path specified as "/.../...").
	 * Possible: absolute paths or stored in the maps directory of the coordination_oru package (e.g., "maps/map-empty");
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 * @param debug If <code>true</code>, it enables showing the content of each GridMap.
	 * ATTENTION: it is required the map to contain all the possible paths (dynamic sizing is off). Also, resolution is automatically constrained in 0.01 <= resolution <= 1.
	 */
	@Deprecated
	public void instantiateFleetMaster(String mapYAMLFile, boolean propagateDelays, boolean debug) {
		this.fleetMasterInterface = new FleetMasterInterface(mapYAMLFile, debug);
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
		this.propagateDelays = propagateDelays;
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * @param mapYAMLFile file to the global map of the environment (path specified as "/.../...").
	 * Possible: absolute paths or stored in the maps directory of the coordination_oru package (e.g., "maps/map-empty");
	 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
	 * ATTENTION: it is required the map to contain all the possible paths (dynamic sizing is off). Also, resolution is automatically constrained in 0.01 <= resolution <= 1.
	 */
	@Deprecated
	public void instantiateFleetMaster(String mapYAMLFile, boolean propagateDelays) {
		instantiateFleetMaster(mapYAMLFile, propagateDelays, false);
	}
	
	/**
	 * Set the parameters for estimating the temporal profile for the trajectories a given robot. 
	 * Use <code>-1</code> when the parameter is unknown (maxSteeringAngleVel equal to <code>-1</code> will automatically force useSteerDriveVel to be <code>false</code>).
	 * @param robotID The robot ID.
	 * @param maxVel The maximum forward linear velocity (m/s).
	 * @param maxVelRev The maximum backward linear velocity (m/s).
	 * @param useSteerDriveVel <code>true</code> if the steering velocity should be used.
	 * @param maxRotationalVel The maximum anti clockwise angular velocity (rad/s).
	 * @param maxRotationalVelRev The maximum clockwise angular velocity (rad/s).
	 * @param maxSteeringAngleVel The maximum clockwise steering velocity (rad/s).
	 * @param maxAcc The maximum linear acceleration (m/s^2).
	 * @param maxRotationalAcc The maximum angular acceleration (rad/s^2).
	 * @param maxSteeringAngleAcc The maximum steering acceleration (rad/s^2).
	 */
	public void setNominalTrajectoryParameters(int robotID, double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc) {
		if (this.fleetMasterInterface != null) 
			this.fleetMasterInterface.setTrajParams(robotID, maxVel, maxVelRev, useSteerDriveVel, maxRotationalVel, maxRotationalVelRev, maxSteeringAngleVel, maxAcc, maxRotationalAcc, maxSteeringAngleAcc);
		else {
			metaCSPLogger.severe("Unable to set Robot" + robotID + " trajectory parameters. Instantiate the fleetmaster first!");
			throw new Error("Unable to set Robot" + robotID + " trajectory parameters. Instantiate the fleetmaster first!");
		}
	}
	
	@Override
	protected void onSettingDefaultFootprint() {
		if (this.fleetMasterInterface != null)
			this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
	}
		
	@Override
	protected void onClearingTe(TrajectoryEnvelope te) {
		if (this.fleetMasterInterface != null) 
			this.fleetMasterInterface.clearPath(te.getID());
	}
	
	@Override
	protected void onAddingTe(TrajectoryEnvelope te) {
		if (!fleetMasterInterface.addPath(te)) 
			metaCSPLogger.severe("Unable to add the path to the fleetmaster gridmap. Check if the map contains the given path.");
	}
	
	/**
	 * Check if the comparison between two delays is meaningful, i.e., when two delays are both negative,
	 * then both the precedence orderings are safe, so another heuristic may be used to decide the precedence order.
	 * @param delay1 The first delay.
	 * @param delay2 The second delay.
	 * @return If at least one of them is positive.
	 */
	protected boolean oneOrderingIsUnsafe(double delay1, double delay2) {
		return (delay1 >= 0 || delay2 >= 0) && !(delay1 == 0 && delay2 == 0);
	}
	
	@Override
	protected void updateDependencies() {
		synchronized(solver) {
			if (this.fleetMasterInterface != null && this.propagateDelays) localCheckAndReviseWithDelayPropagation();
			else { //use "the super" updateDependencies(), eventually with new heuristic, defined in the overridden getOrderWithAdvancedHeuristics function.
				if (this.avoidDeadlockGlobally) globalCheckAndRevise();
				else localCheckAndRevise();
			}
		}
	}
	
	/**
	 * Function to extend the getOrder with more advanced heuristics.
	 * @return <ul>
	 * <li> 1 if robot1 should go before robot2,
	 * <li> -1 if robot2 should go before robot1, 
	 * <li> 0 if the heuristic cannot be used or cannot retrieve a valid estimation.
	 * </ul>
	 */
	@Override
	protected int getOrderWithAdvancedHeuristics(AbstractTrajectoryEnvelopeTracker robotTracker1, RobotReport robotReport1, AbstractTrajectoryEnvelopeTracker robotTracker2, RobotReport robotReport2, CriticalSection cs) {
		if (this.fleetMasterInterface != null && fleetMasterInterface.checkTrajectoryEnvelopeHasBeenAdded(cs.getTe1()) && fleetMasterInterface.checkTrajectoryEnvelopeHasBeenAdded(cs.getTe2())) {
			CumulatedIndexedDelaysList te1TCDelays = new CumulatedIndexedDelaysList();
			CumulatedIndexedDelaysList te2TCDelays = new CumulatedIndexedDelaysList();
			Pair<Double, Double> delays = fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays);
			if (oneOrderingIsUnsafe(delays.getFirst(), delays.getSecond())) {
				if (Math.min(delays.getFirst(), delays.getSecond()) > Double.MAX_VALUE) return 0;
				return delays.getFirst() < delays.getSecond() ? -1 : 1;
			}
		}
		return 0;
	}
	
	//FIXME not the best way if we have to convert types!!
	/*
	 * TTC 
		indices: 0  1  2  3  4  5  6
		values:  6  5  4  3  2  1  0
	   Delays
		indices: 1, 4, 5
		values: 5, 3, 1. //always decreasing.
	   Updated TTC
		indices: 0  1  2  3  4  5  6
		1) from 0 (included) to 1 (excluded) add 5;
		2) from 1 (included) to 4 (excluded) add 3;
		3) from 4 (included) to 5 (excluded) add 1;
		values:  11  8  7  6  3  2  0
	 */
	protected CumulatedIndexedDelaysList toIndexedDelaysList(TreeSet<IndexedDelay> delays, int max_depth) {
		//Handle exceptions
		if (delays == null) {
			metaCSPLogger.severe("Invalid input in function toPropagationTCDelays!!");
			throw new Error("Invalid input in function toPropagationTCDelays!!");
		}
		if (delays.isEmpty() || max_depth < 1) return new CumulatedIndexedDelaysList();
			
		//Cast the type
		ArrayList<Long> indices = new ArrayList<Long>();
		ArrayList<Double> values = new ArrayList<Double>();
		Iterator<IndexedDelay> it = delays.descendingIterator();
		IndexedDelay prev = delays.last();
		while (it.hasNext()) {
			IndexedDelay current = it.next();
			//Check unfeasible values
			if (current.getValue() == Double.NaN) {
				metaCSPLogger.severe("NaN input in function toPropagationTCDelays!!");
				throw new Error("NaN input in function toPropagationTCDelays!!");
			}
			if (current.getValue() == Double.NEGATIVE_INFINITY) {
				metaCSPLogger.severe("-Inf input in function toPropagationTCDelays!!");
				throw new Error("-Inf input in function toPropagationTCDelays!!");
			}
			if (prev.getIndex() < current.getIndex()) {
				metaCSPLogger.severe("Invalid IndexedDelays TreeSet!!");
				throw new Error("Invalid IndexedDelays TreeSet!!");
			}
			
			//Update the value only if positive and only if the index is lower than the max depth
			if (current.getValue() > 0 && current.getValue() < Double.MAX_VALUE && current.getIndex() < max_depth) {
				if (values.size() == 0) {
					//Add the index the first time its value is positive
					indices.add(new Long(current.getIndex()));
					values.add(current.getValue());
				}
				else if (prev.getIndex() == current.getIndex())				
					//Handle multiple delays in the same critical point
					values.set(values.size()-1, values.get(values.size()-1) + current.getValue());
				else {
					//Add the cumulative value if it is not the first.
					indices.add(new Long(current.getIndex()));
					values.add(values.get(values.size()-1) + current.getValue());
				}
			}
			prev = current;
		}
		CumulatedIndexedDelaysList propTCDelays = new CumulatedIndexedDelaysList();
		if (indices.size() > 0) {
			propTCDelays.size = indices.size();
			propTCDelays.indices = ArrayUtils.toPrimitive((Long[]) indices.toArray(new Long[indices.size()]));
			ArrayUtils.reverse(propTCDelays.indices);
			propTCDelays.values = ArrayUtils.toPrimitive((Double[]) values.toArray(new Double[values.size()]));
			ArrayUtils.reverse(propTCDelays.values);
		}
		return propTCDelays;
	}
	
	protected Pair<Double,Double> estimateTimeToCompletionDelays(AbstractTrajectoryEnvelopeTracker robotTracker1, RobotReport robotReport1, TreeSet<IndexedDelay> delaysRobot1, AbstractTrajectoryEnvelopeTracker robotTracker2, RobotReport robotReport2, TreeSet<IndexedDelay> delaysRobot2, CriticalSection cs) {
		if (this.fleetMasterInterface != null && fleetMasterInterface.checkTrajectoryEnvelopeHasBeenAdded(cs.getTe1()) && fleetMasterInterface.checkTrajectoryEnvelopeHasBeenAdded(cs.getTe2())) {
			CumulatedIndexedDelaysList te1TCDelays = toIndexedDelaysList(delaysRobot1, cs.getTe1().getTrajectory().getPoseSteering().length);
			metaCSPLogger.info("[estimateTimeToCompletionDelays] te1TCDelays: " + te1TCDelays.toString());
			CumulatedIndexedDelaysList te2TCDelays = toIndexedDelaysList(delaysRobot2, cs.getTe2().getTrajectory().getPoseSteering().length);
			metaCSPLogger.info("[estimateTimeToCompletionDelays] te2TCDelays: " + te2TCDelays.toString());
			return fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays);
		}
		return new Pair<Double, Double> (Double.NaN, Double.NaN);
	}
	
	protected void localCheckAndReviseWithDelayPropagation() {

		//System.out.println("Caller of updateDependencies(): " + Thread.currentThread().getStackTrace()[2]);
		synchronized(solver) {
			HashMap<Integer,RobotReport> currentReports = new HashMap<Integer,RobotReport>();
			HashMap<Integer,HashSet<Dependency>> currentDeps = new HashMap<Integer,HashSet<Dependency>>();
			HashMap<Integer,HashSet<Dependency>> artificialDependencies = new HashMap<Integer,HashSet<Dependency>>(); 
			HashSet<Dependency> currentReversibleDependencies = new HashSet<Dependency>();
			HashMap<Integer, TreeSet<IndexedDelay>> robotToIndexedTimeCompletionDelay = new HashMap<Integer, TreeSet<IndexedDelay>>();
			Pair<Double, Double> thisEstimatedDelays = new Pair<Double, Double>(Double.NaN, Double.NaN); //delays in the completion time of robot 1 (thisEstimatedDelays[0]) and robot 2 (thisEstimatedDelays[1]).
			
			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				//Update the coordinator view
				RobotReport robotReport = robotTracker.getRobotReport();
				currentReports.put(robotID,this.getRobotReport(robotID));
				robotToIndexedTimeCompletionDelay.put(robotID, new TreeSet<IndexedDelay>());
				
				synchronized(stoppingPoints) {
					if (stoppingPoints.containsKey(robotID)) {
						metaCSPLogger.info("Stopping points Robot"+robotID+": "+stoppingPoints.get(robotID).toString());
						for (int i = 0; i < stoppingPoints.get(robotID).size(); i++) {
							int stoppingPoint = stoppingPoints.get(robotID).get(i);
							int duration = stoppingTimes.get(robotID).get(i);
							if (robotReport.getPathIndex() <= stoppingPoint) {
								Dependency dep = new Dependency(robotTracker.getTrajectoryEnvelope(), null, stoppingPoint, 0);
								if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new HashSet<Dependency>());
								currentDeps.get(robotID).add(dep);
							}
							//Start waiting thread if the stopping point has been reached
							//if (Math.abs(robotReport.getPathIndex()-stoppingPoint) <= 1 && robotReport.getCriticalPoint() == stoppingPoint && !stoppingPointTimers.containsKey(robotID)) {
							if (Math.abs(robotReport.getPathIndex()-stoppingPoint) <= 1 && robotReport.getCriticalPoint() <= stoppingPoint && !stoppingPointTimers.containsKey(robotID)) {
								spawnWaitingThread(robotID, i, duration);
							}
							
							thisEstimatedDelays = new Pair<Double, Double>(duration/1000.0, Double.NaN);
							
							//Set the delay to robotID at stopping point stoppingPoint equal to duration.
							if (!robotToIndexedTimeCompletionDelay.get(robotID).add(new IndexedDelay(robotTracker.getTrajectoryEnvelope().getID(), 0, stoppingPoint, duration/1000.0))) {
								metaCSPLogger.severe("Delay not added for robot " + robotID + " (duplicated critical section, stopping point or wrong teID).");
								throw new Error("Delay not added for robot " + robotID + " (duplicated critical section, stopping point or wrong teID).");
							}
						}
					}
				}
			}
						
			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
				
				depsToCS.clear();				
				this.isBlocked = false;
				
				HashSet<CriticalSection> toRemove = new HashSet<CriticalSection>();
				for (CriticalSection cs : this.allCriticalSections) {
										
					AbstractTrajectoryEnvelopeTracker robotTracker1 = trackers.get(cs.getTe1().getRobotID());
					RobotReport robotReport1 = currentReports.get(cs.getTe1().getRobotID());
					AbstractTrajectoryEnvelopeTracker robotTracker2 = trackers.get(cs.getTe2().getRobotID());
					RobotReport robotReport2 = currentReports.get(cs.getTe2().getRobotID());
					if (this.fleetMasterInterface != null) {
						this.fleetMasterInterface.updateCurrentPathIdx(robotTracker1.getTrajectoryEnvelope().getID(), robotReport1.getPathIndex());
						this.fleetMasterInterface.updateCurrentPathIdx(robotTracker2.getTrajectoryEnvelope().getID(), robotReport2.getPathIndex());
					}
	
					//One or both robots past end of the critical section --> critical section is obsolete
					//This condition is still valid in case of delays (new mission phases <--> new trackers)
					if (robotReport1.getPathIndex() > cs.getTe1End() || robotReport2.getPathIndex() > cs.getTe2End()) {
						toRemove.add(cs);
						this.robotsToCS.remove(cs); //Just in case ... It should already be deleted when one of the robot could not stop before entering it.
						metaCSPLogger.finest("Obsolete critical section\n\t" + cs);
						continue;
					}
					
					//Will be assigned depending on current situation of robot reports...
					AbstractTrajectoryEnvelopeTracker waitingTracker = null;
					AbstractTrajectoryEnvelopeTracker drivingTracker = null;
					int waitingPoint = -1;
					int drivingCurrentIndex = -1;
					
					//The critical section could be still active. One of the two robots could already have exited the critical section,
					//but the information has not been received.					
					if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy || robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
						
						this.isBlocked = true;
						
						//Robot1 is parking in critical section. If it is the driver, make robot 2 wait.
						if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy) {
							waitingTracker = robotTracker2;
							drivingTracker = robotTracker1;
							drivingCurrentIndex = robotReport1.getPathIndex();
							thisEstimatedDelays = new Pair<Double, Double>(0., Double.POSITIVE_INFINITY); //robot 1 is parked in its goal, while robot2 should yield for an unbounded amount of time.
						}
						else if (robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
							waitingTracker = robotTracker1;
							drivingTracker = robotTracker2;	
							drivingCurrentIndex = robotReport2.getPathIndex();
							thisEstimatedDelays = new Pair<Double, Double>(Double.POSITIVE_INFINITY, 0.); //robot 2 is parked in its goal, while robot1 should yield for an unbounded amount of time.
						}
						//Compute the waiting point according to the decided precedence
						waitingPoint = getCriticalPoint(waitingTracker.getRobotID(), cs, drivingCurrentIndex);

						//Make the dependency
						if (!communicatedCPs.containsKey(waitingTracker) && drivingCurrentIndex <= waitingPoint 
								|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
				
							metaCSPLogger.finest("Robot" + drivingTracker.getRobotID() + " is parked, so Robot" + waitingTracker.getRobotID() + " will have to wait");
							if (!currentDeps.containsKey(waitingTracker.getRobotID())) currentDeps.put(waitingTracker.getRobotID(), new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingTracker.getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End());
							currentDeps.get(waitingTracker.getRobotID()).add(dep);
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
							if (!robotToIndexedTimeCompletionDelay.get(waitingTracker.getRobotID()).add(new IndexedDelay(waitingTracker.getTrajectoryEnvelope().getID(), cs.hashCode(), waitingPoint, Double.POSITIVE_INFINITY))) {
								metaCSPLogger.severe("Delay not added for robot " + waitingTracker.getRobotID() + " (duplicated critical section, stopping point or wrong teID).");
								throw new Error("Delay not added for robot " + waitingTracker.getRobotID() + " (duplicated critical section, stopping point or wrong teID).");
							}
						}
					}
					else {//Both robots are driving, let's determine an ordering for them through this critical section
						
						boolean update = true;
						synchronized(lockedRobots) {
							if (lockedRobots.containsValue(cs) && communicatedCPs.containsKey(robotTracker1) && communicatedCPs.containsKey(robotTracker2)) update = false;
						}
						
						//estimate the delays considering the current robot poses.
						thisEstimatedDelays = estimateTimeToCompletionDelays(robotTracker1, robotReport1, robotToIndexedTimeCompletionDelay.get(robotTracker1.getRobotID()), robotTracker2, robotReport2, robotToIndexedTimeCompletionDelay.get(robotTracker2.getRobotID()), cs);
						
						//Check if the robots can stop while changing the current order of accessing the critical section.
						ForwardModel fm1 = getForwardModel(robotReport1.getRobotID());
						ForwardModel fm2 = getForwardModel(robotReport2.getRobotID());
						boolean canStopRobot1 = false;
						boolean canStopRobot2 = false;
						boolean wakeUpinCSRobot1 = false;
						boolean wakeUpinCSRobot2 = false;
					
						if (update) {
							//Force the dependency for the robot footprint
							if (//the last critical point was before the critical section (can stop by induction)
									(communicatedCPs.containsKey(robotTracker1) && communicatedCPs.get(robotTracker1).getFirst() != -1 && communicatedCPs.get(robotTracker1).getFirst() < cs.getTe1Start())
									|| !communicatedCPs.containsKey(robotTracker1) && Math.max(0, robotReport1.getPathIndex()) < cs.getTe1Start())
									canStopRobot1 = true;
							else
								//Due to temporal delays we cannot trust the velocity.
								canStopRobot1 = fm1.canStop(robotTracker1.getTrajectoryEnvelope(), robotReport1, cs.getTe1Start(), false);			
						
							if ((communicatedCPs.containsKey(robotTracker2) && communicatedCPs.get(robotTracker2).getFirst() != -1 && communicatedCPs.get(robotTracker2).getFirst() < cs.getTe2Start())
								|| !communicatedCPs.containsKey(robotTracker2) && Math.max(0, robotReport2.getPathIndex()) < cs.getTe2Start())
								canStopRobot2 = true;
							else canStopRobot2 = fm2.canStop(robotTracker2.getTrajectoryEnvelope(), robotReport2, cs.getTe2Start(), false);
						}
						
						//Both the robots can stop before accessing the critical section --> follow ordering heuristic if FW model allows it
						if (!canStopRobot1 && !canStopRobot2) {
							//Both robots in critical section --> re-impose previously decided dependency if possible
							
							//Update the TreeSets for both the robots (if not already done).
							robotsToCS.remove(cs);
							
							//Handle the particular case of starting from a critical section at the FIRST communication.
							wakeUpinCSRobot1 = !communicatedCPs.containsKey(robotTracker1) && Math.max(robotReport1.getPathIndex(), 0) >= cs.getTe1Start();
							wakeUpinCSRobot2 = !communicatedCPs.containsKey(robotTracker2) && Math.max(robotReport2.getPathIndex(), 0) >= cs.getTe2Start();
							if (wakeUpinCSRobot1 || wakeUpinCSRobot2) {
								//Check if the robot that is parked can exit from critical section.
								drivingTracker = wakeUpinCSRobot1 ? robotTracker1 : robotTracker2;
								waitingTracker = wakeUpinCSRobot1 ? robotTracker2 : robotTracker1;	
								metaCSPLogger.finest("Robot" + robotReport1.getRobotID() + " wake up: " +wakeUpinCSRobot1 + ", Robot"+ robotReport2.getRobotID() + " wake up: " + wakeUpinCSRobot2 +" in CS " + cs);

							}
							else {
								if (this.CSToDepsOrder.containsKey(cs) && this.CSToDepsOrder.get(cs) != null) {
									//The critical section is not new and no re-plan happens.
									//Standard case: there is just one dependency in the previous set. Let's impose that.
									waitingTracker = this.CSToDepsOrder.get(cs).getFirst() == robotTracker1.getRobotID() ? robotTracker1 : robotTracker2;
									drivingTracker = waitingTracker.getRobotID() == robotReport1.getRobotID() ? robotTracker2 : robotTracker1;
									
									//If one of the robot was parked previously in critical section and has just started driving again,
									//then the set of previous constraints contains at least one dependency (due to parking).
								}
								else {
										metaCSPLogger.severe("Both cannot stop but lost critical section to dep. CS: " + cs + ", TE: " + cs.getTe1().getID() + ", " + cs.getTe2().getID() + ".");
										int ahead = isAhead(cs, robotReport1, robotReport2);
										if (ahead == 0) {
											if (!this.CSToDepsOrder.containsKey(cs)) throw new Error("FIXME! Lost dependency and order cannot be restored! Key value not found" );
											else if (this.CSToDepsOrder.get(cs) == null) throw new Error("FIXME! Lost dependency and order cannot be restored! Empty value." );	
										}
										else {
											drivingTracker = ahead == 1 ? robotTracker1 : robotTracker2;
											waitingTracker = ahead == 1 ? robotTracker2 : robotTracker1;
											metaCSPLogger.info("<<<<<<<<< Restoring the previous order with the isAhead function: Robot" + drivingTracker.getRobotID());
										}					
								}			
							}

							drivingCurrentIndex = drivingTracker.getRobotID() == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
																
							//check if the driving robot can really escape
							int waitingCurrentIndex = communicatedCPs.containsKey(waitingTracker) ? 
									//((communicatedCPs.get(trackers.get(waitingRobotID)).getFirst() == -1) ? 
									//		waitingTE.getPathLength()-1 : 
												communicatedCPs.get(waitingTracker).getFirst() : (
														waitingTracker.getRobotID() == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
							int lastIndexOfCSDriving = drivingTracker.getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							//if (lastWaitingRobotCP >= startingWaitingRobotCS) {
							if (!canExitCriticalSection(drivingCurrentIndex, waitingCurrentIndex, drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), lastIndexOfCSDriving)) {
								//it's too late for escaping. Let's create a deadlock: the parked robot should wait for the moving one,
								//while the other should be commanded to do not proceed beyond its last communicated CP. Both the dependencies should be added to the current set.									
								
								//The parked robot should wait for the other at its current path index till the moving robot has not been exited from its critical section.
								int artWaitingPoint = communicatedCPs.containsKey(drivingTracker) ? communicatedCPs.get(drivingTracker).getFirst() :
										(drivingTracker.getRobotID() == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
								int artDrivingCSEnd = waitingTracker.getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
								Dependency dep = new Dependency(drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), Math.max(0, artWaitingPoint), artDrivingCSEnd);
								if (!artificialDependencies.containsKey(drivingTracker.getRobotID())) artificialDependencies.put(drivingTracker.getRobotID(), new HashSet<Dependency>());
								artificialDependencies.get(drivingTracker.getRobotID()).add(dep);
								metaCSPLogger.info("Robot" + drivingTracker.getRobotID() + " cannot escape from CS: " + cs + ". Let's create a deadlock. Add artificial dependency for Robot" + drivingTracker.getRobotID() + " at " + dep.getWaitingPoint() + ".");
							}	
							metaCSPLogger.finest("Both can't. Driving Robot" + drivingTracker.getRobotID() + " at " + drivingCurrentIndex + " makes " + waitingTracker.getRobotID() + " waiting at CS " + cs + ".");
						}
						
	
						
						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						else if (canStopRobot1 && !canStopRobot2) {
							
							//Update the TreeSet for robot 2 (if not already done).
							robotsToCS.remove(cs, false);
							
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							metaCSPLogger.finest("One-can-one-can't-stop (1) and Robot" + drivingTracker.getRobotID() + " (can't) is ahead of Robot" + waitingTracker.getRobotID() + " (can) and CS is: " + cs);						
						}
	
						//Robot 2 can stop before entering critical section, robot 1 can't --> robot 2 waits
						else if (!canStopRobot1 && canStopRobot2) { 
							
							//Update the TreeSet for robot 1 (if not already done).
							robotsToCS.remove(cs, true);
							
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTracker.getRobotID() + " (can't) ahead of Robot" + waitingTracker.getRobotID() + " (can) and CS is: " + cs);
						}
						
						else { //Both the robot can stop.
							//FIXME If both the delays are infinite ... the closest goes first!!
							
							//If robot 1 has priority over robot 2
							if (thisEstimatedDelays.getSecond() < thisEstimatedDelays.getFirst()) { //robot 2 may be delayed for a lower time if it will be required to yield.
								drivingTracker = robotTracker1;
								waitingTracker = robotTracker2;
								metaCSPLogger.finest("Both-can-stop (1) and Robot" + drivingTracker.getRobotID() + " ahead of Robot" + waitingTracker.getRobotID() + " and CS is: " + cs);
							}
	
							//If robot 2 has priority over robot 1
							else {
								drivingTracker = robotTracker2;
								waitingTracker = robotTracker1;
								metaCSPLogger.finest("Both-can-stop (2) and Robot" + drivingTracker.getRobotID() + " ahead of Robot" + waitingTracker.getRobotID() + " and CS is: " + cs);
							}	
						}
						
						//Compute waiting path index point for waiting robot
						waitingPoint = getCriticalPoint(waitingTracker.getRobotID(), cs, drivingTracker.getRobotID() == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
						
						//Impose holding the previous dependence in some cases
						if (wakeUpinCSRobot1 && communicatedCPs.containsKey(robotTracker2)) {
							if (communicatedCPs.get(robotTracker2).getFirst() > waitingPoint) {
								metaCSPLogger.info("Wake-up Robot" + robotReport1.getRobotID() + "; revising waiting point of Robot" + robotReport2.getRobotID() + ": " + waitingPoint + "-->" + communicatedCPs.get(robotTracker2).getFirst());
								waitingPoint = communicatedCPs.get(robotTracker2).getFirst();
								escapingCSToWaitingRobotIDandCP.put(cs, new Pair<Integer,Integer>(robotReport2.getRobotID(),waitingPoint));
							}
						}
						else if (wakeUpinCSRobot2 && communicatedCPs.containsKey(robotTracker1)) {
							if (communicatedCPs.get(robotTracker1).getFirst() > waitingPoint) {
								metaCSPLogger.info("Wake-up Robot" + robotReport2.getRobotID() + "; revising waiting point of Robot" + robotReport1.getRobotID() + ": " + waitingPoint + "-->" + communicatedCPs.get(robotTracker1).getFirst());
								waitingPoint = communicatedCPs.get(robotTracker1).getFirst();
								escapingCSToWaitingRobotIDandCP.put(cs, new Pair<Integer,Integer>(robotReport1.getRobotID(),waitingPoint));
							}
						}
						
						if (escapingCSToWaitingRobotIDandCP.containsKey(cs) && escapingCSToWaitingRobotIDandCP.get(cs).getFirst() == waitingTracker.getRobotID()) {
							waitingPoint = escapingCSToWaitingRobotIDandCP.get(cs).getSecond();
							metaCSPLogger.info("Use escaping waiting point. Make Robot" + waitingTracker.getRobotID() + " stopping at " + waitingPoint + ".");
						}
						
						if (!update) {
							waitingPoint = CSToDepsOrder.get(cs).getSecond();
							metaCSPLogger.info("Locking of Robot" + waitingTracker.getRobotID() + " makes it stopping at " + waitingPoint + "" + ".");
						}
						
						if (waitingPoint >= 0) {		
							//Make new dependency
							int drivingCSEnd =  drivingTracker.getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							if (!currentDeps.containsKey(waitingTracker.getRobotID())) currentDeps.put(waitingTracker.getRobotID(), new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd);
							currentDeps.get(waitingTracker.getRobotID()).add(dep);
							
							//Update the history of decisions for each critical section that has been updated
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
							if (canStopRobot1 && canStopRobot2) currentReversibleDependencies.add(dep);
							double estimatedDelayWaiter = waitingTracker.getRobotID() == robotTracker1.getRobotID() ? thisEstimatedDelays.getFirst() : thisEstimatedDelays.getSecond(); 
							if (!robotToIndexedTimeCompletionDelay.get(waitingTracker.getRobotID()).add(new IndexedDelay(waitingTracker.getTrajectoryEnvelope().getID(), cs.hashCode(), waitingPoint, estimatedDelayWaiter))) {
								metaCSPLogger.severe("Delay not added for robot " + waitingTracker.getRobotID() + " (duplicated critical section, stopping point or wrong teID).");
								throw new Error("Delay not added for robot " + waitingTracker.getRobotID() + " (duplicated critical section, stopping point or wrong teID).");
							}
						}
						else {
							//If robot is asked to wait in an invalid path point, throw error and give up!
							metaCSPLogger.severe("Waiting point < 0 for critical section " + cs);
							throw new Error("Waiting point < 0 for critical section " + cs);
						}
					}						
				}
				
				//Remove obsolete critical sections
				for (CriticalSection cs : toRemove) {
					this.allCriticalSections.remove(cs);
					this.escapingCSToWaitingRobotIDandCP.remove(cs);
				}
				
				//increment the counter
				this.criticalSectionCounter.addAndGet(toRemove.size());
			}

			
			//get current dependencies
			synchronized(currentDependencies) {
				
				//FIXME: already synchronized (so maybe is ok currentDependencies = computeClosestDependencies(currentDeps, artificialDependencies);)
				HashSet<Dependency> closestDeps  = computeClosestDependencies(currentDeps, artificialDependencies);
				currentDependencies.clear();
				currentDependencies.addAll(closestDeps);
				
				//find cycles and revise dependencies if necessary
				currentDeps = findCurrentCycles(currentDeps, artificialDependencies, currentReversibleDependencies, currentReports, robotIDs);
			
				//send revised dependencies
				HashMap<Integer,Dependency> constrainedRobotIDs = new HashMap<Integer,Dependency>();
				for (Dependency dep : currentDependencies) {
					constrainedRobotIDs.put(dep.getWaitingRobotID(),dep);				
				}
				for (int robotID : robotIDs) {
					AbstractTrajectoryEnvelopeTracker tracker = null;
					synchronized (trackers) {
							tracker = trackers.get(robotID);
						}
					int maxDelay = 2*(MAX_TX_DELAY+CONTROL_PERIOD+tracker.getTrackingPeriodInMillis());
					if (constrainedRobotIDs.containsKey(robotID)) {
						Dependency dep = constrainedRobotIDs.get(robotID);
						metaCSPLogger.finest("Set critical point " + dep.getWaitingPoint() + " to Robot" + dep.getWaitingRobotID() +".");
						boolean retransmitt = communicatedCPs.containsKey(tracker) && communicatedCPs.get(tracker).getFirst() == dep.getWaitingPoint() && currentReports.get(robotID).getCriticalPoint() != dep.getWaitingPoint()
								&& ((int)(Calendar.getInstance().getTimeInMillis()-communicatedCPs.get(tracker).getSecond()) > maxDelay);
						setCriticalPoint(dep.getWaitingRobotID(), dep.getWaitingPoint(), retransmitt);
	
					}
					else {
						boolean retransmitt = communicatedCPs.containsKey(tracker) && communicatedCPs.get(tracker).getFirst() == -1 && currentReports.get(robotID).getCriticalPoint() != -1
								&& ((int)(Calendar.getInstance().getTimeInMillis()-communicatedCPs.get(tracker).getSecond()) > maxDelay);
						setCriticalPoint(robotID, -1, retransmitt);
					}
				}
			}
		}
	}


	
}
