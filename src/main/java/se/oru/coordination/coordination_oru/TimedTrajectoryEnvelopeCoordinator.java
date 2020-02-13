package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PropagationTCDelays;

public abstract class TimedTrajectoryEnvelopeCoordinator extends TrajectoryEnvelopeCoordinator {
	protected ArrayList<HashMap<Integer, Double>> csToTimeCompletionDelay = new ArrayList<HashMap<Integer, Double>>();
	protected AbstractFleetMasterInterface fleetMasterInterface = null;
	
	public TimedTrajectoryEnvelopeCoordinator(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
	}

	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * Note: this function should be called before placing the first robot.
	 * ATTENTION: It is required the user to check that all the paths will lay in the given area.
	 * @param origin_x The x origin of the map.
	 * @param origin_y The y origin of the map.
	 * @param origin_theta The theta origin of the map.
	 * @param resolution The resolution of the map (in meters/cell).
	 * @param width Number of columns of the map (in cells).
	 * @param height Number of rows of the map (in cells).
	 * @param debug <code>true</code> enables showing the content of each GridMap.
	 */
	@Deprecated
	public void instantiateFleetMaster(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean debug) {
		this.fleetMasterInterface = new FleetMasterInterface(origin_x, origin_y, origin_theta, resolution, width, height, debug);
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * @param mapYAMLFile file to the global map of the environment (with absolute path specified as "/home/...").
	 * @param debug <code>true</code> enables showing the content of each GridMap.
	 * Note: it is required the map to contain all the possible paths.
	 */
	public void instantiateFleetMaster(String mapYAMLFile, boolean debug) {
		this.fleetMasterInterface = new FleetMasterInterface(mapYAMLFile, debug);
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
	}
	
	/**
	 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
	 * @param mapYAMLFile file to the global map of the environment (with absolute path specified as "/home/...").
	 * Note: it is required the map to contain all the possible paths.
	 */
	public void instantiateFleetMaster(String mapYAMLFile) {
		instantiateFleetMaster(mapYAMLFile, false);
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
		fleetMasterInterface.setTrajParams(robotID, maxVel, maxVelRev, useSteerDriveVel, maxRotationalVel, maxRotationalVelRev, maxSteeringAngleVel, maxAcc, maxRotationalAcc, maxSteeringAngleAcc);
	}
	
	@Override
	protected void onSettingDefaultFootprint() {
		this.fleetMasterInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
	}
		
	@Override
	protected void onClearingTe(TrajectoryEnvelope te) {
		fleetMasterInterface.clearPath(te.getID());
	}
	
	@Override
	protected void onAddingTe(TrajectoryEnvelope te) {
		if (!fleetMasterInterface.addPath(te)) 
			metaCSPLogger.severe("Unable to add the path to the fleetmaster gridmap. Check if the map contains the given path.");
	}
		
	@Override
	//returns true if robot1 should go before robot2
	//returns false if robot2 should go before robot1
	protected boolean getOrder(AbstractTrajectoryEnvelopeTracker robotTracker1, RobotReport robotReport1, AbstractTrajectoryEnvelopeTracker robotTracker2, RobotReport robotReport2, CriticalSection cs) {

		//If both can stop before entering, use ordering function (or closest if no ordering function)
		metaCSPLogger.finest("Both robots can stop at " + cs);
		
		if (fleetMasterInterface.checkTrajectoryEnvelopeHasBeenAdded(cs.getTe1()) && fleetMasterInterface.checkTrajectoryEnvelopeHasBeenAdded(cs.getTe2())) {
			PropagationTCDelays te1TCDelays = new PropagationTCDelays();
			PropagationTCDelays te2TCDelays = new PropagationTCDelays();
			Pair<Double, Double> delays = fleetMasterInterface.queryTimeDelay(cs, te1TCDelays, te2TCDelays);
			return delays.getFirst() < delays.getSecond() ? false : true;
		}
		
		if (yieldIfParking) {
			boolean robot1ParksInCS = cs.getTe1End() == cs.getTe1().getPathLength()-1;
			boolean robot2ParksInCS = cs.getTe2End() == cs.getTe2().getPathLength()-1;
			if (robot1ParksInCS && !robot2ParksInCS) {
				metaCSPLogger.finest("Robot" + cs.getTe1().getRobotID() + " will park in " + cs + ", letting Robot" + cs.getTe2().getRobotID() + " go first");
				return false;
			}
			else if (!robot1ParksInCS && robot2ParksInCS) {
				metaCSPLogger.finest("Robot" + cs.getTe2().getRobotID() + " will park in " + cs + ", letting Robot" + cs.getTe1().getRobotID() + " go first");
				return true;
			}
			else if (robot1ParksInCS && robot2ParksInCS) 
				metaCSPLogger.finest("Both Robot" + cs.getTe1().getRobotID() + " and Robot" + cs.getTe2().getRobotID() + " will park in " + cs + ". Decide according to the heuristic.");
		}
			
		RobotAtCriticalSection r1atcs = new RobotAtCriticalSection(robotReport1, cs);
		RobotAtCriticalSection r2atcs = new RobotAtCriticalSection(robotReport2, cs);
		boolean ret = false;
		if (this.comparators.size() > 0) ret = (this.comparators.compare(r1atcs,r2atcs) < 0);
		//No ordering function, decide an ordering based on distance (closest goes first)
		else ret = ((cs.getTe2Start()-robotReport2.getPathIndex()) > (cs.getTe1Start()-robotReport1.getPathIndex()));
		if (ret && muted.contains(robotReport2.getRobotID())) return false;
		if (!ret && muted.contains(robotReport1.getRobotID())) return true;
		return ret;
	}
	
	@Override
	protected void localCheckAndRevise() {

		//System.out.println("Caller of updateDependencies(): " + Thread.currentThread().getStackTrace()[2]);
		synchronized(solver) {
			HashMap<Integer,RobotReport> currentReports = new HashMap<Integer,RobotReport>();
			HashMap<Integer,HashSet<Dependency>> currentDeps = new HashMap<Integer,HashSet<Dependency>>();
			HashMap<Integer,HashSet<Dependency>> artificialDependencies = new HashMap<Integer,HashSet<Dependency>>(); 
			HashSet<Dependency> currentReversibleDependencies = new HashSet<Dependency>();
			
			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				RobotReport robotReport = robotTracker.getRobotReport();
				//Update the coordinator view
				currentReports.put(robotID,this.getRobotReport(robotID));
				
				if (!(robotTracker instanceof TrajectoryEnvelopeTrackerDummy))
					fleetMasterInterface.updateCurrentPathIdx(robotTracker.getTrajectoryEnvelope().getID(), Math.max(0, robotReport.getPathIndex()));

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
						}
					}
				}
			}
			
			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
				
				depsToCS.clear();
				this.blocked = false;
				
				HashSet<CriticalSection> toRemove = new HashSet<CriticalSection>();
				for (CriticalSection cs : this.allCriticalSections) {
										
					//Will be assigned depending on current situation of robot reports...
					int waitingRobotID = -1;
					int drivingRobotID = -1;
					AbstractTrajectoryEnvelopeTracker waitingTracker = null;
					AbstractTrajectoryEnvelopeTracker drivingTracker = null;
					int drivingCurrentIndex = -1;
					TrajectoryEnvelope waitingTE = null;
					TrajectoryEnvelope drivingTE = null;
	
					AbstractTrajectoryEnvelopeTracker robotTracker1 = trackers.get(cs.getTe1().getRobotID());
					RobotReport robotReport1 = currentReports.get(cs.getTe1().getRobotID());
					AbstractTrajectoryEnvelopeTracker robotTracker2 = trackers.get(cs.getTe2().getRobotID());
					RobotReport robotReport2 = currentReports.get(cs.getTe2().getRobotID());
	
					//One or both robots past end of the critical section --> critical section is obsolete
					//This condition is still valid in case of delays (new mission phases <--> new trackers)
					if (robotReport1.getPathIndex() > cs.getTe1End() || robotReport2.getPathIndex() > cs.getTe2End()) {
						toRemove.add(cs);
						this.robotsToCS.remove(cs); //Just in case ... It should already be deleted when one of the robot could not stop before entering it.
						metaCSPLogger.finest("Obsolete critical section\n\t" + cs);
						continue;
					}
					
					/*DEBUG For debugging purposes
					if (robotsToCS.next(cs.getTe1().getRobotID(),cs.getTe2().getRobotID()) != null && cs.equals(robotsToCS.next(cs.getTe1().getRobotID(),cs.getTe2().getRobotID()))){
						metaCSPLogger.info("Closest critical section for Robot" + cs.getTe1().getRobotID() + " is " + cs.toString());
					}
					if (robotsToCS.next(cs.getTe2().getRobotID(),cs.getTe1().getRobotID()) != null && cs.equals(robotsToCS.next(cs.getTe2().getRobotID(),cs.getTe1().getRobotID()))){
						metaCSPLogger.info("Closest critical section for Robot" + cs.getTe2().getRobotID() + " is " + cs.toString());
					}*/
	
					//The critical section could be still active. One of the two robots could already have exited the critical section,
					//but the information has not been received.
					int waitingPoint = -1;
					
					if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy || robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
						
						boolean createAParkingDep = false;
						this.blocked = true;
						
						//Robot1 is parking in critical section. If it is the driver, make robot 2 wait.
						if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy) {
							drivingRobotID = robotReport1.getRobotID();
							waitingRobotID = robotReport2.getRobotID();
							drivingTE = robotTracker1.getTrajectoryEnvelope();
							waitingTE = robotTracker2.getTrajectoryEnvelope();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							waitingPoint = getCriticalPoint(robotReport2.getRobotID(), cs, robotReport1.getPathIndex());
	
							if (!communicatedCPs.containsKey(waitingTracker) && robotReport1.getPathIndex() <= waitingPoint 
									|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
								createAParkingDep = true;
							}
							
						}
						else if (robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
							drivingRobotID = robotReport2.getRobotID();
							waitingRobotID = robotReport1.getRobotID();
							drivingTE = robotTracker2.getTrajectoryEnvelope();
							waitingTE = robotTracker1.getTrajectoryEnvelope();
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							waitingPoint = getCriticalPoint(robotReport1.getRobotID(), cs, robotReport2.getPathIndex());
													
							if (!communicatedCPs.containsKey(waitingTracker) && robotReport2.getPathIndex() <= waitingPoint
									|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
								createAParkingDep = true;						
							}
						}
						
						if (createAParkingDep) {
							int drivingCSEnd = drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							metaCSPLogger.finest("Robot" + drivingRobotID + " is parked, so Robot" + waitingRobotID + " will have to wait");
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
							currentDeps.get(waitingRobotID).add(dep);
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
						}
					}
					else {//Both robots are driving, let's determine an ordering for them through this critical section
						
						boolean update = true;
						synchronized(lockedRobots) {
							if (lockedRobots.containsValue(cs) && communicatedCPs.containsKey(robotTracker1) && communicatedCPs.containsKey(robotTracker2)) {
								update = false;
							}
						}
						
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
								drivingRobotID = wakeUpinCSRobot1 ? robotReport1.getRobotID() : robotReport2.getRobotID();
								waitingRobotID = wakeUpinCSRobot1 ? robotReport2.getRobotID() : robotReport1.getRobotID();	
								metaCSPLogger.finest("Robot" + robotReport1.getRobotID() + " wake up: " +wakeUpinCSRobot1 + ", Robot"+ robotReport2.getRobotID() + " wake up: " + wakeUpinCSRobot2 +" in CS " + cs);

							}
							else {
								if (this.CSToDepsOrder.containsKey(cs) && this.CSToDepsOrder.get(cs) != null) {
									//The critical section is not new and no re-plan happens.
									//Standard case: there is just one dependency in the previous set. Let's impose that.
									waitingRobotID = this.CSToDepsOrder.get(cs).getFirst();
									drivingRobotID = waitingRobotID == robotReport1.getRobotID() ? robotReport2.getRobotID() : robotReport1.getRobotID();
									
									//If one of the robot was parked previously in critical section and has just started driving again,
									//then the set of previous constraints contains at least one dependency (due to parking).
								}
								else {
										metaCSPLogger.severe("Both cannot stop but lost critical section to dep. CS: " + cs + ", TE: " + cs.getTe1().getID() + ", " + cs.getTe2().getID() + ".");
										int ahead = isAhead(cs,robotReport1,robotReport2);
										if (ahead == 0) {
											if (!this.CSToDepsOrder.containsKey(cs)) throw new Error("FIXME! Lost dependency and order cannot be restored! Key value not found" );
											else if (this.CSToDepsOrder.get(cs) == null) throw new Error("FIXME! Lost dependency and order cannot be restored! Empty value." );	
										}
										else {
											drivingRobotID = ahead == 1 ? robotReport1.getRobotID() : robotReport2.getRobotID();
											waitingRobotID = drivingRobotID == robotReport1.getRobotID() ? robotReport2.getRobotID() : robotReport1.getRobotID();
											metaCSPLogger.info("<<<<<<<<< Restoring the previous order with the isAhead function: Robot" + drivingRobotID);
										}					
								}			
							}

							drivingCurrentIndex = drivingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
							drivingTE = drivingRobotID == robotReport1.getRobotID() ? robotTracker1.getTrajectoryEnvelope() : robotTracker2.getTrajectoryEnvelope();
							waitingTE = waitingRobotID == robotReport1.getRobotID() ? robotTracker1.getTrajectoryEnvelope() : robotTracker2.getTrajectoryEnvelope();	
							waitingTracker = waitingRobotID == robotReport1.getRobotID() ? robotTracker1 : robotTracker2;
							drivingTracker = drivingRobotID == robotReport1.getRobotID() ? robotTracker1 : robotTracker2;
																
							//check if the driving robot can really escape
							int waitingCurrentIndex = communicatedCPs.containsKey(waitingTracker) ? 
									//((communicatedCPs.get(trackers.get(waitingRobotID)).getFirst() == -1) ? 
									//		waitingTE.getPathLength()-1 : 
												communicatedCPs.get(waitingTracker).getFirst() : (
										waitingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
							int lastIndexOfCSDriving = drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							//if (lastWaitingRobotCP >= startingWaitingRobotCS) {
							if (!canExitCriticalSection(drivingCurrentIndex, waitingCurrentIndex, drivingTE, waitingTE,lastIndexOfCSDriving)) {
								//it's too late for escaping. Let's create a deadlock: the parked robot should wait for the moving one,
								//while the other should be commanded to do not proceed beyond its last communicated CP. Both the dependencies should be added to the current set.									
								
								//The parked robot should wait for the other at its current path index till the moving robot has not been exited from its critical section.
								int artWaitingPoint = communicatedCPs.containsKey(drivingTracker) ? communicatedCPs.get(drivingTracker).getFirst() :
										(drivingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
								int artDrivingCSEnd = waitingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
								Dependency dep = new Dependency(drivingTE,waitingTE,Math.max(0, artWaitingPoint),artDrivingCSEnd);
								if (!artificialDependencies.containsKey(drivingRobotID)) artificialDependencies.put(drivingRobotID, new HashSet<Dependency>());
								artificialDependencies.get(drivingRobotID).add(dep);
								metaCSPLogger.info("Robot" + drivingRobotID + " cannot escape from CS: " + cs + ". Let's create a deadlock. Add artificial dependency for Robot" + drivingRobotID + " at " + dep.getWaitingPoint() + ".");
							}	
							metaCSPLogger.finest("Both can't. Driving Robot" + drivingRobotID + " at " + drivingCurrentIndex + " makes " + waitingRobotID + " waiting at CS " + cs + ".");
						}
						
	
						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						else if (canStopRobot1 && !canStopRobot2) {
							
							//Update the TreeSet for robot 2 (if not already done).
							robotsToCS.remove(cs, false);
							
							drivingCurrentIndex = robotReport2.getPathIndex();
							drivingRobotID = robotReport2.getRobotID();
							waitingRobotID = robotReport1.getRobotID();
							drivingTE = robotTracker2.getTrajectoryEnvelope();
							waitingTE = robotTracker1.getTrajectoryEnvelope();
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							metaCSPLogger.finest("One-can-one-can't-stop (1) and Robot" + drivingTE.getRobotID() + " (can't) is ahead of Robot" + waitingTE.getRobotID() + " (can) and CS is: " + cs);						
						}
	
						//Robot 2 can stop before entering critical section, robot 1 can't --> robot 2 waits
						else if (!canStopRobot1 && canStopRobot2) { 
							
							//Update the TreeSet for robot 1 (if not already done).
							robotsToCS.remove(cs, true);
							
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingRobotID = robotReport1.getRobotID();
							waitingRobotID = robotReport2.getRobotID();
							drivingTE = robotTracker1.getTrajectoryEnvelope();
							waitingTE = robotTracker2.getTrajectoryEnvelope();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTE.getRobotID() + " (can't) ahead of Robot" + waitingTE.getRobotID() + " (can) and CS is: " + cs);
						}
						
						else { //Both the robot can stop.
							
							//If robot 1 has priority over robot 2
							if (getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs)) {
								drivingCurrentIndex = robotReport1.getPathIndex();
								drivingRobotID = robotReport1.getRobotID();
								waitingRobotID = robotReport2.getRobotID();
								drivingTE = robotTracker1.getTrajectoryEnvelope();
								waitingTE = robotTracker2.getTrajectoryEnvelope();
								drivingTracker = robotTracker1;
								waitingTracker = robotTracker2;
								metaCSPLogger.finest("Both-can-stop (1) and Robot" + drivingTE.getRobotID() + " ahead of Robot" + waitingTE.getRobotID() + " and CS is: " + cs);
							}
	
							//If robot 2 has priority over robot 1
							else {
								drivingCurrentIndex = robotReport2.getPathIndex();
								drivingRobotID = robotReport2.getRobotID();
								waitingRobotID = robotReport1.getRobotID();
								drivingTE = robotTracker2.getTrajectoryEnvelope();
								waitingTE = robotTracker1.getTrajectoryEnvelope();
								drivingTracker = robotTracker2;
								waitingTracker = robotTracker1;
								metaCSPLogger.finest("Both-can-stop (2) and Robot" + drivingTE.getRobotID() + " ahead of Robot" + waitingTE.getRobotID() + " and CS is: " + cs);
							}	
						}
						
						//Compute waiting path index point for waiting robot
						waitingPoint = getCriticalPoint(waitingRobotID, cs, drivingCurrentIndex);
						
						//Impose holding the previous dependence in some cases
						if (wakeUpinCSRobot1 && communicatedCPs.containsKey(robotTracker2)) {
							if (communicatedCPs.get(robotTracker2).getFirst() > waitingPoint) {
								metaCSPLogger.info("Wake-up Robot"+robotReport1.getRobotID()+"; revising waiting point of Robot" + robotReport2.getRobotID() + ": " + waitingPoint + "-->" + communicatedCPs.get(robotTracker2).getFirst());
								waitingPoint = communicatedCPs.get(robotTracker2).getFirst();
								escapingCSToWaitingRobotIDandCP.put(cs, new Pair<Integer,Integer>(robotReport2.getRobotID(),waitingPoint));
							}
						}
						else if (wakeUpinCSRobot2 && communicatedCPs.containsKey(robotTracker1)) {
							if (communicatedCPs.get(robotTracker1).getFirst() > waitingPoint) {
								metaCSPLogger.info("Wake-up Robot"+robotReport2.getRobotID()+"; revising waiting point of Robot" + robotReport1.getRobotID() + ": " + waitingPoint + "-->" + communicatedCPs.get(robotTracker1).getFirst());
								waitingPoint = communicatedCPs.get(robotTracker1).getFirst();
								escapingCSToWaitingRobotIDandCP.put(cs, new Pair<Integer,Integer>(robotReport1.getRobotID(),waitingPoint));
							}
						}
						
						if (escapingCSToWaitingRobotIDandCP.containsKey(cs) && escapingCSToWaitingRobotIDandCP.get(cs).getFirst() == waitingRobotID) {
							waitingPoint = escapingCSToWaitingRobotIDandCP.get(cs).getSecond();
							metaCSPLogger.info("Use escaping waiting point. Make Robot" + waitingRobotID + " stopping at " + waitingPoint + ".");
						}
						
						if (!update) {
							waitingPoint = CSToDepsOrder.get(cs).getSecond();
							metaCSPLogger.info("Locking of Robot" + waitingRobotID + " makes it stopping at " + waitingPoint + "" + ".");
						}
						
						if (waitingPoint >= 0) {		
							//Make new dependency
							int drivingCSEnd =  drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
							currentDeps.get(waitingRobotID).add(dep);
							
							//Update the history of decisions for each critical section that has been updated
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
							if (canStopRobot1 && canStopRobot2) currentReversibleDependencies.add(dep);
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
