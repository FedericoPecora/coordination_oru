package se.oru.coordination.coordination_oru;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;

import javax.swing.SwingUtilities;

import org.apache.commons.collections.comparators.ComparatorChain;
import org.jgrapht.DirectedGraph;
import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.alg.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.graph.ClassBasedEdgeFactory;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DirectedMultigraph;
import org.jgrapht.graph.DirectedSubgraph;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.metacsp.framework.Constraint;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.PermutationsWithRepetition;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.StringUtils;


/**
 * This class provides coordination for a fleet of robots. An instantiatable {@link TrajectoryEnvelopeCoordinator}
 * must provide an implementation of a time keeping method, a {@link TrajectoryEnvelope} tracker factory, and
 * a criteria with which robots are to be prioritized.
 * 
 * @author fpa
 *
 */
public abstract class TrajectoryEnvelopeCoordinator extends AbstractTrajectoryEnvelopeCoordinator {
	
	//@note: currentOrdersGraph and currentCyclesList should be synchronized with allCriticalSection variable.
	protected SimpleDirectedGraph<Integer,String> currentOrdersGraph = new SimpleDirectedGraph<Integer,String>(String.class);
	protected HashMap<Pair<Integer,Integer>, HashSet<ArrayList<Integer>>> currentCyclesList = new HashMap<Pair<Integer,Integer>, HashSet<ArrayList<Integer>>>();
	
	//Robots currently involved in a re-plan which critical point cannot increase beyond the one used for re-plan
	//till the re-plan has not finished yet.
	protected HashMap<Integer,CriticalSection> lockedRobots = new HashMap<Integer,CriticalSection>();
	
	protected boolean breakDeadlocksByReordering = true;
	protected boolean breakDeadlocksByReplanning = true;
	protected boolean avoidDeadlockGlobally = false;
	
	//True if waiting for deadlocks to happen.
	protected boolean staticReplan = false;
	
	/**
	 * Set whether the coordinator should try to break deadlocks by attempting to re-plan
	 * the path of one of the robots involved in an unsafe cycle.
	 * @param value <code>true</code> if deadlocks should be broken by re-planning.
	 */
	public void setBreakDeadlocksByReplanning(boolean value) {
		this.breakDeadlocksByReplanning = value;
		this.avoidDeadlockGlobally = false;
	}
	
	/**
	 * Set whether the coordinator should try to break deadlocks by disallowing an arbitrary
	 * ordering involved in a loop in the current dependency graph.
	 * @param value <code>true</code> if deadlocks should be broken.
	 */
	public void setBreakDeadlocksByReordering(boolean value) {
		this.breakDeadlocksByReordering = value;
		this.avoidDeadlockGlobally = false;
	}

	/**
	 * Set whether the coordinator should use the global strategy to avoid deadlocks
	 * @param value <code>true</code> if deadlocks should be broken.
	 */
	public void setAvoidDeadlocksGlobally(boolean value) {
		this.avoidDeadlockGlobally = value;
		if (value == true) {
			this.breakDeadlocksByReordering = false;
			this.breakDeadlocksByReplanning = false;
		}
	}
	
	/**
	 * Set whether the coordinator should try to break deadlocks by both arbitrary
	 * re-ordering and re-planning.
	 * @param value <code>true</code> if deadlocks should be broken.
	 */
	@Deprecated
	public void setBreakDeadlocks(boolean value) {
		this.avoidDeadlockGlobally = false;
		this.setBreakDeadlocksByReordering(value);
		this.setBreakDeadlocksByReplanning(value);
	}
	
	
	/**
	 * Set whether waiting for deadlocks to happen before starting re-plan to recover from deadlock.
	 * @param value <code>true</code> if all the robots should yield at their deadlocking critical points 
	 * before starting a recovery strategy based on re-plan.
	 */
	public void setStaticReplan(boolean value) {
		this.staticReplan = value;
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinator}, with control period 1000 msec,
	 * and temporal resolution 1000 (milliseconds).
	 */
	public TrajectoryEnvelopeCoordinator() {
		this(1000, 1000.0);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinator}, with given control period and temporal resolution.
	 * @param CONTROL_PERIOD The control period of the coordinator, e.g., 1000 msec.
	 * @param TEMPORAL_RESOLUTION The temporal resolution of the control period, e.g., 1000 for milliseconds.
	 */
	public TrajectoryEnvelopeCoordinator(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
		this.CONTROL_PERIOD = CONTROL_PERIOD;
		this.TEMPORAL_RESOLUTION = TEMPORAL_RESOLUTION;
		//FOR PRINTING
		if (overlay) {
			//clear screen
			System.out.printf(((char) 0x1b) + "[2J");
			//make window from row 8 to bottom (200)
			System.out.printf(((char) 0x1b) + "[8;200r");
		}
		setupLogging();
	}

	private List<List<Integer>> findSimpleUnsafeCycles(SimpleDirectedGraph<Integer,Dependency> g) {
		
				JohnsonSimpleCycles<Integer, Dependency> cycleFinder = new JohnsonSimpleCycles<Integer, Dependency>(g);
				List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
				List<List<Integer>> unsafeCycles = new ArrayList<List<Integer>>();
				
				//Find all the unsafe cycles
				
				//For each cycle...
				//example [1,2]
				for (List<Integer> cycle : cycles) {
					
					//Get edges along the cycle...
					ArrayList<Dependency> edgesAlongCycle = new ArrayList<Dependency>();
					Collections.reverse(cycle);
					for (int i = 0; i < cycle.size(); i++) {
						Dependency dep = null;
						if (i < cycle.size()-1) dep = g.getEdge(cycle.get(i), cycle.get(i+1));
						else dep = g.getEdge(cycle.get(i), cycle.get(0));
						edgesAlongCycle.add(dep);
					}
					//[1,2], [2,1]

					//Check for unsafety condition, that is:
					//  All along the cycle, check if there are (u,v,dep1) and (v,w,dep2) such that
					//    v.dep2.waitingpoint <= v.dep1.releasingpoint
					//  And if so mark cycle as deadlock
					//[1,2] ,[2,1]			[1,2], [2,3], [3,1]
					//i = 0: [1,2], [2,1]   [1,2], [2,3]
					//i = 1: [2,1], [1,2]	[2,3], [3,1]
					//i = 2:  -             [3,1], [1,2]
					for (int i = 0; i < edgesAlongCycle.size(); i++) {
						boolean safe = true;
						if (unsafePair(edgesAlongCycle.get(i), edgesAlongCycle.get(i < edgesAlongCycle.size()-1 ? i+1 : 0))) {
							safe = false;
						}
						if (safe) {
							metaCSPLogger.finest("Cycle: " + edgesAlongCycle + " is deadlock-free");
							break;
						}
						if (i == edgesAlongCycle.size()-1) {
							metaCSPLogger.info("Cycle: " + edgesAlongCycle + " is NOT deadlock-free");
							unsafeCycles.add(cycle);
						}
					}			
				}
				
				return unsafeCycles;
	}
	

	private SimpleDirectedGraph<Integer,Dependency> depsToGraph(SimpleDirectedGraph<Integer,Dependency> g, HashSet<Dependency> deps) {
		for (Dependency dep : deps) {
			if (!g.containsVertex(dep.getWaitingRobotID())) g.addVertex(dep.getWaitingRobotID());
			if (!g.containsVertex(dep.getDrivingRobotID())) g.addVertex(dep.getDrivingRobotID());
			g.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
		}
		return g;
	}
	
	private HashSet<Dependency> updateCurrentDependencies(HashMap<Integer,TreeSet<Dependency>> allDeps, HashMap<Integer, TreeSet<Dependency>> artificialDeps, Set<Integer> robotIDs) {
		
		metaCSPLogger.finest("currentDeps: " + allDeps.toString() + ".");	
		metaCSPLogger.finest("artificialDeps: " + artificialDeps.toString() + ".");	
		
		HashSet<Dependency> closestDeps = new HashSet<Dependency>();
		for (Integer robotID : robotIDs) {						
			if (allDeps.containsKey(robotID) || artificialDeps.containsKey(robotID)) {
				Dependency firstDep = null;
				Dependency firstArtificialDep = null;
				if (allDeps.containsKey(robotID)){
					firstDep = allDeps.get(robotID).first();
				}
				if (artificialDeps.containsKey(robotID)) {
					firstArtificialDep = artificialDeps.get(robotID).first();
					metaCSPLogger.info("Artificial critical point " + firstArtificialDep.getWaitingPoint() + " for Robot" + firstArtificialDep.getWaitingRobotID() +".");
				}
				
				Dependency depToSend = null; 
				if (firstDep != null)
					depToSend = firstArtificialDep != null ? (firstDep.compareTo(firstArtificialDep) < 0 ? firstDep : firstArtificialDep) : firstDep;
				else
					depToSend = firstArtificialDep;
				closestDeps.add(depToSend);
			}
		}
		return closestDeps;
	}
	
	private void findCurrentCycles(HashMap<Integer,TreeSet<Dependency>> currentDeps, HashMap<Integer,TreeSet<Dependency>> artificialDeps, HashSet<Dependency> reversibleDeps, HashMap<Integer,RobotReport> currentReports, Set<Integer> robotIDs) {
		
		HashMap<Integer,TreeSet<Dependency>> allDeps = new HashMap<Integer, TreeSet<Dependency>>();
		allDeps.putAll(currentDeps);
		
		//1. COMPUTE UNSAFE CYCLES
		
		//Dep graph G = (V,E)
		//  V = robots involved in current deps
		//  E = {(u,v,dep) | exists dep stating that robot u should wait for robot v}
		//Deadlock iff:
		//  Cycle <r_1, r_2, ... r_1> in G
		//  Exists one selection of edges along the cycle such that
		//    Exists (u,v,dep1) and (v,w,dep2) such that
		//      v.dep2.waitingpoint <= v.dep1.releasingpoint
		
		SimpleDirectedGraph<Integer,Dependency> g = new SimpleDirectedGraph<Integer, Dependency>(Dependency.class);
		g = depsToGraph(g, currentDependencies);
		List<List<Integer>> unsafeCycles = findSimpleUnsafeCycles(g);	
		
		//2. IF THERE ARE UNSAFE CYCLES AND RE-ORDERING IS ENABLED, TRY RE-ORDER
		if (breakDeadlocksByReordering) {
			for (List<Integer> cycle : unsafeCycles) {
				//Get edges along the cycle...
				ArrayList<Dependency> reversibleDepsAlongCycle = new ArrayList<Dependency>();
				for (int i = 0; i < cycle.size(); i++) {
					Dependency dep = null;
					if (i < cycle.size()-1) {
						dep = g.getEdge(cycle.get(i), cycle.get(i+1));
					}
					else {
						dep = g.getEdge(cycle.get(i), cycle.get(0));
					}
					if (reversibleDeps.contains(dep) && !artificialDeps.containsKey(dep.getWaitingRobotID())) reversibleDepsAlongCycle.add(dep);
				}
				
				//For each reversible edge ... 
				//(note that if the dependency is reversible, then it is neither an artificial dependency,
				//nor one related to an active parking critical section)
				for (Dependency dep : reversibleDepsAlongCycle) {
					
					if (depsToCS.containsKey(dep)) {
						CriticalSection cs = depsToCS.get(dep);
						
						//Try reversing the precedence
						int waitingRobotID = dep.getDrivingRobotID();
						TrajectoryEnvelope waitingTE = dep.getDrivingTrajectoryEnvelope();
						int drivingRobotID = dep.getWaitingRobotID();
						TrajectoryEnvelope drivingTE = dep.getWaitingTrajectoryEnvelope();
						int drivingCurrentIndex = currentReports.get(drivingRobotID).getPathIndex();
						int waitingPoint = getCriticalPoint(waitingRobotID, cs, drivingCurrentIndex);
						int drivingCSEnd = (drivingRobotID == cs.getTe1().getRobotID()) ? cs.getTe1End() : cs.getTe2End();
						
						Dependency revDep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
						
						//and add the reversed dependency to the new waiting one
						TreeSet<Dependency> allOldDrivingRobotDeps = allDeps.get(dep.getDrivingRobotID());
						allOldDrivingRobotDeps.add(revDep);
						
						//remove the dependency to the old waiting robot
						TreeSet<Dependency> allOldWaitingRobotDeps = allDeps.get(dep.getWaitingRobotID());
						allOldWaitingRobotDeps.remove(dep);
						if (allOldWaitingRobotDeps.isEmpty()) allDeps.remove(dep.getWaitingRobotID());
											
						//update a temporary map between critical sections and orders
						HashMap<CriticalSection, Pair<Integer,Integer>> CSToDepsOrderTmp = new HashMap<CriticalSection, Pair<Integer,Integer>>();
						CSToDepsOrderTmp.putAll(CSToDepsOrder);
						CSToDepsOrderTmp.put(cs, new Pair<Integer,Integer>(waitingRobotID,revDep.getWaitingPoint()));
						
						//create a temporary map between dependencies and critical sections
						HashMap<Dependency,CriticalSection> depsToCSTmp = new HashMap<Dependency, CriticalSection>();
						depsToCSTmp.putAll(depsToCS);
						depsToCSTmp.remove(dep);
						depsToCSTmp.put(revDep,cs);

						//update currentDeps
						HashSet<Dependency> currentDepsTmp = updateCurrentDependencies(allDeps, artificialDeps, robotIDs);
						SimpleDirectedGraph<Integer,Dependency> gTmp = new SimpleDirectedGraph<Integer, Dependency>(Dependency.class);
						gTmp = depsToGraph(gTmp,currentDepsTmp);
						
						//compute cycles again. If the number of cycles is lower, keep this solution
						List<List<Integer>> unsafeCyclesTmp = findSimpleUnsafeCycles(gTmp);
						if (unsafeCyclesTmp.size() < unsafeCycles.size()) {
							metaCSPLogger.info("REVERSING a precedence constraint to break a deadlock: " + revDep.getWaitingRobotID() + " now waits for " + revDep.getDrivingRobotID());
							g = gTmp;
							currentDependencies = currentDepsTmp;
							currentDeps = allDeps;
							unsafeCycles = unsafeCyclesTmp; //FIXME termination
							CSToDepsOrder = CSToDepsOrderTmp;
							depsToCS = depsToCSTmp;
							break;
						}
					}
				}
			}
		}
		
		//2. OTHERWISE, IF RE-PLAN IS ENABLED, TRY REPLANNING
		
		if (breakDeadlocksByReplanning) {
			HashSet<Integer> spawnedThreadRobotSet = new HashSet<Integer>();
			for (List<Integer> cycle : unsafeCycles) {
				boolean spawnThread = true;
				
				//Get edges along the cycle...
				ArrayList<Dependency> depsAlongCycle = new ArrayList<Dependency>();
				HashSet<Integer> deadlockedRobots = new HashSet<Integer>();
				for (int i = 0; i < cycle.size(); i++) {
					Dependency dep = null;
					if (i < cycle.size()-1) {
						dep = g.getEdge(cycle.get(i), cycle.get(i+1));
					}
					else {
						dep = g.getEdge(cycle.get(i), cycle.get(0));
					}
					depsAlongCycle.add(dep);
					deadlockedRobots.add(dep.getWaitingRobotID());
					if (spawnedThreadRobotSet.contains(dep.getWaitingRobotID())) {
						spawnThread = false;
						break;
					}
				}
				
				if (spawnThread) {
					//Get other robots (all the robot in the maximum connected set of a deadlocked one)
					
					/*------------------------------------------------------------------------------------------
					 * Notes on connectivity: example code. While strongly connected components are useful for 
					 * computing cycles, connected components are used to get the set of robots to be considered 
					 * as obstacles when replanning.
					 * -----------------------------------------------------------------------------------------
					 * 
					 * 	SimpleDirectedGraph<Integer,Integer> g = new SimpleDirectedGraph(Integer.class);
						for (int v=1; v< 6; v++) g.addVertex(v);
						g.addEdge(1,2,1);
						g.addEdge(2,3,2);
						g.addEdge(3,1,3);
						g.addEdge(4,2,4);
						g.addEdge(3,5,5);
						ConnectivityInspector<Integer,Integer> connInsp = new ConnectivityInspector<Integer,Integer>(g);
						KosarajuStrongConnectivityInspector<Integer,Integer> kconnInsp = new KosarajuStrongConnectivityInspector<Integer,Integer>(g);
						Set<Integer> cc2 = connInsp.connectedSetOf(2);
						Set<Integer> cc3 = connInsp.connectedSetOf(3);
						List<Set<Integer>> scc = kconnInsp.stronglyConnectedSets();
						System.out.println("cc2: " + cc2.toString() + ", cc3: " + cc3.toString() + ", scc: " + scc.toString());
						//Result: cc2: [1, 2, 3, 4, 5], cc3: [1, 2, 3, 4, 5], scc: [[4], [1, 2, 3], [5]]
						---------------------------------------------------------------------------------------
					 */
					
					ConnectivityInspector<Integer,Dependency> connInsp = new ConnectivityInspector<Integer,Dependency>(g);
					HashSet<Integer> allRobots = (HashSet<Integer>) connInsp.connectedSetOf(cycle.get(0));
					spawnReplanning(depsAlongCycle, allRobots);
					spawnedThreadRobotSet.addAll(deadlockedRobots);
				}
			}
		}
	}
	
	//returns true if robot1 should go before robot2
	//returns false if robot2 should go before robot1
	protected boolean getOrder(AbstractTrajectoryEnvelopeTracker robotTracker1, RobotReport robotReport1, AbstractTrajectoryEnvelopeTracker robotTracker2, RobotReport robotReport2, CriticalSection cs) {

		//If both can stop before entering, use ordering function (or closest if no ordering function)
		metaCSPLogger.finest("Both robots can stop at " + cs);
		
		//NO MORE USED
		if (yieldIfParking) {
			boolean robot1ParksInCS = cs.getTe1End() == cs.getTe1().getPathLength()-1;
			boolean robot2ParksInCS = cs.getTe2End() == cs.getTe2().getPathLength()-1;
			if (robot1ParksInCS && !robot2ParksInCS) {
				metaCSPLogger.info("Robot" + cs.getTe1().getRobotID() + " will park in " + cs + ", letting Robot" + cs.getTe2().getRobotID() + " go first");
				return false;
			}
			else if (!robot1ParksInCS && robot2ParksInCS) {
				metaCSPLogger.info("Robot" + cs.getTe2().getRobotID() + " will park in " + cs + ", letting Robot" + cs.getTe1().getRobotID() + " go first");
				return true;
			}
			else if (robot1ParksInCS && robot2ParksInCS) 
				metaCSPLogger.info("Both Robot" + cs.getTe1().getRobotID() + " and Robot" + cs.getTe2().getRobotID() + " will park in " + cs + ". Decide according to the heuristic.");
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
	protected void updateDependencies() {
		synchronized(solver) {
			if (this.avoidDeadlockGlobally) globalCheckAndRevise();
			else localCheckAndRevise();
		}
	}
	
	protected void localCheckAndRevise() {

		//System.out.println("Caller of updateDependencies(): " + Thread.currentThread().getStackTrace()[2]);
		synchronized(solver) {
			HashMap<Integer,TreeSet<Dependency>> currentDeps = new HashMap<Integer,TreeSet<Dependency>>();
			HashMap<Integer,TreeSet<Dependency>> artificialDependencies = new HashMap<Integer,TreeSet<Dependency>>(); 
			HashSet<Dependency> currentReversibleDependencies = new HashSet<Dependency>();
			
			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				RobotReport robotReport = robotTracker.getRobotReport();
				synchronized(stoppingPoints) {
					if (stoppingPoints.containsKey(robotID)) {
						metaCSPLogger.info("Stopping points Robot"+robotID+": "+stoppingPoints.get(robotID).toString());
						for (int i = 0; i < stoppingPoints.get(robotID).size(); i++) {
							int stoppingPoint = stoppingPoints.get(robotID).get(i);
							int duration = stoppingTimes.get(robotID).get(i);
							if (robotReport.getPathIndex() <= stoppingPoint) {
								Dependency dep = new Dependency(robotTracker.getTrajectoryEnvelope(), null, stoppingPoint, 0);
								if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new TreeSet<Dependency>());
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
			
			HashMap<Integer,RobotReport> currentReports = new HashMap<Integer,RobotReport>();
			
			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
				
				depsToCS.clear();
				
				//Update the coordinator view
				for (int robotID : robotIDs) {
					currentReports.put(robotID,this.getRobotReport(robotID));
				}
				
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
						metaCSPLogger.finest("Obsolete critical section\n\t" + cs);
						continue;
					}
	
					//The critical section could be still active. One of the two robots could already have exited the critical section,
					//but the information has not been received.
					int waitingPoint = -1;
					
					if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy || robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
						
						boolean createAParkingDep = false;
						
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
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
							currentDeps.get(waitingRobotID).add(dep);
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
						}
					}
					else {//Both robots are driving, let's determine an ordering for them through this critical section
						
						boolean update = true;
						synchronized(lockedRobots) {
							if (lockedRobots.containsValue(cs)) {
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
						if (canStopRobot1 && canStopRobot2) {
	
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
	
						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						else if (canStopRobot1 && !canStopRobot2) {
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
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingRobotID = robotReport1.getRobotID();
							waitingRobotID = robotReport2.getRobotID();
							drivingTE = robotTracker1.getTrajectoryEnvelope();
							waitingTE = robotTracker2.getTrajectoryEnvelope();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTE.getRobotID() + " (can't) ahead of Robot" + waitingTE.getRobotID() + " (can) and CS is: " + cs);
						}
	
						else {			
														
							//Both robots in critical section --> re-impose previously decided dependency if possible
						
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
										throw new Error("FIXME! Lost dependency! " );							
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
								if (!artificialDependencies.containsKey(drivingRobotID)) artificialDependencies.put(drivingRobotID, new TreeSet<Dependency>());
								artificialDependencies.get(drivingRobotID).add(dep);
								metaCSPLogger.info("Robot" + drivingRobotID + " cannot escape from CS: " + cs + ". Let's create a deadlock. Add artificial dependency for Robot" + drivingRobotID + " at " + dep.getWaitingPoint() + ".");
							}	
							metaCSPLogger.finest("Both can't. Driving Robot" + drivingRobotID + " at " + drivingCurrentIndex + " makes " + waitingRobotID + " waiting at CS " + cs + ".");
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
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
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
					//this.CSToDepsOrder.remove(cs);
					this.escapingCSToWaitingRobotIDandCP.remove(cs);
				}
				
				//increment the counter
				this.criticalSectionCounter.addAndGet(toRemove.size());
			}
			
			//get current dependencies
			synchronized(currentDependencies) {
				currentDependencies = updateCurrentDependencies(currentDeps, artificialDependencies, robotIDs);
				
				//find cycles and revise dependencies if necessary
				findCurrentCycles(currentDeps, artificialDependencies, currentReversibleDependencies, currentReports, robotIDs);
			
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
	
	private void spawnReplanning(ArrayList<Dependency> deadlockedDeps, HashSet<Integer> allConnectedRobots) {
		
		boolean tryReplanning = true;
		final HashSet<Integer> deadlockedRobots = new HashSet<Integer>();
		final boolean useStaticReplan = staticReplan;
		
		//collect the set of robots for which the re-plan will eventually tried
		for (Dependency dep : deadlockedDeps) {
			deadlockedRobots.add(dep.getWaitingRobotID());
			deadlockedRobots.add(dep.getDrivingRobotID());
			RobotReport rrWaiting = getRobotReport(dep.getWaitingRobotID());
			
			//avoid to re-plan if one of the robots is parked.
			//In case of static re-plan, wait for a deadlock to happen before starting it
			if (inParkingPose(dep.getDrivingRobotID()) || inParkingPose(dep.getWaitingRobotID())
					|| useStaticReplan && ((dep.getWaitingPoint() == 0 ? (rrWaiting.getPathIndex() < 0) : (rrWaiting.getPathIndex() < dep.getWaitingPoint()-1)))) {
				tryReplanning = false;
				break;
			}
		}
		
		//You should lock the robots if you want to start replaning. 
		//In this way, the last critical point communicated when the re-plan is started is forced not to be updated
		synchronized (lockedRobots) {
			boolean tryLocking = true;
			for (int robotID : deadlockedRobots) {
				if (lockedRobots.containsKey(robotID)) {
					tryLocking = false;
					break;
				}
			}
			if (tryReplanning && tryLocking) {
				final HashSet<Integer> allRobots = allConnectedRobots;
				for (Dependency dep : deadlockedDeps) {
					lockedRobots.put(dep.getWaitingRobotID(), depsToCS.get(dep));
				}
				metaCSPLogger.info("Lock robots: " + deadlockedRobots.toString());
				metaCSPLogger.info("Will re-plan for one of the following deadlocked robots: " + deadlockedRobots + " (" + allRobots + ")...");
				new Thread() {
					public void run() {
						rePlanPath(deadlockedRobots, allRobots, useStaticReplan);
					}
				}.start();

			}
		}
	}
	
	private boolean unsafePair(Dependency dep1, Dependency dep2) {
		if (dep2.getWaitingPoint() <= dep1.getReleasingPoint()) return true;
		return false;
	}
	
	protected void rePlanPath(HashSet<Integer> robotsToReplan, HashSet<Integer> allRobots, boolean useStaticReplan) {
		for (int robotID : robotsToReplan) {
			int currentWaitingIndex = -1;
			Pose currentWaitingPose = null;
			Pose currentWaitingGoal = null;
			PoseSteering[] oldPath = null;
			
			//FIXME not synchronized on current dependencies
			for (Dependency dep : getCurrentDependencies()) {
				if (dep.getWaitingRobotID() == robotID) {
					currentWaitingIndex = dep.getWaitingPoint();
					currentWaitingPose = dep.getWaitingPose();
					Trajectory traj = dep.getWaitingTrajectoryEnvelope().getTrajectory();
					oldPath = traj.getPoseSteering();
					currentWaitingGoal = oldPath[oldPath.length-1].getPose();
					break;
				}
			}
			
			int[] otherRobotIDs = new int[allRobots.size()-1];
			int counter = 0;
			for (int otherRobotID : allRobots) if (otherRobotID != robotID) otherRobotIDs[counter++] = otherRobotID;
			
			//FIXME not synchronized on current dependencies
			Geometry[] obstacles = getObstaclesInCriticalPoints(otherRobotIDs);
			
			metaCSPLogger.info("Attempting to re-plan path of Robot" + robotID + " (with obstacles for robots " + Arrays.toString(otherRobotIDs) + ")...");
			AbstractMotionPlanner mp = null;
			if (this.motionPlanners.containsKey(robotID)) mp = this.motionPlanners.get(robotID);
			else mp = this.defaultMotionPlanner;
			synchronized (mp) {
				PoseSteering[] newPath = doReplanning(mp, currentWaitingPose, currentWaitingGoal, obstacles);
				if (newPath != null && newPath.length > 0) {
					PoseSteering[] newCompletePath = new PoseSteering[newPath.length+currentWaitingIndex];
					for (int i = 0; i < newCompletePath.length; i++) {
						if (i < currentWaitingIndex) newCompletePath[i] = oldPath[i];
						else newCompletePath[i] = newPath[i-currentWaitingIndex];
					}
					replacePath(robotID, newCompletePath, currentWaitingIndex, robotsToReplan, useStaticReplan);
					metaCSPLogger.info("Successfully re-planned path of Robot" + robotID);
					break;
				}
				else {
					metaCSPLogger.info("Failed to re-plan path of Robot" + robotID);
				}
			}
		}
		synchronized (lockedRobots) {
			for (int robotID : robotsToReplan) lockedRobots.remove(robotID);
			metaCSPLogger.info("Unlock robots: " + robotsToReplan.toString());
		}
	}
	
	@Override
	protected void cleanUpRobotCS(int robotID, int lastWaitingPoint) {
		synchronized (allCriticalSections) {
			metaCSPLogger.info("Cleaning up critical sections of Robot" + robotID);
			ArrayList<CriticalSection> toRemove = new ArrayList<CriticalSection>();
			//Clear the critical sections for which we have stored a dependency ...
			for (CriticalSection cs : CSToDepsOrder.keySet()) {
				if (cs.getTe1().getRobotID() == robotID || cs.getTe2().getRobotID() == robotID) toRemove.add(cs);
			}
			//... and all the critical sections which are currently alive.
			for (CriticalSection cs : allCriticalSections) {
				if ((cs.getTe1().getRobotID() == robotID || cs.getTe2().getRobotID() == robotID) && !toRemove.contains(cs)) {
					toRemove.add(cs);
					metaCSPLogger.info("WARNING: removing critical section which was not associated to a dependency.");
					//increment the counter
					if (cs.getTe1().getRobotID() == robotID && (cs.getTe1Start() <= lastWaitingPoint || lastWaitingPoint == -1) || 
							cs.getTe2().getRobotID() == robotID && (cs.getTe2Start() <= lastWaitingPoint || lastWaitingPoint == -1))
							this.criticalSectionCounter.incrementAndGet();
				}
			}
			for (CriticalSection cs : toRemove) {
				if (this.avoidDeadlockGlobally) {
					if (CSToDepsOrder.containsKey(cs)) {
						int waitingRobID = CSToDepsOrder.get(cs).getFirst();
						int drivingRobID = cs.getTe1().getRobotID() == waitingRobID ? cs.getTe2().getRobotID() : cs.getTe1().getRobotID(); 
						deleteEdge(new Pair<Integer,Integer>(waitingRobID,drivingRobID));
					}
				}
				CSToDepsOrder.remove(cs);
				allCriticalSections.remove(cs);
				escapingCSToWaitingRobotIDandCP.remove(cs);
			}
		}
	}
	
	/**
	 * Replace the path of a robot's {@link TrajectoryEnvelope} on the fly.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} is to be recomputed.
	 * @param newPath The path based on which the new {@link TrajectoryEnvelope} should be computed.
	 * @param lockedRobotIDs The set of robots which have been locked when the re-plan started.
	 */
	public void replacePath(int robotID, PoseSteering[] newPath, int breakingPathIndex, HashSet<Integer> lockedRobotIDs, boolean useStaticReplan) {
		
		synchronized (solver) {
			
			synchronized (allCriticalSections) {
				//Get current envelope
				TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);
						
				if (viz != null) {
					viz.removeEnvelope(te);
				}
								
				//------------------ (dynamic re-plan) --------------------
				//store the order of precedence (and waiting points) for every 
				//critical section involving the robot which is replanning
				HashMap<CriticalSection,Pair<Integer,Integer>> holdingCS = new HashMap<CriticalSection,Pair<Integer,Integer>>();
				if (!useStaticReplan) {
					for (CriticalSection cs : CSToDepsOrder.keySet()) {
						if (cs.getTe1().getRobotID() == robotID && cs.getTe1Start() <= breakingPathIndex || 
								cs.getTe2().getRobotID() == robotID && cs.getTe2Start() <= breakingPathIndex) 
							holdingCS.put(cs, CSToDepsOrder.get(cs));
					}
				}
				//---------------------------------------------------------
				
				//Remove CSs involving this robot, clearing the history.
				cleanUpRobotCS(te.getRobotID(), breakingPathIndex);
				//---------------------------------------------------------
				
				//Make new envelope
				TrajectoryEnvelope newTE = solver.createEnvelopeNoParking(robotID, newPath, "Driving", this.getFootprint(robotID));
		
				//Notify tracker
				synchronized (trackers) {
					this.trackers.get(robotID).updateTrajectoryEnvelope(newTE);
				}
				
				//Stitch together with rest of constraint network (temporal constraints with parking envelopes etc.)
				for (Constraint con : solver.getConstraintNetwork().getOutgoingEdges(te)) {
					if (con instanceof AllenIntervalConstraint) {
						AllenIntervalConstraint aic = (AllenIntervalConstraint)con;
						if (aic.getTypes()[0].equals(AllenIntervalConstraint.Type.Meets)) {
							TrajectoryEnvelope newEndParking = solver.createParkingEnvelope(robotID, PARKING_DURATION, newTE.getTrajectory().getPose()[newTE.getTrajectory().getPose().length-1], "whatever", getFootprint(robotID));
							TrajectoryEnvelope oldEndParking = (TrajectoryEnvelope)aic.getTo();
		
							solver.removeConstraints(solver.getConstraintNetwork().getIncidentEdges(te));
							solver.removeVariable(te);
							solver.removeConstraints(solver.getConstraintNetwork().getIncidentEdges(oldEndParking));
							solver.removeVariable(oldEndParking);
							
							AllenIntervalConstraint newMeets = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
							newMeets.setFrom(newTE);
							newMeets.setTo(newEndParking);
							solver.addConstraint(newMeets);
		
							break;
						}
					}
				}
				
				if (viz != null) {
					viz.addEnvelope(newTE);
				}
								
				//Add as if it were a new envelope, that way it will be accounted for in computeCriticalSections()
				envelopesToTrack.add(newTE);
				
				//Recompute CSs involving this robot
				computeCriticalSections();
				
				//------------------ (static re-plan or starting from critical section) ----------------------
				if (useStaticReplan || breakingPathIndex == 0) {
					//The robot is waiting at its current critical point. Hence, it can stop by definition.
					//To keep it simple, restart the communication.
					synchronized (trackers) {
						//if (breakingPathIndex == 0) trackers.get(robotID).resetStartingTimeInMillis();
						communicatedCPs.remove(trackers.get(robotID));
					}
				}
				else {
					
					//------------------ (dynamic re-plan) --------------------
					for (CriticalSection cs1 : allCriticalSections) {
						if (cs1.getTe1().getRobotID() == robotID && cs1.getTe1Start() <= breakingPathIndex || 
							cs1.getTe2().getRobotID() == robotID && cs1.getTe2Start() <= breakingPathIndex) {
							for (CriticalSection cs2 : holdingCS.keySet()) {
								if (cs1.getTe1().getRobotID() == cs2.getTe1().getRobotID() && cs1.getTe2().getRobotID() == cs2.getTe2().getRobotID() ||
									cs1.getTe1().getRobotID() == cs2.getTe2().getRobotID() && cs1.getTe2().getRobotID() == cs2.getTe1().getRobotID()) {
									//the same set of robots
									if ((cs1.getTe1().getRobotID() == robotID && cs2.getTe1().getRobotID() == robotID && cs1.getTe1Start() == cs2.getTe1Start() &&
											(cs1.getTe2Start() == cs2.getTe2Start() || cs1.getTe2End() == cs2.getTe2End())) ||
											(cs1.getTe1().getRobotID() == robotID && cs2.getTe2().getRobotID() == robotID && cs1.getTe1Start() == cs2.getTe2Start() &&
											(cs1.getTe2Start() == cs2.getTe1Start() || cs1.getTe2End() == cs2.getTe1End())) ||
											(cs1.getTe2().getRobotID() == robotID && cs2.getTe1().getRobotID() == robotID && cs1.getTe2Start() == cs2.getTe1Start() &&
											(cs1.getTe1Start() == cs2.getTe2Start() || cs1.getTe1End() == cs2.getTe2End())) ||
											(cs1.getTe2().getRobotID() == robotID && cs2.getTe2().getRobotID() == robotID && cs1.getTe2Start() == cs2.getTe2Start() &&
											(cs1.getTe1Start() == cs2.getTe1Start() || cs1.getTe1End() == cs2.getTe1End()))) {
										CSToDepsOrder.put(cs1, holdingCS.get(cs2));
										
										if (this.avoidDeadlockGlobally) {
											//re-add dependency to cyclesList and currentOrdersGraph
											HashSet<Pair<Integer,Integer>> edgesToAdd = new HashSet<Pair<Integer,Integer>>();
											int waitingRobotID = holdingCS.get(cs2).getFirst();
											int drivingRobotID = holdingCS.get(cs2).getFirst() == cs2.getTe1().getRobotID() ? cs2.getTe2().getRobotID() : cs2.getTe1().getRobotID();
											edgesToAdd.add(new Pair<Integer,Integer>(waitingRobotID, drivingRobotID));
											addEdges(edgesToAdd);
										}
									}
								}		
							}
						}
					}
				}
								
				envelopesToTrack.remove(newTE);
				
				synchronized (lockedRobots) {
					for (int ID : lockedRobotIDs) lockedRobots.remove(ID);
					metaCSPLogger.info("Unlock robots: " + lockedRobotIDs.toString());
				}
				
				updateDependencies();											
				
			}
		}
	}
	
	/**
	 * Truncate the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point. This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be truncated.
	 * 	 * @return true if the envelope is successfully truncated.
	 */
	public boolean truncateEnvelope(int robotID) {
		synchronized (solver) {
			
			synchronized (lockedRobots) {
				if (lockedRobots.containsKey(robotID)) return false;
			}
			
			TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);
			AbstractTrajectoryEnvelopeTracker tet = this.trackers.get(robotID); 
			if (!(tet instanceof TrajectoryEnvelopeTrackerDummy)) {
				int earliestStoppingPathIndex = this.getForwardModel(robotID).getEarliestStoppingPathIndex(te, this.getRobotReport(robotID));
				if (earliestStoppingPathIndex != -1) {
					metaCSPLogger.info("Truncating " + te + " at " + earliestStoppingPathIndex);
					
					//Compute and add new TE, remove old TE (both driving and final parking)
					PoseSteering[] truncatedPath = Arrays.copyOf(te.getTrajectory().getPoseSteering(), earliestStoppingPathIndex+1);
					
					//replace the path of this robot (will compute new envelope)
					replacePath(robotID, truncatedPath, truncatedPath.length, new HashSet<Integer>(robotID),false);
					
				}
			}
			
			return true;
		}
	}
	
	/**
	 * Reverse the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point. This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be reversed.
	 * @return true if the envelope is successfully reversed.
	 */
	public boolean reverseEnvelope(int robotID) {
		synchronized (solver) {
			
			synchronized (lockedRobots) {
				if (lockedRobots.containsKey(robotID)) return false;
			}
			
			TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);
			AbstractTrajectoryEnvelopeTracker tet = this.trackers.get(robotID); 
			if (!(tet instanceof TrajectoryEnvelopeTrackerDummy)) {
				int earliestStoppingPathIndex = this.getForwardModel(robotID).getEarliestStoppingPathIndex(te, this.getRobotReport(robotID));
				if (earliestStoppingPathIndex != -1) {
					metaCSPLogger.info("Reversing " + te + " at " + earliestStoppingPathIndex);
					
					//Compute and add new TE, remove old TE (both driving and final parking)
					PoseSteering[] truncatedPath = Arrays.copyOf(te.getTrajectory().getPoseSteering(), earliestStoppingPathIndex+1);
					PoseSteering[] overallPath = new PoseSteering[truncatedPath.length*2-1];
					for (int i = 0; i < truncatedPath.length; i++) overallPath[i] = truncatedPath[i];
					for (int i = 1; i < truncatedPath.length; i++) overallPath[truncatedPath.length-1+i] = truncatedPath[truncatedPath.length-i];
										
					//replace the path of this robot (will compute new envelope)
					replacePath(robotID, overallPath, earliestStoppingPathIndex, new HashSet<Integer>(robotID),false);
					
				}
			}
			
			return true;
		}
	}
	
	protected void deleteEdges(HashSet<Pair<Integer,Integer>> edgesToDelete) {	
		synchronized(allCriticalSections) {
			if (edgesToDelete == null) return;
			synchronized (currentOrdersGraph) {
				synchronized (currentCyclesList) {
					metaCSPLogger.finest("Deleting edges " + edgesToDelete.toString() +".");
					//metaCSPLogger.finest("Graph before deletion: " + currentOrdersGraph.toString());
					for (Pair<Integer,Integer> edge : edgesToDelete) {
						deleteEdge(edge);
					}	
					//metaCSPLogger.info("... after deletion: " + currentOrdersGraph.toString());
				}
			}
		}
	}
	
	protected void deleteEdge(Pair<Integer,Integer> edge) {
		synchronized(allCriticalSections) {
			if (edge == null) return;
			if (currentOrdersGraph.getEdge(edge.getFirst(), edge.getSecond()) != null) {
				if (currentCyclesList.containsKey(edge)) {
					HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> toRemove = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
					for (ArrayList<Integer> cycle : currentCyclesList.get(edge)) {
						for (int i = 0; i < cycle.size(); i++) {
							Pair<Integer,Integer> otherEdge = new Pair<Integer,Integer>(cycle.get(i),cycle.get(i < cycle.size()-1 ? i+1 : 0));
							if (!otherEdge.equals(edge)) {
								if (!toRemove.containsKey(otherEdge)) toRemove.put(otherEdge, new HashSet<ArrayList<Integer>>());
								toRemove.get(otherEdge).add(cycle);
							}
						}
					}
					
					for (Pair<Integer, Integer> key : toRemove.keySet()) {
						if (currentCyclesList.containsKey(key)) {
							currentCyclesList.get(key).removeAll(toRemove.get(key));
							if (currentCyclesList.get(key).isEmpty()) currentCyclesList.remove(key);
						}
					}
					currentCyclesList.remove(edge);
				}
				//hashcode: "s" + startVertex + "t" + targetVertex + "c" + number of edges from startVertex to targetVertex
				String hashCode = currentOrdersGraph.getEdge(edge.getFirst(), edge.getSecond());
				int cmark = hashCode.indexOf("c");
				int numEdge = Integer.valueOf(hashCode.substring(cmark+1, hashCode.length()));
				currentOrdersGraph.removeEdge(edge.getFirst(), edge.getSecond());
				//metaCSPLogger.finest("deleting one edge " + edge.toString() + "updating the counter from " + numEdge);
				if (numEdge > 1) {
					if (!currentOrdersGraph.containsVertex(edge.getFirst())) currentOrdersGraph.addVertex(edge.getFirst());
					if (!currentOrdersGraph.containsVertex(edge.getSecond())) currentOrdersGraph.addVertex(edge.getSecond());
					String newHashCode = new String(hashCode.substring(0,cmark+1).concat(String.valueOf(numEdge-1)));
					currentOrdersGraph.addEdge(edge.getFirst(), edge.getSecond(), newHashCode);
				}
			}
		}
	}
	
	protected void addEdges(HashSet<Pair<Integer,Integer>> edgesToAdd) {
		
		if (edgesToAdd == null || edgesToAdd.isEmpty()) return;
		
		synchronized(allCriticalSections) {
			HashSet<Pair<Integer,Integer>> toAdd = new HashSet<Pair<Integer,Integer>>();
			
			//add the edges if not already in the graph
			for (Pair<Integer,Integer> edge : edgesToAdd) {
				if (!currentOrdersGraph.containsEdge(edge.getFirst(), edge.getSecond())) {
					toAdd.add(edge);
					if (!currentOrdersGraph.containsVertex(edge.getFirst())) currentOrdersGraph.addVertex(edge.getFirst());
					if (!currentOrdersGraph.containsVertex(edge.getSecond())) currentOrdersGraph.addVertex(edge.getSecond());
					String hashCode = new String("s"+ String.valueOf(edge.getFirst()) + "t" + String.valueOf(edge.getSecond()) + "c" + String.valueOf(1));
					currentOrdersGraph.addEdge(edge.getFirst(), edge.getSecond(), hashCode);
					metaCSPLogger.finest("Add edge:" + edge.toString());
				}
				else {
					String hashCode = currentOrdersGraph.getEdge(edge.getFirst(), edge.getSecond());
					int cmark = hashCode.indexOf("c");
					int numEdge = Integer.valueOf(hashCode.substring(cmark+1, hashCode.length()));
					currentOrdersGraph.removeEdge(edge.getFirst(), edge.getSecond());
					if (!currentOrdersGraph.containsVertex(edge.getFirst())) currentOrdersGraph.addVertex(edge.getFirst());
					if (!currentOrdersGraph.containsVertex(edge.getSecond())) currentOrdersGraph.addVertex(edge.getSecond());
					String newHashCode = new String(hashCode.substring(0,cmark+1).concat(String.valueOf(numEdge+1)));
					currentOrdersGraph.addEdge(edge.getFirst(), edge.getSecond(), newHashCode);
				}
			}
			if (toAdd.isEmpty()) return;
			
			//compute strongly connected components
			KosarajuStrongConnectivityInspector<Integer,String> ksccFinder = new KosarajuStrongConnectivityInspector<Integer,String>(currentOrdersGraph);
			List<DirectedSubgraph<Integer,String>> sccs = ksccFinder.stronglyConnectedSubgraphs();
			metaCSPLogger.finest("Connected components: " + sccs.toString());
	
			//update the cycle list
			for (Pair<Integer,Integer> pair : toAdd) {
				//search the strongly connected components containing the two vertices
				for (DirectedSubgraph<Integer,String> ssc : sccs) {
					if (ssc.containsVertex(pair.getFirst()) || ssc.containsVertex(pair.getSecond())) {
						if (ssc.containsVertex(pair.getFirst()) && ssc.containsVertex(pair.getSecond())) {
							//get cycles in this strongly connected components
							JohnsonSimpleCycles<Integer,String> cycleFinder = new JohnsonSimpleCycles<Integer,String>(ssc);
							List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
							metaCSPLogger.finest("Reversed cycles: " + cycles);
							if (!cycles.isEmpty()) {
								for(List<Integer> cycle : cycles) {
									Collections.reverse(cycle);
									//update the list of cycles for each edge
									for (int i = 0; i < cycle.size(); i++) {
										int j = i < cycle.size()-1 ? i+1 : 0;
										Pair<Integer,Integer> edge = new Pair<Integer,Integer>(cycle.get(i), cycle.get(j));
										if (!currentCyclesList.containsKey(edge)) currentCyclesList.put(edge, new HashSet<ArrayList<Integer>>());
										currentCyclesList.get(edge).add((ArrayList<Integer>)cycle);
										//metaCSPLogger.info("edge: " + edge.toString() + "currentCyclesList:" + currentCyclesList.get(edge));
									}
								}
							}
						}
						break; //move to next pair
					}
				}
			}
		}
	}
	
	protected void updateGraph(HashSet<Pair<Integer,Integer>> edgesToDelete, HashSet<Pair<Integer,Integer>> edgesToAdd) {
		synchronized(allCriticalSections) {		
			HashSet<Pair<Integer,Integer>> toDelete = null;
			if (edgesToDelete != null) {
				toDelete = new HashSet<Pair<Integer,Integer>>(edgesToDelete);
				if (edgesToAdd != null) toDelete.removeAll(edgesToAdd);
			}
	
			HashSet<Pair<Integer,Integer>> toAdd = null;
			if (edgesToAdd != null) {
				toAdd = new HashSet<Pair<Integer,Integer>>(edgesToAdd);
				if (edgesToDelete != null) toAdd.removeAll(edgesToDelete);
			}
			
			deleteEdges(toDelete);			
			addEdges(toAdd);
		}
	}
	
	protected void globalCheckAndRevise() {
		
		synchronized(solver) {
			HashMap<Integer,TreeSet<Dependency>> currentDeps = new HashMap<Integer,TreeSet<Dependency>>();
			HashMap<Integer,TreeSet<Dependency>> artificialDependencies = new HashMap<Integer,TreeSet<Dependency>>(); 
			DirectedMultigraph<Integer,Dependency> depsGraph = new DirectedMultigraph<Integer, Dependency>(Dependency.class);
			HashSet<Integer> askForReplan = new HashSet<Integer>();			
			
			HashSet<Pair<Integer,Integer>> edgesToDelete = new HashSet<Pair<Integer,Integer>>();
			HashSet<Pair<Integer,Integer>> edgesToAdd = new HashSet<Pair<Integer,Integer>>();
			HashSet<CriticalSection> reversibleCS = new HashSet<CriticalSection>();
			
			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				RobotReport robotReport = robotTracker.getRobotReport();
				synchronized(stoppingPoints) {
					if (stoppingPoints.containsKey(robotID)) {
						metaCSPLogger.info("Stopping points Robot"+robotID+": "+stoppingPoints.get(robotID).toString());
						for (int i = 0; i < stoppingPoints.get(robotID).size(); i++) {
							int stoppingPoint = stoppingPoints.get(robotID).get(i);
							int duration = stoppingTimes.get(robotID).get(i);
							if (robotReport.getPathIndex() <= stoppingPoint) {
								Dependency dep = new Dependency(robotTracker.getTrajectoryEnvelope(), null, stoppingPoint, 0);
								if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new TreeSet<Dependency>());
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
	
			HashMap<Integer,RobotReport> currentReports = new HashMap<Integer,RobotReport>();
			
			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
				
				depsToCS.clear();
				
				//Update the coordinator view
				for (int robotID : robotIDs) {
					currentReports.put(robotID,this.getRobotReport(robotID));
				}
	
				HashSet<CriticalSection> toRemove = new HashSet<CriticalSection>();
				
				for (CriticalSection cs : this.allCriticalSections) {
										
					//Will be assigned depending on current situation of robot reports...
					int waitingPoint = -1;
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
					if (robotReport1.getPathIndex() > cs.getTe1End() || robotReport2.getPathIndex() > cs.getTe2End()) {
						toRemove.add(cs);
						metaCSPLogger.finest("Obsolete critical section\n\t" + cs);
						continue;
					}
				
					//If the precedence IS CONSTRAINED BY PARKED ROBOTS ...
					if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy || robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
						
						boolean createAParkingDep = false;
						
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
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
							currentDeps.get(waitingRobotID).add(dep);

							//If cs is new, update the of edges to add (otherwise, it is already in the graph)
							if (!CSToDepsOrder.containsKey(cs))
								edgesToAdd.add(new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getDrivingRobotID()));
							else {
								if (CSToDepsOrder.get(cs).getFirst() != dep.getWaitingRobotID()) {
									metaCSPLogger.info("<<<<<<<<<<<<< strange situation!!");
									if (CSToDepsOrder.get(cs).getFirst() == dep.getDrivingRobotID()) edgesToDelete.add(new Pair<Integer,Integer>(dep.getDrivingRobotID(), dep.getWaitingRobotID()));
									edgesToAdd.add(new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getDrivingRobotID()));
								}
									
							}
							
							//update the global graph
							if (!depsGraph.containsEdge(dep)) {
								if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
								if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
								depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
							}
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
						}
					}
					else { //both the robots are driving. Check if the precedence is reversible.
						
						//Avoid to update precedences on critical sections that are locked
						//e.g. due to a re-plan
						boolean update = true;
						synchronized(lockedRobots) {
							if (lockedRobots.containsValue(cs)) {
								update = false;
								metaCSPLogger.info("lockedRobots contains cs " + cs + ", skip update.");
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
						
						//If the precedence IS REVERSIBLE, continue ...
						//we will check if the heuristic order will preserve the liveness
						if (canStopRobot1 && canStopRobot2) {
							reversibleCS.add(cs);
							continue;
						}
						
						//otherwise, 
						//the precedence is CONSTRAINED BY DYNAMICS
						
						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						if (canStopRobot1 && !canStopRobot2) {
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
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingRobotID = robotReport1.getRobotID();
							waitingRobotID = robotReport2.getRobotID();
							drivingTE = robotTracker1.getTrajectoryEnvelope();
							waitingTE = robotTracker2.getTrajectoryEnvelope();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTE.getRobotID() + " (can't) ahead of Robot" + waitingTE.getRobotID() + " (can) and CS is: " + cs);
						}
						
						else {	//Otherwise re-impose previously decided dependency if possible			
							
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
										throw new Error("FIXME! Lost dependency! " );							
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
								if (!artificialDependencies.containsKey(drivingRobotID)) artificialDependencies.put(drivingRobotID, new TreeSet<Dependency>());
								artificialDependencies.get(drivingRobotID).add(dep);
								askForReplan.add(drivingRobotID);
								
								//update the global graph
								if (!depsGraph.containsEdge(dep)) {
									if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
									if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
									depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
								}
								
								metaCSPLogger.info("Robot" + drivingRobotID + " cannot escape from CS: " + cs + ". Let's create a deadlock. Add artificial dependency for Robot" + drivingRobotID + " at " + dep.getWaitingPoint() + ".");
							}	
							metaCSPLogger.finest("Both can't. Driving Robot" + drivingRobotID + " at " + drivingCurrentIndex + " makes " + waitingRobotID + " waiting at CS " + cs + ".");
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
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
							currentDeps.get(waitingRobotID).add(dep);
							
							//If cs is new, update the of edges to add (otherwise, it is already in the graph)
							if (!CSToDepsOrder.containsKey(cs))
								edgesToAdd.add(new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getDrivingRobotID()));
							else {
								if (CSToDepsOrder.get(cs).getFirst() != dep.getWaitingRobotID()) {
									metaCSPLogger.info("<<<<<<<<<<<<< strange situation 2!!");
									if (CSToDepsOrder.get(cs).getFirst() == dep.getDrivingRobotID()) edgesToDelete.add(new Pair<Integer,Integer>(dep.getDrivingRobotID(), dep.getWaitingRobotID()));
									edgesToAdd.add(new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getDrivingRobotID()));
								}
									
							}
							//update the global graph
							if (!depsGraph.containsEdge(dep)) {
								if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
								if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
								depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
							}
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
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
					if (CSToDepsOrder.containsKey(cs)) {
						int waitingRobID = CSToDepsOrder.get(cs).getFirst();
						int drivingRobID = cs.getTe1().getRobotID() == waitingRobID ? cs.getTe2().getRobotID() : cs.getTe1().getRobotID(); 
						edgesToDelete.add(new Pair<Integer,Integer>(waitingRobID, drivingRobID));
						//this.CSToDepsOrder.remove(cs);
					}
					else metaCSPLogger.info("WARNING: Obsolete critical section " + cs + " was not assigned to a dependence.");
					this.escapingCSToWaitingRobotIDandCP.remove(cs);
				}
				
				//increment the counter
				this.criticalSectionCounter.addAndGet(toRemove.size());		
				
				//update graph with all the not reversible constraints
				updateGraph(edgesToDelete, edgesToAdd);
				metaCSPLogger.finest("Constrained precedences leads to graph: " + currentOrdersGraph.toString() + ".");		
			
				//For each reversible constraint, first preload the precedence according to the ESTF heuristic.
				//Then, try to reverse the constraint and decide for the heuristic order if it will preserve liveness.
				//( checkAndAdd() ... )				
				
				/*/////////////////////////////
							PRE-LOAD
				/////////////////////////////*/
				for (CriticalSection cs : reversibleCS) {
					
					//check each edge one by one
					edgesToDelete.clear();
					edgesToAdd.clear();
	
					AbstractTrajectoryEnvelopeTracker robotTracker1 = trackers.get(cs.getTe1().getRobotID());
					RobotReport robotReport1 = currentReports.get(cs.getTe1().getRobotID());
					AbstractTrajectoryEnvelopeTracker robotTracker2 = trackers.get(cs.getTe2().getRobotID());
					RobotReport robotReport2 = currentReports.get(cs.getTe2().getRobotID());
									
					//If the dependency is new, the starting robot should yield (ESTF heuristic).
					//Otherwise, preload the previous precedence order
					boolean robot2Yields = true;	
					if (!CSToDepsOrder.containsKey(cs)) 
						robot2Yields = robotTracker1.getStartingTimeInMillis() < robotTracker2.getStartingTimeInMillis();
					else robot2Yields = CSToDepsOrder.get(cs).getFirst() == robotReport2.getRobotID();
					
					int drivingRobotID = robot2Yields ? robotReport1.getRobotID() : robotReport2.getRobotID();
					int waitingRobotID = robot2Yields ? robotReport2.getRobotID() : robotReport1.getRobotID();
					TrajectoryEnvelope drivingTE = robot2Yields ? robotTracker1.getTrajectoryEnvelope() : robotTracker2.getTrajectoryEnvelope();
					TrajectoryEnvelope waitingTE = robot2Yields ? robotTracker2.getTrajectoryEnvelope() : robotTracker1.getTrajectoryEnvelope();
					int waitingPoint = getCriticalPoint(waitingRobotID, cs, robot2Yields ? robotReport1.getPathIndex() : robotReport2.getPathIndex());								
					
					//Update graphs
					if (waitingPoint >= 0) {
						
						//If cs is new, update the of edges to add (otherwise, it is already in the graph)
						if (!CSToDepsOrder.containsKey(cs)) {
							edgesToAdd.add(new Pair<Integer,Integer>(waitingRobotID, drivingRobotID));
							addEdges(edgesToAdd);
						}
						
						metaCSPLogger.finest("Graph after pre-loading precedences: " + currentOrdersGraph.toString() + ".");		
						
						//Make new dependency
						int drivingCSEnd =  drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
						if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
						Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
						currentDeps.get(waitingRobotID).add(dep);
						
						//update the global graph
						if (!depsGraph.containsEdge(dep)) {
							if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
							if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
							depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
						}
						CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getWaitingPoint()));
						depsToCS.put(dep, cs);
					}
					else {
						//If robot is asked to wait in an invalid path point, throw error and give up!
						metaCSPLogger.severe("Waiting point < 0 for critical section " + cs);
						throw new Error("Waiting point < 0 for critical section " + cs);
					}
				}
				
				/*/////////////////////////////
						CHECK HEURISTIC
				/////////////////////////////*/
				for (CriticalSection cs : reversibleCS) {
										
					//check each edge one by one
					edgesToDelete.clear();
					edgesToAdd.clear();
	
					AbstractTrajectoryEnvelopeTracker robotTracker1 = trackers.get(cs.getTe1().getRobotID());
					RobotReport robotReport1 = currentReports.get(cs.getTe1().getRobotID());
					AbstractTrajectoryEnvelopeTracker robotTracker2 = trackers.get(cs.getTe2().getRobotID());
					RobotReport robotReport2 = currentReports.get(cs.getTe2().getRobotID());
					
					boolean robot2Yields = getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs); //true if robot1 should go before robot2, false vice versa
					boolean robot2YieldsOld = CSToDepsOrder.get(cs).getFirst() == robotReport2.getRobotID();
					
					if (robot2YieldsOld != robot2Yields) {
						metaCSPLogger.info("Trying reversing a precedence at critical section " + cs + ".");
					
						//try reversing the order
		
						int drivingRobotID = robot2Yields ? robotReport1.getRobotID() : robotReport2.getRobotID();
						int waitingRobotID = robot2Yields ? robotReport2.getRobotID() : robotReport1.getRobotID();
						TrajectoryEnvelope drivingTE = robot2Yields ? robotTracker1.getTrajectoryEnvelope() : robotTracker2.getTrajectoryEnvelope();
						TrajectoryEnvelope waitingTE = robot2Yields ? robotTracker2.getTrajectoryEnvelope() : robotTracker1.getTrajectoryEnvelope();
						int waitingPoint = getCriticalPoint(waitingRobotID, cs, robot2Yields ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
						
						//Update graphs
						if (waitingPoint >= 0) {		
												
							//Store previous graph
							DirectedMultigraph<Integer,Dependency> backupDepsGraph = new DirectedMultigraph<Integer, Dependency>(Dependency.class);
							for (int v : depsGraph.vertexSet()) backupDepsGraph.addVertex(v);
							for (Dependency dep : depsGraph.edgeSet()) backupDepsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
							
							SimpleDirectedGraph<Integer,String> backupGraph = new SimpleDirectedGraph<Integer, String>(String.class);
							for (int v : currentOrdersGraph.vertexSet()) backupGraph.addVertex(v);
							for (String edge : currentOrdersGraph.edgeSet()) backupGraph.addEdge(currentOrdersGraph.getEdgeSource(edge), currentOrdersGraph.getEdgeTarget(edge), edge);
							
							HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> backupcurrentCyclesList = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
							backupcurrentCyclesList.putAll(currentCyclesList);
														
							edgesToDelete.add(new Pair<Integer,Integer>(drivingRobotID, waitingRobotID));
							Pair<Integer,Integer> newEdge = new Pair<Integer,Integer>(waitingRobotID, drivingRobotID);
							edgesToAdd.add(newEdge);
							updateGraph(edgesToDelete, edgesToAdd);
							metaCSPLogger.finest("Graph after revising according to the heuristic: " + currentOrdersGraph.toString() + ".");		
							
							//Make new dependency
							int drivingCSEnd =  drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							int drivingCSEndOld =  drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe2End() : cs.getTe1End();
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd);
							Dependency depOld = new Dependency(drivingTE, waitingTE, CSToDepsOrder.get(cs).getSecond(), drivingCSEndOld);
							
							//update the global graph and treeset
							depsGraph.removeEdge(depOld);
							if (!depsGraph.containsEdge(dep)) {
								if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
								if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
								depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
							}
							
							//check if safe (continue here) 
							boolean safe = true;
							//metaCSPLogger.info("currentCyclesList: " + currentCyclesList.toString());
							if (currentCyclesList.get(newEdge) != null) {
								//for each cycle involving the new edge
								for (List<Integer> cycle : currentCyclesList.get(newEdge)) {
									
									//Get edges along the cycle...
									//  Recall: there could be more than one edge per pair of vertices, as this is a multi-graph
									ArrayList<ArrayList<Dependency>> edgesAlongCycle = new ArrayList<ArrayList<Dependency>>();
									for (int i = 0; i < cycle.size(); i++) {
										int j = i < cycle.size()-1 ? i+1 : 0;
										if (cycle.get(i) == dep.getWaitingRobotID() && cycle.get(j) == dep.getDrivingRobotID()) {
											ArrayList<Dependency> array = new ArrayList<Dependency>();
											array.add(dep);
											edgesAlongCycle.add(array);
										}
										else {
											Set<Dependency> allEdges = depsGraph.getAllEdges(cycle.get(i), cycle.get(j));
											if (allEdges == null) metaCSPLogger.info("<<<<<<<<< something wrong.");
											else edgesAlongCycle.add(new ArrayList<Dependency>(allEdges));
										}
									}
									
									//Get all the possible permutation of edges
									int maxSize = 0;
									for (ArrayList<Dependency> oneDepList : edgesAlongCycle) if (oneDepList.size() > maxSize) maxSize = oneDepList.size();
									PermutationsWithRepetition gen = new PermutationsWithRepetition(maxSize, edgesAlongCycle.size());
									int[][] v = gen.getVariations();
									ArrayList<ArrayList<Dependency>> newDeps = new ArrayList<ArrayList<Dependency>>();
									for (int k = 0; k < v.length; k++) {
										int[] oneComb = v[k];
										ArrayList<Dependency> oneSelection = new ArrayList<Dependency>();
										for (int i = 0; i < oneComb.length; i++) {
											if (oneComb[i] < edgesAlongCycle.get(i).size()) oneSelection.add(edgesAlongCycle.get(i).get(oneComb[i]));
											else {
												oneSelection = null;
												break;
											}
										}
										if (oneSelection != null) newDeps.add(oneSelection);
									}
									
									//Check for unsafe cycles, that is:
									//  All along the cycle, check if there are (u,v,dep1) and (v,w,dep2) such that
									//    v.dep2.waitingpoint <= v.dep1.releasingpoint
																
									for (ArrayList<Dependency> depList : newDeps) {
										for (int i = 0; i < depList.size(); i++) {
											safe = true;
											Dependency dep1 = depList.get(i);
											Dependency dep2 = depList.get(i<depList.size()-1 ? i+1 : 0);
											if (unsafePair(dep1,dep2)) safe = false;
											if (safe) break; //if one pair in the cycle is safe, then the cycle is safe
										}
									//  If there exists almost one unsafe cycle, then the precedence cannot be reversed.	
										if(!safe) {
											metaCSPLogger.info("An unsafe cycle: " + depList + ".");
											break;
										}
									}
									
									if(!safe) {
										metaCSPLogger.info("Cycle: " + edgesAlongCycle + " is NOT deadlock-free");
										break;
									}
								}
							}
							if (!safe) {
								depsGraph = backupDepsGraph;
								currentOrdersGraph = backupGraph;
								currentCyclesList = backupcurrentCyclesList;
								metaCSPLogger.info("Restore previous precedence " + depOld + ".");
							}
							else {
								//update the maps
								currentDeps.get(drivingRobotID).remove(depOld);
								if (currentDeps.get(drivingRobotID).isEmpty()) currentDeps.remove(drivingRobotID);
								if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
								currentDeps.get(waitingRobotID).add(dep);
								CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getWaitingPoint()));
								depsToCS.put(dep, cs);
								metaCSPLogger.info("Update precedences " + dep + " according to heuristic.");
							}
							metaCSPLogger.finest("Final graph: " + currentOrdersGraph.toString() + ".");		
						}
					}
				}//end for reversibleCS
			}//end synchronized(allCriticalSections)
			
			//update and communicate critical points
			//get current dependencies
			synchronized(currentDependencies) {
				currentDependencies = updateCurrentDependencies(currentDeps, artificialDependencies, robotIDs);
				
				//The global strategy builds upon the assumption that robots do not start from critical section. 
				//If this is not the case, them pre-loading may bring to unsafe cycles. 
				//To handle this case, switch to a local strategy whenever a robot is starting from a critical section and cannot
				//exit from it.
				/*if (!artificialDependencies.isEmpty()) {
					this.breakDeadlocksByReplanning = true;
					
					//find cycles and revise dependencies if necessary
					findCurrentCycles(currentDeps, artificialDependencies, new HashSet<Dependency>(), currentReports, robotIDs);

					this.breakDeadlocksByReplanning = false;
				}*/
				
				//re-plan for the set of robots that are currently in a critical section
				SimpleDirectedGraph<Integer,Dependency> g = new SimpleDirectedGraph<Integer,Dependency>(Dependency.class);
				g = depsToGraph(g, currentDependencies);
				List<List<Integer>> unsafeCycles = findSimpleUnsafeCycles(g);	
				
				HashSet<Integer> spawnedThreadRobotSet = new HashSet<Integer>();
				for (List<Integer> cycle : unsafeCycles) {
					int robotID = -1;
					for (int ID : askForReplan) {
						if (cycle.contains(ID)) {
							robotID = ID;
							break;
						}
					}
					if (robotID == -1) continue;
					
					//the cycle contains a robot that is asking for re-plan.
					//Find all the deps along the cycle
					boolean spawnThread = true;
					metaCSPLogger.info("Try re-plan for Robot" + robotID+ ".");
					ArrayList<Dependency> depsAlongCycle = new ArrayList<Dependency>();
					for (int i = 0; i < cycle.size(); i++) {
						Dependency dep = g.getEdge(cycle.get(i), cycle.get(i < cycle.size()-1 ? i+1 : 0));
						if (dep != null) depsAlongCycle.add(dep);
						if (spawnedThreadRobotSet.contains(dep.getWaitingRobotID()) || spawnedThreadRobotSet.contains(dep.getDrivingRobotID())) spawnThread = false;
					}
					if (spawnThread) {
						ConnectivityInspector<Integer,Dependency> connInsp = new ConnectivityInspector<Integer,Dependency>(g);
						HashSet<Integer> allRobots = (HashSet<Integer>) connInsp.connectedSetOf(robotID);
						spawnReplanning(depsAlongCycle, allRobots);
						spawnedThreadRobotSet.addAll(cycle);
					}
				}
						
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
					int maxDelay = 2*(MAX_TX_DELAY+CONTROL_PERIOD+tracker.getTrackingPeriodInMillis()) + CONTROL_PERIOD; //add an extra control period to the theoretical upperbound to handle the case o equality
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
			
		}//end synchronized(solver)
		
	}//end checkAndRevise
	
}//end class


//FIXME Strating from critical section with checkAndRevise