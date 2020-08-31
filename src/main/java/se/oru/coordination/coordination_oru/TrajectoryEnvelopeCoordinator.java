package se.oru.coordination.coordination_oru;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.jgrapht.alg.ConnectivityInspector;
import org.jgrapht.alg.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.DirectedMultigraph;
import org.jgrapht.graph.DirectedSubgraph;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.metacsp.framework.Constraint;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.PermutationsWithRepetition;

import com.vividsolutions.jts.geom.Geometry;

import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;


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
	protected SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge> currentOrdersGraph = new SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge>(DefaultWeightedEdge.class);
	protected HashMap<Pair<Integer,Integer>, HashSet<ArrayList<Integer>>> currentCyclesList = new HashMap<Pair<Integer,Integer>, HashSet<ArrayList<Integer>>>();
	
	//Robots currently involved in a re-plan which critical point cannot increase beyond the one used for re-plan
	//till the re-plan has not finished yet.
	protected HashMap<Integer,Dependency> lockedRobots = new HashMap<Integer,Dependency>();

	protected boolean breakDeadlocksByReordering = true;
	protected boolean breakDeadlocksByReplanning = true;
	protected AtomicBoolean avoidDeadlockGlobally = new AtomicBoolean(false);
	protected AtomicInteger nonliveStatesDetected = new AtomicInteger(0);
	protected AtomicInteger nonliveStatesAvoided = new AtomicInteger(0);
	protected AtomicInteger currentOrdersHeurusticallyDecided = new AtomicInteger(0);
	protected List<List<Integer>> nonliveCyclesOld = new ArrayList<List<Integer>>();
	protected AtomicInteger replanningTrialsCounter = new AtomicInteger(0);
	protected AtomicInteger successfulReplanningTrialsCounter = new AtomicInteger(0);

	//True if waiting for deadlocks to happen.
	protected boolean staticReplan = false;

	protected boolean isDeadlocked = false;
	protected boolean isBlocked = false;


	/**
	 * Get whether there is a robot in a blocked situation (waiting for a parked robot).
	 * @return <code>true</code> iff a robot is waiting for another robot that is parked.
	 */
	public boolean isBlocked() {
		return this.isBlocked;
	}

	
	/**
	 * Set whether the coordinator should try to break deadlocks by either global re-ordering, or by re-ordering and/or re-planning.
	 * @param global <code>true</code> if deadlocks should be broken via global re-ordering (complete, complexity: O(2^nlogn) in the worst case).
	 * @param reorder <code>true</code> if deadlocks should be broken via local re-ordering (incomplete, complexity: O(n^2) in the worst case).
	 * @param replan <code>true</code> if deadlocks should be broken via local re-planning (incomplete).
	 */
	public void setBreakDeadlocks(boolean global, boolean reorder, boolean replan) {
		if (global && (reorder||replan)) {
			metaCSPLogger.severe("Enable either the global or the local strategies for deadlock prevention!! Using default values (i.e., enabling local re-order + replan).");
			this.avoidDeadlockGlobally.getAndSet(false);
			this.breakDeadlocksByReordering = true;
			this.breakDeadlocksByReplanning = true;
			return;
		}
		this.avoidDeadlockGlobally.getAndSet(global);
		this.breakDeadlocksByReordering = reorder;
		this.breakDeadlocksByReplanning = replan;
	}


	/**
	 * Set whether waiting for deadlocks to happen before starting re-plan to recover from deadlock.
	 * @param value <code>true</code> if all the robots should yield at their deadlocking critical points 
	 * before starting a recovery strategy based on re-plan.
	 * Default value: false.
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

	@Override
	protected String[] getStatistics() {

		String CONNECTOR_BRANCH = (char)0x251C + "" + (char)0x2500 + " ";
		String CONNECTOR_LEAF = (char)0x2514 + "" + (char)0x2500 + " ";
		ArrayList<String> ret = new ArrayList<String>();
		int numVar = solver.getConstraintNetwork().getVariables().length;
		int numCon = solver.getConstraintNetwork().getConstraints().length;

		synchronized (trackers) {
			ret.add("Status @ "  + getCurrentTimeInMillis() + " ms");
			ret.add(CONNECTOR_BRANCH + "Eff period ..... " + EFFECTIVE_CONTROL_PERIOD + " ms");
			ret.add(CONNECTOR_BRANCH + "Network ........ " + numVar + " variables, " + numCon + " constraints");
			HashSet<Integer> allRobots = new HashSet<Integer>();
			for (Integer robotID : trackers.keySet()) {
				allRobots.add(robotID);
			}
			String st = CONNECTOR_BRANCH + "Robots view .... ";
			for (Integer robotID : allRobots) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				RobotReport rr = getRobotReport(robotID); 
				if (rr != null ) {
					int currentPP = rr.getPathIndex();
					st += tracker.getTrajectoryEnvelope().getComponent();
					if (tracker instanceof TrajectoryEnvelopeTrackerDummy) st += " (P)";
					else st += " (D)";
					st += ": " + currentPP + "   ";
				}
			}
			ret.add(st);
		}
		synchronized (currentDependencies) {
			ret.add(CONNECTOR_BRANCH + "Dependencies ... " + currentDependencies);
		}
		ret.add(CONNECTOR_BRANCH + "Total number of obsolete critical sections ... " + criticalSectionCounter.get() + ".");
		ret.add(CONNECTOR_BRANCH + "Total messages sent: ... " + totalMsgsSent.get() + ",, retransmitted: " + totalMsgsReTx.get() + ", number of replicas: " + numberOfReplicas + ".");
		ret.add(CONNECTOR_BRANCH + "Total nonlive states detected: ... " + nonliveStatesDetected.get() + ", avoided: " + nonliveStatesAvoided.get() + ", revised according to heuristic: " + currentOrdersHeurusticallyDecided.get() + ".");
		ret.add(CONNECTOR_LEAF + "Total re-planned path: ... " + replanningTrialsCounter.get() + ", successful: " + successfulReplanningTrialsCounter.get() + ".");
		return ret.toArray(new String[ret.size()]);
	}

	private List<List<Integer>> findSimpleNonliveCycles(SimpleDirectedGraph<Integer,Dependency> g) {

		JohnsonSimpleCycles<Integer, Dependency> cycleFinder = new JohnsonSimpleCycles<Integer, Dependency>(g);
		List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
		List<List<Integer>> nonliveCycles = new ArrayList<List<Integer>>();

		//Find all the nonlive cycles

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

			//Check for nonlive condition, that is:
			//  All along the cycle, check if there are (u,v,dep1) and (v,w,dep2) such that
			//    v.dep2.waitingpoint <= v.dep1.releasingpoint
			//  And if so mark cycle as deadlock
			//[1,2] ,[2,1]			[1,2], [2,3], [3,1]
			//i = 0: [1,2], [2,1]   [1,2], [2,3]
			//i = 1: [2,1], [1,2]	[2,3], [3,1]
			//i = 2:  -             [3,1], [1,2]
			for (int i = 0; i < edgesAlongCycle.size(); i++) {
				boolean safe = true;
				if (nonlivePair(edgesAlongCycle.get(i), edgesAlongCycle.get(i < edgesAlongCycle.size()-1 ? i+1 : 0))) {
					safe = false;
				}
				if (safe) {
					metaCSPLogger.finest("Cycle: " + edgesAlongCycle + " is deadlock-free");
					break;
				}
				if (i == edgesAlongCycle.size()-1) {
					metaCSPLogger.info("Cycle: " + edgesAlongCycle + " is NOT deadlock-free. Current cycle list contains this cycle? :" + currentCyclesList.containsValue(cycle));
					nonliveCycles.add(cycle);
				}
			}			
		}

		return nonliveCycles;
	}


	private SimpleDirectedGraph<Integer,Dependency> depsToGraph(HashSet<Dependency> deps) {
		SimpleDirectedGraph<Integer,Dependency> g = new SimpleDirectedGraph<Integer,Dependency>(Dependency.class);
		for (Dependency dep : deps) {
			g.addVertex(dep.getWaitingRobotID());
			g.addVertex(dep.getDrivingRobotID());
			if (!g.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep))
				metaCSPLogger.severe("<<<<<<<<< Add dependency fails (13). Dep: " + dep);	
		}
		return g;
	}

	private HashSet<Dependency> computeClosestDependencies(HashMap<Integer,HashSet<Dependency>> allDeps, HashMap<Integer, HashSet<Dependency>> artificialDeps) {

		HashSet<Dependency> closestDeps = new HashSet<Dependency>();

		if (allDeps == null && artificialDeps == null || allDeps.isEmpty() && artificialDeps.isEmpty()) {
			return closestDeps;
		}

		metaCSPLogger.finest("currentDeps: " + allDeps.toString() + ".");	
		metaCSPLogger.finest("artificialDeps: " + artificialDeps.toString() + ".");	

		//Fill the TreeSet
		HashMap<Integer, TreeSet<Dependency>> allDepsTree = new HashMap<Integer, TreeSet<Dependency>>();
		for (Integer key : allDeps.keySet()) allDepsTree.put(key, new TreeSet<Dependency>(allDeps.get(key)));
		HashMap<Integer, TreeSet<Dependency>> artificialDepsTree = new HashMap<Integer, TreeSet<Dependency>>();
		for (Integer key : artificialDeps.keySet()) artificialDepsTree.put(key, new TreeSet<Dependency>(artificialDeps.get(key)));

		HashSet<Integer> robotIDs = new HashSet<Integer>();
		robotIDs.addAll(allDeps.keySet());
		robotIDs.addAll(artificialDeps.keySet());

		for (Integer robotID : robotIDs) {						
			if (allDeps.containsKey(robotID) || artificialDeps.containsKey(robotID)) {
				Dependency firstDep = null;
				Dependency firstArtificialDep = null;
				if (allDeps.containsKey(robotID)){
					firstDep = allDepsTree.get(robotID).first();
				}
				if (artificialDeps.containsKey(robotID)) {
					firstArtificialDep = artificialDepsTree.get(robotID).first();
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

	/**
	 * Check whether there is or will be a deadlock.
	 * @return <code>true</code> iff there is currently a deadlock, or the situation is
	 * such that a deadlock will occur (non-live state).
	 */
	public boolean isDeadlocked() {
		return this.isDeadlocked;
	}

	private HashMap<Integer,HashSet<Dependency>> findCurrentCycles(HashMap<Integer,HashSet<Dependency>> currentDeps, HashMap<Integer,HashSet<Dependency>> artificialDeps, HashSet<Dependency> reversibleDeps, HashMap<Integer,RobotReport> currentReports, Set<Integer> robotIDs) {

		HashMap<Integer,HashSet<Dependency>> allDeps = new HashMap<Integer, HashSet<Dependency>>();
		for (int robotID : currentDeps.keySet()) {
			HashSet<Dependency> set = new HashSet<Dependency>();
			set.addAll(currentDeps.get(robotID));
			allDeps.put(robotID, set);
		}

		//1. COMPUTE NONLIVE CYCLES

		//Dep graph G = (V,E)
		//  V = robots involved in current deps
		//  E = {(u,v,dep) | exists dep stating that robot u should wait for robot v}
		//Deadlock iff:
		//  Cycle <r_1, r_2, ... r_1> in G
		//  Exists one selection of edges along the cycle such that
		//    Exists (u,v,dep1) and (v,w,dep2) such that
		//      v.dep2.waitingpoint <= v.dep1.releasingpoint

		SimpleDirectedGraph<Integer,Dependency> g = depsToGraph(currentDependencies);
		List<List<Integer>> nonliveCycles = findSimpleNonliveCycles(g);

		this.isDeadlocked = nonliveCycles.size() > 0;

		// ... keep tracks of size and old cycles for statistics
		List<List<Integer>> nonliveCyclesNew = new ArrayList<List<Integer>>();
		nonliveCyclesNew.addAll(nonliveCycles);
		nonliveCyclesNew.removeAll(nonliveCyclesOld);
		nonliveStatesDetected.addAndGet(nonliveCyclesNew.size());
		nonliveCyclesOld.clear();
		nonliveCyclesOld.addAll(nonliveCycles);

		//2. IF THERE ARE NONLIVE CYCLES AND RE-ORDERING IS ENABLED, TRY RE-ORDER
		if (breakDeadlocksByReordering) {

			int counter = 0;

			while (counter < nonliveCycles.size()) { //if empty no problem

				//get one cycle
				List<Integer> cycle = nonliveCycles.get(counter);
				counter++;

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

						HashMap<Integer,HashSet<Dependency>> allDepsTmp = new HashMap<Integer,HashSet<Dependency>>(allDeps);

						//and add the reversed dependency to the new waiting one
						if (!allDepsTmp.containsKey(waitingRobotID)) allDepsTmp.put(waitingRobotID, new HashSet<Dependency>());
						allDepsTmp.get(revDep.getWaitingRobotID()).add(revDep);

						//remove the dependency to the old waiting robot
						allDepsTmp.get(dep.getWaitingRobotID()).remove(dep);
						if (allDepsTmp.get(dep.getWaitingRobotID()).isEmpty()) allDepsTmp.remove(dep.getWaitingRobotID());

						//update a temporary map between critical sections and orders
						HashMap<CriticalSection, Pair<Integer,Integer>> CSToDepsOrderTmp = new HashMap<CriticalSection, Pair<Integer,Integer>>(CSToDepsOrder);
						CSToDepsOrderTmp.put(cs, new Pair<Integer,Integer>(revDep.getWaitingRobotID(), revDep.getWaitingPoint()));

						//create a temporary map between dependencies and critical sections
						HashMap<Dependency,CriticalSection> depsToCSTmp = new HashMap<Dependency, CriticalSection>(depsToCS);
						depsToCSTmp.remove(dep);
						depsToCSTmp.put(revDep, cs);

						//update currentDeps
						HashSet<Dependency> currentDepsTmp = computeClosestDependencies(allDepsTmp, artificialDeps);
						SimpleDirectedGraph<Integer,Dependency> gTmp = depsToGraph(currentDepsTmp);

						//compute cycles again. If the number of cycles is lower, keep this solution
						List<List<Integer>> nonliveCyclesTmp = findSimpleNonliveCycles(gTmp);
						if (nonliveCyclesTmp.size() < nonliveCycles.size()) {
							nonliveStatesAvoided.incrementAndGet();
							metaCSPLogger.info("REVERSING a precedence constraint to break a deadlock: " + revDep.getWaitingRobotID() + " now waits for " + revDep.getDrivingRobotID());						

							//reset the counter
							counter = 0;

							//update all the variables according to the new choice
							g = gTmp;
							currentDependencies.clear();
							currentDependencies.addAll(currentDepsTmp);				
							allDeps = allDepsTmp;
							nonliveCycles = nonliveCyclesTmp;					
							CSToDepsOrder.clear();
							CSToDepsOrder.putAll(CSToDepsOrderTmp);							
							depsToCS.clear();
							depsToCS.putAll(depsToCSTmp);	

							break;
						}
						//else you should not update the solution (the cycle will reversed by re-planning if possible)
					}
				}
			}
		}

		//2. OTHERWISE, IF RE-PLAN IS ENABLED, TRY REPLANNING

		if (breakDeadlocksByReplanning) {

			HashSet<Integer> spawnedThreadRobotSet = new HashSet<Integer>();
			for (List<Integer> cycle : nonliveCycles) {
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
					if (spawnedThreadRobotSet.contains(dep.getWaitingRobotID()) || spawnedThreadRobotSet.contains(dep.getDrivingRobotID())) {
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

					//TODO: see issue on Github
					ConnectivityInspector<Integer,Dependency> connInsp = new ConnectivityInspector<Integer,Dependency>(g);
					HashSet<Integer> allRobots = (HashSet<Integer>) connInsp.connectedSetOf(cycle.get(0));
					spawnReplanning(depsAlongCycle, allRobots);
					spawnedThreadRobotSet.addAll(deadlockedRobots);
				}
			}
		}

		return allDeps;
	}

	//returns true if robot1 should go before robot2
	//returns false if robot2 should go before robot1
	protected boolean getOrder(AbstractTrajectoryEnvelopeTracker robotTracker1, RobotReport robotReport1, AbstractTrajectoryEnvelopeTracker robotTracker2, RobotReport robotReport2, CriticalSection cs) {

		//If both can stop before entering, use ordering function (or closest if no ordering function)
		metaCSPLogger.finest("Both robots can stop at " + cs);

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
	protected void updateDependencies() {
		synchronized(solver) {
			if (this.avoidDeadlockGlobally.get()) globalCheckAndRevise();
			else localCheckAndRevise();
		}
	}

	protected void localCheckAndRevise() {

		//System.out.println("Caller of updateDependencies(): " + Thread.currentThread().getStackTrace()[2]);
		synchronized(solver) {
			HashMap<Integer,RobotReport> currentReports = new HashMap<Integer,RobotReport>();
			HashMap<Integer,HashSet<Dependency>> currentDeps = new HashMap<Integer,HashSet<Dependency>>();
			HashMap<Integer,HashSet<Dependency>> artificialDependencies = new HashMap<Integer,HashSet<Dependency>>(); 
			HashSet<Dependency> currentReversibleDependencies = new HashSet<Dependency>();
			HashMap<Integer,Integer> earliestStoppingPoints = new HashMap<Integer,Integer>();

			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				//Update the coordinator view
				RobotReport robotReport = robotTracker.getRobotReport();
				currentReports.put(robotID, robotReport);
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
				synchronized(lockedRobots) {
					if (lockedRobots.containsKey(robotID)) 	{
						//We should enforce the last robot stopping point while it is involved in a re-plan.
						if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new HashSet<Dependency>());
						currentDeps.get(robotID).add(lockedRobots.get(robotID));
					}			
				}
			}



			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
				
				//FIXME Add a CriticalSectionManager class
				if (allCriticalSections.size() > 0) 
					for (int robotID : robotIDs) 
						earliestStoppingPoints.put(robotID, getForwardModel(robotID).getEarliestStoppingPathIndex(trackers.get(robotID).getTrajectoryEnvelope(), currentReports.get(robotID)));

				depsToCS.clear();			
				this.isBlocked = false;

				HashSet<CriticalSection> toRemove = new HashSet<CriticalSection>();
				for (CriticalSection cs : this.allCriticalSections) {

					//Will be assigned depending on current situation of robot reports...
					AbstractTrajectoryEnvelopeTracker waitingTracker = null;
					AbstractTrajectoryEnvelopeTracker drivingTracker = null;
					int drivingCurrentIndex = -1;

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
						this.isBlocked = true;

						//Robot1 is parking in critical section. If it is the driver, make robot 2 wait.
						if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy) {
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							waitingPoint = getCriticalPoint(robotReport2.getRobotID(), cs, drivingCurrentIndex);

							if (!communicatedCPs.containsKey(waitingTracker) && robotReport1.getPathIndex() <= waitingPoint 
									|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
								createAParkingDep = true;
							}

						}
						else if (robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
							drivingCurrentIndex = robotReport2.getPathIndex();
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							waitingPoint = getCriticalPoint(robotReport1.getRobotID(), cs, drivingCurrentIndex);

							if (!communicatedCPs.containsKey(waitingTracker) && robotReport2.getPathIndex() <= waitingPoint
									|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
								createAParkingDep = true;						
							}
						}

						if (createAParkingDep) {
							int drivingCSEnd = drivingTracker.getTrajectoryEnvelope().getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							metaCSPLogger.finest("Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " is parked, so Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " will have to wait");
							if (!currentDeps.containsKey(waitingTracker.getTrajectoryEnvelope().getRobotID())) currentDeps.put(waitingTracker.getTrajectoryEnvelope().getRobotID(), new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd);
							currentDeps.get(waitingTracker.getTrajectoryEnvelope().getRobotID()).add(dep);
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
						}
					}
					else {//Both robots are driving, let's determine an ordering for them through this critical section


						//Check if the robots can stop while changing the current order of accessing the critical section.
						boolean canStopRobot1 = false;
						boolean canStopRobot2 = false;
						boolean wakeUpinCSRobot1 = false;
						boolean wakeUpinCSRobot2 = false;

						
						//Force the dependency for the robot footprint
						if (//the last critical point was before the critical section (can stop by induction)
								(communicatedCPs.containsKey(robotTracker1) && communicatedCPs.get(robotTracker1).getFirst() != -1 && communicatedCPs.get(robotTracker1).getFirst() < cs.getTe1Start())
								|| !communicatedCPs.containsKey(robotTracker1) && Math.max(0, robotReport1.getPathIndex()) < cs.getTe1Start())
							canStopRobot1 = true;
						else
							//Due to temporal delays we cannot trust the velocity.
							canStopRobot1 =  earliestStoppingPoints.get(robotReport1.getRobotID()) < cs.getTe1Start();

						if ((communicatedCPs.containsKey(robotTracker2) && communicatedCPs.get(robotTracker2).getFirst() != -1 && communicatedCPs.get(robotTracker2).getFirst() < cs.getTe2Start())
								|| !communicatedCPs.containsKey(robotTracker2) && Math.max(0, robotReport2.getPathIndex()) < cs.getTe2Start())
							canStopRobot2 = true;
						else canStopRobot2 = earliestStoppingPoints.get(robotReport2.getRobotID()) < cs.getTe2Start();


						//Both the robots can stop before accessing the critical section --> follow ordering heuristic if FW model allows it
						if (canStopRobot1 && canStopRobot2) {

							//If robot 1 has priority over robot 2
							if (getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs)) {
								drivingCurrentIndex = robotReport1.getPathIndex();
								drivingTracker = robotTracker1;
								waitingTracker = robotTracker2;
								metaCSPLogger.finest("Both-can-stop (1) and Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " ahead of Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " and CS is: " + cs);
							}

							//If robot 2 has priority over robot 1
							else {
								drivingCurrentIndex = robotReport2.getPathIndex();
								drivingTracker = robotTracker2;
								waitingTracker = robotTracker1;
								metaCSPLogger.finest("Both-can-stop (2) and Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " ahead of Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " and CS is: " + cs);
							}	
						}

						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						else if (canStopRobot1 && !canStopRobot2) {
							drivingCurrentIndex = robotReport2.getPathIndex();
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							metaCSPLogger.finest("One-can-one-can't-stop (1) and Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " (can't) is ahead of Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " (can) and CS is: " + cs);						
						}

						//Robot 2 can stop before entering critical section, robot 1 can't --> robot 2 waits
						else if (!canStopRobot1 && canStopRobot2) {
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " (can't) ahead of Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " (can) and CS is: " + cs);
						}

						else {			

							//Both robots in critical section --> re-impose previously decided dependency if possible
							int drivingRobotID = -1;
							int waitingRobotID = -1;

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
									
									//Try to recover the lost order.
									int ahead = 0;
									//Only the leading robot may be already commanded to go beyond the critical section end.
									if (communicatedCPs.containsKey(robotTracker1) && (communicatedCPs.get(robotTracker1).getFirst() == -1 || communicatedCPs.get(robotTracker1).getFirst() > cs.getTe1End())) ahead = 1;
									else if (communicatedCPs.containsKey(robotTracker2) && (communicatedCPs.get(robotTracker2).getFirst() == -1 || communicatedCPs.get(robotTracker2).getFirst() > cs.getTe2End())) ahead = -1;

									//Otherwise, if both the robots are inside the critical section check their poses.
									if (ahead == 0) ahead = isAhead(cs, robotReport1, robotReport2);
																		
									//Otherwise, the dependency is lost. Try an error. FIXME
									if (ahead == 0) {
										if (!this.CSToDepsOrder.containsKey(cs)) throw new Error("FIXME! Lost dependency and order cannot be restored! Key value not found. RobotReport1: " + robotReport1.toString() + ", RobotReport2: " + robotReport2.toString()
										+", last communicated CPs: (" + (communicatedCPs.containsKey(robotTracker1) ? communicatedCPs.get(robotTracker1).getFirst() : "null") + "," + (communicatedCPs.containsKey(robotTracker2) ? communicatedCPs.get(robotTracker2).getFirst() : "null),") + 
										"), cs: "+ cs.toString());
										else if (this.CSToDepsOrder.get(cs) == null) throw new Error("FIXME! Lost dependency and order cannot be restored! Empty value." );	
									}
									else {
										drivingRobotID = ahead == 1 ? robotReport1.getRobotID() : robotReport2.getRobotID();
										waitingRobotID = drivingRobotID == robotReport1.getRobotID() ? robotReport2.getRobotID() : robotReport1.getRobotID();
										metaCSPLogger.info("<<<<<<<<< Restoring the lost order via estimation for Robot" + drivingRobotID);
									}					
								}			
							}

							drivingCurrentIndex = drivingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
							drivingTracker = drivingRobotID == robotReport1.getRobotID() ? robotTracker1 : robotTracker2;
							waitingTracker = waitingRobotID == robotReport1.getRobotID() ? robotTracker1 : robotTracker2;

							//check if the driving robot can really escape
							int waitingCurrentIndex = communicatedCPs.containsKey(waitingTracker) ? 
									//((communicatedCPs.get(trackers.get(waitingRobotID)).getFirst() == -1) ? 
									//		waitingTE.getPathLength()-1 : 
									communicatedCPs.get(waitingTracker).getFirst() : (
											waitingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
									int lastIndexOfCSDriving = drivingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
									//if (lastWaitingRobotCP >= startingWaitingRobotCS) {
									if (!canExitCriticalSection(drivingCurrentIndex, waitingCurrentIndex, drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(),lastIndexOfCSDriving)) {
										//it's too late for escaping. Let's create a deadlock: the parked robot should wait for the moving one,
										//while the other should be commanded to do not proceed beyond its last communicated CP. Both the dependencies should be added to the current set.									

										//The parked robot should wait for the other at its current path index till the moving robot has not been exited from its critical section.
										int artWaitingPoint = communicatedCPs.containsKey(drivingTracker) ? communicatedCPs.get(drivingTracker).getFirst() :
											(drivingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
										int artDrivingCSEnd = waitingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
										Dependency dep = new Dependency(drivingTracker.getTrajectoryEnvelope(),waitingTracker.getTrajectoryEnvelope(),Math.max(0, artWaitingPoint),artDrivingCSEnd);
										if (!artificialDependencies.containsKey(drivingRobotID)) artificialDependencies.put(drivingRobotID, new HashSet<Dependency>());
										artificialDependencies.get(drivingRobotID).add(dep);
										metaCSPLogger.info("Robot" + drivingRobotID + " cannot escape from CS: " + cs + ". Let's create a deadlock. Add artificial dependency for Robot" + drivingRobotID + " at " + dep.getWaitingPoint() + ".");
									}	
									metaCSPLogger.finest("Both can't. Driving Robot" + drivingRobotID + " at " + drivingCurrentIndex + " makes " + waitingRobotID + " waiting at CS " + cs + ".");
						}

						//Compute waiting path index point for waiting robot
						waitingPoint = getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex);

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

						if (escapingCSToWaitingRobotIDandCP.containsKey(cs) && escapingCSToWaitingRobotIDandCP.get(cs).getFirst() == waitingTracker.getTrajectoryEnvelope().getRobotID()) {
							waitingPoint = escapingCSToWaitingRobotIDandCP.get(cs).getSecond();
							metaCSPLogger.info("Use escaping waiting point. Make Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " stopping at " + waitingPoint + ".");
						}

						if (waitingPoint >= 0) {		
							//Make new dependency
							int drivingCSEnd =  drivingTracker.getTrajectoryEnvelope().getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							if (!currentDeps.containsKey(waitingTracker.getTrajectoryEnvelope().getRobotID())) currentDeps.put(waitingTracker.getTrajectoryEnvelope().getRobotID(), new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd);
							currentDeps.get(waitingTracker.getTrajectoryEnvelope().getRobotID()).add(dep);

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

		//You should lock the robots if you want to start re-planning. 
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
					lockedRobots.put(dep.getWaitingRobotID(), new Dependency(dep.getWaitingTrajectoryEnvelope(), null, dep.getWaitingPoint(), 0));
				}
				metaCSPLogger.finest("Locking robots: " + deadlockedRobots.toString());
				metaCSPLogger.info("Will re-plan for one of the following deadlocked robots: " + deadlockedRobots + " (" + allRobots + ")...");
				new Thread() {
					public void run() {
						rePlanPath(deadlockedRobots, allRobots, useStaticReplan);
					}
				}.start();

			}
		}
	}

	private boolean nonlivePair(Dependency dep1, Dependency dep2) {
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
			Geometry[] obstacles = null;
			int[] otherRobotIDs = null;
			synchronized (getCurrentDependencies()) {
				for (Dependency dep : getCurrentDependencies()) {
					if (dep.getWaitingRobotID() == robotID) {
						currentWaitingIndex = dep.getWaitingPoint();
						currentWaitingPose = dep.getWaitingPose();
						if (currentWaitingPose == null) throw new Error("Waiting pose should not be null in dep: " + dep);
						Trajectory traj = dep.getWaitingTrajectoryEnvelope().getTrajectory();
						oldPath = traj.getPoseSteering();
						currentWaitingGoal = oldPath[oldPath.length-1].getPose();
						break;
					}
				}

				otherRobotIDs = new int[allRobots.size()-1];
				int counter = 0;
				for (int otherRobotID : allRobots) if (otherRobotID != robotID) otherRobotIDs[counter++] = otherRobotID;

				//FIXME not synchronized on current dependencies
				obstacles = getObstaclesInCriticalPoints(otherRobotIDs);
			}

			metaCSPLogger.info("Attempting to re-plan path of Robot" + robotID + " (with obstacles for robots " + Arrays.toString(otherRobotIDs) + ")...");
			AbstractMotionPlanner mp = null;
			if (this.motionPlanners.containsKey(robotID)) mp = this.motionPlanners.get(robotID);
			else {
				metaCSPLogger.severe("Motion planner is not initialized for Robot" + robotID + ", cannot replan");
				continue;
			}
			synchronized (mp) {
				PoseSteering[] newPath = doReplanning(mp, currentWaitingPose, currentWaitingGoal, obstacles);
				replanningTrialsCounter.incrementAndGet();
				if (newPath != null && newPath.length > 0) {
					PoseSteering[] newCompletePath = new PoseSteering[newPath.length+currentWaitingIndex];
					for (int i = 0; i < newCompletePath.length; i++) {
						if (i < currentWaitingIndex) newCompletePath[i] = oldPath[i];
						else newCompletePath[i] = newPath[i-currentWaitingIndex];
					}
					replacePath(robotID, newCompletePath, currentWaitingIndex, robotsToReplan, useStaticReplan);
					successfulReplanningTrialsCounter.incrementAndGet();
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
			metaCSPLogger.finest("Unlocking robots: " + robotsToReplan.toString());
		}
	}

	@Override
	protected void setupInferenceCallback() {
		
		this.stopInference = false;
		this.inference = new Thread("Coordinator inference") {
			
			@Override
			public void run() {

				String fileName = new String(System.getProperty("user.home")+File.separator+"coordinator_stat.txt");
				initStat(fileName, "Init statistics: @"+Calendar.getInstance().getTimeInMillis() + "\n");
				String stat = new String(//"Legend: at each control period\n\t 1. elapsed time to compute critical sections\n\t 2.elapsed time to update dependencies\n\t 3. number of new critical sections\n\t"
						"elapsedTimeComputeCriticalSections\t elapsedTimeUpdateDependencies\t numberNewCriticalSections\t numberAllCriticalSections\t numberNewAddedMissions \t numberDrivingRobots\t expectedSleepingTime\t effectiveSleepingTime\t printStatisticsTime\t effectiveTc");
				if (avoidDeadlockGlobally.get()) stat = stat + new String("\t numberReversedPrecedenceOrder\t numberCurrentCycles");
				stat.concat("\n");				
				writeStat(fileName, stat);
				
				long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
				long elapsedTimeComputeCriticalSections = -1;
				long elapsedTimeUpdateDependencies = -1;
				int numberNewCriticalSections = -1;
				int numberAllCriticalSections = -1;
				int numberNewAddedMissions = 0;
				int numberDrivingRobots = 0;
				int MAX_ADDED_MISSIONS = 1;
				long expectedSleepingTime = -1;
				long effectiveSleepingTime = -1;
				long printStatisticsTime = -1;
				
				while (!stopInference) {
					elapsedTimeComputeCriticalSections = -1;
					elapsedTimeUpdateDependencies = -1;
					numberNewCriticalSections = -1;
					numberAllCriticalSections = -1;
					numberNewAddedMissions = 0;
					numberDrivingRobots = 0;
					
					synchronized (solver) {	
						for (Integer robotID : trackers.keySet()) 
							if (!(trackers.get(robotID) instanceof TrajectoryEnvelopeTrackerDummy)) numberDrivingRobots++;
						
						if (!missionsPool.isEmpty()) {
							
							//FIXME critical sections should be computed incrementally/asynchronously
							while (!missionsPool.isEmpty() && numberNewAddedMissions < MAX_ADDED_MISSIONS) {
								Pair<TrajectoryEnvelope,Long> te = missionsPool.pollFirst();
								envelopesToTrack.add(te.getFirst());
								numberNewAddedMissions++;
							}
							numberNewCriticalSections = allCriticalSections.size();
							elapsedTimeComputeCriticalSections = Calendar.getInstance().getTimeInMillis();
							computeCriticalSections();
							elapsedTimeComputeCriticalSections = Calendar.getInstance().getTimeInMillis()-elapsedTimeComputeCriticalSections;
							numberAllCriticalSections = allCriticalSections.size();
							numberNewCriticalSections = numberAllCriticalSections-numberNewCriticalSections;
							
							startTrackingAddedMissions();
						}
						elapsedTimeUpdateDependencies = Calendar.getInstance().getTimeInMillis();
						updateDependencies();
						elapsedTimeUpdateDependencies = Calendar.getInstance().getTimeInMillis()-elapsedTimeUpdateDependencies;
						numberAllCriticalSections = allCriticalSections.size();
						
						if (!quiet) {
							printStatisticsTime = Calendar.getInstance().getTimeInMillis();
							printStatistics();
							printStatisticsTime = Calendar.getInstance().getTimeInMillis()-printStatisticsTime;
						}
						if (overlay) overlayStatistics();
					}
					
					//Sleep a little...
					expectedSleepingTime = Math.max(500,CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate);
					effectiveSleepingTime = Calendar.getInstance().getTimeInMillis();
					if (CONTROL_PERIOD > 0) {
						try { Thread.sleep(expectedSleepingTime); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					effectiveSleepingTime = Calendar.getInstance().getTimeInMillis()-effectiveSleepingTime;
					
					EFFECTIVE_CONTROL_PERIOD = (int)(Calendar.getInstance().getTimeInMillis()-threadLastUpdate);
					threadLastUpdate = Calendar.getInstance().getTimeInMillis();
					
					if (inferenceCallback != null) inferenceCallback.performOperation();
										
					stat = new String(elapsedTimeComputeCriticalSections + "\t" + elapsedTimeUpdateDependencies + "\t" + numberNewCriticalSections + "\t" + numberAllCriticalSections + "\t" + numberNewAddedMissions + "\t" + numberDrivingRobots
							+ "\t" + expectedSleepingTime + "\t" + effectiveSleepingTime + "\t" + printStatisticsTime + "\t" + EFFECTIVE_CONTROL_PERIOD);
					if (avoidDeadlockGlobally.get()) stat = stat + new String("\t" + currentOrdersHeurusticallyDecided.get() + "\t" + currentCyclesList.size());
					stat.concat("\n");
					writeStat(fileName, stat);
				}
				
			}
		};
		inference.setPriority(Thread.MAX_PRIORITY);
		inference.start();
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
					metaCSPLogger.severe("<<<<<<<< WARNING: Cleaning up a critical section which was not associated to a dependency: " + cs + ".");
					//increment the counter
					if (cs.getTe1().getRobotID() == robotID && (cs.getTe1Start() <= lastWaitingPoint || lastWaitingPoint == -1) || 
							cs.getTe2().getRobotID() == robotID && (cs.getTe2Start() <= lastWaitingPoint || lastWaitingPoint == -1))
						this.criticalSectionCounter.incrementAndGet();
				}
			}

			for (CriticalSection cs : toRemove) {
				if (this.avoidDeadlockGlobally.get()) {
					if (CSToDepsOrder.containsKey(cs)) {
						int waitingRobID = CSToDepsOrder.get(cs).getFirst();
						int drivingRobID = cs.getTe1().getRobotID() == waitingRobID ? cs.getTe2().getRobotID() : cs.getTe1().getRobotID(); 
						deleteEdge(new Pair<Integer,Integer>(waitingRobID,drivingRobID));
						//metaCSPLogger.info(currentCyclesList.toString());
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
	 * @param breakingPathIndex Last point on the current path to preserve. 
	 * @param lockedRobotIDs The set of robots which have been locked when the re-plan started.
	 * @param <code>true</code> whether the robot should be at the current critical point before starting a re-plan.
	 */
	public void replacePath(int robotID, PoseSteering[] newPath, int breakingPathIndex, HashSet<Integer> lockedRobotIDs, boolean useStaticReplan) {

		synchronized (solver) {

			synchronized (allCriticalSections) {
				//Get current envelope
				TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);

				if (viz != null) {
					viz.removeEnvelope(te);
				}
				
				metaCSPLogger.info("Replacing TE " + te.getID() + " (Robot" + robotID + ") with breaking point " + breakingPathIndex + ".");

				//------------------ (dynamic re-plan) --------------------
				//store the order of precedence (and waiting points) for every 
				//critical section involving the robot which is re-planning
				HashMap<CriticalSection,Pair<Integer,Integer>> holdingCS = new HashMap<CriticalSection,Pair<Integer,Integer>>();
				if (!useStaticReplan) {
					for (CriticalSection cs : CSToDepsOrder.keySet()) {
						if (cs.getTe1().getRobotID() == robotID && cs.getTe1Start() <= breakingPathIndex || 
								cs.getTe2().getRobotID() == robotID && cs.getTe2Start() <= breakingPathIndex) {
							Pair<Integer,Integer> order = new Pair<Integer,Integer>(CSToDepsOrder.get(cs).getFirst(), CSToDepsOrder.get(cs).getSecond());
							holdingCS.put(cs, order);
							metaCSPLogger.finest("Saving " + order.toString() + " for critical section " + cs + ".");
						}
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

									//We should restore all the dependency order which cannot be safely reverted.
									//Each new critical section (cs1 = [start11,end11], [start12,end12]) containing the breaking point 
									//can be matched to its related old one (one cs2 = [start21,end21], [start22,end22]) by reasoning about the spatial overlaps between TEs' intervals. 
									//Note that start and end of each critical sections are both included (i.e., still overlapping configurations) 
									//-- see {@link CriticalSection}, {@link AbstractTrajectoryEnvelopeCoordinaror}, function: getCriticalSections).

									int start11 = cs1.getTe1().getRobotID() == robotID ? cs1.getTe1Start() : cs1.getTe2Start();
									int start12 = cs1.getTe1().getRobotID() == robotID ? cs1.getTe2Start() : cs1.getTe1Start();
									int end11 = cs1.getTe1().getRobotID() == robotID ? cs1.getTe1End() : cs1.getTe2End();
									int end12 = cs1.getTe1().getRobotID() == robotID ? cs1.getTe2End() : cs1.getTe1End();
									int start21 = cs2.getTe1().getRobotID() == robotID ? cs2.getTe1Start() : cs2.getTe2Start();
									int start22 = cs2.getTe1().getRobotID() == robotID ? cs2.getTe2Start() : cs2.getTe1Start();
									int end21 = cs2.getTe1().getRobotID() == robotID ? cs2.getTe1End() : cs2.getTe2End();
									int end22 = cs2.getTe1().getRobotID() == robotID ? cs2.getTe2End() : cs2.getTe1End();
									
									//Here we should consider all the cases of spatial overlap between the new critical section (cs1 = [start11,end11], [start12,end12]) and the old 
									//critical section (cs2 = [start21,end21], [start22,end22]). Note that start and end are still overlapping configurations (see {@link CriticalSection}, {@link AbstractTrajectoryEnvelopeCoordinaror}, function: getCriticalSections).
									if ((start21 <= start11 && start11 <= end21) && !(end22 < start12 || end12 < start22)) {
										metaCSPLogger.finest("Restoring  the order of critical section " + cs2 + "(" + holdingCS.get(cs2).toString() + ") for critical section " + cs1 + ".");
										CSToDepsOrder.put(cs1, holdingCS.get(cs2));

										if (this.avoidDeadlockGlobally.get()) {
											//re-add dependency to cyclesList and currentOrdersGraph
											HashMap<Pair<Integer,Integer>,Integer> edgesToAdd = new HashMap<Pair<Integer,Integer>,Integer>();
											int waitingRobotID = holdingCS.get(cs2).getFirst();
											int drivingRobotID = waitingRobotID == cs2.getTe1().getRobotID() ? cs2.getTe2().getRobotID() : cs2.getTe1().getRobotID();
											edgesToAdd.put(new Pair<Integer,Integer>(waitingRobotID, drivingRobotID),1);
											addEdges(edgesToAdd);
										}
										break;
									}
								}		
							}
						}
					}
				}

				envelopesToTrack.remove(newTE);

				synchronized (lockedRobots) {
					for (int ID : lockedRobotIDs) lockedRobots.remove(ID);
					metaCSPLogger.finest("Unlocking robots: " + lockedRobotIDs.toString());
				}

				updateDependencies();											

			}
		}
	}

	/**
	 * Truncate the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point. This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be truncated.
	 * @return <code>true</code> iff the envelope is successfully truncated.
	 */
	public boolean truncateEnvelope(int robotID) {
		return this.truncateEnvelope(robotID, true);
	}

	/**
	 * Truncate the {@link TrajectoryEnvelope} of a given robot. 
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be truncated.
	 * @param ensureDynamicFeasibility If <code>true</code>, truncate at the closest dynamically-feasible path point, which is computed via the robot's {@link ForwardModel}; if <code>false</code>, truncate at the current path index.
	 * @return <code>true</code> iff the envelope is successfully truncated.
	 */
	public boolean truncateEnvelope(int robotID, boolean ensureDynamicFeasibility) {

		//This does not seem necessary - why would you need a motion planner if you are just truncating?
		//		if(!this.motionPlanners.containsKey(robotID) && this.getDefaultMotionPlanner() == null) {
		//			metaCSPLogger.severe("Motion planner not initialized (neither specific for Robot" + robotID + ", nor a default one).");
		//			return false;
		//		}

		synchronized (solver) {

			synchronized (lockedRobots) {
				if (lockedRobots.containsKey(robotID)) return false;
			}

			TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);
			AbstractTrajectoryEnvelopeTracker tet = this.trackers.get(robotID); 

			if (!(tet instanceof TrajectoryEnvelopeTrackerDummy)) {

				int earliestStoppingPathIndex = -1;
				if (ensureDynamicFeasibility) earliestStoppingPathIndex = this.getForwardModel(robotID).getEarliestStoppingPathIndex(te, this.getRobotReport(robotID));
				else earliestStoppingPathIndex = this.getRobotReport(robotID).getPathIndex();

				if (earliestStoppingPathIndex != -1) {
					metaCSPLogger.info("Truncating " + te + " at " + earliestStoppingPathIndex);

					//Compute and add new TE, remove old TE (both driving and final parking)
					PoseSteering[] truncatedPath = Arrays.copyOf(te.getTrajectory().getPoseSteering(), earliestStoppingPathIndex+1);

					//replace the path of this robot (will compute new envelope)
					replacePath(robotID, truncatedPath, truncatedPath.length-1, new HashSet<Integer>(robotID),false);

				}
			}

			return true;
		}
	}

	/**
	 * Reverse the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point.
	 * This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be reversed.
	 * @return true if the envelope is successfully reversed.
	 */
	public boolean reverseEnvelope(int robotID) {

		//TODO: figure out why someone thought we need a motion planner to reverse!
		//if(!this.motionPlanners.containsKey(robotID) && this.getDefaultMotionPlanner() == null) {
		/*
		if(!this.motionPlanners.containsKey(robotID)) {
			metaCSPLogger.severe("Motion planner not initialized for Robot" + robotID + ", cannot reverse envelope");
			return false;
		}
		*/

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
					replacePath(robotID, overallPath, truncatedPath.length-1, new HashSet<Integer>(robotID),false);

				}
			}

			return true;
		}
	}

	protected void deleteEdge(Pair<Integer,Integer> edge) {
		HashMap<Pair<Integer,Integer>, Integer> edgesToDelete = new HashMap<Pair<Integer,Integer>, Integer>();
		edgesToDelete.put(edge, 1);
		deleteEdges(edgesToDelete);
	}
	
	protected void deleteEdges(HashMap<Pair<Integer,Integer>, Integer> edgesToDelete) {

		synchronized(allCriticalSections) {

			if (edgesToDelete == null || edgesToDelete.isEmpty()) return;

			for (Pair<Integer,Integer> edge : edgesToDelete.keySet()) {
				Integer occurrence = edgesToDelete.get(edge);
				if (occurrence == null || occurrence == 0) {
					metaCSPLogger.severe("<<<<<<<<<< Found edge " + edge.toString() + " with invalid weigth. Skipping deletion." );
					continue;
				}
				DefaultWeightedEdge e = currentOrdersGraph.getEdge(edge.getFirst(), edge.getSecond());
				if (e != null) {
					int numEdge = (int) currentOrdersGraph.getEdgeWeight(e);
					if (numEdge > occurrence) currentOrdersGraph.setEdgeWeight(e, numEdge-occurrence);
					else {
						//FIXME Waste of time is probably here
						metaCSPLogger.finest("Removing the edge: " + edge.toString());
						currentOrdersGraph.removeEdge(edge.getFirst(), edge.getSecond());
						if (currentCyclesList.containsKey(edge)) {
							HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> toRemove = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
							for (ArrayList<Integer> cycle : currentCyclesList.get(edge)) {
								for (int i = 0; i < cycle.size(); i++) {
									Pair<Integer,Integer> key = new Pair<Integer,Integer>(cycle.get(i), cycle.get(i < cycle.size()-1 ? i+1 : 0));
									if (!toRemove.containsKey(key)) toRemove.put(key, new HashSet<ArrayList<Integer>>());
									toRemove.get(key).add(cycle);
								}
							}
							for (Pair<Integer, Integer> key : toRemove.keySet()) {
								currentCyclesList.get(key).removeAll(toRemove.get(key));
								if (currentCyclesList.get(key).isEmpty()) currentCyclesList.remove(key);
							}
						}
					}
				}
			}
		}
	}

	protected void addEdges(HashMap<Pair<Integer,Integer>, Integer> edgesToAdd) {

		if (edgesToAdd == null || edgesToAdd.isEmpty()) return;
		
		synchronized(allCriticalSections) {

			HashSet<Pair<Integer,Integer>> toAdd = new HashSet<Pair<Integer,Integer>>();
	
			//add the edges if not already in the graph
			for (Pair<Integer,Integer> edge : edgesToAdd.keySet()) {
				Integer occurrence = edgesToAdd.get(edge);
				if (occurrence == null || occurrence == 0) {
					metaCSPLogger.severe("<<<<<<<<<< Found edge " + edge.toString() + " with invalid weigth. Skipping addition." );
					continue;
				}
				if (!currentOrdersGraph.containsEdge(edge.getFirst(),edge.getSecond())) {
					toAdd.add(edge);
					currentOrdersGraph.addVertex(edge.getFirst());
					currentOrdersGraph.addVertex(edge.getSecond());
					DefaultWeightedEdge e = currentOrdersGraph.addEdge(edge.getFirst(),edge.getSecond());
					if (e == null) System.out.println("<<<<<<<<< Add dependency order fails (12). Edge: " + edge.getFirst() + "->" + edge.getSecond());				
					currentOrdersGraph.setEdgeWeight(e, occurrence);
					metaCSPLogger.finest("Add " + occurrence + " edges:" + edge.toString());
				}
				else {
					DefaultWeightedEdge e = currentOrdersGraph.getEdge(edge.getFirst(),edge.getSecond());
					currentOrdersGraph.setEdgeWeight(e,currentOrdersGraph.getEdgeWeight(e)+occurrence);
				}
			}
			if (toAdd.isEmpty()) return;
	
			//compute strongly connected components
			KosarajuStrongConnectivityInspector<Integer,DefaultWeightedEdge> ksccFinder = new KosarajuStrongConnectivityInspector<Integer,DefaultWeightedEdge>(currentOrdersGraph);
			List<DirectedSubgraph<Integer,DefaultWeightedEdge>> sccs = ksccFinder.stronglyConnectedSubgraphs();
			metaCSPLogger.finest("Connected components: " + sccs.toString());
	
			//update the cycle list. Use a map to avoid recomputing cycles of each connected component.
			for (Pair<Integer,Integer> pair : toAdd) {
				//search the strongly connected components containing the two vertices
				for (DirectedSubgraph<Integer,DefaultWeightedEdge> scc : sccs) {
					if (scc.containsVertex(pair.getFirst()) ||	scc.containsVertex(pair.getSecond())) {
						if (scc.containsVertex(pair.getFirst()) && scc.containsVertex(pair.getSecond())) {
							//get cycles in this strongly connected components
							JohnsonSimpleCycles<Integer,DefaultWeightedEdge> cycleFinder = new JohnsonSimpleCycles<Integer,DefaultWeightedEdge>(scc);
							List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
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
						break;
					}
				}
			}
		}
	}

	protected void updateGraph(HashMap<Pair<Integer,Integer>, Integer> edgesToDelete, HashMap<Pair<Integer,Integer>, Integer> edgesToAdd) {

		synchronized(allCriticalSections) {		

			HashMap<Pair<Integer,Integer>, Integer> toDelete = null;
			if (edgesToDelete != null && !edgesToDelete.isEmpty()) {
				toDelete = new HashMap<Pair<Integer,Integer>, Integer>();
				for (Pair<Integer,Integer> edge : edgesToDelete.keySet()) {
					if (edgesToAdd.containsKey(edge) && edgesToAdd.get(edge) != null && edgesToAdd.get(edge) < edgesToDelete.get(edge)) 
						toDelete.put(edge, edgesToDelete.get(edge)-edgesToAdd.get(edge));
					else toDelete.put(edge, edgesToDelete.get(edge));
				}
			}
			
			HashMap<Pair<Integer,Integer>, Integer> toAdd = null;
			if (edgesToAdd != null && !edgesToAdd.isEmpty()) {
				toAdd = new HashMap<Pair<Integer,Integer>, Integer>();
				for (Pair<Integer,Integer> edge : edgesToAdd.keySet()) {
					if (edgesToDelete.containsKey(edge) && edgesToDelete.get(edge) != null && edgesToDelete.get(edge) < edgesToAdd.get(edge)) 
						toAdd.put(edge, edgesToAdd.get(edge)-edgesToDelete.get(edge));
					else toAdd.put(edge, edgesToAdd.get(edge));
				}
			}

			deleteEdges(toDelete);			
			addEdges(toAdd);

		}
	}

	protected void globalCheckAndRevise() {

		synchronized(solver) {
			HashMap<Integer,RobotReport> currentReports = new HashMap<Integer,RobotReport>();
			HashMap<Integer,HashSet<Dependency>> currentDeps = new HashMap<Integer,HashSet<Dependency>>();
			HashMap<Integer,HashSet<Dependency>> artificialDependencies = new HashMap<Integer,HashSet<Dependency>>(); 
			DirectedMultigraph<Integer,Dependency> depsGraph = new DirectedMultigraph<Integer, Dependency>(Dependency.class);
			HashSet<Integer> askForReplan = new HashSet<Integer>();			
			HashMap<Integer, Integer> earliestStoppingPoints = new HashMap<Integer,Integer>();

			HashMap<Pair<Integer,Integer>,Integer> edgesToDelete = new HashMap<Pair<Integer,Integer>,Integer> ();
			HashMap<Pair<Integer,Integer>,Integer> edgesToAdd = new HashMap<Pair<Integer,Integer>,Integer> ();
			HashSet<CriticalSection> reversibleCS = new HashSet<CriticalSection>();

			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				//Update the coordinator view
				RobotReport robotReport = robotTracker.getRobotReport();
				currentReports.put(robotID, robotReport);
				synchronized(stoppingPoints) {
					if (stoppingPoints.containsKey(robotID)) {
						metaCSPLogger.info("Stopping points Robot"+robotID+": "+stoppingPoints.get(robotID).toString());
						for (int i = 0; i < stoppingPoints.get(robotID).size(); i++) {
							int stoppingPoint = stoppingPoints.get(robotID).get(i);
							int duration = stoppingTimes.get(robotID).get(i);
							if (robotReport.getPathIndex() <= stoppingPoint) {
								Dependency dep = new Dependency(robotTracker.getTrajectoryEnvelope(), null, stoppingPoint, 0);
								if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new HashSet<Dependency>());
								if (!currentDeps.get(robotID).add(dep)) {
									metaCSPLogger.severe("<<<<<<<<< Add dependency fails (1). Dep: " + dep);
								}
							}
							//Start waiting thread if the stopping point has been reached
							//if (Math.abs(robotReport.getPathIndex()-stoppingPoint) <= 1 && robotReport.getCriticalPoint() == stoppingPoint && !stoppingPointTimers.containsKey(robotID)) {
							if (Math.abs(robotReport.getPathIndex()-stoppingPoint) <= 1 && robotReport.getCriticalPoint() <= stoppingPoint && !stoppingPointTimers.containsKey(robotID)) {
								spawnWaitingThread(robotID, i, duration);
							}
						}
					}
					synchronized(lockedRobots) {
						if (lockedRobots.containsKey(robotID)) 	{
							//We should enforce the last robot stopping point while it is involved in a re-plan.
							if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new HashSet<Dependency>());
							currentDeps.get(robotID).add(lockedRobots.get(robotID));
							metaCSPLogger.info("Add Locking dependency to Robot" + robotID + ". Dep: " + lockedRobots.get(robotID));
						}			
					}
				}
			}



			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
								
				//FIXME Add a CriticalSectionManager class
				if (allCriticalSections.size() > 0) 
					for (int robotID : robotIDs) 
						earliestStoppingPoints.put(robotID, getForwardModel(robotID).getEarliestStoppingPathIndex(trackers.get(robotID).getTrajectoryEnvelope(), currentReports.get(robotID)));

				depsToCS.clear();			
				this.isBlocked = false;

				HashSet<CriticalSection> toRemove = new HashSet<CriticalSection>();
				for (CriticalSection cs : this.allCriticalSections) {

					//Will be assigned depending on current situation of robot reports...
					int waitingPoint = -1;
					int drivingCurrentIndex = -1;
					AbstractTrajectoryEnvelopeTracker drivingTracker = null;
					AbstractTrajectoryEnvelopeTracker waitingTracker = null;

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
						this.isBlocked = true;

						//Robot1 is parking in critical section. If it is the driver, make robot 2 wait.
						if (robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy) {
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							waitingPoint = getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex);

							if (!communicatedCPs.containsKey(waitingTracker) && robotReport1.getPathIndex() <= waitingPoint 
									|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
								createAParkingDep = true;
							}

						}
						else if (robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy) {
							drivingCurrentIndex = robotReport2.getPathIndex();
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							waitingPoint = getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex);

							if (!communicatedCPs.containsKey(waitingTracker) && robotReport2.getPathIndex() <= waitingPoint
									|| communicatedCPs.containsKey(waitingTracker) && communicatedCPs.get(waitingTracker).getFirst() != -1 && communicatedCPs.get(waitingTracker).getFirst() <= waitingPoint) {
								createAParkingDep = true;						
							}
						}

						if (createAParkingDep) {
							int drivingCSEnd = drivingTracker.getTrajectoryEnvelope().getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							metaCSPLogger.finest("Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " is parked, so Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " will have to wait");
							if (!currentDeps.containsKey(waitingTracker.getTrajectoryEnvelope().getRobotID())) currentDeps.put(waitingTracker.getTrajectoryEnvelope().getRobotID(), new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd);
							if (!currentDeps.get(waitingTracker.getTrajectoryEnvelope().getRobotID()).add(dep)) {
								metaCSPLogger.severe("<<<<<<<<< Add dependency fails (2). Dep: " + dep);
							}

							//If cs is new, update the of edges to add (otherwise, it is already in the graph)
							if (!CSToDepsOrder.containsKey(cs)) {
								Pair<Integer,Integer> edge = new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getDrivingRobotID());
								if (edgesToAdd.containsKey(edge)) edgesToAdd.put(edge, edgesToAdd.get(edge)+1);
								else edgesToAdd.put(edge, 1);
							}

							//update the global graph
							if (!depsGraph.containsEdge(dep)) {
								if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
								if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
								if (!depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep))
									metaCSPLogger.severe("<<<<<<<<< Add dependency fails (3). Dep: " + dep);
							}
							CSToDepsOrder.put(cs, new Pair<Integer,Integer>(dep.getWaitingRobotID(),dep.getWaitingPoint()));
							depsToCS.put(dep, cs);
						}
					}
					else { //both the robots are driving. Check if the precedence is reversible.

						//Check if the robots can stop while changing the current order of accessing the critical section.
						boolean canStopRobot1 = false;
						boolean canStopRobot2 = false;
						boolean wakeUpinCSRobot1 = false;
						boolean wakeUpinCSRobot2 = false;


						//Force the dependency for the robot footprint
						if (//the last critical point was before the critical section (can stop by induction)
								(communicatedCPs.containsKey(robotTracker1) && communicatedCPs.get(robotTracker1).getFirst() != -1 && communicatedCPs.get(robotTracker1).getFirst() < cs.getTe1Start())
								|| !communicatedCPs.containsKey(robotTracker1) && Math.max(0, robotReport1.getPathIndex()) < cs.getTe1Start())
							canStopRobot1 = true;
						else
							//Due to temporal delays we cannot trust the velocity.
							canStopRobot1 = earliestStoppingPoints.get(robotReport1.getRobotID()) < cs.getTe1Start();
						if ((communicatedCPs.containsKey(robotTracker2) && communicatedCPs.get(robotTracker2).getFirst() != -1 && communicatedCPs.get(robotTracker2).getFirst() < cs.getTe2Start())
								|| !communicatedCPs.containsKey(robotTracker2) && Math.max(0, robotReport2.getPathIndex()) < cs.getTe2Start())
							canStopRobot2 = true;
						else canStopRobot2 = earliestStoppingPoints.get(robotReport2.getRobotID()) < cs.getTe2Start();


						//For each reversible constraint, we pre-load the precedence according to the FCFS heuristic or to the previous decided ones.
						//We will check if the heuristic after.
						if (canStopRobot1 && canStopRobot2) {
							reversibleCS.add(cs);
							boolean robot2Yields;	
							if (!CSToDepsOrder.containsKey(cs)) {
								if (robotTracker1.getStartingTimeInMillis() < robotTracker2.getStartingTimeInMillis() ||
										robotTracker1.getStartingTimeInMillis() == robotTracker2.getStartingTimeInMillis() && robotReport1.getRobotID() < robotReport2.getRobotID()) 
									robot2Yields = true;
								else robot2Yields = false;
							}
							else robot2Yields = (CSToDepsOrder.get(cs).getFirst() == robotReport2.getRobotID());
							metaCSPLogger.finest("Robot" + robotTracker1.getRobotReport().getRobotID() + ": " + robotTracker1.getStartingTimeInMillis() + ", " +
									"Robot" + robotTracker2.getRobotReport().getRobotID() + ": " + robotTracker2.getStartingTimeInMillis() + ", yielding: " + (robot2Yields ?
											robotReport2.getRobotID() : robotReport1.getRobotID()));
							drivingCurrentIndex = robot2Yields ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
							drivingTracker = robot2Yields ? robotTracker1 : robotTracker2;
							waitingTracker = robot2Yields ? robotTracker2 : robotTracker1;
						}

						//otherwise, 
						//the precedence is CONSTRAINED BY DYNAMICS

						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						else if (canStopRobot1 && !canStopRobot2) {
							drivingCurrentIndex = robotReport2.getPathIndex();
							drivingTracker = robotTracker2;
							waitingTracker = robotTracker1;
							metaCSPLogger.finest("One-can-one-can't-stop (1) and Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " (can't) is ahead of Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " (can) and CS is: " + cs);						
						}

						//Robot 2 can stop before entering critical section, robot 1 can't --> robot 2 waits
						else if (!canStopRobot1 && canStopRobot2) {
							drivingCurrentIndex = robotReport1.getPathIndex();
							drivingTracker = robotTracker1;
							waitingTracker = robotTracker2;
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTracker.getTrajectoryEnvelope().getRobotID() + " (can't) ahead of Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " (can) and CS is: " + cs);
						}

						else {	//Otherwise re-impose previously decided dependency if possible			
							
							int drivingRobotID = -1;
							int waitingRobotID = -1;

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
									
									//Try to recover the lost order.
									int ahead = 0;
									//Only the leading robot may be already commanded to go beyond the critical section end.
									if (communicatedCPs.containsKey(robotTracker1) && (communicatedCPs.get(robotTracker1).getFirst() == -1 || communicatedCPs.get(robotTracker1).getFirst() > cs.getTe1End())) ahead = 1;
									else if (communicatedCPs.containsKey(robotTracker2) && (communicatedCPs.get(robotTracker2).getFirst() == -1 || communicatedCPs.get(robotTracker2).getFirst() > cs.getTe2End())) ahead = -1;

									//Otherwise, if both the robots are inside the critical section check their poses.
									if (ahead == 0) ahead = isAhead(cs, robotReport1, robotReport2);
									
									//Otherwise, the dependency is lost. Try an error. FIXME
									if (ahead == 0) {
										if (!this.CSToDepsOrder.containsKey(cs)) throw new Error("FIXME! Lost dependency and order cannot be restored! Key value not found. RobotReport1: " + robotReport1.toString() + ", RobotReport2: " + robotReport2.toString()
										+", last communicated CPs: (" + (communicatedCPs.containsKey(robotTracker1) ? communicatedCPs.get(robotTracker1).getFirst() : "null") + "," + (communicatedCPs.containsKey(robotTracker2) ? communicatedCPs.get(robotTracker2).getFirst() : "null),") + 
										"), cs: "+ cs.toString());
										else if (this.CSToDepsOrder.get(cs) == null) throw new Error("FIXME! Lost dependency and order cannot be restored! Empty value." );	
									}
									else {
										drivingRobotID = ahead == 1 ? robotReport1.getRobotID() : robotReport2.getRobotID();
										waitingRobotID = drivingRobotID == robotReport1.getRobotID() ? robotReport2.getRobotID() : robotReport1.getRobotID();
										metaCSPLogger.info("<<<<<<<<< Restoring the lost order via estimation for Robot" + drivingRobotID);
									}					
								}			
							}

							drivingCurrentIndex = drivingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
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
									if (!canExitCriticalSection(drivingCurrentIndex, waitingCurrentIndex, drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(),lastIndexOfCSDriving)) {
										//it's too late for escaping. Let's create a deadlock: the parked robot should wait for the moving one,
										//while the other should be commanded to do not proceed beyond its last communicated CP. Both the dependencies should be added to the current set.									

										//The parked robot should wait for the other at its current path index till the moving robot has not been exited from its critical section.
										int artWaitingPoint = communicatedCPs.containsKey(drivingTracker) ? communicatedCPs.get(drivingTracker).getFirst() :
											(drivingRobotID == robotReport1.getRobotID() ? robotReport1.getPathIndex() : robotReport2.getPathIndex());
										int artDrivingCSEnd = waitingRobotID == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
										Dependency dep = new Dependency(drivingTracker.getTrajectoryEnvelope(),waitingTracker.getTrajectoryEnvelope(),Math.max(0, artWaitingPoint),artDrivingCSEnd);
										if (!artificialDependencies.containsKey(drivingRobotID)) artificialDependencies.put(drivingRobotID, new HashSet<Dependency>());
										artificialDependencies.get(drivingRobotID).add(dep);
										askForReplan.add(drivingRobotID);

										//update the global graph
										if (!depsGraph.containsEdge(dep)) {
											if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
											if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
											if (!depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep))
												metaCSPLogger.severe("<<<<<<<<< Add dependency fails (5a). Dep: " + dep);
										}

										metaCSPLogger.info("Robot" + drivingRobotID + " cannot escape from CS: " + cs + ". Let's create a deadlock. Add artificial dependency for Robot" + drivingRobotID + " at " + dep.getWaitingPoint() + ".");
									}	
									metaCSPLogger.finest("Both can't. Driving Robot" + drivingRobotID + " at " + drivingCurrentIndex + " makes " + waitingRobotID + " waiting at CS " + cs + ".");
						}
						//Compute waiting path index point for waiting robot
						waitingPoint = getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex);

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

						if (escapingCSToWaitingRobotIDandCP.containsKey(cs) && escapingCSToWaitingRobotIDandCP.get(cs).getFirst() == waitingTracker.getTrajectoryEnvelope().getRobotID()) {
							waitingPoint = escapingCSToWaitingRobotIDandCP.get(cs).getSecond();
							metaCSPLogger.info("Use escaping waiting point. Make Robot" + waitingTracker.getTrajectoryEnvelope().getRobotID() + " stopping at " + waitingPoint + ".");
						}

						if (waitingPoint >= 0) {		
							//Make new dependency
							int drivingCSEnd =  drivingTracker.getTrajectoryEnvelope().getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							if (!currentDeps.containsKey(waitingTracker.getTrajectoryEnvelope().getRobotID())) currentDeps.put(waitingTracker.getTrajectoryEnvelope().getRobotID(), new HashSet<Dependency>());
							Dependency dep = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd);
							if (!currentDeps.get(waitingTracker.getTrajectoryEnvelope().getRobotID()).add(dep)) {
								metaCSPLogger.severe("<<<<<<<<< Add dependency fails (4). Dep: " + dep);
							}

							//If cs is new, update the of edges to add (otherwise, it is already in the graph)
							if (!CSToDepsOrder.containsKey(cs)) {
								Pair<Integer,Integer> edge = new Pair<Integer,Integer>(dep.getWaitingRobotID(), dep.getDrivingRobotID());
								if (edgesToAdd.containsKey(edge)) edgesToAdd.put(edge, edgesToAdd.get(edge)+1);
								else edgesToAdd.put(edge,1);
							}

							//update the global graph
							if (!depsGraph.containsEdge(dep)) {
								if (!depsGraph.containsVertex(dep.getWaitingRobotID())) depsGraph.addVertex(dep.getWaitingRobotID());
								if (!depsGraph.containsVertex(dep.getDrivingRobotID())) depsGraph.addVertex(dep.getDrivingRobotID());
								if (!depsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep))
									metaCSPLogger.severe("<<<<<<<<< Add dependency fails (5). Dep: " + dep);
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
				}
				
				//Remove obsolete critical sections
				for (CriticalSection cs : toRemove) {
					this.allCriticalSections.remove(cs);
					if (CSToDepsOrder.containsKey(cs)) {
						int waitingRobID = CSToDepsOrder.get(cs).getFirst();
						int drivingRobID = cs.getTe1().getRobotID() == waitingRobID ? cs.getTe2().getRobotID() : cs.getTe1().getRobotID(); 
						Pair<Integer,Integer> edge = new Pair<Integer,Integer>(waitingRobID, drivingRobID);
						if (edgesToDelete.containsKey(edge)) edgesToDelete.put(edge, edgesToDelete.get(edge)+1);
						else edgesToDelete.put(edge,1);
					}
					else metaCSPLogger.info("WARNING: Obsolete critical section " + cs + " was not assigned to a dependence.");
					this.escapingCSToWaitingRobotIDandCP.remove(cs);
				}

				//increment the counter
				this.criticalSectionCounter.addAndGet(toRemove.size());		

				//update graph with all the not reversible constraints
				updateGraph(edgesToDelete, edgesToAdd);
				metaCSPLogger.finest("Constrained precedences leads to graph: " + currentOrdersGraph.toString() + ".");		

				//Let's try to reverse reversible constraints according to the user defined heuristic if this preserves liveness.		

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

					boolean robot2Yields = getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs);
					//true if robot1 should go before robot2, false vice versa
					boolean robot2YieldsOld = CSToDepsOrder.get(cs).getFirst() == robotReport2.getRobotID();

					if (robot2YieldsOld != robot2Yields) {
						metaCSPLogger.finest("Trying reversing a precedence at critical section " + cs + ".");

						//try reversing the order
						int drivingCurrentIndex = robot2Yields ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
						AbstractTrajectoryEnvelopeTracker drivingTracker = robot2Yields ? robotTracker1 : robotTracker2;
						AbstractTrajectoryEnvelopeTracker waitingTracker = robot2Yields ? robotTracker2 : robotTracker1;
						int waitingPoint = getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex);

						//Update graphs
						if (waitingPoint >= 0) {		
							
							//Store previous graph (FIXME)
							DirectedMultigraph<Integer,Dependency> backupDepsGraph = new DirectedMultigraph<Integer, Dependency>(Dependency.class);
							for (int v : depsGraph.vertexSet()) backupDepsGraph.addVertex(v);
							for (Dependency dep : depsGraph.edgeSet()) {
								if (!backupDepsGraph.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep)) 
									metaCSPLogger.severe("<<<<<<<<< Add dependency fails (6). Dep: " + dep);
							}

							SimpleDirectedWeightedGraph<Integer, DefaultWeightedEdge> backupGraph = new SimpleDirectedWeightedGraph<Integer, DefaultWeightedEdge>(DefaultWeightedEdge.class);
							for (int v : currentOrdersGraph.vertexSet()) backupGraph.addVertex(v);
							for (DefaultWeightedEdge e : currentOrdersGraph.edgeSet()) {
								DefaultWeightedEdge e_ = backupGraph.addEdge(currentOrdersGraph.getEdgeSource(e), currentOrdersGraph.getEdgeTarget(e));
								if (e_ == null) metaCSPLogger.severe("<<<<<<<<< Add egde fails (7). Edge: " + e.toString());
								backupGraph.setEdgeWeight(e_, currentOrdersGraph.getEdgeWeight(e));
							}

							HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> backupcurrentCyclesList = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
							for (Pair<Integer,Integer> key : currentCyclesList.keySet())
								backupcurrentCyclesList.put(key, new HashSet<ArrayList<Integer>>(currentCyclesList.get(key)));
							
							edgesToDelete.put(new Pair<Integer,Integer>(drivingTracker.getTrajectoryEnvelope().getRobotID(), waitingTracker.getTrajectoryEnvelope().getRobotID()), 1);
							Pair<Integer,Integer> newEdge = new Pair<Integer,Integer>(waitingTracker.getTrajectoryEnvelope().getRobotID(), drivingTracker.getTrajectoryEnvelope().getRobotID());
							edgesToAdd.put(newEdge, 1);
							updateGraph(edgesToDelete,edgesToAdd);
							metaCSPLogger.finest("Graph after revising according to the heuristic: " + currentOrdersGraph.toString() + ".");		

							//Make new dependency
							int drivingCSEnd =  drivingTracker.getTrajectoryEnvelope().getRobotID() == cs.getTe1().getRobotID() ? cs.getTe1End() : cs.getTe2End();
							int drivingCSEndOld =  drivingTracker.getTrajectoryEnvelope().getRobotID()  == cs.getTe1().getRobotID() ? cs.getTe2End() : cs.getTe1End();
							Dependency depNew = new Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd);
							Dependency depOld = new Dependency(drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), CSToDepsOrder.get(cs).getSecond(), drivingCSEndOld);

							//update the global graph and treeset
							depsGraph.removeEdge(depOld);
							if (!depsGraph.containsVertex(depNew.getWaitingRobotID())) depsGraph.addVertex(depNew.getWaitingRobotID());
							if (!depsGraph.containsVertex(depNew.getDrivingRobotID())) depsGraph.addVertex(depNew.getDrivingRobotID());
							if (!depsGraph.addEdge(depNew.getWaitingRobotID(), depNew.getDrivingRobotID(), depNew))
								metaCSPLogger.severe("<<<<<<<<< Add dependency fails (10). Dep: " + depNew);

							//check if safe (continue here) 
							boolean safe = true;
							if (currentCyclesList.containsKey(newEdge)) {
								//for each cycle involving the new edge
								for (List<Integer> cycle : currentCyclesList.get(newEdge)) {

									//Get edges along the cycle...
									//  Recall: there could be more than one edge per pair of vertices, as this is a multi-graph
									ArrayList<ArrayList<Dependency>> edgesAlongCycle = new ArrayList<ArrayList<Dependency>>();
									for (int i = 0; i < cycle.size(); i++) {
										int j = i < cycle.size()-1 ? i+1 : 0;
										if (cycle.get(i) == depNew.getWaitingRobotID() && cycle.get(j) == depNew.getDrivingRobotID()) {
											ArrayList<Dependency> array = new ArrayList<Dependency>();
											array.add(depNew);
											edgesAlongCycle.add(array);
										}
										else {
											Set<Dependency> allEdges = depsGraph.getAllEdges(cycle.get(i), cycle.get(j));
											if (allEdges == null) {
												metaCSPLogger.severe("<<<<<<<<< Unfound one vertex between " + cycle.get(i) + " and "+ cycle.get(j));
												//metaCSPLogger.info(depsGraph.toString());
												//metaCSPLogger.info(currentCyclesList.toString());
											}
											else {
												if (allEdges.isEmpty()) {
													metaCSPLogger.severe("<<<<<<<<< Unfound deps from: " + cycle.get(i) + " to: "+ cycle.get(j) + " when reversing dep  " + depOld + " to " + depNew);
													metaCSPLogger.severe("currentCyclesList: " + currentCyclesList.get(newEdge).toString());
													metaCSPLogger.severe("currentOrdersGraph contains it? " + currentOrdersGraph.containsEdge(cycle.get(i), cycle.get(j)));
													metaCSPLogger.severe("backupDepsGraph contains it? " + backupDepsGraph.containsEdge(cycle.get(i), cycle.get(j)));
													metaCSPLogger.severe("backupGraph contains it? " + backupGraph.containsEdge(cycle.get(i), cycle.get(j)));
													boolean got = false;
													if (currentDeps.containsKey(cycle.get(i))) {
														for (Dependency dep1 : currentDeps.get(cycle.get(i))) {
															if (dep1.getDrivingRobotID() == cycle.get(j)) {
																got = true;
																break;
															}
														}
													}
													metaCSPLogger.severe("currentDeps contains it? " + got);
												}
												edgesAlongCycle.add(new ArrayList<Dependency>(allEdges));
											}
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

									//Check for nonlive cycles, that is:
									//  All along the cycle, check if there are (u,v,dep1) and (v,w,dep2) such that
									//    v.dep2.waitingpoint <= v.dep1.releasingpoint

									for (ArrayList<Dependency> depList : newDeps) {
										for (int i = 0; i < depList.size(); i++) {
											safe = true;
											Dependency dep1 = depList.get(i);
											Dependency dep2 = depList.get(i < depList.size()-1 ? i+1 : 0);
											if (nonlivePair(dep1,dep2)) safe = false;
											if (safe) break; //if one pair in the cycle is safe, then the cycle is safe
										}
										//  If there exists almost one nonlive cycle, then the precedence cannot be reversed.	
										if(!safe) {
											metaCSPLogger.finest("A nonlive cycle: " + depList + ".");
											break;
										}
									}

									if(!safe) {
										metaCSPLogger.finest("Cycle: " + edgesAlongCycle + " is NOT deadlock-free");
										break;
									}
								}
							}

							if (!safe) {								
								metaCSPLogger.finest("Restore previous precedence " + depOld + ".");
								depsGraph = backupDepsGraph;
								currentOrdersGraph = backupGraph;
								currentCyclesList = backupcurrentCyclesList;
								nonliveStatesAvoided.incrementAndGet();
							}
							else {
								//update the maps
								if (!currentDeps.get(depOld.getWaitingRobotID()).remove(depOld)) metaCSPLogger.severe("<<<<<<<< Error in removing dep: " + depOld);
								if (currentDeps.get(depOld.getWaitingRobotID()).isEmpty()) currentDeps.remove(depOld.getWaitingRobotID());
								if (!currentDeps.containsKey(waitingTracker.getTrajectoryEnvelope().getRobotID())) currentDeps.put(waitingTracker.getTrajectoryEnvelope().getRobotID(), new HashSet<Dependency>());
								if (!currentDeps.get(waitingTracker.getTrajectoryEnvelope().getRobotID()).add(depNew)) {
									metaCSPLogger.severe("<<<<<<<<< Add dependency fails (11). Dep: " + depNew);
								}
								CSToDepsOrder.put(cs, new Pair<Integer,Integer>(depNew.getWaitingRobotID(), depNew.getWaitingPoint()));
								depsToCS.put(depNew, cs);
								metaCSPLogger.finest("Update precedences " + depNew + " according to heuristic.");
								currentOrdersHeurusticallyDecided.incrementAndGet();
							}
							metaCSPLogger.finest("Final graph: " + currentOrdersGraph.toString() + ".");	

						}
					}
				}//end for reversibleCS*/
			}//end synchronized(allCriticalSections)

			//update and communicate critical points
			//get current dependencies
			synchronized(currentDependencies) {

				//FIXME: already synchronized (so maybe is ok currentDependencies = computeClosestDependencies(currentDeps, artificialDependencies);)
				HashSet<Dependency> closestDeps  = computeClosestDependencies(currentDeps, artificialDependencies);
				currentDependencies.clear();
				currentDependencies.addAll(closestDeps);

				//The global strategy builds upon the assumption that robots do not start from critical section. 
				//If this is not the case, them pre-loading may bring to nonlive cycles. 
				//To handle this case, switch to a local strategy whenever a robot is starting from a critical section and cannot
				//exit from it.
				/*if (!artificialDependencies.isEmpty()) {
					this.breakDeadlocksByReplanning = true;

					//find cycles and revise dependencies if necessary
					findCurrentCycles(currentDeps, artificialDependencies, new HashSet<Dependency>(), currentReports, robotIDs);

					this.breakDeadlocksByReplanning = false;
				}*/

				//re-plan for the set of robots that are currently in a critical section
				SimpleDirectedGraph<Integer,Dependency> g = depsToGraph(currentDependencies);
				List<List<Integer>> nonliveCycles = findSimpleNonliveCycles(g);
				nonliveStatesDetected.addAndGet(nonliveCycles.size());

				HashSet<Integer> spawnedThreadRobotSet = new HashSet<Integer>();
				for (List<Integer> cycle : nonliveCycles) {
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
								&& ((int)(Calendar.getInstance().getTimeInMillis()-communicatedCPs.get(tracker).getSecond().longValue()) > maxDelay);
						setCriticalPoint(dep.getWaitingRobotID(), dep.getWaitingPoint(), retransmitt);

					}
					else {
						boolean retransmitt = communicatedCPs.containsKey(tracker) && communicatedCPs.get(tracker).getFirst() == -1 && currentReports.get(robotID).getCriticalPoint() != -1
								&& ((int)(Calendar.getInstance().getTimeInMillis()-communicatedCPs.get(tracker).getSecond().longValue()) > maxDelay);
						setCriticalPoint(robotID, -1, retransmitt);
					}
				}				
			}

		}//end synchronized(solver)

	}//end checkAndRevise

}//end class


//FIXME Strating from critical section with checkAndRevise