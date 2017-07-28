package se.oru.coordination.coordination_oru;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.graph.DirectedMultigraph;
import org.metacsp.framework.Constraint;
import org.metacsp.framework.Variable;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.JTSDrawingPanel;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.MultiPolygon;

import edu.uci.ics.jung.graph.DirectedSparseMultigraph;

/**
 * This class provides coordination for a fleet of robots. An instantiatable {@link TrajectoryEnvelopeCoordinator}
 * must provide an implementation of a time keeping method, a {@link TrajectoryEnvelope} tracker factory, and
 * a criteria with which robots are to be prioritized.
 * 
 * @author fpa
 *
 */
public abstract class TrajectoryEnvelopeCoordinator {
	
	public static final int PARKING_DURATION = 3000;
	protected static final long STOPPING_TIME = 5000;
	protected int CONTROL_PERIOD;
	protected double TEMPORAL_RESOLUTION;
	
	protected boolean overlay = false;
	
	protected TrajectoryEnvelopeSolver solver = null;
	protected JTSDrawingPanel panel = null;
	protected ArrayList<TrajectoryEnvelope> envelopesToTrack = new ArrayList<TrajectoryEnvelope>();
	protected HashMap<CriticalSection,Dependency> criticalSectionsToDeps = new HashMap<CriticalSection, Dependency>();
	protected ArrayList<CriticalSection> allCriticalSections = new ArrayList<CriticalSection>();
	protected HashMap<Integer,ArrayList<Integer>> stoppingPoints = new HashMap<Integer,ArrayList<Integer>>();
	protected HashMap<Integer,Thread> stoppingPointTimers = new HashMap<Integer,Thread>();
	
	protected HashMap<Integer,AbstractTrajectoryEnvelopeTracker> trackers = new HashMap<Integer, AbstractTrajectoryEnvelopeTracker>();
	protected HashSet<Dependency> currentDependencies = new HashSet<Dependency>();
	
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(TrajectoryEnvelopeCoordinator.class);
	protected String logDirName = null;
	
	protected HashMap<AbstractTrajectoryEnvelopeTracker,Integer> communicatedCPs = new HashMap<AbstractTrajectoryEnvelopeTracker, Integer>();
	
	protected Map mapMetaConstraint = null;

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
	
	protected void setupLogging() {
		//logDirName = "log-" + Calendar.getInstance().getTimeInMillis();
		logDirName = "logs";
		File dir = new File(logDirName);
		dir.mkdir();
		MetaCSPLogging.setLogDir(logDirName);
	}
	
	public TrajectoryEnvelopeSolver getSolver() {
		return this.solver;
	}
	
	/**
	 * Instruct a given robot's tracker that it may not navigate beyond a given 
	 * path index.  
	 * @param robotID The ID of the robot.
	 * @param criticalPoint The index of the path pose beyond which the robot should not navigate.
	 */
	public void setCriticalPoint(int robotID, int criticalPoint) {
		if (!communicatedCPs.containsKey(trackers.get(robotID)) || !communicatedCPs.get(trackers.get(robotID)).equals(criticalPoint) ) {
			communicatedCPs.put(trackers.get(robotID), criticalPoint);
			trackers.get(robotID).setCriticalPoint(criticalPoint);
		}
	}
	
	/**
	 * Get the current state of a given robot.
	 * @param robotID The ID of the robot of which the state should be returned.
	 * @return The current state of a given robot.
	 */
	public RobotReport getRobotReport(int robotID) {
		return trackers.get(robotID).getRobotReport();
	}

	//Define the footprint (same for all robots)
	//NOTE: coordinates must be in CCW or CW order
	protected Coordinate[] footprint = new Coordinate[] {
			new Coordinate(-1.7, 0.7),	//back left
			new Coordinate(-1.7, -0.7),	//back right
			new Coordinate(2.7, -0.7),	//front right
			new Coordinate(2.7, 0.7)	//front left
	};
	
	/**
	 * Set the footprint of this TrajectoryEnvelope, which is used for computing the spatial envelope.
	 * Provide the bounding polygon of the machine assuming its reference point is in (0,0), and its
	 * orientation is aligned with the x-axis. The coordinates must be in CW or CCW order.
	 * @param coordinates The coordinates delimiting bounding polygon of the footprint.
	 */
	public void setFootprint(Coordinate ... coordinates) {
		this.footprint = coordinates; 
	}
	
	/**
	 * Call this method to setup the solvers that manage the {@link TrajectoryEnvelope} representation
	 * underlying the coordinator.
	 * @param origin The origin of time (milliseconds).
	 * @param horizon The maximum time (milliseconds).
	 */
	public void setupSolver(long origin, long horizon) {
		
		//Create meta solver and solver
		solver = new TrajectoryEnvelopeSolver(origin, horizon);
				
		//Start a thread that checks and enforces dependencies at every clock tick
		this.setupInferenceCallback();
		
	}
		
	/**
	 * Get the current time of the system, in milliseconds.
	 * @return The current time of the system, in milliseconds.
	 */
	public abstract long getCurrentTimeInMillis(); 
	
	public void placeRobot(final int robotID, Pose currentPose) {
		this.placeRobot(robotID, currentPose, null, currentPose.toString());
	}
	
	public void placeRobot(final int robotID, TrajectoryEnvelope parking) {
		this.placeRobot(robotID, null, parking, null);
	}
	
	public void placeRobot(final int robotID, Pose currentPose, TrajectoryEnvelope parking, String location) {
		
		if (solver == null) {
			metaCSPLogger.severe("Solver not initialized, please call method setupSolver() first!");
			throw new Error("Solver not initialized, please call method setupSolver() first!");
		}

		synchronized (solver) {	
			//Create a new parking envelope
			long time = getCurrentTimeInMillis();
			
			//Can provide null parking or null currentPose, but not both
			if (parking == null) parking = solver.createParkingEnvelope(robotID, PARKING_DURATION, currentPose, location, footprint);
			else currentPose = parking.getTrajectory().getPose()[0];
			
			AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(time, time));
			release.setFrom(parking);
			release.setTo(parking);
			if (!solver.addConstraint(release)) {
				metaCSPLogger.severe("Could not release " + parking + " with constriant " + release);
				throw new Error("Could not release " + parking + " with constriant " + release);
			}
			metaCSPLogger.info("Placed " + parking.getComponent() + " in pose " + currentPose + ": " + parking);
			
			TrackingCallback cb = new TrackingCallback() {
				@Override
				public void beforeTrackingStart() { }
				@Override
				public void onTrackingStart() { }
				@Override
				public void onNewGroundEnvelope() { }
				@Override
				public void beforeTrackingFinished() { }
				@Override
				public void onTrackingFinished() { }
			};
			
			//Now start the tracker for this parking (will be ended by call to addMissions for this robot)
			final TrajectoryEnvelopeCoordinator tec = this;
			final TrajectoryEnvelopeTrackerDummy tracker = new TrajectoryEnvelopeTrackerDummy(parking, 300, TEMPORAL_RESOLUTION, solver, cb) {
				@Override
				public long getCurrentTimeInMillis() {
					return tec.getCurrentTimeInMillis();
				}
				
				@Override
				public void onPositionUpdate() { }
				
			};
						
			synchronized (trackers) {
				trackers.put(robotID, tracker);
			}
			
		}
	}
	
	
	//Robot 2 follows robot 1
	protected int getCriticalPoint(TrajectoryEnvelope te1, TrajectoryEnvelope te2, int currentPIR1, int te1Start, int te1End, int te2Start) {
		
		//Number of additional path points robot 2 should stay behind robot 1
		int TRAILING_PATH_POINTS = 3;
		
		//How far ahead in robot 1's trajectory should we look ahead for collisions
		//(all the way to end of critical section if robots can drive in opposing directions)
		int LOOKAHEAD = te1End; //5;
		
		//How far into the critical section has robot 1 reached 
		int depthRobot1 = currentPIR1-te1Start;
		
		//If robot 1 not in critical section yet, return critical section start as waiting point for robot 2
		if (depthRobot1 < 0) return Math.max(0, te2Start-TRAILING_PATH_POINTS);	
		
		//Compute sweep of robot 1's footprint from current position to LOOKAHEAD
		Pose robot1Pose = te1.getTrajectory().getPose()[currentPIR1];
		Geometry robot1InPose = TrajectoryEnvelope.getFootprint(te1.getFootprint(), robot1Pose.getX(), robot1Pose.getY(), robot1Pose.getTheta());
		for (int i = 1; currentPIR1+i < LOOKAHEAD && currentPIR1+i < te1.getTrajectory().getPose().length; i++) {
			Pose robot1NextPose = te1.getTrajectory().getPose()[currentPIR1+i];
			robot1InPose = robot1InPose.union(TrajectoryEnvelope.getFootprint(te1.getFootprint(), robot1NextPose.getX(), robot1NextPose.getY(), robot1NextPose.getTheta()));			
		}
		
		//Starting from the same depth into the critical section as that of robot 1,
		//and decreasing the pose index backwards down to te2Start,
		//return the pose index as soon as robot 2's footprint does not intersect with robot 1's sweep
		for (int i = Math.min(te2.getTrajectory().getPose().length-1, te2Start+depthRobot1); i > te2Start-1; i--) {
			Pose robot2Pose = te2.getTrajectory().getPose()[i];
			Geometry robot2InPose = TrajectoryEnvelope.getFootprint(te2.getFootprint(), robot2Pose.getX(), robot2Pose.getY(), robot2Pose.getTheta());
			if (!robot1InPose.intersects(robot2InPose)) {
				return Math.max(0, i-TRAILING_PATH_POINTS);
			}
		}

		//The only situation where the above has not returned is when robot 2 should
		//stay "parked", therefore wait at index 0
		return Math.max(0, te2Start-TRAILING_PATH_POINTS);
		
//		//The above should have returned, as i is decremented until robot 2 is before the critical section
//		System.out.println("Robot" + te1.getRobotID() + ": start=" +te1Start + " depth=" + depthRobot1 + " Robot" + te2.getRobotID() + ": start=" + te2Start);
//		metaCSPLogger.severe("Could not determine CP for " + te2);
//		throw new Error("Could not determine CP for " + te2);
	}
	
	protected int getCriticalPoint() {
		int BUFFER = 10;
		return BUFFER;
	}
	
	//Spawn a waiting thread at this stopping point
	protected void spawnWaitingThread(final int robotID, final int index) {
		Thread stoppingPointTimer = new Thread() {
			private long startTime = Calendar.getInstance().getTimeInMillis();
			public void run() {
				while (Calendar.getInstance().getTimeInMillis()-startTime < STOPPING_TIME) {
					try { Thread.sleep(100); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
				stoppingPoints.get(robotID).remove(index);
				stoppingPointTimers.remove(robotID);
				//waitingTracker.setCriticalPoint(-1);
			}
		};
		stoppingPointTimer.start();
		stoppingPointTimers.put(robotID,stoppingPointTimer);		
	}
	
	// Deadlock:
	//  R1-x/R2-y
	//  R2-z/R1-w 
	// (R1-x/R2-y)              --> deadlock iff (y>z AND w>x)
	// 
	//  R1-x1/R2-x2
	//  R2-x3/R3-x4
	//  R3-x5/R4-x6
	//  ...
	//  Rj-x(2j-1)/R(j+1)-x(2j)
	//  ...
	//  Rn-x(2n-1)/R1-x(2n)     
	// (R1-x1/R2-x2)            --> deadlock iff x(2j)>x(2j+1) forall j in [1..n] AND x(2n)>x1
	
	
	private void findCycles() {
		DirectedMultigraph<Integer,Dependency> g = new DirectedMultigraph<Integer, Dependency>(Dependency.class);
		for (Dependency dep : currentDependencies) {
			if (!g.containsVertex(dep.getWaitingRobotID())) g.addVertex(dep.getWaitingRobotID());
			if (!g.containsVertex(dep.getDrivingRobotID())) g.addVertex(dep.getDrivingRobotID());
			g.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
		}
		JohnsonSimpleCycles<Integer, Dependency> cycleFinder = new JohnsonSimpleCycles<Integer, Dependency>(g);
		List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
		for (List<Integer> cycle : cycles) {
			ArrayList<ArrayList<Dependency>> edgesAlongCycle = new ArrayList<ArrayList<Dependency>>();
			Collections.reverse(cycle);
			for (int i = 0; i < cycle.size(); i++) {
				if (i < cycle.size()-1) {
					edgesAlongCycle.add(new ArrayList<Dependency>(g.getAllEdges(cycle.get(i), cycle.get(i+1))));
				}
				else {
					edgesAlongCycle.add(new ArrayList<Dependency>(g.getAllEdges(cycle.get(i), cycle.get(0))));
				}
			}

			
			for (int i = 0; i < edgesAlongCycle.size()-1; i++) {
				boolean safe = true;
				for (Dependency dep1 : edgesAlongCycle.get(i)) {
					for (Dependency dep2 : edgesAlongCycle.get(i+1)) {
						if (unsafePair(dep1, dep2)) {
							safe = false;
						}
					}
				}
				if (safe) {
					System.out.println("Cycle: " + edgesAlongCycle + " is deadlock-free");
					break;
				}
				if (i == edgesAlongCycle.size()-2) System.out.println("Cycle: " + edgesAlongCycle + " is NOT deadlock-free");
			}
			
		}
	}
	
	private boolean unsafePair(Dependency dep1, Dependency dep2) {
		if (dep2.getWaitingPoint() < dep1.getReleasingPoint()) return true;
		return false;
	}
	
	//Update and set the critical points
	protected void updateDependencies() {

		HashMap<Integer,TreeSet<Dependency>> currentDeps = new HashMap<Integer,TreeSet<Dependency>>();

		//Make deps from un-reached stopping points
		Set<Integer> robotIDs = trackers.keySet();
		for (int robotID : robotIDs) {
			AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
			RobotReport robotReport = robotTracker.getRobotReport();
			synchronized(stoppingPoints) {
				if (stoppingPoints.containsKey(robotID)) {
					for (int i = 0; i < stoppingPoints.get(robotID).size(); i++) {
						int stoppingPoint = stoppingPoints.get(robotID).get(i);
						if (robotReport.getPathIndex() <= stoppingPoint) {
							Dependency dep = new Dependency(robotTracker.getTrajectoryEnvelope(), null, stoppingPoint, 0, robotTracker, null);
							if (!currentDeps.containsKey(robotID)) currentDeps.put(robotID, new TreeSet<Dependency>());
							currentDeps.get(robotID).add(dep);					
						}
						//Start waiting thread if the stopping point has been reached
						if (Math.abs(robotReport.getPathIndex()-stoppingPoint) <= 1 && robotReport.getCriticalPoint() == stoppingPoint && !stoppingPointTimers.containsKey(robotID)) {
							spawnWaitingThread(robotID, i);
						}
					}
				}
			}
		}
		
		//Make deps from critical sections, and remove obsolete critical sections
		synchronized(allCriticalSections) {
			
			ArrayList<CriticalSection> toRemove = new ArrayList<CriticalSection>();
			for (CriticalSection cs : allCriticalSections) {
				
				//Will be assigned depending on current situation of robot reports...
				int waitingRobotID = -1;
				int drivingRobotID = -1;
				AbstractTrajectoryEnvelopeTracker waitingTracker = null;
				AbstractTrajectoryEnvelopeTracker drivingTracker = null;
				int waitingCurrentIndex = -1;
				int drivingCurrentIndex = -1;
				TrajectoryEnvelope waitingTE = null;
				TrajectoryEnvelope drivingTE = null;
				int waitingCSStart = -1;
				int drivingCSStart = -1;
				int waitingCSEnd = -1;
				int drivingCSEnd = -1;
				
				AbstractTrajectoryEnvelopeTracker robotTracker1 = trackers.get(cs.getTe1().getRobotID());
				RobotReport robotReport1 = robotTracker1.getRobotReport();
				AbstractTrajectoryEnvelopeTracker robotTracker2 = trackers.get(cs.getTe2().getRobotID());
				RobotReport robotReport2 = robotTracker2.getRobotReport();
				if (!(robotTracker1 instanceof TrajectoryEnvelopeTrackerDummy) && !(robotTracker2 instanceof TrajectoryEnvelopeTrackerDummy)) {
					//One or both robots past end of the critical section --> critical section is obsolete
					if (robotReport1.getPathIndex() > cs.getTe1End() || robotReport2.getPathIndex() > cs.getTe2End()) {
						toRemove.add(cs);
						metaCSPLogger.finest("OBSOLETE critical section\n\t" + cs);
						continue;
					}
					//Neither robot has reached the critical section --> follow ordering or closest robot if ordering = null
					else if (robotReport1.getPathIndex() < cs.getTe1Start() && robotReport2.getPathIndex() < cs.getTe2Start()) {
						
						Dependency previousDep = criticalSectionsToDeps.get(cs);
						if (previousDep != null) {
							if (previousDep.getWaitingRobotID() == robotTracker1.getTrajectoryEnvelope().getRobotID()) {
								waitingCurrentIndex = robotReport1.getPathIndex();
								drivingCurrentIndex = robotReport2.getPathIndex();
								waitingTE = cs.getTe1();
								drivingTE = cs.getTe2();
								waitingCSStart = cs.getTe1Start();
								drivingCSStart = cs.getTe2Start();
								waitingCSEnd = cs.getTe1End();
								drivingCSEnd = cs.getTe2End();
							}
							else {
								waitingCurrentIndex = robotReport2.getPathIndex();
								drivingCurrentIndex = robotReport1.getPathIndex();
								waitingTE = cs.getTe2();
								drivingTE = cs.getTe1();
								waitingCSStart = cs.getTe2Start();
								drivingCSStart = cs.getTe1Start();
								waitingCSEnd = cs.getTe2End();
								drivingCSEnd = cs.getTe1End();
							}
							metaCSPLogger.finest("R1 (OUT) / R2 (OUT) --> reusing previous ordering\n\t" + cs);
						}

						else if (getOrdering() != null) {
							if (getOrdering().compare(robotTracker1, robotTracker2) > 0) {
								waitingCurrentIndex = robotReport1.getPathIndex();
								drivingCurrentIndex = robotReport2.getPathIndex();
								waitingTE = cs.getTe1();
								drivingTE = cs.getTe2();
								waitingCSStart = cs.getTe1Start();
								drivingCSStart = cs.getTe2Start();
								waitingCSEnd = cs.getTe1End();
								drivingCSEnd = cs.getTe2End();															
							}
							else {
								waitingCurrentIndex = robotReport2.getPathIndex();
								drivingCurrentIndex = robotReport1.getPathIndex();
								waitingTE = cs.getTe2();
								drivingTE = cs.getTe1();
								waitingCSStart = cs.getTe2Start();
								drivingCSStart = cs.getTe1Start();
								waitingCSEnd = cs.getTe2End();
								drivingCSEnd = cs.getTe1End();
							}
						}
						
						else {
							boolean robot2Closest = ((cs.getTe2Start()-robotReport2.getPathIndex()) < (cs.getTe1Start()-robotReport1.getPathIndex())); 
							waitingCurrentIndex = robot2Closest ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
							drivingCurrentIndex = robot2Closest ? robotReport2.getPathIndex() : robotReport1.getPathIndex();
							waitingTE = robot2Closest ? cs.getTe1() : cs.getTe2();
							drivingTE = robot2Closest ? cs.getTe2() : cs.getTe1();
							waitingCSStart = robot2Closest ? cs.getTe1Start() : cs.getTe2Start();
							drivingCSStart = robot2Closest ? cs.getTe2Start() : cs.getTe1Start();
							waitingCSEnd = robot2Closest ? cs.getTe1End() : cs.getTe2End();
							drivingCSEnd = robot2Closest ? cs.getTe2End() : cs.getTe1End();
							metaCSPLogger.finest("R1 (OUT) / R2 (OUT) --> " + (robot2Closest ? "R2 > R1" : "R1 > R2") + "\n\t" + cs);
						}
					}
					
					//Robot 1 has not reached critical section, robot 2 in critical section --> robot 1 waits
					else if (robotReport1.getPathIndex() < cs.getTe1Start() && robotReport2.getPathIndex() >= cs.getTe2Start()) {
						waitingCurrentIndex = robotReport1.getPathIndex();
						drivingCurrentIndex = robotReport2.getPathIndex();
						waitingTE = cs.getTe1();
						drivingTE = cs.getTe2();
						waitingCSStart = cs.getTe1Start();
						drivingCSStart = cs.getTe2Start();
						waitingCSEnd = cs.getTe1End();
						drivingCSEnd = cs.getTe2End();
						metaCSPLogger.finest("R1 (OUT) / R2 (IN) --> R2 > R1\n\t" + cs);
					}
					//Robot 2 has not reached critical section, robot 1 in critical section --> robot 2 waits
					else if (robotReport1.getPathIndex() >= cs.getTe1Start() && robotReport2.getPathIndex() < cs.getTe2Start()) {
						waitingCurrentIndex = robotReport2.getPathIndex();
						drivingCurrentIndex = robotReport1.getPathIndex();
						waitingTE = cs.getTe2();
						drivingTE = cs.getTe1();
						waitingCSStart = cs.getTe2Start();
						drivingCSStart = cs.getTe1Start();
						waitingCSEnd = cs.getTe2End();
						drivingCSEnd = cs.getTe1End();
						metaCSPLogger.finest("R1 (IN) / R2 (OUT) --> R1 > R2\n\t" + cs);
					}
					//Both robots in critical section --> re-impose previously decided dependency
					else {
						Dependency previousDep = criticalSectionsToDeps.get(cs);
						//A previous dep must exist because we assume robots cannot start both within the same critical section
						if (previousDep == null) {
							metaCSPLogger.severe("Could not coordinate at critical section " + cs);
							throw new Error("Could not coordinate at critical section " + cs);
						}
						if (previousDep.getWaitingRobotID() == robotTracker1.getTrajectoryEnvelope().getRobotID()) {
							waitingCurrentIndex = robotReport1.getPathIndex();
							drivingCurrentIndex = robotReport2.getPathIndex();
							waitingTE = cs.getTe1();
							drivingTE = cs.getTe2();
							waitingCSStart = cs.getTe1Start();
							drivingCSStart = cs.getTe2Start();
							waitingCSEnd = cs.getTe1End();
							drivingCSEnd = cs.getTe2End();
						}
						else {
							waitingCurrentIndex = robotReport2.getPathIndex();
							drivingCurrentIndex = robotReport1.getPathIndex();
							waitingTE = cs.getTe2();
							drivingTE = cs.getTe1();
							waitingCSStart = cs.getTe2Start();
							drivingCSStart = cs.getTe1Start();
							waitingCSEnd = cs.getTe2End();
							drivingCSEnd = cs.getTe1End();
						}
						metaCSPLogger.finest("R1 (IN) / R2 (IN) critical section: " + cs);
					}
					
					waitingRobotID = waitingTE.getRobotID();
					drivingRobotID = drivingTE.getRobotID();
					waitingTracker = trackers.get(waitingRobotID);
					drivingTracker = trackers.get(drivingRobotID);

					//Compute waiting path index point for waiting robot
					int waitingPoint = getCriticalPoint(drivingTE, waitingTE, drivingCurrentIndex, drivingCSStart, drivingCSEnd, waitingCSStart);
					//int waitingPoint = Math.max(0,waitingCSStart-1);
					if (waitingPoint >= 0) {		
						//Make new dependency
						//Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCurrentIndex+1, waitingTracker, drivingTracker);
						Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd, waitingTracker, drivingTracker);
						if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
						currentDeps.get(waitingRobotID).add(dep);
						criticalSectionsToDeps.put(cs, dep);
					}
					else {
						metaCSPLogger.severe("Waiting point < 0 for critical section " + cs);
						throw new Error("Waiting point < 0 for critical section " + cs);
					}
				}
			}
			
			//Remove obsolete critical sections
			for (CriticalSection cs : toRemove) {
				this.criticalSectionsToDeps.remove(cs);
				this.allCriticalSections.remove(cs);
			}
		}

		synchronized(currentDependencies) {
			currentDependencies.clear();
			for (Integer robotID : robotIDs) {
				if (!currentDeps.containsKey(robotID)) {
					setCriticalPoint(robotID, -1);
				}
				else {
					Dependency firstDep = currentDeps.get(robotID).first();
					setCriticalPoint(firstDep.getWaitingTracker().getTrajectoryEnvelope().getRobotID(), firstDep.getWaitingPoint());
					currentDependencies.add(firstDep);
				}
			}
			findCycles();
		}
	}
	
	/**
	 * Defines the criteria with which robots are given priorities. Note that 
	 * order of adding {@link Mission}s pre-empts the ordering given by this method. In other words,
	 * this method provides an ordering for robots whose {@link Mission}s were added
	 * in one batch. 
	 * @return A criteria to prioritize robots driving a batch of {@link Mission}s.
	 */
	public abstract Comparator<AbstractTrajectoryEnvelopeTracker> getOrdering();
	
	/**
	 * Update the set of current critical sections. This should be called every time
	 * a new set of {@link Mission}s has been successfully added.
	 */
	public void computeCriticalSections() {

		synchronized(allCriticalSections) {

			//Collect all driving envelopes
			ArrayList<TrajectoryEnvelope> tes = new ArrayList<TrajectoryEnvelope>();
			for (AbstractTrajectoryEnvelopeTracker atet : trackers.values()) {
				if (!(atet instanceof TrajectoryEnvelopeTrackerDummy)) {
					tes.add(atet.getTrajectoryEnvelope());
				}
			}
			
			//Compute critical sections between driving and new envelopes
			for (int i = 0; i < tes.size(); i++) {
				for (int j = 0; j < envelopesToTrack.size(); j++) {
					for (CriticalSection cs : getCriticalSections(tes.get(i), envelopesToTrack.get(j))) {
						this.allCriticalSections.add(cs);
					}
				}
			}	

			//Compute critical sections between new envelopes
			for (int i = 0; i < envelopesToTrack.size(); i++) {
				for (int j = i+1; j < envelopesToTrack.size(); j++) {
					for (CriticalSection cs : getCriticalSections(envelopesToTrack.get(i), envelopesToTrack.get(j))) {
						this.allCriticalSections.add(cs);
					}
				}
			}
			
			metaCSPLogger.info("There are now " + allCriticalSections.size() + " critical sections");
		}
	}

	protected CriticalSection[] getCriticalSections(TrajectoryEnvelope te1, TrajectoryEnvelope te2) {
		GeometricShapeVariable poly1 = te1.getEnvelopeVariable();
		GeometricShapeVariable poly2 = te2.getEnvelopeVariable();
		Geometry shape1 = ((GeometricShapeDomain)poly1.getDomain()).getGeometry();
		Geometry shape2 = ((GeometricShapeDomain)poly2.getDomain()).getGeometry();
		ArrayList<CriticalSection> css = new ArrayList<CriticalSection>();
		if (shape1.intersects(shape2)) {
			PoseSteering[] path1 = te1.getTrajectory().getPoseSteering();
			PoseSteering[] path2 = te2.getTrajectory().getPoseSteering();
			int te1Start = -1;
			int te2Start = -1;
			int te1End = -1;
			int te2End = -1;
			Geometry gc = shape1.intersection(shape2);
			for (int i = 0; i < gc.getNumGeometries(); i++) {
				Geometry g = gc.getGeometryN(i);
				boolean started = false;
				for (int j = 0; j < path1.length; j++) {
					Geometry placement1 = te1.makeFootprint(path1[j]);
					if (!started && placement1.intersects(g)) {
						started = true;
						te1Start = j;
					}
					else if (started && !placement1.intersects(g)) {
						te1End = j;
						break;
					}
					if (j == path1.length-1) {
						te1End = path1.length-1;						
					}
				}
				started = false;
				for (int j = 0; j < path2.length; j++) {
					Geometry placement2 = te2.makeFootprint(path2[j]);
					if (!started && placement2.intersects(g)) {
						started = true;
						te2Start = j;
					}
					else if (started && !placement2.intersects(g)) {
						te2End = j;
						break;
					}
					if (j == path2.length-1) {
						te2End = path2.length-1;						
					}
				}
				CriticalSection oneCS = new CriticalSection(te1, te2, te1Start, te2Start, te1End, te2End);
				css.add(oneCS);
			}
		}
		return css.toArray(new CriticalSection[css.size()]);
	}

	protected void cleanUp(TrajectoryEnvelope te) {
		synchronized(solver) {
			metaCSPLogger.info("Cleaning up " + te);
			Constraint[] consToRemove = solver.getConstraintNetwork().getIncidentEdgesIncludingDependentVariables(te);
			solver.removeConstraints(consToRemove);
			solver.removeVariable(te);
		}
	}

	/**
	 * Get the {@link TrajectoryEnvelope} currently being tracked for a given robot. 
	 * @param robotID The ID of the robot for which to retrieve the {@link TrajectoryEnvelope} currently
	 * being tracked.
	 * @return The {@link TrajectoryEnvelope} currently being tracked for the given robot.
	 */
	public TrajectoryEnvelope getCurrentSuperEnvelope(int robotID) {
		return trackers.get(robotID).getTrajectoryEnvelope();
	}

	/**
	 * Start the trackers associated to the last batch of {@link Mission}s that has been added.
	 */
	public void startTrackingAddedMissions() {
		
		synchronized(solver) {
			for (final TrajectoryEnvelope te : envelopesToTrack) {
				TrajectoryEnvelopeTrackerDummy startParkingTracker = (TrajectoryEnvelopeTrackerDummy)trackers.get(te.getRobotID());
				final TrajectoryEnvelope startParking = startParkingTracker.getTrajectoryEnvelope();
				//Create end parking envelope
				final TrajectoryEnvelope endParking = solver.createParkingEnvelope(te.getRobotID(), PARKING_DURATION, te.getTrajectory().getPose()[te.getTrajectory().getPose().length-1], "whatever", footprint);
	
				//Driving meets final parking
				AllenIntervalConstraint meets1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
				meets1.setFrom(te);
				meets1.setTo(endParking);
	
				if (!solver.addConstraints(meets1)) {
					metaCSPLogger.severe("ERROR: Could not add constriants " + meets1);
					throw new Error("Could not add constriants " + meets1);		
				}
	
				//Add onStart call back that cleans up parking tracker
				//Note: onStart is triggered only when earliest start of this tracker's envelope is < current time
				TrackingCallback cb = new TrackingCallback() {
	
					@Override
					public void beforeTrackingStart() {
						//Sleep for one control period
						//(allows to impose critical points before tracking actually starts)
						try { Thread.sleep(CONTROL_PERIOD); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					
					@Override
					public void onTrackingStart() { }
					
					@Override
					public void onNewGroundEnvelope() { }
					
					@Override
					public void beforeTrackingFinished() { }
					
					@Override
					public void onTrackingFinished() {
						
						metaCSPLogger.info("Tracking finished for " + te);

						//reset stopping points
						synchronized(stoppingPoints) {
							stoppingPoints.remove(te.getRobotID());
						}
						
						//remove critical sections in which this robot is involved
						synchronized(allCriticalSections) {
							ArrayList<CriticalSection> toRemove = new ArrayList<CriticalSection>();
							for (CriticalSection cs : allCriticalSections) {
								if (cs.getTe1().getRobotID() == te.getRobotID() || cs.getTe2().getRobotID() == te.getRobotID()) toRemove.add(cs);
							}
							for (CriticalSection cs : toRemove) allCriticalSections.remove(cs);
							
						}
						
						//clean up the old parking envelope
						cleanUp(startParking);
						
						//clean up this tracker's TE
						cleanUp(te);
						
						//remove communicatedCP entry
						communicatedCPs.remove(trackers.get(te.getRobotID()));
						
						//make a new parking tracker (park the robot)
						placeRobot(te.getRobotID(), null, endParking, null);
					
					}
	
				};
				
				//Make a new tracker for the driving trajectory envelope
				final AbstractTrajectoryEnvelopeTracker tracker = getNewTracker(te, cb);
	
				synchronized (trackers) {
					trackers.put(te.getRobotID(), tracker);
				}
				
				//Now we can signal the parking that it can end (i.e., its deadline will no longer be prolonged)
				//Note: the parking tracker will anyway wait to exit until earliest end time has been reached
				startParkingTracker.finishParking();			
			}
			envelopesToTrack.clear();
		}
	}
	
	/**
	 * Add one or more missions for one or more robots. If more than one mission is specified for
	 * a robot <code>r</code>, then all the robot's missions are concatenated.
	 * NOTE: For each robot <code>r</code>, all missions should be either defined with a path file or with
	 * an array of {@link PoseSteering}s (pathfile- and path-specified missions cannot be mixed for one robot).
	 *   
	 * @param missions One or more {@link Mission}s, for one or more robots.
	 * @return <code>true</code> iff for all {@link Mission}s the relevant robot is not already
	 * engaged in another mission.
	 */
	public boolean addMissions(Mission ... missions) {

		if (solver == null) {
			metaCSPLogger.severe("Solvers not initialized, please call method setupSolver()");
			throw new Error("Solvers not initialized, please call method setupSolver()");
		}

		HashMap<Integer,ArrayList<Mission>> robotsToMissions = new HashMap<Integer,ArrayList<Mission>>();
		for (Mission m : missions) {
			if (robotsToMissions.get(m.getRobotID()) == null) robotsToMissions.put(m.getRobotID(), new ArrayList<Mission>());
			robotsToMissions.get(m.getRobotID()).add(m);
		}
		
		for (Entry<Integer,ArrayList<Mission>> e : robotsToMissions.entrySet()) {
			int robotID = e.getKey();
			if (!isFree(robotID)) return false;
		}
		
		for (Entry<Integer,ArrayList<Mission>> e : robotsToMissions.entrySet()) {
			int robotID = e.getKey();
			synchronized (solver) {
				//Get start parking tracker and envelope
				TrajectoryEnvelopeTrackerDummy startParkingTracker = (TrajectoryEnvelopeTrackerDummy)trackers.get(robotID);
				TrajectoryEnvelope startParking = startParkingTracker.getTrajectoryEnvelope();
				ArrayList<Constraint> consToAdd = new ArrayList<Constraint>();
//				String finalDestLocation = "";
				ArrayList<String> pathFiles = new ArrayList<String>();
				ArrayList<PoseSteering> overallPath = new ArrayList<PoseSteering>();
				if (e.getValue().get(0).getPathFile() == null) pathFiles = null;
				else overallPath = null;
				for (Mission m : e.getValue()) {						
					if (pathFiles != null) pathFiles.add(m.getPathFile());
					else {
						for (PoseSteering ps : m.getPath()) {
							overallPath.add(ps);
						}
					}
//					finalDestLocation = m.getToLocation();
				}

				//Create a big overall driving envelope
				TrajectoryEnvelope te = null;
				if (pathFiles != null) te = solver.createEnvelopeNoParking(robotID, pathFiles.toArray(new String[pathFiles.size()]), "Driving", footprint);
				else te = solver.createEnvelopeNoParking(robotID, overallPath.toArray(new PoseSteering[overallPath.size()]), "Driving", footprint);
				
				//Add destinations as stopping points
				synchronized(stoppingPoints) {
					for (int i = 0; i < e.getValue().size()-1; i++) {
						Mission m = e.getValue().get(i);
						Pose destPose = m.getToPose();
						int stoppingPoint = te.getSequenceNumber(new Coordinate(destPose.getX(), destPose.getY()));
						if (!stoppingPoints.keySet().contains(robotID)) stoppingPoints.put(robotID, new ArrayList<Integer>());
						if (!stoppingPoints.get(robotID).contains(stoppingPoint)) stoppingPoints.get(robotID).add(stoppingPoint);
					}
					metaCSPLogger.info("Stopping points along trajectory for Robot" + robotID + ": " + stoppingPoints);
				}
				
				//Put in more realistic DTs computed with the RK4 integrator
//				System.out.println(">> Computing DTs...");
//				double dts[] = TrajectoryEnvelopeTrackerRK4.computeDTs(te.getTrajectory(), this.getMaxVelocity(), this.getMaxAcceleration());
//				te.getTrajectory().setDTs(dts);
//				te.updateDuration();
//				System.out.println("<< done computing DTs");

				//Start parking meets driving, so that driving does not start before parking is finished
				AllenIntervalConstraint meets = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
				meets.setFrom(startParking);
				meets.setTo(te);
				consToAdd.add(meets);
			
				if (!solver.addConstraints(consToAdd.toArray(new Constraint[consToAdd.size()]))) {
					metaCSPLogger.severe("ERROR: Could not add constriants " + consToAdd);
					throw new Error("Could not add constriants " + consToAdd);						
				}
				
				envelopesToTrack.add(te);
			}
		}
		
		return true;
	}

	/**
	 * Sets up a GUI which shows the current status of robots, overlayed on a given map .
	 */
	public void setupGUI() {
		this.setupGUI(null);
	}
	
	/**
	 * Sets up a GUI which shows the current status of robots, overlayed on a given map .
	 * @param mapYAMLFile A map file to overlay onto the GUI.
	 */
	public void setupGUI(String mapYAMLFile) {
		//Show everything in a GUI (vehicle positions are updated in real time by the trackers, see below)
		panel = JTSDrawingPanel.makeEmpty("Current status of robots");		
		if (mapYAMLFile != null) panel.setMap(mapYAMLFile);
	}
	
	/**
	 * Scale and translate the view of the GUI to include all geometries. 
	 */
	public void centerView() {
//		this.panel.centerView();
		this.panel.reinitVisualization();
	}
	
	//Update viz (keep geoms alive)
	protected void updateVisualization() {
		for (TrajectoryEnvelope te : solver.getRootTrajectoryEnvelopes()) {
			GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
			panel.addGeometry("_"+te.getID(), dom.getGeometry(), true, false);
//			for (Variable gteV : te.getRecursivelyDependentVariables()) {
//				TrajectoryEnvelope gte = (TrajectoryEnvelope)gteV;
//				if (!gte.hasSubEnvelopes()) {
//					GeometricShapeDomain gdom = (GeometricShapeDomain)gte.getEnvelopeVariable().getDomain();
//					panel.addGeometry(""+gte.getID(), gdom.getGeometry(), false, false);						
//				}
//			}//CME
			panel.removeOldGeometries(CONTROL_PERIOD*2);
			panel.updatePanel();
		}
	}

	protected String[] getStatistics() {
		synchronized (trackers) {
			String CONNECTOR_BRANCH = (char)0x251C + "" + (char)0x2500 + " ";
			String CONNECTOR_LEAF = (char)0x2514 + "" + (char)0x2500 + " ";
			ArrayList<String> ret = new ArrayList<String>();
			int numVar = solver.getConstraintNetwork().getVariables().length;
			int numCon = solver.getConstraintNetwork().getConstraints().length;
			ret.add("Status @ "  + getCurrentTimeInMillis() + " ms");
			
			ret.add(CONNECTOR_BRANCH + "Network ........ " + numVar + " variables, " + numCon + " constriants");
			HashSet<Integer> allRobots = new HashSet<Integer>();
			for (Integer robotID : trackers.keySet()) {
				allRobots.add(robotID);
			}
			String st = CONNECTOR_BRANCH + "Robots ......... ";
			for (Integer robotID : allRobots) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				RobotReport rr = tracker.getRobotReport(); 
				int currentPP = rr.getPathIndex();
				st += tracker.te.getComponent();
				if (tracker instanceof TrajectoryEnvelopeTrackerDummy) st += " (P)";
				else st += " (D)";
				st += ": " + currentPP + "   ";
			}
			ret.add(st);
			ret.add(CONNECTOR_LEAF + "Dependencies ... " + currentDependencies);
			return ret.toArray(new String[ret.size()]);
		}
	}

	protected void overlayStatistics() {
		String[] stats = getStatistics();
        System.out.printf(((char) 0x1b) + "[H");
        System.out.printf(((char) 0x1b) + "[1m");
    	for (int l = 0; l < stats.length; l++) {
            System.out.printf(((char) 0x1b) + "[1B" + ((char) 0x1b) + "[2K\r" + stats[l]);
    	}
        System.out.printf(((char) 0x1b) + "[0m");
        System.out.printf(((char) 0x1b) + "[200B\r");
	}
	
	//Print some statistics
	protected void printStatistics() {
		for (String s : getStatistics()) {
			metaCSPLogger.info(s);
		}		
	}

	protected void setupInferenceCallback() {
		
		Thread inference = new Thread("Coordinator inference") {
			public void run() {
				while (true) {
					synchronized(solver) {						
						updateVisualization();
						printStatistics();
						if (overlay) overlayStatistics();
						updateDependencies();
					}
					
					//Sleep a little...
					try { Thread.sleep(CONTROL_PERIOD); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
			}
		};
		inference.start();
	}
	
	/**
	 * Factory method that returns a trajectory envelope tracker, which is the class implementing the
	 * interface with real or simulated robots.
	 * @param te The reference {@link TrajectoryEnvelope} that should be driven. 
	 * @param cb A callback that is called every tracking period.
	 * @return An instance of a trajectory envelope tracker.
	 */
	public abstract AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb);
	
	protected boolean isFree(int robotID) {
		AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
		if (!(tracker instanceof TrajectoryEnvelopeTrackerDummy)) return false;
		TrajectoryEnvelopeTrackerDummy trackerDummy = (TrajectoryEnvelopeTrackerDummy)tracker;
		return (!trackerDummy.isParkingFinished());
	}
		
}
