package se.oru.coordination.coordination_oru;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.nio.file.Files;
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
import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.graph.DirectedMultigraph;
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
public abstract class TrajectoryEnvelopeCoordinator {
	
	public static String TITLE = "coordination_oru - Online coordination for multiple robots";
	public static String COPYRIGHT = "Copyright \u00a9 2017-2018 Federico Pecora";

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

	public static final int PARKING_DURATION = 3000;
	protected static final int DEFAULT_STOPPING_TIME = 5000;
	protected int CONTROL_PERIOD;
	protected double TEMPORAL_RESOLUTION;
	public static int EFFECTIVE_CONTROL_PERIOD = 0;

	protected boolean overlay = false;
	protected boolean quiet = false;
	
	protected boolean checkCollisions = false;
	protected ArrayList<CollisionEvent> collisionsList = new ArrayList<CollisionEvent>();
	protected Thread collisionThread = null;
	
	protected AtomicInteger totalMsgsSent = new AtomicInteger(0);
	protected AtomicInteger totalMsgsRetransmitted = new AtomicInteger(0);
																			
	protected TrajectoryEnvelopeSolver solver = null;

	//protected JTSDrawingPanel panel = null;
	protected FleetVisualization viz = null;
	protected ArrayList<TrajectoryEnvelope> envelopesToTrack = new ArrayList<TrajectoryEnvelope>();
	protected ArrayList<TrajectoryEnvelope> currentParkingEnvelopes = new ArrayList<TrajectoryEnvelope>();
	protected ArrayList<CriticalSection> allCriticalSections = new ArrayList<CriticalSection>();
	protected HashMap<CriticalSection,HashSet<Dependency>> criticalSectionsToDeps = new HashMap<CriticalSection, HashSet<Dependency>>(); 
	//FIXME CHECK (Federico)
	//  0 dependencies -> first assignment. There should NEVER happen: Both cannot stop.
	//  1 dependency -> standard case.
	// >1 dependencies -> There should NEVER happen.
	protected HashMap<Integer,ArrayList<Integer>> stoppingPoints = new HashMap<Integer,ArrayList<Integer>>();
	protected HashMap<Integer,ArrayList<Integer>> stoppingTimes = new HashMap<Integer,ArrayList<Integer>>();
	protected HashMap<Integer,Thread> stoppingPointTimers = new HashMap<Integer,Thread>();

	protected HashMap<Integer,AbstractTrajectoryEnvelopeTracker> trackers = new HashMap<Integer, AbstractTrajectoryEnvelopeTracker>();
	protected HashSet<Dependency> currentDependencies = new HashSet<Dependency>();

	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(TrajectoryEnvelopeCoordinator.class);
	protected String logDirName = null;

	protected HashMap<AbstractTrajectoryEnvelopeTracker,Pair<Integer,Long>> communicatedCPs = new HashMap<AbstractTrajectoryEnvelopeTracker, Pair<Integer,Long>>();
	protected HashMap<AbstractTrajectoryEnvelopeTracker,Integer> externalCPCounters = new HashMap<AbstractTrajectoryEnvelopeTracker, Integer>();

	protected Map mapMetaConstraint = null;

	protected ComparatorChain comparators = new ComparatorChain();
	protected HashMap<Integer,ForwardModel> forwardModels = new HashMap<Integer, ForwardModel>();

	protected HashMap<Integer,Coordinate[]> footprints = new HashMap<Integer, Coordinate[]>();
	protected HashMap<Integer,Double> maxFootprintDimensions = new HashMap<Integer, Double>();

	protected HashSet<Dependency> disallowedDependencies = new HashSet<Dependency>();

	protected HashSet<Integer> muted = new HashSet<Integer>();

	protected boolean yieldIfParking = true;
	protected boolean checkEscapePoses = true;
	protected boolean breakDeadlocksByReordering = true;
	protected boolean breakDeadlocksByReplanning = true;

	protected HashMap<Integer,TrackingCallback> trackingCallbacks = new HashMap<Integer, TrackingCallback>();	
	protected Callback inferenceCallback = null;
	
	protected HashSet<HashSet<Integer>> replanningSpawned = new HashSet<HashSet<Integer>>();
	
	//Network knowledge
	protected double packetLossProbability = NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS;
	public static int MAX_TX_DELAY = NetworkConfiguration.getMaximumTxDelay();
	protected double maxFaultsProbability = NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS;
	protected int numberOfReplicas = 1;
	
	/**
	 * FIXME parameter to be defined in the constructor?
	 */
	public void setCheckCollisions(boolean checkCollision) {
		this.checkCollisions = checkCollision;
	}
		
	/**
	 * Returning the number of messages required by each send to be effective
	 * (i.e. the probability of unsuccessful delivery will be lower than the threshold maxFaultsProbability)
	 */
	public int getNumberOfReplicas() {
		return numberOfReplicas;
	}
	
	
	/**
	 * Utility method to treat internal resources from this library as filenames.
	 * @param resource The internal resource to be loaded.
	 * @return The absolute path of a temporary file which contains a copy of the resource.
	 */
	public static String getResourceAsFileName(String resource) {
		Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		ClassLoader classLoader = TrajectoryEnvelopeCoordinator.class.getClassLoader();
		File source = new File(classLoader.getResource(resource).getFile());
		File dest = new File("." + 1+rand.nextInt(1000) + ".tempfile");
		try { Files.copy(source.toPath(), dest.toPath()); }
		catch (IOException e) { e.printStackTrace(); }
		return dest.getAbsolutePath();
	}
	
	/**
	 * The default footprint used for robots if none is specified.
	 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
	 */
	public static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			new Coordinate(-1.7, 0.7),	//back left
			new Coordinate(-1.7, -0.7),	//back right
			new Coordinate(2.7, -0.7),	//front right
			new Coordinate(2.7, 0.7)	//front left
	};

	/**
	 * Dimension of the default footprint.
	 */
	public static double MAX_DEFAULT_FOOTPRINT_DIMENSION = 4.4;

	/**
	 * Set whether this {@link TrajectoryEnvelopeCoordinator} should print info at every period.
	 * @param value Set to <code>true</code> if this {@link TrajectoryEnvelopeCoordinator} should print info at every period.
	 */
	public void setQuiet(boolean value) {
		this.quiet = value;
	}
	
	/**
	 * Set the network parameters (packet loss probability, max delay and max faults probability).
	 * Then, compute the number of messages numberOfReplicas required for each send to be effective
	 * (the probability of receiving a message after numberOfReplicas trials is assumed to have a geometric distribution).
	 * @param packetLossProbability The probability for a message to be lost.
	 * @param MAX_TX_DELAY The maximum transmission delay.
	 * @param maxFaultsProbability The maximum admitted probability for an information to be lost.
	 */
	public void setNetworkParameters(double packetLossProbability, int MAX_TX_DELAY, double maxFaultsProbability) {
		this.packetLossProbability = packetLossProbability;
		this.MAX_TX_DELAY = MAX_TX_DELAY;
		this.maxFaultsProbability = maxFaultsProbability;	
		this.numberOfReplicas =  (packetLossProbability > 0 && maxFaultsProbability > 0) ? (int)Math.ceil(Math.log(maxFaultsProbability)/Math.log(packetLossProbability)) : 1;
		metaCSPLogger.info("Number of replicas for each send: " + numberOfReplicas);
	}
	
	/**
	 * Set a {@link Callback} that will be called at every cycle.
	 * @param cb A {@link Callback} that will be called at every cycle.
	 */
	public void setInferenceCallback(Callback cb) {
		this.inferenceCallback = cb;
	}
	
	/**
	 * Get the control period of this {@link TrajectoryEnvelopeCoordinator}.
	 * @return the control period (in milliseconds) of this {@link TrajectoryEnvelopeCoordinator}.
	 */
	public int getControlPeriod() {
		return this.CONTROL_PERIOD;
	}

	/**
	 * Set whether the coordinator should try to break deadlocks by attempting to re-plan
	 * the path of one of the robots involved in an unsafe cycle.
	 * @param value <code>true</code> if deadlocks should be broken by re-planning.
	 */
	public void setBreakDeadlocksByReplanning(boolean value) {
		this.breakDeadlocksByReplanning = value;
	}
	
	/**
	 * Set whether the coordinator should try to break deadlocks by disallowing an arbitrary
	 * ordering involved in a loop in the dependency multigraph.
	 * @param value <code>true</code> if deadlocks should be broken.
	 */
	public void setBreakDeadlocksByReordering(boolean value) {
		this.breakDeadlocksByReordering = value;
	}

	/**
	 * Set whether the coordinator should try to break deadlocks by both arbitrary
	 * re-ordering and re-planning.
	 * @param value <code>true</code> if deadlocks should be broken.
	 */
	@Deprecated
	public void setBreakDeadlocks(boolean value) {
		this.setBreakDeadlocksByReordering(value);
		this.setBreakDeadlocksByReplanning(value);
	}

	/**
	 * Set whether robots that will park in a critical section should yield to other robots.
	 * @param value <code>true</code> if robots that will park in a critical section should yield to other robots.
	 */
	public void setYieldIfParking(boolean value) {
		this.yieldIfParking = value;
	}

	/**
	 * Set whether completely overlapping paths should lead to a warning.
	 * @param value <code>true</code> if completely overlapping paths should lead to a warning.
	 */
	public void setCheckEscapePoses(boolean value) {
		this.checkEscapePoses = value;
	}

	/**
	 * Toggle mute/unmute communication with a given robot. 
	 * @param robotID The robot to toggle mute/unmute communication with.
	 */
	public void toggleMute(int robotID) {
		if (muted.contains(robotID)) muted.remove(robotID);
		else muted.add(robotID);
	}

	/**
	 * Mute communication with a given robot. 
	 * @param robotID The robot to mute communication with.
	 */
	public void mute(int robotID) {
		muted.add(robotID);
	}

	/**
	 * Get the IDs of robots that are muted.
	 * @return The IDs of robots that are muted.
	 */
	public int[] getMuted() {
		int[] ret = new int[muted.size()];
		int counter = 0;
		for (Integer m : muted) ret[counter++] = m; 
		return ret;
	}

	/**
	 * Unmute communication with a given robot. 
	 * @param robotID The robot to unmute communication with.
	 */
	public void unMute(int robotID) {
		muted.remove(robotID);
	}

	private double getMaxFootprintDimension(int robotID) {
		if (this.footprints.containsKey(robotID)) return maxFootprintDimensions.get(robotID);
		return MAX_DEFAULT_FOOTPRINT_DIMENSION;
	}

	/**
	 * Get the {@link Coordinate}s defining the default footprint of robots.
	 * @return The {@link Coordinate}s defining the default footprint of robots.
	 */
	public Coordinate[] getDefaultFootprint() {
		return DEFAULT_FOOTPRINT;
	}

	/**
	 * Get the {@link Coordinate}s defining the footprint of a given robot.
	 * @param robotID the ID of the robot
	 * @return The {@link Coordinate}s defining the footprint of a given robot.
	 */
	public Coordinate[] getFootprint(int robotID) {
		if (this.footprints.containsKey(robotID)) return this.footprints.get(robotID);
		return DEFAULT_FOOTPRINT;
	}

	/**
	 * Get a {@link Geometry} representing the default footprint of robots.
	 * @return A {@link Geometry} representing the default footprint of robots.
	 */
	public Geometry getDefaultFootprintPolygon() {
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(DEFAULT_FOOTPRINT);
		return fpGeom;
	}

	/**
	 * Get a {@link Geometry} representing the footprint of a given robot.
	 * @param robotID the ID of the robot
	 * @return A {@link Geometry} representing the footprint of a given robot.
	 */
	public Geometry getFootprintPolygon(int robotID) {
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(getFootprint(robotID));
		return fpGeom;
	}

	/**
	 * Set the {@link ForwardModel} of a given robot.
	 * @param robotID The ID of the robot.
	 * @param fm The robot's {@link ForwardModel}.
	 */
	public void setForwardModel(int robotID, ForwardModel fm) {
		this.forwardModels.put(robotID, fm);
	}

	/**
	 * Get the {@link ForwardModel} of a given robot.
	 * @param robotID The ID of the robot.
	 * @return The {@link ForwardModel} of the robot.
	 */
	public ForwardModel getForwardModel(int robotID) {
		if (forwardModels.containsKey(robotID)) return forwardModels.get(robotID);
		System.out.println("Returning default FM for " + robotID);
		return new ForwardModel() {
			@Override
			public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex, boolean useVelocity) {
				return true;
			}
			@Override
			public int getEarliestStoppingPathIndex(TrajectoryEnvelope te, RobotReport currentState) {
				// TODO Auto-generated method stub
				return 0;
			}
		};
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

	protected void setupLogging() {
		//logDirName = "log-" + Calendar.getInstance().getTimeInMillis();
		logDirName = "logs";
		File dir = new File(logDirName);
		dir.mkdir();
		MetaCSPLogging.setLogDir(logDirName);
	}

	/**
	 * Get the {@link TrajectoryEnvelopeSolver} underlying this coordinator. This solver maintains the {@link TrajectoryEnvelope}s and temporal constraints
	 * among them.
	 * @return The {@link TrajectoryEnvelopeSolver} underlying this coordinator.
	 */
	public TrajectoryEnvelopeSolver getSolver() {
		return this.solver;
	}

	/**
	 * Instruct a given robot's tracker that it may not navigate beyond a given 
	 * path index.  
	 * @param robotID The ID of the robot.
	 * @param criticalPoint The index of the path pose beyond which the robot should not navigate.
	 */
	public void setCriticalPoint(final int robotID, final int criticalPoint) {
			
		final AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
	
		//If the robot is not muted
		if (tracker != null && !muted.contains(robotID)) {
			long timeNow = Calendar.getInstance().getTimeInMillis();
			
			//UDP transmission -> we can assume the message to be lost FIXME
			boolean retransmitt = false;//communicatedCPs.containsKey(tracker) && communicatedCPs.get(tracker).getFirst().equals(criticalPoint) && ((int)(timeNow-communicatedCPs.get(tracker).getSecond()) > 2*(MAX_TX_DELAY+CONTROL_PERIOD+tracker.getTrackingPeriodInMillis()));
			
			if (!communicatedCPs.containsKey(tracker) || !communicatedCPs.get(tracker).getFirst().equals(criticalPoint) || retransmitt ) {
				communicatedCPs.put(tracker, new Pair<Integer,Long>(criticalPoint, timeNow));
				externalCPCounters.replace(tracker,externalCPCounters.get(tracker)+1);
				tracker.setCriticalPoint(criticalPoint, externalCPCounters.get(tracker)%Integer.MAX_VALUE);
				
				//for statistics
				totalMsgsSent.set(totalMsgsSent.get()+1); 
				if (retransmitt) totalMsgsRetransmitted.set(totalMsgsRetransmitted.get()+1);
			}
		}
	}

	/**
	 * Get the current state of a given robot.
	 * @param robotID The ID of the robot of which the state should be returned.
	 * @return The current state of a given robot.
	 */
	public RobotReport getRobotReport(int robotID) {
		
		//Read the last message received
		synchronized (trackers) {
			if (!trackers.containsKey(robotID)) return null;
			return trackers.get(robotID).getLastRobotReport();
		}
		
	}

	/**
	 * Set the default footprint of robots, which is used for computing spatial envelopes.
	 * Provide the bounding polygon of the machine assuming its reference point is in (0,0), and its
	 * orientation is aligned with the x-axis. The coordinates must be in CW or CCW order.
	 * @param coordinates The coordinates delimiting bounding polygon of the footprint.
	 */
	public void setDefaultFootprint(Coordinate ... coordinates) {
		DEFAULT_FOOTPRINT = coordinates;
		MAX_DEFAULT_FOOTPRINT_DIMENSION = computeMaxFootprintDimension(coordinates);
	}


	/**
	 * Set the default footprint of robots, which is used for computing spatial envelopes.
	 * Provide the bounding polygon of the machine assuming its reference point is in (0,0), and its
	 * orientation is aligned with the x-axis. The coordinates must be in CW or CCW order.
	 * @param coordinates The coordinates delimiting bounding polygon of the footprint.
	 */
	@Deprecated
	public void setFootprint(Coordinate ... coordinates) {
		DEFAULT_FOOTPRINT = coordinates;
		MAX_DEFAULT_FOOTPRINT_DIMENSION = computeMaxFootprintDimension(coordinates);
	}

	/**
	 * Set the footprint of a given robot, which is used for computing spatial envelopes.
	 * Provide the bounding polygon of the machine assuming its reference point is in (0,0), and its
	 * orientation is aligned with the x-axis. The coordinates must be in CW or CCW order.
	 * @param robotID The ID of the robot.
	 * @param coordinates The coordinates delimiting bounding polygon of the footprint.
	 */
	public void setFootprint(int robotID, Coordinate ... coordinates) {
		this.footprints.put(robotID, coordinates);
		maxFootprintDimensions.put(robotID,computeMaxFootprintDimension(coordinates));
	}

	private double computeMaxFootprintDimension(Coordinate[] coords) {
		ArrayList<Double> fpX = new ArrayList<Double>();
		ArrayList<Double> fpY = new ArrayList<Double>();
		for (Coordinate coord : coords) {
			fpX.add(coord.x);
			fpY.add(coord.y);
		}
		Collections.sort(fpX);
		Collections.sort(fpY);
		return Math.max(fpX.get(fpX.size()-1)-fpX.get(0), fpY.get(fpY.size()-1)-fpY.get(0));
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

	/**
	 * Place a robot with a given ID in a given {@link Pose}.
	 * @param robotID The ID of the robot.
	 * @param currentPose The {@link Pose} in which to place the robot.
	 */
	public void placeRobot(final int robotID, Pose currentPose) {
		this.placeRobot(robotID, currentPose, null, currentPose.toString());
	}

	/**
	 * Place a robot with a given ID in the first {@link Pose} of a given {@link TrajectoryEnvelope}.
	 * @param robotID The ID of the robot.
	 * @param parking The {@link TrajectoryEnvelope} in which the robot is parked.
	 */
	public void placeRobot(final int robotID, TrajectoryEnvelope parking) {
		this.placeRobot(robotID, null, parking, null);
	}

	/**
	 * Place a robot with a given ID in a given {@link Pose} of a given {@link TrajectoryEnvelope},
	 * labeled with a given string.
	 * @param robotID The ID of the robot.
	 * @param currentPose The {@link Pose} of the robot.
	 * @param parking The {@link TrajectoryEnvelope} in which the robot is parked.
	 * @param location A label representing the {@link TrajectoryEnvelope}.
	 */
	public void placeRobot(final int robotID, Pose currentPose, TrajectoryEnvelope parking, String location) {

		if (solver == null) {
			metaCSPLogger.severe("Solver not initialized, please call method setupSolver() first!");
			throw new Error("Solver not initialized, please call method setupSolver() first!");
		}

		synchronized (solver) {	
			//Create a new parking envelope
			long time = getCurrentTimeInMillis();

			//Can provide null parking or null currentPose, but not both
			if (parking == null) parking = solver.createParkingEnvelope(robotID, PARKING_DURATION, currentPose, location, getFootprint(robotID));
			else currentPose = parking.getTrajectory().getPose()[0];

			AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(time, time));
			release.setFrom(parking);
			release.setTo(parking);
			if (!solver.addConstraint(release)) {
				metaCSPLogger.severe("Could not release " + parking + " with constriant " + release);
				throw new Error("Could not release " + parking + " with constriant " + release);
			}
			metaCSPLogger.info("Placed " + parking.getComponent() + " in pose " + currentPose + ": " + parking);

			TrackingCallback cb = new TrackingCallback(parking) {
				@Override
				public void beforeTrackingStart() {
					if (trackingCallbacks.containsKey(robotID)) {
						trackingCallbacks.get(robotID).myTE = this.myTE; 
						trackingCallbacks.get(robotID).beforeTrackingStart();
					}
				}
				@Override
				public void onTrackingStart() {
					if (trackingCallbacks.containsKey(robotID)) trackingCallbacks.get(robotID).onTrackingStart();
				}
				@Override
				public void onNewGroundEnvelope() {
					if (trackingCallbacks.containsKey(robotID)) trackingCallbacks.get(robotID).onNewGroundEnvelope();
				}
				@Override
				public void beforeTrackingFinished() {
					if (trackingCallbacks.containsKey(robotID)) trackingCallbacks.get(robotID).beforeTrackingFinished();
				}
				@Override
				public void onTrackingFinished() {
					if (trackingCallbacks.containsKey(robotID)) trackingCallbacks.get(robotID).onTrackingFinished();
				}
				@Override
				public String[] onPositionUpdate() {
					if (trackingCallbacks.containsKey(robotID)) return trackingCallbacks.get(robotID).onPositionUpdate();
					return null;
				}
			};
			
			//Now start the tracker for this parking (will be ended by call to addMissions for this robot)
			final TrajectoryEnvelopeTrackerDummy tracker = new TrajectoryEnvelopeTrackerDummy(parking, 300, TEMPORAL_RESOLUTION, this, cb) {
				@Override
				public long getCurrentTimeInMillis() {
					return tec.getCurrentTimeInMillis();
				}
			};

			currentParkingEnvelopes.add(tracker.getTrajectoryEnvelope());				

			synchronized (trackers) {
				externalCPCounters.remove(trackers.get(robotID));
				trackers.remove(robotID);
				
				trackers.put(robotID, tracker);
				externalCPCounters.put(tracker, -1);
			}			
		}
			
	}

	/**
	 * Get the {@link FleetVisualization} that is used for displaying the current fleet.
	 * @return The {@link FleetVisualization} that is used for displaying the current fleet.
	 */
	public FleetVisualization getVisualization() {
		return this.viz;
	}

	/**
	 * Get the list of current dependencies between robots.
	 * @return A list of {@link Dependency} objects.
	 */
	public HashSet<Dependency> getCurrentDependencies() {
		return this.currentDependencies;
	}

	/*
	 * Get the path index beyond which a robot should not navigate, given the {@link TrajectoryEnvelope} of another robot.  
	 * @param te1 The {@link TrajectoryEnvelope} of the leading robot.
	 * @param te2 The {@link TrajectoryEnvelope} of the yielding robot.
	 * @param currentPIR1 The current path index of the leading robot.
	 * @param te1Start The path index
	 * @param te1End
	 * @param te2Start
	 * @return The path index beyond which a robot should not navigate, given the {@link TrajectoryEnvelope} of another robot.
	 */
	private int getCriticalPoint(int yieldingRobotID, CriticalSection cs, int leadingRobotCurrentPathIndex) {

		//Number of additional path points robot 2 should stay behind robot 1
		int TRAILING_PATH_POINTS = 3;

		int leadingRobotStart = -1;
		int yieldingRobotStart = -1;
		int leadingRobotEnd = -1;
		int yieldingRobotEnd = -1;
		TrajectoryEnvelope leadingRobotTE = null;
		TrajectoryEnvelope yieldingRobotTE = null;
		if (cs.getTe1().getRobotID() == yieldingRobotID) {
			leadingRobotStart = cs.getTe2Start();
			yieldingRobotStart = cs.getTe1Start();
			leadingRobotEnd = cs.getTe2End();
			yieldingRobotEnd = cs.getTe1End();
			leadingRobotTE = cs.getTe2();
			yieldingRobotTE = cs.getTe1();
		}
		else {
			leadingRobotStart = cs.getTe1Start();
			yieldingRobotStart = cs.getTe2Start();
			leadingRobotEnd = cs.getTe1End();
			yieldingRobotEnd = cs.getTe2End();
			leadingRobotTE = cs.getTe1();
			yieldingRobotTE = cs.getTe2();			
		}

		if (leadingRobotCurrentPathIndex <= leadingRobotStart) {
			return Math.max(0, yieldingRobotStart-TRAILING_PATH_POINTS);
		}

		//Compute sweep of robot 1's footprint from current position to LOOKAHEAD
		Pose leadingRobotPose = leadingRobotTE.getTrajectory().getPose()[leadingRobotCurrentPathIndex];
		Geometry leadingRobotInPose = TrajectoryEnvelope.getFootprint(leadingRobotTE.getFootprint(), leadingRobotPose.getX(), leadingRobotPose.getY(), leadingRobotPose.getTheta());
		if (leadingRobotCurrentPathIndex < leadingRobotEnd) {
			for (int i = leadingRobotCurrentPathIndex+1; i <= leadingRobotEnd; i++) {
				Pose leadingRobotNextPose = leadingRobotTE.getTrajectory().getPose()[i];
				try {
					leadingRobotInPose = leadingRobotInPose.union(TrajectoryEnvelope.getFootprint(leadingRobotTE.getFootprint(), leadingRobotNextPose.getX(), leadingRobotNextPose.getY(), leadingRobotNextPose.getTheta()));			
				} catch (Exception e) { e.printStackTrace(); }
			}
		}

		//Return pose at which yielding robot should stop given driving robot's projected sweep
		for (int i = yieldingRobotStart; i < yieldingRobotEnd; i++) {
			Pose yieldingRobotPose = yieldingRobotTE.getTrajectory().getPose()[i];
			Geometry yieldingRobotInPose = TrajectoryEnvelope.getFootprint(yieldingRobotTE.getFootprint(), yieldingRobotPose.getX(), yieldingRobotPose.getY(), yieldingRobotPose.getTheta());
			if (leadingRobotInPose.intersects(yieldingRobotInPose)) {
				return Math.max(0, i-TRAILING_PATH_POINTS);
			}
		}

		//The only situation where the above has not returned is when robot 2 should
		//stay "parked", therefore wait at index 0
		return Math.max(0, yieldingRobotStart-TRAILING_PATH_POINTS);

	}
	/**
	 * Returns <code>true</code> iff the given robot is at a stopping point.
	 * @param robotID The ID of a robot.
	 * @return <code>true</code> iff the given robot is at a stopping point.
	 */
	public boolean atStoppingPoint(int robotID) {
		return stoppingPointTimers.containsKey(robotID);
	}
	
	//Spawn a waiting thread at this stopping point
	protected void spawnWaitingThread(final int robotID, final int index, final int duration) {
		Thread stoppingPointTimer = new Thread() {
			private long startTime = Calendar.getInstance().getTimeInMillis();
			@Override
			public void run() {
				metaCSPLogger.info("Waiting thread starts for " + robotID);
				while (Calendar.getInstance().getTimeInMillis()-startTime < duration) {
					try { Thread.sleep(100); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
				metaCSPLogger.info("Waiting thread finishes for " + robotID);
				synchronized(stoppingPoints) {
					stoppingPoints.get(robotID).remove((int)index);
					stoppingTimes.get(robotID).remove((int)index);
					stoppingPointTimers.remove(robotID);
				}
			}
		};
		stoppingPointTimers.put(robotID,stoppingPointTimer);
		stoppingPointTimer.start();
	}

	private void findCycles() {
		
		//Dep multi-graph G = (V,E)
		//  V = robots incolved in deps
		//  E = {(u,v,dep) | exists dep stating that robot u should wait for robot v}
		//Deadlock iff:
		//  Cycle <r_1, r_2, ... r_1> in G
		//  Exists one selection of edges along the cycle (this is a multi-graph) such that
		//    Exists (u,v,dep1) and (v,w,dep2) such that
		//      v.dep2.waitingpoint <= v.dep1.releasingpoint
		
		DirectedMultigraph<Integer,Dependency> g = new DirectedMultigraph<Integer, Dependency>(Dependency.class);
		for (Dependency dep : currentDependencies) {
			if (!g.containsVertex(dep.getWaitingRobotID())) g.addVertex(dep.getWaitingRobotID());
			if (!g.containsVertex(dep.getDrivingRobotID())) g.addVertex(dep.getDrivingRobotID());
			g.addEdge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep);
		}
		JohnsonSimpleCycles<Integer, Dependency> cycleFinder = new JohnsonSimpleCycles<Integer, Dependency>(g);
		List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
		
		//For each cycle...
		for (List<Integer> cycle : cycles) {
			
			//Get edges along the cycle...
			//  Recall: there could be more than one edge per pair of verts, as this is a multi-graph
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

			//Check for unsafe cycles, that is:
			//  All along the cycle, check if there are (u,v,dep1) and (v,w,dep2) such that
			//    v.dep2.waitingpoint <= v.dep1.releasingpoint
			//  And if so mark cycle as deadlock
			Dependency anUnsafeDep = null;
			for (int i = 0; i < edgesAlongCycle.size()-1; i++) {
				boolean safe = true;
				for (Dependency dep1 : edgesAlongCycle.get(i)) {
					for (Dependency dep2 : edgesAlongCycle.get(i+1)) {
						if (unsafePair(dep1, dep2)) {
							safe = false;
							anUnsafeDep = dep1;
						}
					}
				}
				if (safe) {
					metaCSPLogger.finest("Cycle: " + edgesAlongCycle + " is deadlock-free");
					break;
				}
				if (i == edgesAlongCycle.size()-2) {
					metaCSPLogger.info("Cycle: " + edgesAlongCycle + " is NOT deadlock-free");
					if (breakDeadlocksByReplanning) spawnReplanning(edgesAlongCycle);
					if (breakDeadlocksByReordering) {
						synchronized (disallowedDependencies) {
							disallowedDependencies.add(anUnsafeDep);
						}
					}
				}
			}
		}
	}
	
	
	private boolean inParkingPose(int robotID) {
		synchronized(trackers) {
			return trackers.containsKey(robotID) && trackers.get(robotID) instanceof TrajectoryEnvelopeTrackerDummy || !communicatedCPs.containsKey(trackers.get(robotID));
		}
	}
	
	private void spawnReplanning(ArrayList<ArrayList<Dependency>> deadlockedDeps) {
		
		int maxSize = 0;
		for (ArrayList<Dependency> oneDepList : deadlockedDeps) if (oneDepList.size() > maxSize) maxSize = oneDepList.size();
		PermutationsWithRepetition gen = new PermutationsWithRepetition(maxSize, deadlockedDeps.size());
		//Combination c = new Combination(maxSize, deadlockedDeps.size());
		int[][] v = gen.getVariations();
		ArrayList<ArrayList<Dependency>> newDeps = new ArrayList<ArrayList<Dependency>>();
		for (int k = 0; k < v.length; k++) {
			int[] oneComb = v[k];
			//System.out.println("Selecting: " + Arrays.toString(oneComb));
			ArrayList<Dependency> oneSelection = new ArrayList<Dependency>();
			for (int i = 0; i < oneComb.length; i++) {
				if (oneComb[i] < deadlockedDeps.get(i).size()) oneSelection.add(deadlockedDeps.get(i).get(oneComb[i]));
				else {
					oneSelection = null;
					break;
				}
			}
			if (oneSelection != null) newDeps.add(oneSelection);
		}
		for (ArrayList<Dependency> depList : newDeps) {
			//System.out.println("DEPLIST: " + depList);
			boolean tryReplanning = true;
			final HashSet<Integer> deadlockedRobots = new HashSet<Integer>();
			for (Dependency dep : depList) {
				deadlockedRobots.add(dep.getWaitingRobotID());
				deadlockedRobots.add(dep.getDrivingRobotID());
				if (inParkingPose(dep.getDrivingRobotID()) || inParkingPose(dep.getWaitingRobotID())) {
					tryReplanning = false;
					break;
				}
			}
			
			synchronized (replanningSpawned) {
				if (tryReplanning && !replanningSpawned.contains(deadlockedRobots)) {
					//Get other robots
					final HashSet<Integer> allRobots = new HashSet<Integer>();
					for (Integer robotID : deadlockedRobots) {
						for (Dependency dep : currentDependencies) {
							if (dep.getDrivingRobotID() == robotID) allRobots.add(dep.getWaitingRobotID());
							else if (dep.getWaitingRobotID() == robotID) allRobots.add(dep.getDrivingRobotID());
						}
					}
	
					replanningSpawned.add(deadlockedRobots);
					metaCSPLogger.info("Will re-plan for one of the following deadlocked robots: " + deadlockedRobots + " (" + allRobots + ")...");
					new Thread() {
						public void run() {
							rePlanPath(deadlockedRobots, allRobots);
						}
					}.start();
	
				}
			}
		}
	}
	
	protected Geometry[] getObstaclesInCriticalPoints(int ... robotIDs) {
		//Compute one obstacle per given robot, placed in the robot's waiting pose
		ArrayList<Geometry> ret = new ArrayList<Geometry>();
		for (int robotID : robotIDs) {
			for (Dependency dep : getCurrentDependencies()) {
				int waitingID = dep.getWaitingRobotID();
				if (robotID == waitingID) {
					Pose waitingPose = dep.getWaitingPose();
					int waitingPoint = dep.getWaitingPoint();
					Geometry currentFP = makeObstacles(waitingID, waitingPose)[0]; 

					//In case the robot has stopped a little beyond the critical point
					int currentPoint = this.getRobotReport(robotID).getPathIndex();
					if (currentPoint != -1 && currentPoint > waitingPoint) {
						Pose currentPose = dep.getWaitingTrajectoryEnvelope().getTrajectory().getPose()[currentPoint];
						currentFP = makeObstacles(waitingID, currentPose)[0];
						System.out.println("Oops: " + waitingPoint + " < " + currentPoint);
					}
					
					ret.add(currentFP);
				}
			}
		}
		return ret.toArray(new Geometry[ret.size()]);
	}
	
	protected Geometry[] getObstaclesFromWaitingRobots(int robotID) {
		//Compute one obstacle per robot that is waiting for this robot, placed in the waiting robot's waiting pose
		ArrayList<Geometry> ret = new ArrayList<Geometry>();
		for (Dependency dep : getCurrentDependencies()) {
			int drivingID = dep.getDrivingRobotID();
			int waitingID = dep.getWaitingRobotID();
			if (robotID == drivingID) {
				Pose waitingPose  = dep.getWaitingTrajectoryEnvelope().getTrajectory().getPose()[dep.getWaitingPoint()];
				ret.add(makeObstacles(waitingID, waitingPose)[0]);
			}
		}
		return ret.toArray(new Geometry[ret.size()]);
	}
	
	protected abstract PoseSteering[] doReplanning(Pose fromPose, Pose toPose, Geometry ... obstaclesToConsider);
	
	protected void rePlanPath(HashSet<Integer> robotsToReplan, HashSet<Integer> allRobots) {
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
			PoseSteering[] newPath = doReplanning(currentWaitingPose, currentWaitingGoal, obstacles);
			if (newPath != null && newPath.length > 0) {
				PoseSteering[] newCompletePath = new PoseSteering[newPath.length+currentWaitingIndex];
				for (int i = 0; i < newCompletePath.length; i++) {
					if (i < currentWaitingIndex) newCompletePath[i] = oldPath[i];
					else newCompletePath[i] = newPath[i-currentWaitingIndex];
				}
				replacePath(robotID, newCompletePath); //FIXME -> the new path may consider as obstacles some old poses.
				metaCSPLogger.info("Successfully re-planned path of Robot" + robotID);
				break;
			}
			else {
				metaCSPLogger.info("Failed to re-plan path of Robot" + robotID);
			}
		}
		synchronized (replanningSpawned) {
			replanningSpawned.remove(robotsToReplan);
		}
	}
	
	/**
	 * Generate obstacles representing the placement(s) of a given robot in given poses.
	 * @param robotID The ID of the robot whose footprint should be used.
	 * @param obstaclePoses The poses of the footprint.
	 * @return A {@link Geometry} that has the shape of the given robot's footprint, placed in each of the the given {@link Pose}s. 
	 */
	public Geometry[] makeObstacles(int robotID, Pose ... obstaclePoses) {
		ArrayList<Geometry> ret = new ArrayList<Geometry>();
		for (Pose p : obstaclePoses) {
			GeometryFactory gf = new GeometryFactory();
			Coordinate[] footprint = this.getFootprint(robotID);
			Coordinate[] newFoot = new Coordinate[footprint.length+1];
			for (int j = 0; j < footprint.length; j++) {
				newFoot[j] = footprint[j];
			}
			newFoot[footprint.length] = footprint[0];
			Geometry obstacle = gf.createPolygon(newFoot);
			AffineTransformation at = new AffineTransformation();
			at.rotate(p.getTheta());
			at.translate(p.getX(), p.getY());
			obstacle = at.transform(obstacle);
			ret.add(obstacle);
			metaCSPLogger.fine("Made obstacle for Robot" + robotID + " in pose " + p);
		}
		return ret.toArray(new Geometry[ret.size()]);
	}

	private boolean unsafePair(Dependency dep1, Dependency dep2) {
		if (dep2.getWaitingPoint() <= dep1.getReleasingPoint()) return true;
		return false;
	}

	//returns true if robot1 should go before robot2
	//returns false if robot2 should go before robot1
	private boolean getOrder(AbstractTrajectoryEnvelopeTracker robotTracker1, RobotReport robotReport1, AbstractTrajectoryEnvelopeTracker robotTracker2, RobotReport robotReport2, CriticalSection cs) {

		//If both can stop before entering, use ordering function (or closest if no ordering function)
		metaCSPLogger.finest("Both robots can stop at " + cs);
		
		//NO MORE USED
//		if (yieldIfParking) {
//			if (cs.getTe1End() == cs.getTe1().getPathLength()-1) {
//				metaCSPLogger.info("Robot" + cs.getTe1().getRobotID() + " will park in " + cs + ", letting Robot" + cs.getTe2().getRobotID() + " go first");
//				return false;
//			}
//			if (cs.getTe2End() == cs.getTe2().getPathLength()-1) {
//				metaCSPLogger.info("Robot" + cs.getTe2().getRobotID() + " will park in " + cs + ", letting Robot" + cs.getTe1().getRobotID() + " go first");
//				return true;
//			}
//		}
		
		//one possible ordering
		synchronized (disallowedDependencies) {
			for (Dependency dep : disallowedDependencies) {			
				//due to delays, the robot may have already entered the critical section, so the precedence cannot be disallowed
				boolean canBeRemoved1 = communicatedCPs.containsKey(robotTracker1) && communicatedCPs.get(robotTracker1).getFirst() != -1 && (communicatedCPs.get(robotTracker1).getFirst() < cs.getTe1Start());
				boolean canBeRemoved2 = communicatedCPs.containsKey(robotTracker2) && communicatedCPs.get(robotTracker2).getFirst() != -1 && (communicatedCPs.get(robotTracker2).getFirst() < cs.getTe2Start());
				if (canBeRemoved1 && canBeRemoved2) {
					if (dep.getWaitingRobotID() == robotReport1.getRobotID() &&
								dep.getDrivingRobotID() == robotReport2.getRobotID() && 
								cs.getTe2End() == dep.getReleasingPoint()) {
							metaCSPLogger.info("DISALLOWED " + dep.getWaitingRobotID() + " waits for " + dep.getDrivingRobotID());
							return true;
						}
						else if (dep.getWaitingRobotID() == robotReport2.getRobotID() &&
								dep.getDrivingRobotID() == robotReport1.getRobotID() && cs.getTe1End() == dep.getReleasingPoint()) {
							metaCSPLogger.info("DISALLOWED " + dep.getWaitingRobotID() + " waits for " + dep.getDrivingRobotID());
							return false;
						}
					}
				}
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

	private boolean isAhead(CriticalSection cs, int lastPathIndex1, int lastPathIndex2) {
		//Robot 1 is ahead --> return true
		PoseSteering[] pathRobot1 = cs.getTe1().getTrajectory().getPoseSteering();
		PoseSteering[] pathRobot2 = cs.getTe2().getTrajectory().getPoseSteering();
		double dist1 = 0.0;
		double dist2 = 0.0;
		for (int i = cs.getTe1Start(); i < lastPathIndex1-1; i++) {
			dist1 += pathRobot1[i].getPose().getPosition().distance(pathRobot1[i+1].getPose().getPosition());
		}
		for (int i = cs.getTe2Start(); i < lastPathIndex2-1; i++) {
			dist2 += pathRobot2[i].getPose().getPosition().distance(pathRobot2[i+1].getPose().getPosition());
		}
		//metaCSPLogger.finest("Dist R" + rr1.getRobotID() + " = " + dist1 + "; Dist R" + rr2.getRobotID() + " = " + dist2);
		return (dist1 > dist2);
	}

	//Update and set the critical points
	private void updateDependencies() {

		//System.out.println("Caller of updateDependencies(): " + Thread.currentThread().getStackTrace()[2]);
		synchronized(solver) {
			HashMap<Integer,TreeSet<Dependency>> currentDeps = new HashMap<Integer,TreeSet<Dependency>>();
			HashSet<Dependency> currentCsToDeps = new HashSet<Dependency>();
	
			//Make deps from un-reached stopping points
			Set<Integer> robotIDs = trackers.keySet();
			for (int robotID : robotIDs) {
				AbstractTrajectoryEnvelopeTracker robotTracker = trackers.get(robotID);
				RobotReport robotReport = robotTracker.getRobotReport();
				synchronized(stoppingPoints) {
					if (stoppingPoints.containsKey(robotID)) {
						for (int i = 0; i < stoppingPoints.get(robotID).size(); i++) {
							int stoppingPoint = stoppingPoints.get(robotID).get(i);
							int duration = stoppingTimes.get(robotID).get(i);
							if (robotReport.getPathIndex() <= stoppingPoint) {
								Dependency dep = new Dependency(robotTracker.getTrajectoryEnvelope(), null, stoppingPoint, 0, robotTracker, null);
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
	
			//Make deps from critical sections, and remove obsolete critical sections
			synchronized(allCriticalSections) {
	
				ArrayList<CriticalSection> toRemove = new ArrayList<CriticalSection>();
				for (CriticalSection cs : this.allCriticalSections) {
					
					//Store the current set of dependencies related to this critical section.
					currentCsToDeps.clear();
					
					//Will be assigned depending on current situation of robot reports...
					int waitingRobotID = -1;
					int drivingRobotID = -1;
					AbstractTrajectoryEnvelopeTracker waitingTracker = null;
					AbstractTrajectoryEnvelopeTracker drivingTracker = null;
					int drivingCurrentIndex = -1;
					TrajectoryEnvelope waitingTE = null;
					TrajectoryEnvelope drivingTE = null;
	
					AbstractTrajectoryEnvelopeTracker robotTracker1 = trackers.get(cs.getTe1().getRobotID());
					RobotReport robotReport1 = this.getRobotReport(cs.getTe1().getRobotID());
					AbstractTrajectoryEnvelopeTracker robotTracker2 = trackers.get(cs.getTe2().getRobotID());
					RobotReport robotReport2 = this.getRobotReport(cs.getTe2().getRobotID());
	
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
							int drivingCSEnd = (drivingRobotID == cs.getTe1().getRobotID()) ? cs.getTe1End() : cs.getTe2End();
							metaCSPLogger.finest("Robot" + drivingRobotID + " is parked, so Robot" + waitingRobotID + " will have to wait");	
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd, waitingTracker, drivingTracker);
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
							currentDeps.get(waitingRobotID).add(dep);
							currentCsToDeps.add(dep);
						}
					}
					else { //Both robots are driving, let's determine an ordering for them through this critical section
																
						//Check if the robots can stop while changing the current order of accessing the critical section.
						ForwardModel fm1 = getForwardModel(robotReport1.getRobotID());
						ForwardModel fm2 = getForwardModel(robotReport2.getRobotID());
						boolean canStopRobot1 = false;
						boolean canStopRobot2 = false;
					
						if (cs.getTe1End() == 0 && cs.getTe1Start() == 0) {
							//force the other robots to wait the robot to leave its footprint.
							canStopRobot1 = false; 
						}
						else {
							if (//the last critical point was before the critical section (can stop by induction)
								(communicatedCPs.containsKey(robotTracker1) && communicatedCPs.get(robotTracker1).getFirst() != -1 && communicatedCPs.get(robotTracker1).getFirst() < cs.getTe1Start())
								|| !communicatedCPs.containsKey(robotTracker1))
								canStopRobot1 = true;
							else
								//Due to temporal delays we cannot trust the velocity.
								canStopRobot1 = fm1.canStop(robotTracker1.getTrajectoryEnvelope(), robotReport1, cs.getTe1Start(), false);			
						}
	
						
						if (cs.getTe2End() == 0 && cs.getTe2Start() == 0) canStopRobot2 = false;
						else {
							if ((communicatedCPs.containsKey(robotTracker2) && communicatedCPs.get(robotTracker2).getFirst() != -1 && communicatedCPs.get(robotTracker2).getFirst() < cs.getTe2Start())
								|| !communicatedCPs.containsKey(robotTracker2))
								canStopRobot2 = true;
							else canStopRobot2 = fm2.canStop(robotTracker2.getTrajectoryEnvelope(), robotReport2, cs.getTe2Start(), false);
						}
						
						//FIXME handle parking in the goal
						//the robot is starting from parking. This constraint will automatically lead to a deadlock if the other robot is already in critical section,
						//while it allows to leave the parking in the other case.
						if (!communicatedCPs.containsKey(robotTracker1)) {
							if (cs.getTe1End() != 0 && cs.getTe1Start() == 0 && canStopRobot2)
								canStopRobot2 = communicatedCPs.get(robotTracker2).getFirst() < cs.getTe2Start();
						}
						
						if (!communicatedCPs.containsKey(robotTracker2)) {
							if (cs.getTe2End() != 0 && cs.getTe2Start() == 0 && canStopRobot1)
								canStopRobot1 = communicatedCPs.get(robotTracker1).getFirst() < cs.getTe1Start();
						}
						
						//Both the robots can stop before accessing the critical section --> follow ordering heuristic if FW model allows it
						if (canStopRobot1 && canStopRobot2) {
	
							//If robot 1 has priority over robot 2
							if (getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs)) {
								drivingCurrentIndex = robotReport1.getPathIndex();
								waitingTE = cs.getTe2();
								drivingTE = cs.getTe1();
								metaCSPLogger.finest("Both-can-stop (1) and Robot" + drivingTE.getRobotID() + " ahead of Robot" + waitingTE.getRobotID() + " and CS is: " + cs);
							}
	
							//If robot 2 has priority over robot 1
							else {
								drivingCurrentIndex = robotReport2.getPathIndex();
								waitingTE = cs.getTe1();
								drivingTE = cs.getTe2();
								metaCSPLogger.finest("Both-can-stop (2) and Robot" + drivingTE.getRobotID() + " ahead of Robot" + waitingTE.getRobotID() + " and CS is: " + cs);
							}
	
						}
	
						//Robot 1 can stop before entering the critical section, robot 2 can't --> robot 1 waits
						else if (canStopRobot1 && !canStopRobot2) {
							drivingCurrentIndex = robotReport2.getPathIndex();
							waitingTE = cs.getTe1();
							drivingTE = cs.getTe2();
							metaCSPLogger.finest("One-can-one-can't-stop (1) and Robot" + drivingTE.getRobotID() + " (can't) is ahead of Robot" + waitingTE.getRobotID() + " (can) and CS is: " + cs);
						}
	
						//Robot 2 can stop before entering critical section, robot 1 can't --> robot 2 waits
						else if (!canStopRobot1 && canStopRobot2) {
							drivingCurrentIndex = robotReport1.getPathIndex();
							waitingTE = cs.getTe2();
							drivingTE = cs.getTe1();
							metaCSPLogger.finest("One-can-one-can't-stop (2) and Robot" + drivingTE.getRobotID() + " (can't) ahead of Robot" + waitingTE.getRobotID() + " (can) and CS is: " + cs);
						}
	
						else {			
							
							//Both robots in critical section --> re-impose previously decided dependency
							
							//Check the previous decided order
							if (this.criticalSectionsToDeps.containsKey(cs) && this.criticalSectionsToDeps.get(cs).size() == 1) {
								//The critical section is not new and no re-plan happens.
								//Standard case: there is just one dependency in the previous set. Let's impose that.
								for (Dependency dep : this.criticalSectionsToDeps.get(cs)) {
									drivingRobotID = dep.getDrivingRobotID();
									waitingRobotID = dep.getWaitingRobotID();
								}
								
								//If one of the robot was parked previously in critical section and has just started driving again,
								//then the set of previous constraints contains at least one dependency (due to parking).
							}
							else {
									metaCSPLogger.severe("Both cannot stop but lost critical section to dep. CS: " + cs);
									throw new Error("FIXME! Lost dependency! " );							
							}
							
							drivingCurrentIndex = (drivingRobotID == robotReport1.getRobotID()) ? robotReport1.getPathIndex() : robotReport2.getPathIndex();
							drivingTE = (drivingRobotID == robotReport1.getRobotID()) ? cs.getTe1() : cs.getTe2();
							waitingTE = (waitingRobotID == robotReport1.getRobotID()) ? cs.getTe1() : cs.getTe2();
							metaCSPLogger.finest("Both-can't-stop and Robot" + drivingTE.getRobotID() + " ahead of Robot" + waitingTE.getRobotID() + " and CS is: " + cs);
						}
	
						waitingRobotID = waitingTE.getRobotID();
						drivingRobotID = drivingTE.getRobotID();
						waitingTracker = trackers.get(waitingRobotID);
						drivingTracker = trackers.get(drivingRobotID);
	
						//Compute waiting path index point for waiting robot
						waitingPoint = getCriticalPoint(waitingRobotID, cs, drivingCurrentIndex);
						if (waitingPoint >= 0) {		
							//Make new dependency
							int drivingCSEnd = (drivingRobotID == cs.getTe1().getRobotID()) ? cs.getTe1End() : cs.getTe2End();
							Dependency dep = new Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd, waitingTracker, drivingTracker);
							if (!currentDeps.containsKey(waitingRobotID)) currentDeps.put(waitingRobotID, new TreeSet<Dependency>());
							currentDeps.get(waitingRobotID).add(dep);
							currentCsToDeps.add(dep);
						}
						else {
							//If robot is asked to wait in an invalid path point, throw error and give up!
							metaCSPLogger.severe("Waiting point < 0 for critical section " + cs);
							throw new Error("Waiting point < 0 for critical section " + cs);
						}
					}
					
					//Update the map
					HashSet<Dependency> depsToAdd = new HashSet<Dependency>();
					for (Dependency dep : currentCsToDeps) {
						depsToAdd.add(dep);
					}
					this.criticalSectionsToDeps.put(cs, depsToAdd);
				}
	
				//Remove obsolete critical sections
				for (CriticalSection cs : toRemove) {
					this.allCriticalSections.remove(cs);
					this.criticalSectionsToDeps.remove(cs);
				}
			}
	
			synchronized(currentDependencies) {
				currentDependencies.clear();
				for (Integer robotID : robotIDs) {
					if (!currentDeps.containsKey(robotID)) {
						setCriticalPoint(robotID, -1);
					}
					else {
						Dependency firstDep  = currentDeps.get(robotID).first();
						setCriticalPoint(firstDep.getWaitingRobotID(), firstDep.getWaitingPoint());
						currentDependencies.add(firstDep);
					}
				}
				findCycles();
			}
		}
	}
	
	private boolean canExitCriticalSection(int drivingCurrentIndex, int waitingCurrentIndex, TrajectoryEnvelope drivingTE, TrajectoryEnvelope waitingTE, int lastIndexOfCSDriving) {
		drivingCurrentIndex = Math.max(drivingCurrentIndex,0);
		waitingCurrentIndex = Math.max(waitingCurrentIndex,0);
		Geometry placementWaiting = waitingTE.makeFootprint(waitingTE.getTrajectory().getPoseSteering()[waitingCurrentIndex]);
		for (int i = drivingCurrentIndex; i <= lastIndexOfCSDriving; i++) {
			Geometry placementDriving = drivingTE.makeFootprint(drivingTE.getTrajectory().getPoseSteering()[i]);
			if (placementWaiting.intersects(placementDriving)) return false;
		}
		return true;
	}


	/**
	 * Add a criterion for determining the order of robots through critical sections
	 * (comparator of {@link AbstractTrajectoryEnvelopeTracker}s). 
	 * Comparators are considered in the order in which they are added.
	 * @param c A new comparator for determining robot ordering through critical sections.
	 */
	public void addComparator(Comparator<RobotAtCriticalSection> c) {
		this.comparators.addComparator(c);
	}
	
	/**
	 * Update the set of current critical sections. This should be called every time
	 * a new set of {@link Mission}s has been successfully added.
	 */
	public void computeCriticalSections() {

		int numberOfCriticalSections = 0;
		
		synchronized(allCriticalSections) {

			//Collect all driving envelopes
			ArrayList<TrajectoryEnvelope> drivingEnvelopes = new ArrayList<TrajectoryEnvelope>();
			for (AbstractTrajectoryEnvelopeTracker atet : trackers.values()) {
				if (!(atet instanceof TrajectoryEnvelopeTrackerDummy)) {
					drivingEnvelopes.add(atet.getTrajectoryEnvelope());
				}
			}

			//Compute critical sections between driving and new envelopes
			for (int i = 0; i < drivingEnvelopes.size(); i++) {
				for (int j = 0; j < envelopesToTrack.size(); j++) {	
					if (!envelopesToTrack.get(j).equals(drivingEnvelopes.get(i))) {
						for (CriticalSection cs : getCriticalSections(drivingEnvelopes.get(i), envelopesToTrack.get(j))) {
							this.allCriticalSections.add(cs);
							this.criticalSectionsToDeps.put(cs, new HashSet<Dependency>());
						}
					}
				}
			}	

			//Compute critical sections between new envelopes
			for (int i = 0; i < envelopesToTrack.size(); i++) {
				for (int j = i+1; j < envelopesToTrack.size(); j++) {
					for (CriticalSection cs : getCriticalSections(envelopesToTrack.get(i), envelopesToTrack.get(j))) {
						this.allCriticalSections.add(cs);
						this.criticalSectionsToDeps.put(cs, new HashSet<Dependency>());
					}
				}
			}

			//Compute critical sections between driving envelopes and current parking envelopes	
			for (int i = 0; i < drivingEnvelopes.size(); i++) {
				for (int j = 0; j < currentParkingEnvelopes.size(); j++) {
					if (drivingEnvelopes.get(i).getRobotID() != currentParkingEnvelopes.get(j).getRobotID()) {
						for (CriticalSection cs : getCriticalSections(drivingEnvelopes.get(i), currentParkingEnvelopes.get(j))) {
							this.allCriticalSections.add(cs);
							this.criticalSectionsToDeps.put(cs, new HashSet<Dependency>());
						}
					}
				}
			}

			//Compute critical sections between NEW driving envelopes and current parking envelopes	
			for (int i = 0; i < envelopesToTrack.size(); i++) {
				for (int j = 0; j < currentParkingEnvelopes.size(); j++) {
					if (envelopesToTrack.get(i).getRobotID() != currentParkingEnvelopes.get(j).getRobotID()) {
						for (CriticalSection cs : getCriticalSections(envelopesToTrack.get(i), currentParkingEnvelopes.get(j))) {
							this.allCriticalSections.add(cs);
							this.criticalSectionsToDeps.put(cs, new HashSet<Dependency>());
						}
					}
				}
			}			
			filterCriticalSections();
			
			numberOfCriticalSections = this.allCriticalSections.size();
			metaCSPLogger.info("There are now " + numberOfCriticalSections + " critical sections");
		}
		
		if (checkCollisions && (collisionThread == null) && (numberOfCriticalSections > 0)) {
			// Start the collision checking thread. 
			// The tread will be alive until there will be almost one critical section.
			collisionThread = new Thread("Collision checking thread.") {

				
				@Override
				public void run() {
					metaCSPLogger.info("Starting the collision checking thread.");
					ArrayList<CriticalSection> previousCollidingCS = new ArrayList<CriticalSection>();
					ArrayList<CriticalSection> newCollidingCS = new ArrayList<CriticalSection>();
					
					while(true) {
						newCollidingCS.clear();
						
						//collisions can happen only in critical sections						
						synchronized (allCriticalSections) {
							if (allCriticalSections.isEmpty())
								break; //break the thread if there are no critical sections to control
							
							for (CriticalSection cs : allCriticalSections) {
								//check if both the robots are inside the critical section
								
								//FIXME sample the real pose of the robots (ok in simulation, but not otherwise)
								RobotReport robotReport1, robotReport2;
								AbstractTrajectoryEnvelopeTracker tracker1, tracker2;
								try {
									synchronized (trackers)	{
										tracker1 = trackers.get(cs.getTe1().getRobotID());
										robotReport1 = tracker1.getRobotReport();
										tracker2 = trackers.get(cs.getTe2().getRobotID());
										robotReport2 = tracker2.getRobotReport();
									}
								}
								catch (NullPointerException e) {
									continue; //skip this cycle
								}
								
								if ( robotReport1 != null && robotReport2 != null &&
								(robotReport1.getPathIndex() < cs.getTe1End()) && (robotReport1.getPathIndex() > cs.getTe1Start()) && //robot1 is inside
								(robotReport2.getPathIndex() < cs.getTe2End()) && (robotReport2.getPathIndex() > cs.getTe2Start())  	//robot2 is inside
								) {
									//place robot  in pose and get geometry
									PoseSteering[] path1 = cs.getTe1().getTrajectory().getPoseSteering();
									Geometry placement1 = cs.getTe1().makeFootprint(path1[robotReport1.getPathIndex()]);
									
									PoseSteering[] path2 = cs.getTe2().getTrajectory().getPoseSteering();
									Geometry placement2 = cs.getTe2().makeFootprint(path2[robotReport2.getPathIndex()]);
									
									//check intersection
									if (placement1.intersects(placement2)) {
										if (!previousCollidingCS.contains(cs)) {
											metaCSPLogger.info(" * NEW COLLISION *");
											CollisionEvent ce = new CollisionEvent(Calendar.getInstance().getTimeInMillis(),robotReport1,robotReport2);
											synchronized (collisionsList) {
												collisionsList.add(ce);
											}		
											newCollidingCS.add(cs);
										}
									}
									else if (previousCollidingCS.contains(cs))
										previousCollidingCS.remove(cs); //remove the ones that are not colliding anymore
								}
							}
						}
						previousCollidingCS.addAll(newCollidingCS);
						
						try { Thread.sleep((long)(1000.0/30.0)); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					metaCSPLogger.info("Ending the collision checking thread.");
				}
					
			};
			collisionThread.start();
		}
	}
	
	/**
	 * Replace the path of a robot's {@link TrajectoryEnvelope} on the fly.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} is to be recomputed.
	 * @param newPath The path based on which the new {@link TrajectoryEnvelope} should be computed.
	 */
	public void replacePath(int robotID, PoseSteering[] newPath) {
		
		synchronized (solver) {
			
			synchronized (allCriticalSections) {
				//Get current envelope
				TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);
						
				if (viz != null) {
					viz.removeEnvelope(te);
				}
				
				//Remove CSs involving this robot
				HashMap<CriticalSection,HashSet<Dependency>> toRemove = new HashMap<CriticalSection,HashSet<Dependency>>();
				for (CriticalSection cs : this.criticalSectionsToDeps.keySet()) {
					if (cs.getTe1().equals(te) || cs.getTe2().equals(te)) {
						toRemove.put(cs, this.criticalSectionsToDeps.get(cs));
					}
				}
				
				//FIXME!! Compute and store the set of dependencies that should be maintained.
				for (CriticalSection cs : toRemove.keySet()) {
					this.allCriticalSections.remove(cs);
					this.criticalSectionsToDeps.remove(cs);
				}
			
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
				
				//-------------------------------------
				
				//Make new envelope containing only the new part of the path
				int lastCriticalPoint = -1;
				AbstractTrajectoryEnvelopeTracker robotTracker = null;
				synchronized (trackers) {
					robotTracker = this.trackers.get(robotID);
					lastCriticalPoint = communicatedCPs.get(robotTracker).getFirst();
				}
				PoseSteering[] truncatedPath = Arrays.copyOf(newTE.getTrajectory().getPoseSteering(), lastCriticalPoint+1);
				TrajectoryEnvelope truncatedTE = solver.createEnvelopeNoParking(robotID, truncatedPath, "Driving", this.getFootprint(robotID));
				
				//Stitch together with rest of constraint network (temporal constraints with parking envelopes etc.)
				for (Constraint con : solver.getConstraintNetwork().getOutgoingEdges(te)) {
					if (con instanceof AllenIntervalConstraint) {
						AllenIntervalConstraint aic = (AllenIntervalConstraint)con;
						if (aic.getTypes()[0].equals(AllenIntervalConstraint.Type.Meets)) {
							TrajectoryEnvelope newEndParking = solver.createParkingEnvelope(robotID, PARKING_DURATION, truncatedTE.getTrajectory().getPose()[newTE.getTrajectory().getPose().length-1], "whatever", getFootprint(robotID));
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
				
				//Add as if it were a new envelope, that way it will be accounted for in computeCriticalSections()
				envelopesToTrack.add(truncatedTE);
				
				//Recompute CSs involving this robot (only on the new piece of the path) 
				computeCriticalSections();
				
				//For each CS, replace the truncated trajectory envelope with the complete one, shift start and end.
				ArrayList<CriticalSection> toAdd = new ArrayList<CriticalSection>(); 
				ArrayList<CriticalSection> newToRemove = new ArrayList<CriticalSection>(); 
				for (CriticalSection cs1 : this.allCriticalSections) {
					if (cs1.getTe1().getRobotID() == robotID) {
						CriticalSection cs3 = new CriticalSection(newTE,cs1.getTe2(),cs1.getTe1Start()+lastCriticalPoint+1,cs1.getTe2Start(),cs1.getTe1End()+lastCriticalPoint+1,cs1.getTe2End());
						toAdd.add(cs3);
						newToRemove.add(cs1);
					}
					else if (cs1.getTe2().getRobotID() == robotID) {
						CriticalSection cs3 = new CriticalSection(cs1.getTe1(),newTE,cs1.getTe2Start(),cs1.getTe2Start()+lastCriticalPoint+1,cs1.getTe2End(),cs1.getTe2End()+lastCriticalPoint+1);
						toAdd.add(cs3);
						newToRemove.add(cs1);
					}
				}
				
				//remove not shifted CS involving the robot
				for (CriticalSection cs : newToRemove) {
					this.allCriticalSections.remove(cs);
					this.criticalSectionsToDeps.remove(cs);
				}
				//add the shifted ones an empty set of dependencies
				for (CriticalSection cs : toAdd) {
					this.allCriticalSections.add(cs);
					this.criticalSectionsToDeps.put(cs,new HashSet<Dependency>());
				}
				
				//Then, stick together the old CS with the new ones.
				toAdd.clear();
				newToRemove.clear();
				HashMap<CriticalSection,HashSet<Dependency>> toMerge = new HashMap<CriticalSection,HashSet<Dependency>>();
				
				//(TrajectoryEnvelope te1, TrajectoryEnvelope te2, int te1Start, int te2Start, int te1End, int te2End)
				
				for (CriticalSection cs2 : toRemove.keySet()) {
					//cs2 is related to this robot?
					if (cs2.getTe1().getRobotID() == robotID || cs2.getTe2().getRobotID() == robotID) {
						//something should be done. If it ends before the last communicated CP, then it should be added and stop.
						if ((cs2.getTe1().getRobotID() == robotID && cs2.getTe1End() < lastCriticalPoint) || (cs2.getTe2().getRobotID() == robotID && cs2.getTe2End() < lastCriticalPoint)) {
							toAdd.add(cs2);
						}
						else {
							//check if the OLD critical section should be merged with a NEW one
							for (CriticalSection cs1 : this.allCriticalSections) {
								if ((cs1.getTe1().getRobotID() == cs2.getTe1().getRobotID() && cs1.getTe2().getRobotID() == cs2.getTe2().getRobotID()) ||
										(cs1.getTe1().getRobotID() == cs2.getTe2().getRobotID() && cs1.getTe2().getRobotID() == cs2.getTe1().getRobotID())) {
									if (cs1.getTe1().getRobotID() == robotID && cs1.getTe1Start() == lastCriticalPoint + 1) {
										int newStart = (cs2.getTe1().getRobotID() == robotID) ? cs2.getTe1Start() : cs2.getTe2Start();
										int newEnd = (cs1.getTe1().getRobotID() == robotID) ? cs1.getTe1End() : cs1.getTe2End();
										CriticalSection cs3 = new CriticalSection(cs1.getTe1(),cs1.getTe2(),newStart,cs1.getTe2Start(),newEnd,cs1.getTe2End());
										toMerge.put(cs3,toRemove.get(cs2));
										newToRemove.add(cs1);
										break;
									}
									else if (cs1.getTe2().getRobotID() == robotID && cs1.getTe2Start() == lastCriticalPoint + 1) {
										int newStart = (cs2.getTe1().getRobotID() == robotID) ? cs2.getTe1Start() : cs2.getTe2Start();
										int newEnd = (cs1.getTe1().getRobotID() == robotID) ? cs1.getTe1End() : cs1.getTe2End();
										CriticalSection cs3 = new CriticalSection(cs1.getTe1(),cs1.getTe2(),cs1.getTe1Start(),newStart,cs1.getTe1End(),newEnd);
										toMerge.put(cs3,toRemove.get(cs2));
										newToRemove.add(cs1);
										break;
									}
								}
							}
						}
					}
				}
				
				//restore the set of CS related to the first part of the path with the related set of dependencies
				for (CriticalSection cs : toAdd) {
					this.allCriticalSections.add(cs);
					this.criticalSectionsToDeps.put(cs,toRemove.get(cs));
				}
				
				//replace the CS that should be merged
				for (CriticalSection cs : newToRemove) {
					this.allCriticalSections.remove(cs);
					this.criticalSectionsToDeps.remove(cs);
				}
				for (CriticalSection cs : toMerge.keySet()) {
					this.allCriticalSections.add(cs);
					this.criticalSectionsToDeps.put(cs,toMerge.get(cs));
				}
				
				//remove the truncated envelope
				envelopesToTrack.remove(truncatedTE);
				cleanUp(truncatedTE);
				
				updateDependencies();							
				
			}
		}
	}
	
	/**
	 * Truncate the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point. This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be truncated.
	 */
	public void truncateEnvelope(int robotID) {
		synchronized (solver) {
			TrajectoryEnvelope te = this.getCurrentTrajectoryEnvelope(robotID);
			AbstractTrajectoryEnvelopeTracker tet = this.trackers.get(robotID); 
			if (!(tet instanceof TrajectoryEnvelopeTrackerDummy)) {
				int earliestStoppingPathIndex = this.getForwardModel(robotID).getEarliestStoppingPathIndex(te, this.getRobotReport(robotID));
				if (earliestStoppingPathIndex != -1) {
					metaCSPLogger.info("Truncating " + te + " at " + earliestStoppingPathIndex);
					
					//Compute and add new TE, remove old TE (both driving and final parking)
					PoseSteering[] truncatedPath = Arrays.copyOf(te.getTrajectory().getPoseSteering(), earliestStoppingPathIndex+1);
					
					//replace the path of this robot (will compute new envelope)
					replacePath(robotID, truncatedPath);
					
				}
			}
		}
	}
	
	/**
	 * Reverse the {@link TrajectoryEnvelope} of a given robot at the closest dynamically-feasible path point. This path point is computed via the robot's {@link ForwardModel}.
	 * @param robotID The ID of the robot whose {@link TrajectoryEnvelope} should be reversed.
	 */
	public void reverseEnvelope(int robotID) {
		synchronized (solver) {
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
					replacePath(robotID, overallPath);
					
				}
			}
		}
	}
	
	
	private void filterCriticalSections() {
		ArrayList<CriticalSection> toAdd = new ArrayList<CriticalSection>();
		ArrayList<CriticalSection> toRemove = new ArrayList<CriticalSection>();
		int passNum = 0;
		do {
			passNum++;
			toAdd.clear();
			toRemove.clear();
			for (CriticalSection cs1 : this.allCriticalSections) {
				for (CriticalSection cs2 : this.allCriticalSections) {
					//If CS1 and CS2 are about the same pair of robots
					if (!cs1.equals(cs2) && cs1.getTe1().equals(cs2.getTe1()) && cs1.getTe2().equals(cs2.getTe2())) {
						int start11 = cs1.getTe1Start();
						int start12 = cs1.getTe2Start();
						int start21 = cs2.getTe1Start();
						int start22 = cs2.getTe2Start();
						int end11 = cs1.getTe1End();
						int end12 = cs1.getTe2End();
						int end21 = cs2.getTe1End();
						int end22 = cs2.getTe2End();
						//CS1 before CS2
						if (start11 < start21) {
							//CS1 ends after CS2 starts
							if (end11 >= start21) {
								toRemove.add(cs1);
								toRemove.add(cs2);
								int newStart1 = start11;
								int newEnd1 = end21;
								int newStart2 = -1;
								int newEnd2 = -1;
								if (start12 < start22) {
									newStart2 = start12;
									newEnd2 = end22;
								}
								else {
									newStart2 = start22;
									newEnd2 = end12;
								}
								CriticalSection newCS = new CriticalSection(cs1.getTe1(), cs1.getTe2(), newStart1, newStart2, newEnd1, newEnd2); 
								toAdd.add(newCS);
								metaCSPLogger.finest("(Pass " + passNum + ") MERGED (ends-after-start): " + cs1 + " + " + cs2 + " = " + newCS);
							}
						}
					}
				}	
			}
			for (CriticalSection cs : toRemove) {
				this.allCriticalSections.remove(cs);
				this.criticalSectionsToDeps.remove(cs);
			}
			for (CriticalSection cs : toAdd) {
				this.allCriticalSections.add(cs);
				this.criticalSectionsToDeps.put(cs, new HashSet<Dependency>());
			}
		}
		while (!toAdd.isEmpty() || !toRemove.isEmpty());
		
		toRemove.clear();
		for (int i = 0; i < this.allCriticalSections.size(); i++) {
			for (int j = i+1; j < this.allCriticalSections.size(); j++) {
				CriticalSection cs1 = this.allCriticalSections.get(i);
				CriticalSection cs2 = this.allCriticalSections.get(j);
				int start11 = cs1.getTe1Start();
				int start12 = cs1.getTe2Start();
				int start21 = cs2.getTe1Start();
				int start22 = cs2.getTe2Start();
				int end11 = cs1.getTe1End();
				int end12 = cs1.getTe2End();
				int end21 = cs2.getTe1End();
				int end22 = cs2.getTe2End();
				//If CS1 and CS2 are about the same pair of robots
				if (!cs1.equals(cs2) && cs1.getTe1().equals(cs2.getTe1()) && cs1.getTe2().equals(cs2.getTe2())) {
					//CS1 and CS2 are identical
					if (start11 == start21 && end11 == end21 && start12 == start22 && end12 == end22) {
						toRemove.add(cs1);
						metaCSPLogger.finest("(Pass " + passNum + ") Removed one of " + cs1 + " and " + cs2);
					}
				}
			}
		}
		for (CriticalSection cs : toRemove) {
			this.allCriticalSections.remove(cs);
			this.criticalSectionsToDeps.remove(cs);
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

			if (checkEscapePoses) {
				//Check that there is an "escape pose" along the paths 
				boolean safe = false;
				for (int j = 0; j < path1.length; j++) {
					Geometry placement1 = te1.makeFootprint(path1[j]);
					if (!placement1.intersects(shape2)) {
						safe = true;
						break;
					}
				}
				if (path1.length == 1 || path2.length == 1) safe = true;
				if (!safe) {
					metaCSPLogger.severe("** WARNING ** Cannot coordinate as one envelope is completely overlapped by the other!");
					metaCSPLogger.severe("** " + te1 + " <--> " + te2);
					//throw new Error("Cannot coordinate as one envelope is completely overlapped by the other!");
				}

				safe = false;
				for (int j = 0; j < path2.length; j++) {
					Geometry placement2 = te2.makeFootprint(path2[j]);
					if (!placement2.intersects(shape1)) {
						safe = true;
						break;
					}
				}
				if (path1.length == 1 || path2.length == 1) safe = true;
				if (!safe) {
					metaCSPLogger.severe("** WARNING ** Cannot coordinate as one envelope is completely overlapped by the other!");
					metaCSPLogger.severe("** " + te1 + " <--> " + te2);
					//throw new Error("Cannot coordinate as one envelope is completely overlapped by the other!");
				}
			}

			Geometry gc = shape1.intersection(shape2);
			ArrayList<Geometry> allIntersections = new ArrayList<Geometry>();
			if (gc.getNumGeometries() == 1) {
				allIntersections.add(gc);
			}
			else {
				for (int i = 1; i < gc.getNumGeometries(); i++) {
					Geometry prev = gc.getGeometryN(i-1);
					Geometry next = gc.getGeometryN(i);					
					if (prev.distance(next) < getMaxFootprintDimension(te1.getRobotID()) || prev.distance(next) < getMaxFootprintDimension(te2.getRobotID())) {
						allIntersections.add(prev.union(next).convexHull());
					}
					else {
						allIntersections.add(prev);
						if (i == gc.getNumGeometries()-1) allIntersections.add(next);
					}
				}
			}

			for (int i = 0; i < allIntersections.size(); i++) {
				ArrayList<CriticalSection> cssOneIntersectionPiece = new ArrayList<CriticalSection>();
				ArrayList<Integer> te1Starts = new ArrayList<Integer>();
				ArrayList<Integer> te1Ends = new ArrayList<Integer>();
				ArrayList<Integer> te2Starts = new ArrayList<Integer>();
				ArrayList<Integer> te2Ends = new ArrayList<Integer>();
				Geometry g = allIntersections.get(i);
				boolean started = false;
				for (int j = 0; j < path1.length; j++) {
					Geometry placement1 = te1.makeFootprint(path1[j]);
					if (!started && placement1.intersects(g)) {
						started = true;
						te1Starts.add(j);
					}
					else if (started && !placement1.intersects(g)) {
						te1Ends.add(j);
						started = false;
					}
					if (started && j == path1.length-1) {
						te1Ends.add(path1.length-1);
					}
				}
				started = false;
				for (int j = 0; j < path2.length; j++) {
					Geometry placement2 = te2.makeFootprint(path2[j]);
					if (!started && placement2.intersects(g)) {
						started = true;
						te2Starts.add(j);
					}
					else if (started && !placement2.intersects(g)) {
						te2Ends.add(j);
						started = false;
					}
					if (started && j == path2.length-1) {
						te2Ends.add(path2.length-1);
					}
				}
				for (int k1 = 0; k1 < te1Starts.size(); k1++) {
					for (int k2 = 0; k2 < te2Starts.size(); k2++) {
						CriticalSection oneCS = new CriticalSection(te1, te2, te1Starts.get(k1), te2Starts.get(k2), te1Ends.get(k1), te2Ends.get(k2));
						//css.add(oneCS);
						cssOneIntersectionPiece.add(oneCS);
					}					
				}
				
				
				// SPURIOUS INTERSECTIONS (can ignore)
				if (te1Starts.size() == 0 || te2Starts.size() == 0) {
					cssOneIntersectionPiece.clear();
				}

				// ASYMMETRIC INTERSECTIONS OF ENVELOPES
				// There are cases in which there are more starts along one envelope than along the other
				// (see the Epiroc underground mining example).
				// These "holes" may or may not be big enough to accommodate a robot. Those that are not
				// should be filtered, as they falsely indicate that the critical section ends for a little bit
				// before restarting. Because of this, such situations may lead to collision.
				// Here, we take a conservative approach: instead of verifying whether
				// the "hole" is big enough to really accommodate a robot so that it does not collide with
				// the other envelope, we simply filter out all of these cases. We do this by joining the
				// critical sections around holes.
				else if (te1Starts.size() != te2Starts.size()) {
					if (te1Starts.size() == 0 || te2Starts.size() == 0) System.out.println("CRAP: te1Starts is " + te1Starts + " and te2Starts is " + te2Starts);
					metaCSPLogger.info("Asymmetric intersections of envelopes for Robot" + te1.getRobotID() + ", Robot" + te2.getRobotID() + ":");
					metaCSPLogger.info("   Original : " + cssOneIntersectionPiece);
					CriticalSection oldCSFirst = cssOneIntersectionPiece.get(0);
					CriticalSection oldCSLast = cssOneIntersectionPiece.get(cssOneIntersectionPiece.size()-1);
					CriticalSection newCS = new CriticalSection(te1, te2, oldCSFirst.getTe1Start(), oldCSFirst.getTe2Start(), oldCSLast.getTe1End(), oldCSLast.getTe2End());
					cssOneIntersectionPiece.clear();
					cssOneIntersectionPiece.add(newCS);
					metaCSPLogger.info("   Refined  : " + cssOneIntersectionPiece);
				}

				css.addAll(cssOneIntersectionPiece);

			}
		}
		
		return css.toArray(new CriticalSection[css.size()]);
	}

	protected void cleanUp(TrajectoryEnvelope te) {
		synchronized (solver) {
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
	 * Add a {@link TrackingCallback} that will be called by the tracker of a given robot.
	 * @param robotID The ID of the robot to which the callback should be attached.
	 * @param cb A callback object.
	 */
	public void addTrackingCallback(int robotID, TrackingCallback cb) {
		trackingCallbacks.put(robotID, cb);
	}

	/**
	 * Start the trackers associated to the last batch of {@link Mission}s that has been added.
	 */
	public void startTrackingAddedMissions() {
		
		//FIXME: if this is not placed into the control loop (), then a robot can pass from (P) to (D) without 
		//affecting the set of dependencies.

		synchronized (solver) {
			for (final TrajectoryEnvelope te : envelopesToTrack) {
				TrajectoryEnvelopeTrackerDummy startParkingTracker = null;
				synchronized (trackers) {
					startParkingTracker = (TrajectoryEnvelopeTrackerDummy)trackers.get(te.getRobotID());
				}
				final TrajectoryEnvelope startParking = startParkingTracker.getTrajectoryEnvelope();
				//Create end parking envelope
				final TrajectoryEnvelope endParking = solver.createParkingEnvelope(te.getRobotID(), PARKING_DURATION, te.getTrajectory().getPose()[te.getTrajectory().getPose().length-1], "whatever", getFootprint(te.getRobotID()));

				//Driving meets final parking
				AllenIntervalConstraint meets1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
				meets1.setFrom(te);
				meets1.setTo(endParking);
				//System.out.println("Made end parking: " + endParking + " with con: " + meets1);

				if (!solver.addConstraints(meets1)) {
					metaCSPLogger.severe("ERROR: Could not add constraints " + meets1);
					throw new Error("Could not add constraints " + meets1);		
				}

				//Add onStart call back that cleans up parking tracker
				//Note: onStart is triggered only when earliest start of this tracker's envelope is < current time
				TrackingCallback cb = new TrackingCallback(te) {
					
					private long lastEnvelopeRefresh = Calendar.getInstance().getTimeInMillis();
					private boolean trackingFinished = false;
					
					@Override
					public void beforeTrackingStart() {
						
						if (trackingCallbacks.containsKey(myTE.getRobotID())) {
							trackingCallbacks.get(myTE.getRobotID()).myTE = this.myTE;
							trackingCallbacks.get(myTE.getRobotID()).beforeTrackingStart();
						}

						//canStartTracking becomes true when setCriticalPoint is called once
						while (!trackers.containsKey(myTE.getRobotID()) || !trackers.get(myTE.getRobotID()).canStartTracking()) {
							try { Thread.sleep(100); }
							catch (InterruptedException e) { e.printStackTrace(); }							
						}
						
//						//Sleep for one control period
//						//(allows to impose critical points before tracking actually starts)
//						try { Thread.sleep(CONTROL_PERIOD); }
//						catch (InterruptedException e) { e.printStackTrace(); }
					}

					@Override
					public void onTrackingStart() {
						if (trackingCallbacks.containsKey(myTE.getRobotID())) trackingCallbacks.get(myTE.getRobotID()).onTrackingStart();
						if (viz != null) viz.addEnvelope(myTE);
					}

					@Override
					public void onNewGroundEnvelope() {
						if (trackingCallbacks.containsKey(myTE.getRobotID())) trackingCallbacks.get(myTE.getRobotID()).onNewGroundEnvelope();
					}

					@Override
					public void beforeTrackingFinished() {
						this.trackingFinished = true;
						if (trackingCallbacks.containsKey(myTE.getRobotID())) trackingCallbacks.get(myTE.getRobotID()).beforeTrackingFinished();
					}

					@Override
					public void onTrackingFinished() {

						synchronized (solver) {
							metaCSPLogger.info("Tracking finished for " + myTE);
							
							if (trackingCallbacks.containsKey(myTE.getRobotID())) trackingCallbacks.get(myTE.getRobotID()).onTrackingFinished();

							if (viz != null) viz.removeEnvelope(myTE);

							//reset stopping points
							synchronized(stoppingPoints) {
								stoppingPoints.remove(myTE.getRobotID());
								stoppingTimes.remove(myTE.getRobotID());
							}

							//remove critical sections in which this robot is involved
							synchronized(allCriticalSections) {
								ArrayList<CriticalSection> toRemove = new ArrayList<CriticalSection>();
								for (CriticalSection cs : allCriticalSections) {
									if (cs.getTe1().getRobotID() == myTE.getRobotID() || cs.getTe2().getRobotID() == myTE.getRobotID()) toRemove.add(cs);
								}
								for (CriticalSection cs : toRemove) {
									allCriticalSections.remove(cs);
									criticalSectionsToDeps.remove(cs);
								}

							}

							//clean up the old parking envelope
							cleanUp(startParking);
							currentParkingEnvelopes.remove(startParking);

							//Find end parking...
							TrajectoryEnvelope myEndParking = null;
							for (Constraint con : solver.getConstraintNetwork().getOutgoingEdges(myTE)) {
								if (con instanceof AllenIntervalConstraint) {
									AllenIntervalConstraint aic = (AllenIntervalConstraint)con;
									if (aic.getTypes()[0].equals(AllenIntervalConstraint.Type.Meets)) {
										myEndParking = (TrajectoryEnvelope)aic.getTo();
										break;
									}
								}
							}
							
							//clean up this tracker's TE
							cleanUp(myTE);

							//remove communicatedCP entry
							communicatedCPs.remove(trackers.get(myTE.getRobotID()));

							//Make a new parking tracker for the found end parking (park the robot)
							placeRobot(myTE.getRobotID(), null, myEndParking, null);

							synchronized (disallowedDependencies) {
								ArrayList<Dependency> toRemove = new ArrayList<Dependency>();
								for (Dependency dep : disallowedDependencies) {
									if (dep.getDrivingRobotID() == myTE.getRobotID() || dep.getWaitingRobotID() == myTE.getRobotID()) {
										toRemove.add(dep);
									}
								}
								for (Dependency dep : toRemove) disallowedDependencies.remove(dep);
							}
							computeCriticalSections();
							updateDependencies();							
						}

					}

					@Override
					public String[] onPositionUpdate() {
						if (viz != null && !trackingFinished && viz.periodicEnvelopeRefreshInMillis() > 0) {
							long timeNow = Calendar.getInstance().getTimeInMillis();
							if (timeNow-lastEnvelopeRefresh > viz.periodicEnvelopeRefreshInMillis()) {
								viz.addEnvelope(myTE);
								lastEnvelopeRefresh = timeNow;
							}
						}
						if (trackingCallbacks.containsKey(myTE.getRobotID())) return trackingCallbacks.get(myTE.getRobotID()).onPositionUpdate();
						return null;
					}

				};

				synchronized (trackers) {
					externalCPCounters.remove(trackers.get(te.getRobotID()));
					trackers.remove(te.getRobotID());

					//Make a new tracker for the driving trajectory envelope
					AbstractTrajectoryEnvelopeTracker tracker = getNewTracker(te, cb);
					trackers.put(te.getRobotID(), tracker);
					externalCPCounters.put(tracker, -1);
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
				ArrayList<PoseSteering> overallPath = new ArrayList<PoseSteering>();
				for (Mission m : e.getValue()) {						
					for (PoseSteering ps : m.getPath()) {
						overallPath.add(ps);
					}
				}

				//Create a big overall driving envelope
				TrajectoryEnvelope te = null;
				te = solver.createEnvelopeNoParking(robotID, overallPath.toArray(new PoseSteering[overallPath.size()]), "Driving", getFootprint(robotID));

				//Add mission stopping points
				synchronized(stoppingPoints) {
					for (int i = 0; i < e.getValue().size(); i++) {
						Mission m = e.getValue().get(i);
						for (Entry<Pose,Integer> entry : m.getStoppingPoints().entrySet()) {
							Pose stoppingPose = entry.getKey();
							int stoppingPoint = te.getSequenceNumber(new Coordinate(stoppingPose.getX(), stoppingPose.getY()));
							if (stoppingPoint == te.getPathLength()-1) stoppingPoint -= 2;
							int duration = entry.getValue();
							if (!stoppingPoints.keySet().contains(robotID)) {
								stoppingPoints.put(robotID, new ArrayList<Integer>());
								stoppingTimes.put(robotID, new ArrayList<Integer>());
							}
							if (!stoppingPoints.get(robotID).contains(stoppingPoint)) {
								stoppingPoints.get(robotID).add(stoppingPoint);
								stoppingTimes.get(robotID).add(duration);
							}
						}
					}
				
					//If many missions, add destinations as stopping points
					for (int i = 0; i < e.getValue().size()-1; i++) {
						Mission m = e.getValue().get(i);
						Pose destPose = m.getToPose();
						int stoppingPoint = te.getSequenceNumber(new Coordinate(destPose.getX(), destPose.getY()));
						if (!stoppingPoints.keySet().contains(robotID)) {
							stoppingPoints.put(robotID, new ArrayList<Integer>());
							stoppingTimes.put(robotID, new ArrayList<Integer>());
						}
						if (!stoppingPoints.get(robotID).contains(stoppingPoint)) {
							stoppingPoints.get(robotID).add(stoppingPoint);
							stoppingTimes.get(robotID).add(DEFAULT_STOPPING_TIME);
						}
					}
					if (stoppingPoints.get(robotID) != null) metaCSPLogger.info("Stopping points along trajectory for Robot" + robotID + ": " + stoppingPoints.get(robotID));
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
	 * Sets up a GUI which shows the current status of robots.
	 */
	public void setVisualization(FleetVisualization viz) {
		this.viz = viz;
	}

	private void setPriorityOfEDT(final int prio) {
		try {
			SwingUtilities.invokeAndWait(new Runnable() {
				public void run() {
					Thread.currentThread().setPriority(prio);
				}});
		}
		catch (InvocationTargetException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
		catch (InterruptedException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}		
	}

	/**
	 * Get the current {@link TrajectoryEnvelope} of a robot.
	 * @param robotID The robotID.
	 * @return The current {@link TrajectoryEnvelope} of a robot.
	 */
	public TrajectoryEnvelope getCurrentTrajectoryEnvelope(int robotID) {
		return trackers.get(robotID).getTrajectoryEnvelope();
	}

	protected String[] getStatistics() {
		
		String CONNECTOR_BRANCH = (char)0x251C + "" + (char)0x2500 + " ";
		String CONNECTOR_LEAF = (char)0x2514 + "" + (char)0x2500 + " ";
		ArrayList<String> ret = new ArrayList<String>();
		int numVar = solver.getConstraintNetwork().getVariables().length;
		int numCon = solver.getConstraintNetwork().getConstraints().length;
		
		synchronized (trackers) {

			ret.add("Status @ "  + getCurrentTimeInMillis() + " ms");
			ret.add(CONNECTOR_BRANCH + "Eff period ..... " + EFFECTIVE_CONTROL_PERIOD + " ms");
			ret.add(CONNECTOR_BRANCH + "Network ........ " + numVar + " variables, " + numCon + " constriants");
			HashSet<Integer> allRobots = new HashSet<Integer>();
			for (Integer robotID : trackers.keySet()) {
				allRobots.add(robotID);
			}
			String st = CONNECTOR_BRANCH + "Robots really .. ";
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
			st = CONNECTOR_BRANCH + "Robots view .... ";
			for (Integer robotID : allRobots) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				RobotReport rr = getRobotReport(robotID); 
				if (rr != null ) {
					int currentPP = rr.getPathIndex();
					st += tracker.te.getComponent();
					if (tracker instanceof TrajectoryEnvelopeTrackerDummy) st += " (P)";
					else st += " (D)";
					st += ": " + currentPP + "   ";
				}
			}
			ret.add(st);
		}
		synchronized (currentDependencies) {
			ret.add(CONNECTOR_BRANCH + "Dependencies ... " + currentDependencies);
			if (checkCollisions) {
				int numberOfCollisions = 0;
				synchronized (collisionsList) {
					numberOfCollisions = collisionsList.size();		
					if (numberOfCollisions>0) {
						ret.add(CONNECTOR_BRANCH + "Number of collisions ... " + numberOfCollisions + ".");
						for (CollisionEvent ce : collisionsList) {
							ret.add(CONNECTOR_BRANCH + " ....................... " + ce.toString());
						}
					}
					else
						ret.add(CONNECTOR_LEAF + "Number of collisions ... " + numberOfCollisions + ".");
				}
				
			}
			int totReTx = totalMsgsRetransmitted.get();
			int totTx = totalMsgsSent.get();
			ret.add(CONNECTOR_LEAF + "Transmission ..... reTx:" + totReTx + ", Tx: " + totTx + ", rate: " + ((totTx > 0) ? (float)totReTx/totTx : 0) + ".");
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
			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			public void run() {
				while (true) {
					synchronized (solver) {						
						if (!quiet) printStatistics();
						if (overlay) overlayStatistics();
						updateDependencies();
					}

					//Sleep a little...
					if (CONTROL_PERIOD > 0) {
						try { Thread.sleep(CONTROL_PERIOD); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}

					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
					EFFECTIVE_CONTROL_PERIOD = (int)(threadCurrentUpdate-threadLastUpdate);
					threadLastUpdate = threadCurrentUpdate;
					
					if (inferenceCallback != null) inferenceCallback.performOperation();

				}
			}
		};
		inference.setPriority(Thread.MAX_PRIORITY);
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

	/**
	 * Determine if a robot is free to accept a new mission (that is, the robot is in state WAITING_FOR_TASK).
	 * @param robotID The ID of the robot.
	 * @return <code>true</code> iff the robot is free to accept a new mission (that is, the robot is in state WAITING_FOR_TASK).
	 */
	public boolean isFree(int robotID) {
		synchronized (solver) {
			if (muted.contains(robotID)) return false;
			for (TrajectoryEnvelope te : envelopesToTrack) if (te.getRobotID() == robotID) return false;
			synchronized (trackers) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				if (!(tracker instanceof TrajectoryEnvelopeTrackerDummy)) return false;
				TrajectoryEnvelopeTrackerDummy trackerDummy = (TrajectoryEnvelopeTrackerDummy)tracker;
				return (!trackerDummy.isParkingFinished());
			}
		}
	}

	private static void printLicense() {
		System.out.println("\n"+TrajectoryEnvelopeCoordinator.TITLE);
		System.out.println(TrajectoryEnvelopeCoordinator.COPYRIGHT+"\n");
		if (TrajectoryEnvelopeCoordinator.LICENSE != null) {
			List<String> lic = StringUtils.fitWidth(TrajectoryEnvelopeCoordinator.PRIVATE_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		else {
			List<String> lic = StringUtils.fitWidth(TrajectoryEnvelopeCoordinator.PUBLIC_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		System.out.println();
	}
	
	public void computeCriticalSectionsAndStartTrackingAddedMission() {
		synchronized (solver) {
			computeCriticalSections();
			updateDependencies();
			startTrackingAddedMissions();
		}
	}

}
