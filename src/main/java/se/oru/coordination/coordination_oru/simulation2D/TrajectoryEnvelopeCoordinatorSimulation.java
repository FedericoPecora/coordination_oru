package se.oru.coordination.coordination_oru.simulation2D;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.atomic.AtomicInteger;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.UI.Callback;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import geometry_msgs.Point;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.CollisionEvent;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Dependency;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeTrackerDummy;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;

public class TrajectoryEnvelopeCoordinatorSimulation extends TrajectoryEnvelopeCoordinator {

	protected static final long START_TIME = Calendar.getInstance().getTimeInMillis();
	protected double MAX_VELOCITY;
	protected double MAX_ACCELERATION;
	protected int trackingPeriodInMillis;
	protected boolean useInternalCPs = true;
	
	protected boolean fake = false;
	protected boolean checkCollisions = false;
	protected ArrayList<CollisionEvent> collisionsList = new ArrayList<CollisionEvent>();
	protected Thread collisionThread = null;
	
	protected AtomicInteger totalMsgsLost = new AtomicInteger(0);
	protected AtomicInteger totalPacketsLost = new AtomicInteger(0);
	
	/**
	 * Enable the collision checking thread.
	 * @param enable <code>true</code>  if the thread for checking collisions should be enabled.
	 */
	public void setCheckCollisions(boolean enable) {
		this.checkCollisions = enable;
	}
		
	/**
	 * Enable fake coordination.
	 * @param enable <code>true</code> whether the coordinator does not impose any precedence constraints.
	 */
	public void setFakeCoordination(boolean fake) {
		this.fake = fake;
	}

	/** 
	 * Just for statistic purposes (simulation).
	 */
	public void incrementLostMsgsCounter() {
		this.totalMsgsLost.incrementAndGet();
	}
	
	/** 
	 * Just for statistic purposes (simulation).
	 */
	public void incrementLostPacketsCounter() {
		this.totalPacketsLost.incrementAndGet();
	}
	
	public int getTrackingPeriod() {
		return trackingPeriodInMillis;
	}

	public double getTemporalResolution() {
		return 1000.0;
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation() {
		this(1000, 1000, 10.0, 1.0, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(1000, 1000, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with the following default values:
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 * and given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
		this(CONTROL_PERIOD, TEMPORAL_RESOLUTION, 10.0, 1.0, 30);
	}

	private ArrayList<Integer> computeStoppingPoints(PoseSteering[] poses) {
		ArrayList<Integer> ret = new ArrayList<Integer>();
		double prevTheta = poses[0].getTheta();
		if (poses.length > 1) prevTheta = Math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX());
		for (int i = 0; i < poses.length-1; i++) {
			double theta = Math.atan2(poses[i+1].getY() - poses[i].getY(), poses[i+1].getX() - poses[i].getX());
			double deltaTheta = (theta-prevTheta);
			prevTheta = theta;
			if (Math.abs(deltaTheta) > Math.PI/2 && Math.abs(deltaTheta) < 1.9*Math.PI) {
				ret.add(i);
			}
		}
		return ret;
	}

	@Override
	public boolean addMissions(Mission... missions) {
		HashMap<Mission, HashMap<Pose, Integer>> userStoppingPoints = new HashMap<Mission, HashMap<Pose,Integer>>();
		if (this.useInternalCPs) {
			for (Mission m : missions) {
				PoseSteering[] path = m.getPath();
				ArrayList<Integer> sps = computeStoppingPoints(path);
				userStoppingPoints.put(m, m.getStoppingPoints());
				for (Integer i : sps) m.setStoppingPoint(path[i-1].getPose(), 100);				
			}
		}
		if (!super.addMissions(missions)) {
			if (this.useInternalCPs) {
				for (Mission m : missions) {
					m.clearStoppingPoints();
					for (Pose p : userStoppingPoints.get(m).keySet()) {
						m.setStoppingPoint(p, userStoppingPoints.get(m).get(p));
					}
				}			
			}
			return false;
		}
		return true;
	}


	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulation} with given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 * @param MAX_VELOCITY The maximum speed of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param MAX_ACCELERATION The maximum acceleration/deceleration of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param trackingPeriodInMillis The tracking period in milliseconds (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 */
	public TrajectoryEnvelopeCoordinatorSimulation(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION, int trackingPeriodInMillis) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION);
		this.MAX_VELOCITY = MAX_VELOCITY;
		this.MAX_ACCELERATION = MAX_ACCELERATION;
		this.trackingPeriodInMillis = trackingPeriodInMillis;
	}

	/**
	 * Enable (default) or disable the use of internal critical points in the {@link TrajectoryEnvelopeTrackerRK4} trackers.
	 * @param value <code>true</code> if these critical points should be used to slow down, <code>false</code> otherwise.
	 */
	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}

	@Override
	public AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb) {

		TrajectoryEnvelopeTrackerRK4 ret = new TrajectoryEnvelopeTrackerRK4(te, trackingPeriodInMillis, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, this, cb) {

			//Method for measuring time in the trajectory envelope tracker
			@Override
			public long getCurrentTimeInMillis() {
				return tec.getCurrentTimeInMillis();
			}
		};
		//ret.setUseInternalCriticalPoints(this.useInternalCPs);
		ret.setUseInternalCriticalPoints(false);
		return ret;
	}

	//Method for measuring time in the trajectory envelope coordinator
	@Override
	public long getCurrentTimeInMillis() {
		return Calendar.getInstance().getTimeInMillis()-START_TIME;
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
			String st = CONNECTOR_BRANCH + "Robots really .. ";
			for (Integer robotID : allRobots) {
				AbstractTrajectoryEnvelopeTracker tracker = trackers.get(robotID);
				RobotReport rr = tracker.getRobotReport(); 
				int currentPP = rr.getPathIndex();
				st += tracker.getTrajectoryEnvelope().getComponent();
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
		if (checkCollisions) {
			int numberOfCollisions = 0;
			synchronized (collisionsList) {
				numberOfCollisions = collisionsList.size();		
				ret.add(CONNECTOR_BRANCH + "Number of collisions ... " + numberOfCollisions + ".");
				if (numberOfCollisions>0) {
					for (CollisionEvent ce : collisionsList) {
						ret.add(CONNECTOR_BRANCH + " ....................... " + ce.toString());
					}
				}
			}
			
		}
		ret.add(CONNECTOR_BRANCH + "Total number of obsolete critical sections ... " + criticalSectionCounter.get() + ".");
		ret.add(CONNECTOR_BRANCH + "Total messages sent: ... " + totalMsgsSent.get() + ", lost: " + totalMsgsLost.get() + ", retransmitted: " + totalMsgsReTx.get() + ". Packets lost: " + totalPacketsLost.get() + ", number of replicas: " + numberOfReplicas + ".");
		ret.add(CONNECTOR_BRANCH + "Total unalive states detected: ... " + nonliveStatesDetected.get() + ", avoided: " + nonliveStatesAvoided.get() + ", revised according to heuristic: " + currentOrdersHeurusticallyDecided.get() + ".");
		ret.add(CONNECTOR_LEAF + "Total re-planned path: ... " + replanningTrialsCounter.get() + ", successful: " + successfulReplanningTrialsCounter.get() + ".");
		return ret.toArray(new String[ret.size()]);
	}
	
	@Override
	public void onCriticalSectionUpdate() {
		
		if (checkCollisions && (collisionThread == null) && (this.allCriticalSections.size() > 0)) {
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
								(robotReport1.getPathIndex() <= cs.getTe1End()) && (robotReport1.getPathIndex() >= cs.getTe1Start()) && //robot1 is inside
								(robotReport2.getPathIndex() <= cs.getTe2End()) && (robotReport2.getPathIndex() >= cs.getTe2Start())  	//robot2 is inside
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
	
	@Override
	protected void updateDependencies() {
		synchronized(solver) {
			if (this.fake) {
				for (int robotID : trackers.keySet()) setCriticalPoint(robotID, -1, true);
				return;
			}
			if (this.avoidDeadlockGlobally.get()) globalCheckAndRevise();
			else localCheckAndRevise(); 
			
		}
	}
}
