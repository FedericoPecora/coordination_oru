package se.oru.coordination.coordination_oru.simulation2D;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;


public class TrajectoryEnvelopeCoordinatorSimulationWithPedestrians extends TrajectoryEnvelopeCoordinator {

	protected static final long START_TIME = Calendar.getInstance().getTimeInMillis();
	protected double MAX_VELOCITY;
	protected double MAX_ACCELERATION;
	protected int trackingPeriodInMillis;
	protected boolean useInternalCPs = true;
	
	protected HashMap<Integer, PedestrianTrajectory> pedestrianTrajectoryMap = new HashMap<Integer, PedestrianTrajectory>();

	public void addPedestrianTrajectory(int robotID, PedestrianTrajectory traj) {
		pedestrianTrajectoryMap.put(robotID, traj);
	}
	public int getTrackingPeriod() {
		return trackingPeriodInMillis;
	}

	public double getTemporalResolution() {
		return 1000.0;
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulationWithPedestrians} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulationWithPedestrians() {
		this(1000, 1000, 10.0, 1.0, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulationWithPedestrians} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulationWithPedestrians} with the following default values:
	 * <ul>
	 * <li><code>CONTROL_PERIOD</code> = 1000</li>
	 * <li><code>TEMPORAL_RESOLUTION</code> = 1000</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 */
	public TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(double MAX_VELOCITY, double MAX_ACCELERATION) {
		this(1000, 1000, MAX_VELOCITY, MAX_ACCELERATION, 30);
	}

	/**
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulationWithPedestrians} with the following default values:
	 * <li><code>MAX_VELOCITY</code> = 10.0</li>
	 * <li><code>MAX_ACCELERATION</code> = 1.0</li>
	 * <li><code>trackingPeriodInMillis</code> = 30</li>
	 * <li><code>PARKING_DURATION</code> = 3000</li>
	 * </ul>
	 * and given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 */
	public TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
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
	 * Create a new {@link TrajectoryEnvelopeCoordinatorSimulationWithPedestrians} with given parameters.
	 * @param CONTROL_PERIOD The control period of the coordinator (e.g., 1000 msec)
	 * @param TEMPORAL_RESOLUTION The temporal resolution at which the control period is specified (e.g., 1000)
	 * @param MAX_VELOCITY The maximum speed of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param MAX_ACCELERATION The maximum acceleration/deceleration of a robot (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 * @param trackingPeriodInMillis The tracking period in milliseconds (used to appropriately instantiate the {@link TrajectoryEnvelopeTrackerRK4} instances).
	 */
	public TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION, int trackingPeriodInMillis) {
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
		
		AbstractTrajectoryEnvelopeTracker ret = null;

		// This needs to change later. For now all uncontrollable entities are pedestrians. 
		if(this.pedestrianTrajectoryMap.containsKey(te.getRobotID()))  {
			ret = new TrajectoryEnvelopeTrackerPedestrian(te, trackingPeriodInMillis, TEMPORAL_RESOLUTION, this, cb, this.pedestrianTrajectoryMap.get(te.getRobotID())); }
		else {
			ret = new TrajectoryEnvelopeTrackerRK4(te, trackingPeriodInMillis, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, this, cb) {

				//Method for measuring time in the trajectory envelope tracker
				@Override
				public long getCurrentTimeInMillis() {
					return Calendar.getInstance().getTimeInMillis()-START_TIME;
				}
			};
		}
		return ret;
	}

	//Method for measuring time in the trajectory envelope coordinator
	@Override
	public long getCurrentTimeInMillis() {
		return Calendar.getInstance().getTimeInMillis()-START_TIME;
	}
	
//	@Override
//	protected PoseSteering[] doReplanning(Pose fromPose, Pose toPose, Geometry... obstaclesToConsider) {
//		if (this.defaultMotionPlanner == null) return null;
//		defaultMotionPlanner.setStart(fromPose);
//		defaultMotionPlanner.setGoals(toPose);
//		defaultMotionPlanner.clearObstacles();
//		if (obstaclesToConsider != null && obstaclesToConsider.length > 0) defaultMotionPlanner.addObstacles(obstaclesToConsider);
//		if (defaultMotionPlanner.plan()) return defaultMotionPlanner.getPath();
//		return null;
//	}

}
