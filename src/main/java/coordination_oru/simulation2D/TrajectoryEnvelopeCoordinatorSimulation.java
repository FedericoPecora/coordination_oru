package coordination_oru.simulation2D;

import java.util.Calendar;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import coordination_oru.AbstractTrajectoryEnvelopeTracker;
import coordination_oru.Dependency;
import coordination_oru.RobotReport;
import coordination_oru.TrackingCallback;
import coordination_oru.TrajectoryEnvelopeCoordinator;

public class TrajectoryEnvelopeCoordinatorSimulation extends TrajectoryEnvelopeCoordinator {

	protected static final long START_TIME = Calendar.getInstance().getTimeInMillis();

	// -- the control period of the coordinator
	protected int CONTROL_PERIOD;
	// -- the temporal resolution in which times are expressed (milliseconds)
	protected double TEMPORAL_RESOLUTION;
	// -- the maximum velocity of a robot
	protected double MAX_VELOCITY;
	// -- the maximum acceleration (-deceleration) of a robot
	protected double MAX_ACCELERATION;
	// -- the tracking period of trajectory envelope trackers
	protected int trackingPeriodInMillis;			
	// -- the minimum duration of parking
	protected long PARKING_DURATION;
	
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
		this.CONTROL_PERIOD = 1000;
		this.TEMPORAL_RESOLUTION = 1000;
		this.MAX_VELOCITY = 10.0;
		this.MAX_ACCELERATION = 1.0;
		this.trackingPeriodInMillis = 30;			
		this.PARKING_DURATION = 3000;
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
		this.CONTROL_PERIOD = CONTROL_PERIOD;
		this.TEMPORAL_RESOLUTION = TEMPORAL_RESOLUTION;
		this.MAX_VELOCITY = MAX_VELOCITY;
		this.MAX_ACCELERATION = MAX_ACCELERATION;
		this.trackingPeriodInMillis = trackingPeriodInMillis;
		this.PARKING_DURATION = 3000;
	}

	@Override
	public AbstractTrajectoryEnvelopeTracker getNewTracker(TrajectoryEnvelope te, TrackingCallback cb) {
		
		TrajectoryEnvelopeTrackerRK4 ret = new TrajectoryEnvelopeTrackerRK4(te, trackingPeriodInMillis, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, getSolver(), cb) {
			
			//What should happen when a robot reaches a new pose along the path
			//In this implementation, simply update the GUI
			@Override
			public void onPositionUpdate() {
				
				//Update the position of the robot in the GUI
				RobotReport rr = getRobotReport();
				double x = rr.getPose().getX();
				double y = rr.getPose().getY();
				double theta = rr.getPose().getTheta();
				panel.addGeometry("R" + te.getRobotID(), TrajectoryEnvelope.getFootprint(te.getFootprint(), x, y, theta), false, true, false, "#FF0000");
				
				//Draw an arrow if there is a critical point
				RobotReport rrWaiting = getRobotReport();
				synchronized (currentDependencies) {
					for (Dependency dep : currentDependencies) {
						if (dep.getWaitingTracker().equals(this)) {
							if (dep.getDrivingTracker() != null) {
								RobotReport rrDriving = dep.getDrivingTracker().getRobotReport();
								String arrowIdentifier = "_"+dep.getWaitingRobotID()+"-"+dep.getDrivingRobotID();
								panel.addArrow(arrowIdentifier, rrWaiting.getPose(), rrDriving.getPose());
							}
							else {
								String arrowIdentifier = "_"+dep.getWaitingRobotID()+"-"+dep.getDrivingRobotID();
								panel.addArrow(arrowIdentifier, rrWaiting.getPose(), traj.getPose()[rrWaiting.getCriticalPoint()]);										
							}
						}
					}							
				}

				//Refresh the GUI
				panel.updatePanel();
			}

			//Method for measuring time in the trajectory envelope tracker
			@Override
			public long getCurrentTimeInMillis() {
				return Calendar.getInstance().getTimeInMillis()-START_TIME;
			}
		};
		return ret;
	}

	//Method for measuring time in the trajectory envelope coordinator
	@Override
	public long getCurrentTimeInMillis() {
		return Calendar.getInstance().getTimeInMillis()-START_TIME;
	}

	//Criteria according to which envelopes (i.e., the robots that drive them) precede each other
	@Override
	public Comparator<TrajectoryEnvelope> getOrdering() {
		Comparator<TrajectoryEnvelope> comp = new  Comparator<TrajectoryEnvelope>() {
			@Override
			public int compare(TrajectoryEnvelope o1, TrajectoryEnvelope o2) {
				return o1.getRobotID()-o2.getRobotID();
			}
		};
		return comp;
	}

}