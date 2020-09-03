package se.oru.coordination.coordination_oru;

import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;

/**
 * This {@link AbstractTrajectoryEnvelopeTracker} is used to represent static robots, that is, robots
 * that are parked in a given pose.
 * 
 * @author fpa
 *
 */
public abstract class TrajectoryEnvelopeTrackerDummy extends AbstractTrajectoryEnvelopeTracker implements Runnable {
	
	private Thread th = null;
	private boolean parkingFinished = false;
	private int currentIndex = -1;
	private long DELTA_FUTURE = 0;
	
	/**
	 * Create a new {@link TrajectoryEnvelopeTrackerDummy} representing that a robot with a given ID is parked
	 * in a particular {@link TrajectoryEnvelope}.
	 * @param te The {@link TrajectoryEnvelope} where the robot is parked.
	 * @param timeStep The period at which the tracker should check whether it has "finished" parking. 
	 * @param temporalResolution The temporal resolution in which the timeStep is expressed.
	 * @param solver The {@link TrajectoryEnvelopeSolver} that maintains the parking {@link TrajectoryEnvelope}.
	 * @param cb An optional callback that will be called during tracking.
	 */
	public TrajectoryEnvelopeTrackerDummy(TrajectoryEnvelope te, int timeStep, double temporalResolution, AbstractTrajectoryEnvelopeCoordinator tec, TrackingCallback cb) {
		super(te, temporalResolution, tec, timeStep, cb);
		this.te = te;
		this.traj = te.getTrajectory();
		this.temporalResolution = temporalResolution;
		this.th = new Thread(this, "Parking tracker " + te.getComponent());
		this.th.start();
	}
	
	@Override
	protected void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te) { }
	
	@Override
	public void startTracking() { }
	
	@Override
	protected void setCriticalPoint(int criticalPointToSet) { }
		
	@Override
	public RobotReport getRobotReport() {
		return new RobotReport(te.getRobotID(), traj.getPose()[0], currentIndex, 0.0, 0.0, -1);
	}

	
	/**
	 * Instructs the {@link TrajectoryEnvelopeSolver} that the robot has ceased to be parked here.
	 */
	public void finishParking() {		
		this.parkingFinished = true;
		synchronized(th) {
			th.notify();
		}
	}
	
	/**
	 * Assesses whether the robot is still parked here.
	 * @return <code>true</code> iff the robot is still parked.
	 */
	public boolean isParkingFinished() {
		return parkingFinished;
	}
	
//	private boolean canInterrupt() {
//		for (AllenIntervalConstraint meets : getConstriants(AllenIntervalConstraint.Type.Meets, te, tec.getSolver())) {
//			TrajectoryEnvelope driving = (TrajectoryEnvelope)meets.getTo();
//			AllenIntervalConstraint[] startsCons = getConstriants(AllenIntervalConstraint.Type.Starts, driving, tec.getSolver());
//			TrajectoryEnvelope drivingFirstSubEnv = driving;
//			if (startsCons != null && startsCons.length > 0) drivingFirstSubEnv = (TrajectoryEnvelope)startsCons[0].getFrom();
//			for (AllenIntervalConstraint before : getConstriants(AllenIntervalConstraint.Type.BeforeOrMeets, drivingFirstSubEnv, tec.getSolver())) {
//				//All the befores that involve the next driving
//				if (before.getTo().equals(drivingFirstSubEnv)) {
//					//Someone depends on when this parking finishes, so cannot interrupt!
//					metaCSPLogger.info("Robot " + te.getRobotID() + " should wait to end parking: " + before);
//					return false;
//				}
//			}
//		}
//		//Nobody depends on when this parking finishes, so can interrupt now...
//		metaCSPLogger.info("Robot " + te.getRobotID() + " can end parking (end time bounds are [" + te.getTemporalVariable().getEET() + ", " + te.getTemporalVariable().getLET() + "])");
//		return true;
//	}
	
	@Override
	public void run() {

		//Just do prolong the earliest end time until finished by external call to finishParking()
		metaCSPLogger.info("Parking starts for Robot " + te.getRobotID());
		updateDeadline(this.te, DELTA_FUTURE);
		onPositionUpdate();
		/*while (!parkingFinished) {
			updateDeadline(this.te, DELTA_FUTURE);
			onPositionUpdate();
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}*/		
		synchronized(th) {
			try {
				th.wait();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				metaCSPLogger.severe(e.toString());
			}
		}
		metaCSPLogger.info("Parking finishes for Robot " + te.getRobotID());
		
		fixDeadline(te, 0);
		
//		//Now wait until earliest end time (as there could be some driving envelope after this which should not start)
//		while (this.te.getTemporalVariable().getEET() > getCurrentTimeInMillis()) {
//			
//			//Find out if nobody is waiting, in which case break early!			
//			synchronized(solver) {
//				if (canInterrupt()) {
//					fixDeadline(te, 0);
//					break;
//				}
//			}
//			
//			try { Thread.sleep(trackingPeriodInMillis); }
//			catch (InterruptedException e) { e.printStackTrace(); }
//		}
		
		//Deadline will now be fixed (by superclass) so it is decided that this parking cannot be prolonged
		//(and thus scheduling will not be able to move FW any driving envelopes that are met by this parking)
		
		//Signals thread in abstract tracker to stop
		currentIndex++;
		
	}
	
	@Override
	public void onPositionUpdate() {
		if (tec.getVisualization() != null) tec.getVisualization().displayRobotState(te.getFootprint(), getRobotReport());
	}

	
}