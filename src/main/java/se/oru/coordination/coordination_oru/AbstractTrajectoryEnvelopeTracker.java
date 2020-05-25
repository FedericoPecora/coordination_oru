package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.logging.Logger;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.Variable;
import org.metacsp.meta.spatioTemporal.paths.Map;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;

import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4;

/**
 * This class provides the basic functionalities of a {@link TrajectoryEnvelope} tracker. Implementing
 * this class is equivalent to providing the interface to a particular robot system. An example implementation
 * is the {@link TrajectoryEnvelopeTrackerRK4} class, which implements a simulated robot whose motion is tracked via
 * Runge-Kutta numerical integration (RK4).
 * 
 * @author fpa
 *
 */
public abstract class AbstractTrajectoryEnvelopeTracker {

	protected AbstractTrajectoryEnvelopeCoordinator tec = null;
	protected TrajectoryEnvelope te = null;
	protected Trajectory traj = null;
	protected double temporalResolution = 0.0;
	protected Integer externalCPCounter = -1;
	protected Integer reportCounter = -1;
	protected int criticalPoint = -1;
	protected HashSet<TrajectoryEnvelope> startedGroundEnvelopes = new HashSet<TrajectoryEnvelope>();
	protected HashSet<TrajectoryEnvelope> finishedGroundEnvelopes = new HashSet<TrajectoryEnvelope>();
	protected HashMap<TrajectoryEnvelope,AllenIntervalConstraint> deadlines = new HashMap<TrajectoryEnvelope, AllenIntervalConstraint>();
	protected int trackingPeriodInMillis = 0;
	protected TrackingCallback cb = null;
	protected Map mapMetaConstraint = null; 
	protected boolean calledOnTrackingStart = false;
	protected boolean calledStartTracking = false;
	protected Callback extraStatusCallback = null;
	protected boolean canStartTracking = false;
	protected long startingTimeInMillis = -1;
	
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(AbstractTrajectoryEnvelopeTracker.class);

	/**
	 * Create a new {@link AbstractTrajectoryEnvelopeTracker} to track a given {@link TrajectoryEnvelope},
	 * with a given tracking period in a given temporal resolution. The tracker will post temporal constraints
	 * to the given solver representing when the robot transitions from one sub-envelope to the next. An optional
	 * callback function will be called at every period.
	 * @param te The {@link TrajectoryEnvelope} to track.
	 * @param temporalResolution The temporal unit of measure in which the period is represented. 
	 * @param solver The {@link TrajectoryEnvelopeSolver} to which temporal constraints will be posted.
	 * @param trackingPeriodInMillis The tracking period.
	 * @param cb An optional callback function.
	 */
	public AbstractTrajectoryEnvelopeTracker(TrajectoryEnvelope te, double temporalResolution, AbstractTrajectoryEnvelopeCoordinator tec, int trackingPeriodInMillis, TrackingCallback cb) {
		this.te = te;
		this.traj = te.getTrajectory();
		this.externalCPCounter = -1;
		this.criticalPoint = -1;
		this.temporalResolution = temporalResolution;
		this.startingTimeInMillis = tec.getCurrentTimeInMillis();
		this.tec = tec;
		this.trackingPeriodInMillis = trackingPeriodInMillis;
		this.cb = cb;
		startMonitoringThread();
	}
	
	/**
	 * Return the coordination time (in milli-seconds) at which the tracker has started its mission.
	 */
	public long getStartingTimeInMillis() {
		return this.startingTimeInMillis;
	}
	
	/**
	 * Return the coordination time (in milli-seconds) at which the tracker has started its mission.
	 */
	public void resetStartingTimeInMillis() {
		this.startingTimeInMillis= tec.getCurrentTimeInMillis();
	}
	
	/**
	 * Operations that should be performed when the {@link TrajectoryEnvelope} of this tracker
	 * is updated online.
	 * @param te The updated {@link TrajectoryEnvelope}.
	 */
	protected abstract void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te);
	
	/**
	 * Update the {@link TrajectoryEnvelope} of this tracker (used for truncating/reversing/re-planning {@link TrajectoryEnvelope}s online).
	 * This method calls the {@link #onTrajectoryEnvelopeUpdate(TrajectoryEnvelope)} method of the particular implementation
	 * of this {@link AbstractTrajectoryEnvelopeTracker} class.
	 * @param te The new {@link TrajectoryEnvelope} of this tracker.
	 */
	public void updateTrajectoryEnvelope(TrajectoryEnvelope te) {
		synchronized(tec.getSolver()) {
			metaCSPLogger.info("Updating trajectory Robot" +this.te.getRobotID()+". TEID: " + this.te.getID() + "--> TEID: " + te.getID()+ ".");
			this.te = te;
			this.cb.updateTrajectoryEnvelope(te);
			this.traj = te.getTrajectory();
			this.onTrajectoryEnvelopeUpdate(te);
		}
	}
	
	@Deprecated
	public void setMapMetaConstraint(Map mapMetaConstraint) {
		this.mapMetaConstraint = mapMetaConstraint;
	}
	
	/**
	 * Used by the {@link TrajectoryEnvelopeCoordinator} to indicate that this tracker can start the tracking thread.
	 */
	public void setCanStartTracking() {
		this.canStartTracking = true;
	}

	/**
	 * Indicates whether this tracker can start the tracking thread
	 * (this is set to true by the {@link TrajectoryEnvelopeCoordinator} when appropriate).
	 */
	public boolean canStartTracking() {
		return this.canStartTracking;
	}
	
	/**
	 * Determines if the robot has entered a particular sub-{@link TrajectoryEnvelope}. 
	 * @param env A sub-{@link TrajectoryEnvelope} of this tracker's {@link TrajectoryEnvelope}.
	 * @return <code>true</code> iff the robot has entered the given sub-{@link TrajectoryEnvelope}.
	 */
	public boolean isStarted(TrajectoryEnvelope env) {
		return this.startedGroundEnvelopes.contains(env);
	}

	/**
	 * Determines whether this tracker tracks a given {@link TrajectoryEnvelope}.
	 * @param env The {@link TrajectoryEnvelope} to check.
	 * @return <code>true</code> iff this tracker tracks the given {@link TrajectoryEnvelope}.
	 */
	public boolean tracksEnvelope(TrajectoryEnvelope env) {
		for (TrajectoryEnvelope subEnv : this.getAllSubEnvelopes()) {
			if (subEnv.equals(env)) return true;
		}
		return false;
	}

	/**
	 * Determines if the robot has exited a particular sub-{@link TrajectoryEnvelope}. 
	 * @param env A sub-{@link TrajectoryEnvelope} of this tracker's {@link TrajectoryEnvelope}.
	 * @return <code>true</code> iff the robot has exited the given sub-{@link TrajectoryEnvelope}.
	 */
	public boolean isFinished(TrajectoryEnvelope env) {
		if (!this.tracksEnvelope(env)) return true;
		return this.finishedGroundEnvelopes.contains(env);
	}

	/**
	 * This method should implement the mechanisms for notifying a robot of a new critical point.
	 * @param criticalPoint The critical point to set (index of pose along the reference trajectory
	 * beyond which the robot may not navigate). 
	 */
	protected abstract void setCriticalPoint(int criticalPoint);
	
	/**
	 * This method should implement the mechanisms for notifying a robot of a new critical point, caring about network delays.
	 * Critical point are "timestamped" so that only critical points more recent than the already known will be notified.
	 * @param criticalPoint The critical point to set (index of pose along the reference trajectory
	 * beyond which the robot may not navigate). 
	 * @param externalCPCounter A counter related to the current notification ("timestamp").
	 */
	public void setCriticalPoint(int criticalPointToSet, int externalCPCounter) {

		if (
				(externalCPCounter < this.externalCPCounter && externalCPCounter-this.externalCPCounter > Integer.MAX_VALUE/2.0) ||
				(this.externalCPCounter > externalCPCounter && this.externalCPCounter-externalCPCounter < Integer.MAX_VALUE/2.0)) {
			metaCSPLogger.info("Ignored critical point " + criticalPointToSet + " related to counter " + externalCPCounter + " because counter is already at " + this.externalCPCounter + ".");
			return;
		}
		setCriticalPoint(criticalPointToSet);
		this.externalCPCounter = externalCPCounter;
		
		
	}
	
	/**
	 * Set the current report counter.
	 * @param The current report counter.
	 */
	public void setReportCounter(int reportCounter) {
		this.reportCounter = reportCounter;
	}
	
	/**
	 * Returns the current report counter.
	 * @return The current report counter.
	 */
	public int getReportCounter() {
		return this.reportCounter;
	}
	
	/**
	 * Returns the current critical point.
	 * @return The current critical point.
	 */
	public int getCriticalPoint() {
		return this.criticalPoint;
	}
	
	/**
	 * Returns the tracking period in millis.
	 * @return The the tracking period in millis.
	 */
	public int getTrackingPeriodInMillis() {
		return this.trackingPeriodInMillis;
	}
	
	/**
	 * This method should return a {@link RobotReport}, describing the last known state of the robot.
	 * @return A {@link RobotReport}, describing the last known state state of the robot.
	 */
	public RobotReport getLastRobotReport() {
		return getRobotReport();
	}
	
	/**
	 * This method should return a {@link RobotReport}, describing the current state of the robot.
	 * @return A {@link RobotReport}, describing the current state of the robot.
	 */
	public abstract RobotReport getRobotReport();

	protected void onPositionUpdate() {
	
		String[] extraRobotState = null;
		if (cb != null) {
			extraRobotState = cb.onPositionUpdate();
		}

		if (tec.getVisualization() != null) {
			//Update the position of the robot in the GUI
			RobotReport rr = getRobotReport();
			tec.getVisualization().displayRobotState(te.getFootprint(), rr, extraRobotState);
			
			//Draw an arrow if there is a critical point
			RobotReport rrWaiting = getRobotReport();
			synchronized (tec.getCurrentDependencies()) {
				for (Dependency dep : tec.getCurrentDependencies()) {
					synchronized (tec.trackers) {
						AbstractTrajectoryEnvelopeTracker waitingTrackers = tec.trackers.get(dep.getWaitingRobotID());
						AbstractTrajectoryEnvelopeTracker drivingTrackers = tec.trackers.get(dep.getDrivingRobotID());
						if (waitingTrackers.equals(this)) {
							if (drivingTrackers != null) {
								RobotReport rrDriving = drivingTrackers.getRobotReport();
								String arrowIdentifier = "_"+dep.getWaitingRobotID()+"-"+dep.getDrivingRobotID();
								tec.getVisualization().displayDependency(rrWaiting, rrDriving, arrowIdentifier);
							}
						}
					}
				}							
			}

			tec.getVisualization().updateVisualization();
		}
	}
	
	/**
	 * Should return the current time in milliseconds.
	 * @return The current time in milliseconds.
	 */
	public abstract long getCurrentTimeInMillis();

	protected static AllenIntervalConstraint[] getConstriants(AllenIntervalConstraint.Type type, TrajectoryEnvelope env, TrajectoryEnvelopeSolver solver) {
		ArrayList<AllenIntervalConstraint> ret = new ArrayList<AllenIntervalConstraint>();
		Constraint[] incidentEdges = solver.getConstraintNetwork().getIncidentEdges(env);
		if (incidentEdges != null) {
			for (Constraint con : incidentEdges) {
				AllenIntervalConstraint aic = (AllenIntervalConstraint)con;
				if (aic.getFrom().equals(env) || aic.getTo().equals(env)) {
					if (aic.getTypes()[0].equals(type)) ret.add(aic);
				}
			}
		}
		return ret.toArray(new AllenIntervalConstraint[ret.size()]);
	}
	
	protected void updateDeadline(TrajectoryEnvelope trajEnv, long delta) {
		synchronized(tec.getSolver()) {
			long time = getCurrentTimeInMillis()+delta;
			if (time > trajEnv.getTemporalVariable().getEET()) {
				tec.getSolver().removeConstraint(deadlines.get(trajEnv));
				long bound1 = Math.max(time, trajEnv.getTemporalVariable().getEET());
				//long bound1 = time;
				long bound2 = APSPSolver.INF;
				//metaCSPLogger.info("Finishing @ " + time + " " + trajEnv + " (ET bounds: [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() + "])");
				AllenIntervalConstraint deadline = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Deadline, new Bounds(bound1, bound2));
				deadline.setFrom(trajEnv);
				deadline.setTo(trajEnv);
				boolean added = tec.getSolver().addConstraint(deadline);
				if (!added) {
					metaCSPLogger.severe("ERROR: Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
					throw new Error("Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
				}
				else deadlines.put(trajEnv, deadline);
			}
		}
	}

	protected void fixDeadline(TrajectoryEnvelope trajEnv, long delta) {
		synchronized(tec.getSolver()) {
			long time = getCurrentTimeInMillis()+delta;
			if (time > trajEnv.getTemporalVariable().getEET()) {
				tec.getSolver().removeConstraint(deadlines.get(trajEnv));
				long bound1 = Math.max(time, trajEnv.getTemporalVariable().getEET());
				//long bound1 = time;
				long bound2 = bound1;
				metaCSPLogger.info("Finishing @ " + time + " " + trajEnv + " (ET bounds: [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() + "])");
				AllenIntervalConstraint deadline = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Deadline, new Bounds(bound1, bound2));
				deadline.setFrom(trajEnv);
				deadline.setTo(trajEnv);
				boolean added = tec.getSolver().addConstraint(deadline);
				if (!added) {
					metaCSPLogger.severe("ERROR: Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
					throw new Error("Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
				}
				else deadlines.put(trajEnv, deadline);
			}
		}
	}

	protected void setRelease(TrajectoryEnvelope trajEnv) {
		synchronized(tec.getSolver()) {
			long time = getCurrentTimeInMillis();
			time = Math.max(time, trajEnv.getTemporalVariable().getEST());
			metaCSPLogger.info("Releasing @ " + time + " " + trajEnv + " (ST bounds: [" + trajEnv.getTemporalVariable().getEST() + "," + trajEnv.getTemporalVariable().getLST() + "])");
			AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(time, time));
			release.setFrom(trajEnv);
			release.setTo(trajEnv);
			boolean added = tec.getSolver().addConstraint(release);
			if (!added) {
				metaCSPLogger.severe("ERROR: Could not add release " + release + " constraint on envelope " + trajEnv + " whose ST bounds are [" + trajEnv.getTemporalVariable().getEST() + "," + trajEnv.getTemporalVariable().getLST() +"]");
				throw new Error("Could not add release " + release + " constraint on envelope " + trajEnv + " whose ST bounds are [" + trajEnv.getTemporalVariable().getEST() + "," + trajEnv.getTemporalVariable().getLST() +"]");
			}
		
		}
		
	}
	
	
	
	private TrajectoryEnvelope[] getAllSubEnvelopes() {
		Variable[] allVars = te.getRecursivelyDependentVariables();
		TrajectoryEnvelope[] allSubEnvelopes = new TrajectoryEnvelope[allVars.length];
		for (int i = 0; i < allVars.length; i++) {
			allSubEnvelopes[i] = (TrajectoryEnvelope)allVars[i];
		}
		return allSubEnvelopes;
	}
		
	/**
	 * Specifies what happens when tracking starts.
	 */
	public abstract void startTracking();
	
//	private void printStartedGroundEnvelopes() {
//		metaCSPLogger.info("*** STARTED GTEs of Robot " + te.getRobotID());
//		for (TrajectoryEnvelope sse : startedGroundEnvelopes) {
//			metaCSPLogger.info("\t*** " + sse + " TV: " + sse.getTemporalVariable());
//		}
//	}
	
	/**
	 * Returns <code>true</code> iff tracking has started.
	 * @return <code>true</code> iff tracking has started.
	 */
	public boolean trackingStrated() {
		return calledStartTracking;
	}
	
	protected void startMonitoringThread() {
				
		//Start a thread that monitors the sub-envelopes and finishes them when appropriate
		Thread monitorSubEnvelopes = new Thread("Abstract tracker " + te.getComponent()) {
			@Override
			public void run() {	

				int prevSeqNumber = -1;

				if (cb != null) cb.beforeTrackingStart();

				//Monitor the sub-envelopes...
				while (true) {
					
					//Track if past start time
					if (te.getTemporalVariable().getEST() <= getCurrentTimeInMillis()) {
		
						if (cb != null && !calledOnTrackingStart) {
							calledOnTrackingStart = true;
							cb.onTrackingStart();
						}
						
						if (!calledStartTracking) {
							calledStartTracking = true;							
							startTracking();
						}

						//if (!startedGroundEnvelopes.isEmpty()) printStartedGroundEnvelopes();
						
						RobotReport rr = null;
						while ((rr = tec.getRobotReport(te.getRobotID())) == null) {
							metaCSPLogger.info("(waiting for "+te.getComponent()+"'s tracker to come online)");
							try { Thread.sleep(100); }
							catch (InterruptedException e) { e.printStackTrace(); }
						}

						//Get current sequence number from robot report...
						int currentSeqNumber = rr.getPathIndex();
	
						//Get all ground envelopes of this super-envelope that are not finished (except the last one)...
						for (TrajectoryEnvelope subEnv : getAllSubEnvelopes()) {
							if (subEnv.hasSuperEnvelope()) {
								if (subEnv.getSequenceNumberStart() <= currentSeqNumber && !startedGroundEnvelopes.contains(subEnv)) {
									startedGroundEnvelopes.add(subEnv);
									metaCSPLogger.info(">>>> Dispatched (ground envelope) " + subEnv);
									if (cb != null) cb.onNewGroundEnvelope();
								}
								if (subEnv.getSequenceNumberEnd() < currentSeqNumber && !finishedGroundEnvelopes.contains(subEnv)) {
									finishedGroundEnvelopes.add(subEnv);
									metaCSPLogger.info("<<<< Finished (ground envelope) " + subEnv);
									if (subEnv.getSequenceNumberEnd() < te.getSequenceNumberEnd()) fixDeadline(subEnv, 0);
								}
								else if (!finishedGroundEnvelopes.contains(subEnv) && currentSeqNumber > prevSeqNumber) {
									updateDeadline(subEnv, 0);
								}
							}							
						}
						
						//Stop when last path point reached (or we missed that report and the path point is now 0)
						if (te.getSequenceNumberEnd() == currentSeqNumber || (currentSeqNumber < prevSeqNumber && currentSeqNumber <= 0)) {
							metaCSPLogger.info("At last path point (current: " + currentSeqNumber + ", prev: " + prevSeqNumber + ") of " + te + "...");
							for (TrajectoryEnvelope toFinish : startedGroundEnvelopes) {
								if (!finishedGroundEnvelopes.contains(toFinish)) {
									metaCSPLogger.info("<<<< Finished (ground envelope) " + toFinish);
									finishedGroundEnvelopes.add(toFinish);
								}
							}
							break;
						}
						
						//Update previous seq number
						prevSeqNumber = currentSeqNumber;
					}
				
					//Sleep a little...
					try { Thread.sleep(trackingPeriodInMillis); }
					catch (InterruptedException e) { e.printStackTrace(); }

				}

				synchronized(tec.getSolver()) { 
					if (cb != null) cb.beforeTrackingFinished();
					finishTracking();
					if (cb != null) cb.onTrackingFinished();
				}
				
			}
		};
		
		monitorSubEnvelopes.start();
	}
	
	protected void finishTracking() {
		metaCSPLogger.info("<<<< Finished (super envelope) " + this.te);
		if (!(this instanceof TrajectoryEnvelopeTrackerDummy)) fixDeadline(te, 0);
	}

	/**
	 * Returns the {@link TrajectoryEnvelope} that this tracker is tracking.
	 * @return The {@link TrajectoryEnvelope} that this tracker is tracking.
	 */
	public TrajectoryEnvelope getTrajectoryEnvelope() {
		return this.te;
	}
	
	
}
