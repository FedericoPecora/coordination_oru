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

	protected TrajectoryEnvelope te = null;
	protected Trajectory traj = null;
	protected double temporalResolution = 0.0;
	protected int criticalPoint = -1;
	protected HashSet<TrajectoryEnvelope> startedGroundEnvelopes = new HashSet<TrajectoryEnvelope>();
	protected HashSet<TrajectoryEnvelope> finishedGroundEnvelopes = new HashSet<TrajectoryEnvelope>();
	protected HashMap<TrajectoryEnvelope,AllenIntervalConstraint> deadlines = new HashMap<TrajectoryEnvelope, AllenIntervalConstraint>();
	protected TrajectoryEnvelopeSolver solver = null;
	protected int trackingPeriodInMillis = 0;
	protected TrackingCallback cb = null;
	protected Map mapMetaConstraint = null; 
	protected boolean calledOnTrackingStart = false;
	protected boolean calledStartTracking = false;
	
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
	public AbstractTrajectoryEnvelopeTracker(TrajectoryEnvelope te, double temporalResolution, TrajectoryEnvelopeSolver solver, int trackingPeriodInMillis, TrackingCallback cb) {
		this.te = te;
		this.traj = te.getTrajectory();
		this.criticalPoint = -1;
		this.temporalResolution = temporalResolution;
		this.solver = solver;
		this.trackingPeriodInMillis = trackingPeriodInMillis;
		this.cb = cb;
		startMonitoringThread();
	}
	
	public void setMapMetaConstraint(Map mapMetaConstraint) {
		this.mapMetaConstraint = mapMetaConstraint;
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
	public abstract void setCriticalPoint(int criticalPoint);
	
	/**
	 * Returns the current critical point.
	 * @return The current critical point.
	 */
	public int getCriticalPoint() {
		return this.criticalPoint;
	}
	
	/**
	 * This method should return a {@link RobotReport}, describing the current state of the robot.
	 * @return A {@link RobotReport}, describing the current state of the robot.
	 */
	public abstract RobotReport getRobotReport();
		
	/**
	 * Defines what should happen when the robot reaches a new pose along the reference trajectory.
	 */
	public abstract void onPositionUpdate();
	
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
		synchronized(solver) {
			long time = getCurrentTimeInMillis()+delta;
			if (time > trajEnv.getTemporalVariable().getEET()) {
				solver.removeConstraint(deadlines.get(trajEnv));
				long bound1 = Math.max(time, trajEnv.getTemporalVariable().getEET());
				//long bound1 = time;
				long bound2 = APSPSolver.INF;
				//metaCSPLogger.info("Finishing @ " + time + " " + trajEnv + " (ET bounds: [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() + "])");
				AllenIntervalConstraint deadline = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Deadline, new Bounds(bound1, bound2));
				deadline.setFrom(trajEnv);
				deadline.setTo(trajEnv);
				boolean added = solver.addConstraint(deadline);
				if (!added) {
					metaCSPLogger.severe("ERROR: Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
					throw new Error("Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
				}
				else deadlines.put(trajEnv, deadline);
			}
		}
	}

	protected void fixDeadline(TrajectoryEnvelope trajEnv, long delta) {
		synchronized(solver) {
			long time = getCurrentTimeInMillis()+delta;
			if (time > trajEnv.getTemporalVariable().getEET()) {
				solver.removeConstraint(deadlines.get(trajEnv));
				long bound1 = Math.max(time, trajEnv.getTemporalVariable().getEET());
				//long bound1 = time;
				long bound2 = bound1;
				metaCSPLogger.info("Finishing @ " + time + " " + trajEnv + " (ET bounds: [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() + "])");
				AllenIntervalConstraint deadline = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Deadline, new Bounds(bound1, bound2));
				deadline.setFrom(trajEnv);
				deadline.setTo(trajEnv);
				boolean added = solver.addConstraint(deadline);
				if (!added) {
					metaCSPLogger.severe("ERROR: Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
					throw new Error("Could not add deadline constraint " + deadline + " whose ET bounds are [" + trajEnv.getTemporalVariable().getEET() + "," + trajEnv.getTemporalVariable().getLET() +"]");
				}
				else deadlines.put(trajEnv, deadline);
			}
		}
	}

	protected void setRelease(TrajectoryEnvelope trajEnv) {
		synchronized(solver) {
			long time = getCurrentTimeInMillis();
			time = Math.max(time, trajEnv.getTemporalVariable().getEST());
			metaCSPLogger.info("Releasing @ " + time + " " + trajEnv + " (ST bounds: [" + trajEnv.getTemporalVariable().getEST() + "," + trajEnv.getTemporalVariable().getLST() + "])");
			AllenIntervalConstraint release = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Release, new Bounds(time, time));
			release.setFrom(trajEnv);
			release.setTo(trajEnv);
			boolean added = solver.addConstraint(release);
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
						while ((rr = getRobotReport()) == null) {
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
									//prevSeqNumber = currentSeqNumber;
									updateDeadline(subEnv, 0);
								}
							}							
						}
						
						//Stop when last path point reached (or we missed that report and the path point is now 0)
						if (te.getSequenceNumberEnd() == currentSeqNumber || currentSeqNumber < prevSeqNumber) {
							metaCSPLogger.info("At last path point of " + te + "...");
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

				if (cb != null) cb.beforeTrackingFinished();

				synchronized(solver) { 
					finishTracking();
				}
				if (cb != null) cb.onTrackingFinished();
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
