package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

/**
 * A dependency is a tuple (teWaiting, teDriving, waitingPoint, thresholdPoint), representing
 * the fact that the robot navigating along {@link TrajectoryEnvelope} teWaiting should
 * not go beyond path index waitingPoint until the robot navigating along teDriving reaches
 * path index thresholdPoint.
 * 
 * @author fpa
 *
 */
public class Dependency implements Comparable<Dependency> {
	
	private int robotIDWaiting, robotIDDriving;
	private TrajectoryEnvelope teWaiting, teDriving;
	private int waitingPoint, thresholdPoint;
	private AbstractTrajectoryEnvelopeTracker waitingTracker, drivingTracker;
	
	public Dependency(TrajectoryEnvelope teWaiting, TrajectoryEnvelope teDriving, int waitingPoint, int thresholdPoint, AbstractTrajectoryEnvelopeTracker waitingTracker, AbstractTrajectoryEnvelopeTracker drivingTracker) {
		this.teWaiting = teWaiting;
		this.teDriving = teDriving;
		this.waitingPoint = waitingPoint;
		this.thresholdPoint = thresholdPoint;
		this.waitingTracker = waitingTracker;
		this.drivingTracker = drivingTracker;
		this.robotIDWaiting = teWaiting.getRobotID();
		if (teDriving != null) this.robotIDDriving = teDriving.getRobotID();
		else this.robotIDDriving = 0;
	}
	
	@Override
	public int hashCode() {
		long code = Long.parseLong(teWaiting.getRobotID()+"0"+waitingPoint+"0"+this.robotIDDriving+"0"+thresholdPoint);
		code = code%Integer.MAX_VALUE;
		return (int)code;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof Dependency)) return false;
		Dependency other = (Dependency)obj;
		return this.teWaiting.equals(other.teWaiting) && this.teDriving.equals(other.teDriving) && this.waitingPoint == other.waitingPoint && this.thresholdPoint == other.thresholdPoint;
	}
	
	@Override
	public int compareTo(Dependency other) {
		if (this.waitingPoint != other.waitingPoint) return this.waitingPoint-other.waitingPoint;
		else return this.thresholdPoint-other.thresholdPoint;
	}
	
	@Override
	public String toString() {
		int drivingTEID = 0;
		if (teDriving != null) drivingTEID = teDriving.getID();
		return getWaitingRobotID()+"/"+waitingPoint+"(TE" + teWaiting.getID() + ")-"+getDrivingRobotID()+"/"+thresholdPoint+"(TE" + drivingTEID + ")";
	}
	
	public Pose getWaitingPose() {
		return teWaiting.getTrajectory().getPose()[this.getWaitingPoint()];
	}
	
	public Pose getReleasingPose() {
		return teDriving.getTrajectory().getPose()[this.getReleasingPoint()];
	}
	
	public TrajectoryEnvelope getWaitingTrajectoryEnvelope() {
		return this.teWaiting;
	}
	
	public TrajectoryEnvelope getDrivingTrajectoryEnvelope() {
		return this.teDriving;
	}
	
	public int getWaitingPoint() {
		return this.waitingPoint;
	}
	
	public int getReleasingPoint() {
		return this.thresholdPoint;
	}
	
	public AbstractTrajectoryEnvelopeTracker getWaitingTracker() {
		return this.waitingTracker;
	}
	
	public AbstractTrajectoryEnvelopeTracker getDrivingTracker() {
		return this.drivingTracker;
	}
	
	public int getWaitingRobotID() {
		return this.robotIDWaiting;
	}

	public int getDrivingRobotID() {
		return this.robotIDDriving;
	}
	
}

