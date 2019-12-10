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
	
	public Dependency(TrajectoryEnvelope teWaiting, TrajectoryEnvelope teDriving, int waitingPoint, int thresholdPoint) {
		this.teWaiting = teWaiting;
		this.teDriving = teDriving;
		this.waitingPoint = waitingPoint;
		this.thresholdPoint = thresholdPoint;
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
	/**Function to compare two dependencies.
	 * @param obj The dependency to be compared.
	 * @return true if the two dependencies are related to the same trajectory envelopes and have the same
	 * waiting and threshold points.
	 * @ATTENTION: the compareTo() and the equals() functions give different results in case of
	 * dependencies involving different pairs of robots, but with the same critical point. 
	 * While the compareTo() orders Dependencies according to critical points, the equals also consider
	 * the pair of the robots involved. 
	 * @PLEASE, BE SURE THE RIGHT FUNCTION WILL BE USED TO ADD OR REMOVE DEPENDENCIES FROM A SPECIFIC DATA STRUCTURE.
	 */
	public boolean equals(Object obj) {
		if (!(obj instanceof Dependency)) return false;
		Dependency other = (Dependency)obj;
		return this.teWaiting.equals(other.teWaiting) && this.teDriving.equals(other.teDriving) && this.waitingPoint == other.waitingPoint && this.thresholdPoint == other.thresholdPoint;
	}
	
	@Override
	/**Function for ordering dependencies according to the closest waiting point.
	 * @param other The dependency to be compared.
	 * @ATTENTION: the compareTo() and the equals() functions give different results in case of
	 * dependencies involving different pairs of robots, but with the same critical point. 
	 * While the compareTo() orders Dependencies according to critical points, the equals also consider
	 * the pair of the robots involved. 
	 * @PLEASE, BE SURE THE RIGHT FUNCTION WILL BE USED TO ADD OR REMOVE DEPENDENCIES FROM A SPECIFIC DATA STRUCTURE.
	 */
	public int compareTo(Dependency other) {
		if (this.waitingPoint != other.waitingPoint) return this.waitingPoint-other.waitingPoint;
		return this.thresholdPoint-other.thresholdPoint;
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
	
	public int getWaitingRobotID() {
		return this.robotIDWaiting;
	}

	public int getDrivingRobotID() {
		return this.robotIDDriving;
	}
	
}

