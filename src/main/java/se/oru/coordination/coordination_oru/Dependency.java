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
	
	/**
	 * Class to store the static data (teWaiting, teDriving, thresholdPoint) of a {@link Dependency}.
	 * 
	 * @author anna
	 *
	 */
	public class DependencyStaticData {
		
		/**
		 * Class constructor.
		 * @param teWaiting The {@link TrajectoryEnvelope} of the yielding robot.
		 * @param teDriving The {@link TrajectoryEnvelope} of the leading robot.
		 * @param thresholdPoint The path index along the leading robot's path to be reached from the leading robot to deactivate the constraint.
		 */
		DependencyStaticData(TrajectoryEnvelope teWaiting, TrajectoryEnvelope teDriving, int thresholdPoint) {
			this.teWaiting = teWaiting;
			this.teDriving = teDriving;
			this.thresholdPoint = thresholdPoint;
		}
		private final int thresholdPoint;
		private final TrajectoryEnvelope teWaiting, teDriving;
							
		public TrajectoryEnvelope getTeWaiting() {
			return this.teWaiting;
		}
		
		public TrajectoryEnvelope getTeDriving() {
			return this.teDriving;
		}
		
		public int getThresholdPoint() {
			return this.thresholdPoint;
		}
	}
	
	private final DependencyStaticData data;
	private int waitingPoint;
	
	public Dependency(TrajectoryEnvelope teWaiting, TrajectoryEnvelope teDriving, int waitingPoint, int thresholdPoint) {
		this.data = new DependencyStaticData(teWaiting, teDriving, thresholdPoint);
		this.waitingPoint = waitingPoint;
	}
	
	@Override
	public int hashCode() {
		long code = Long.parseLong(this.getWaitingRobotID() +"0" + this.waitingPoint + "0"+ this.getDrivingRobotID() + "0" + this.getReleasingPoint());
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
		return this.getWaitingTrajectoryEnvelope().equals(other.getWaitingTrajectoryEnvelope()) && 
				this.getDrivingTrajectoryEnvelope().equals(other.getDrivingTrajectoryEnvelope()) && 
				this.waitingPoint == other.waitingPoint && this.getReleasingPoint() == other.getReleasingPoint();
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
		return this.data.getThresholdPoint()-other.data.getThresholdPoint();
	}
	
	@Override
	public String toString() {
		int drivingTEID = 0;
		if (this.data.getTeDriving() != null) drivingTEID = this.data.getTeDriving().getID();
		return getWaitingRobotID() + "/" + this.waitingPoint + "(TE" + getWaitingTrajectoryEnvelope().getID() + ")-" + getDrivingRobotID() + "/" + this.getReleasingPoint() +"(TE" + drivingTEID + ")";
	}
	
	public Pose getWaitingPose() {
		return data.getTeWaiting().getTrajectory().getPose()[this.getWaitingPoint()];
	}
	
	public Pose getReleasingPose() {
		return data.getTeDriving().getTrajectory().getPose()[this.getReleasingPoint()];
	}
	
	public TrajectoryEnvelope getWaitingTrajectoryEnvelope() {
		return this.data.getTeWaiting();
	}
	
	public TrajectoryEnvelope getDrivingTrajectoryEnvelope() {
		return this.data.getTeDriving();
	}
	
	public int getWaitingPoint() {
		return this.waitingPoint;
	}
	
	public int getReleasingPoint() {
		return this.data.getThresholdPoint();
	}
	
	public int getWaitingRobotID() {
		return this.data.getTeWaiting().getRobotID();
	}

	public int getDrivingRobotID() {
		return (this.data.getTeDriving() != null) ? this.data.getTeDriving().getRobotID() : 0;
	}
	
}

