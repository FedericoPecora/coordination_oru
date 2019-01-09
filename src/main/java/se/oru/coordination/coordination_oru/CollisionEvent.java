package se.oru.coordination.coordination_oru;

public class CollisionEvent {
	protected long time = 0;
	protected RobotReport[] reports = new RobotReport[2];
	
	/**
	 * Create a {@link CollisionEvent} with information on time and robots involved.
	 * @param time The time when the collision happens.
	 * @param reports The reports of the robots where the collision happens (the current ones, without delays).
	 */
	public CollisionEvent(long time, RobotReport report1, RobotReport report2) {
		this.time = time;
		this.reports[0] = report1;
		this.reports[1] = report2;
	};
	
	/**
	 * Get the time when the collision happens.
	 * @return The time when the collision happens.
	 */
	public long getTime() {
		return this.time;
	};
	
	/**
	 * Get the reports of the robots where the collision happens (the current ones, without delays).
	 * @return The reports of the robots.
	 */
	public RobotReport[] getReports() {
		return this.reports;
	};
		
	/**
	 * Get an informative string related to the collision event.
	 * @return The critical section where the collision happens.
	 */
	public String toString() {
		return "Robots: [" + this.reports[0].getRobotID() +"," + this.reports[1].getRobotID()+"], PathIndices: [" + this.reports[0].getPathIndex() +", " + this.reports[1].getPathIndex() + "], Poses: "+ this.reports[0].getPose().toString() + ", " + this.reports[1].getPose().toString() + ".";
	};
}

