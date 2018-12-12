package se.oru.coordination.coordination_oru;

public class CollisionEvent {
	protected long time = 0;
	protected CriticalSection cs;
	protected RobotReport[] reports = new RobotReport[2];
	
	/**
	 * Create a {@link CollisionEvent} with information on time and robots involved.
	 * @param time The time when the collision happens.
	 * @param cs The critical section where the collision happens. 
	 * @param reports The reports of the robots where the collision happens (the current ones, without delays).
	 */
	public CollisionEvent(long time, RobotReport report1, RobotReport report2, CriticalSection cs) {
		this.time = time;
		this.reports[0] = report1;
		this.reports[1] = report2;
		this.cs = cs;
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
	 * Get the critical section where the collision happens.
	 * @return The critical section where the collision happens.
	 */
	public CriticalSection getCriticalSection() {
		return this.cs;
	};
}
