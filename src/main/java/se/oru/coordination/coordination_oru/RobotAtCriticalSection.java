package se.oru.coordination.coordination_oru;

public class RobotAtCriticalSection {
	
	private AbstractTrajectoryEnvelopeTracker tet;
	private CriticalSection cs;
	
	public RobotAtCriticalSection(AbstractTrajectoryEnvelopeTracker tet, CriticalSection cs) {
		this.tet = tet;
		this.cs = cs;
	}
	
	public AbstractTrajectoryEnvelopeTracker getTrajectoryEnvelopeTracker() {
		return this.tet;
	}
	
	public CriticalSection getCriticalSection() {
		return this.cs;
	}

}
