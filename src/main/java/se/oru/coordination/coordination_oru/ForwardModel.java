package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

public interface ForwardModel {
	
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex, boolean useVelocity);
	
	public int getEarliestStoppingPathIndex(TrajectoryEnvelope te, RobotReport currentState);

}