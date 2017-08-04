package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

public interface ForwardModel {
	
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex);

}