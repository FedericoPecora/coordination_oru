package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

public interface ForwardModel {
	
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex, boolean useVelocity);
	
	public default int getEarliestStoppingPathIndex(TrajectoryEnvelope te, RobotReport currentState) {
		return getStoppingPathIndicesBounds(te, currentState, 1)[0];
	}
	
	public int[] getStoppingPathIndicesBounds(TrajectoryEnvelope te, RobotReport currentState, int numberOfAdditionalCoordinationPeriods);

}