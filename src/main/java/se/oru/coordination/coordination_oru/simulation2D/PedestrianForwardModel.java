package se.oru.coordination.coordination_oru.simulation2D;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.ForwardModel;
import se.oru.coordination.coordination_oru.RobotReport;

public class PedestrianForwardModel implements ForwardModel {

	@Override
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex, boolean useVelocity) {
		return true;
	}

	@Override
	public int getEarliestStoppingPathIndex(TrajectoryEnvelope te, RobotReport currentState) {
		return 0;
	}

}
