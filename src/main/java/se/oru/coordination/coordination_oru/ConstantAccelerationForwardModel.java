package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.simulation2D.State;

public class ConstantAccelerationForwardModel implements ForwardModel {
		
	private double maxAccel, maxVel;
	
	public ConstantAccelerationForwardModel(double maxAccel, double maxVel) {
		this.maxAccel = maxAccel;
		this.maxVel = maxVel;	
	}
		
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex) {
		double distance = se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.computeDistance(te.getTrajectory(), (currentState.getPathIndex() != -1 ? currentState.getPathIndex() : 0), targetPathIndex);
		State state = new State(0.0, currentState.getVelocity());
		double time = 0.0;
		double deltaTime = 0.0001;
		while (state.getVelocity() > 0) {
			if (state.getPosition() > distance) return false;
			se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			time += deltaTime;
		}
		return true;
	}

}
