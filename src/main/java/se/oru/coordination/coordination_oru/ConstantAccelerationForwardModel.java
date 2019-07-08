package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.simulation2D.State;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4;

public class ConstantAccelerationForwardModel implements ForwardModel {
		
	private double maxAccel, maxVel;
	private double temporalResolution = -1;
	private int trackingPeriodInMillis = 0;
	private int controlPeriodInMillis = -1;
	
	public ConstantAccelerationForwardModel(double maxAccel, double maxVel, double temporalResolution, int controlPeriodInMillis, int trackingPeriodInMillis) {
		this.maxAccel = maxAccel;
		this.maxVel = maxVel;	
		this.temporalResolution = temporalResolution;
		this.controlPeriodInMillis = controlPeriodInMillis;
		this.trackingPeriodInMillis = trackingPeriodInMillis;
	}

	@Override
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex, boolean useVelocity) {
		if (useVelocity && currentState.getVelocity() <= 0.0) return true;
		double distance = se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.computeDistance(te.getTrajectory(), (currentState.getPathIndex() != -1 ? currentState.getPathIndex() : 0), targetPathIndex);
		State state = new State(0.0, currentState.getVelocity());
		double time = 0.0;
		double deltaTime = 0.0001;
		long lookaheadInMillis = 2*(this.controlPeriodInMillis + TrajectoryEnvelopeCoordinator.MAX_TX_DELAY + trackingPeriodInMillis);
		if (lookaheadInMillis > 0) {
			while (time*this.temporalResolution < lookaheadInMillis) {
				se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
				time += deltaTime;
			}
		}
		//decelerate from maximum to stop
		while (state.getVelocity() > 0) {
			if (state.getPosition() > distance) return false;
			se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			time += deltaTime;
		}
		return true;
	}

	private int getPathIndex(TrajectoryEnvelope te, State auxState) {
		if (auxState == null) return -1;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Trajectory traj = te.getTrajectory();
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return currentPathIndex;
	}
	
	@Override
	public int getEarliestStoppingPathIndex(TrajectoryEnvelope te, RobotReport currentState) {
		State state = new State(currentState.getDistanceTraveled(), currentState.getVelocity());
		double time = 0.0;
		double deltaTime = 0.0001;
		long lookaheadInMillis = 2*(this.controlPeriodInMillis + TrajectoryEnvelopeCoordinator.MAX_TX_DELAY + trackingPeriodInMillis);
		if (lookaheadInMillis > 0) {
			while (time*temporalResolution < lookaheadInMillis) {
				se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel*1.1);
				time += deltaTime;
			}
		}
		while (state.getVelocity() > 0) {
			se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel*0.9);
			time += deltaTime;
		}
		return getPathIndex(te,state);
	}

}
