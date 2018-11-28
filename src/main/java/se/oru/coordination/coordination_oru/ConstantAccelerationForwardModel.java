package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.simulation2D.State;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4;

public class ConstantAccelerationForwardModel implements ForwardModel {
		
	private double maxAccel, maxVel;
	private int CONTROL_PERIOD = -1;
	private double TEMPORAL_RESOLUTION = -1;
	private int numControlPeriodsFreeRun = 1;
	private int addtionalDelayInMillis = 0;
	private int delay = 0;

	@Deprecated
	public ConstantAccelerationForwardModel(double maxAccel, double maxVel) {
		this.maxAccel = maxAccel;
		this.maxVel = maxVel;	
	}
	
	@Deprecated
	public ConstantAccelerationForwardModel(double maxAccel, double maxVel, int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, int numControlPeriodsFreeRun, int additionalDelayInMillis) {
		this.maxAccel = maxAccel;
		this.maxVel = maxVel;	
		this.CONTROL_PERIOD = CONTROL_PERIOD;
		this.TEMPORAL_RESOLUTION = TEMPORAL_RESOLUTION;
		this.numControlPeriodsFreeRun = numControlPeriodsFreeRun;
		this.addtionalDelayInMillis = additionalDelayInMillis;
	}

	@Deprecated
	public ConstantAccelerationForwardModel(double maxAccel, double maxVel, int CONTROL_PERIOD, double TEMPORAL_RESOLUTION) {
		this(maxAccel,maxVel,CONTROL_PERIOD,TEMPORAL_RESOLUTION,2,2*NetworkConfiguration.MAXIMUM_TX_DELAY);
	}

	public ConstantAccelerationForwardModel(double maxAccel, double maxVel, int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, int additionaDelayInMillis) {
		this.maxAccel = maxAccel;
		this.maxVel = maxVel;
		this.TEMPORAL_RESOLUTION = TEMPORAL_RESOLUTION;
		this.delay = 2*NetworkConfiguration.MAXIMUM_TX_DELAY+2*CONTROL_PERIOD+additionaDelayInMillis;
	}


	//TODO: Total delay before deceleration = 2*CONTROL_PEROID + TXDelay
	//  -- One CONTROL_PERIOD because the state could be at most that old
	//  -- One CONTROL_PERIOD because the coordinator may TX at end of CONTORL_PERIOD
	//  -- TXDelay because that is the time it take for the message to arrive at the robot
	@Override
	public boolean canStop(TrajectoryEnvelope te, RobotReport currentState, int targetPathIndex) {
		if (currentState.getVelocity() <= 0.0) return true;
		double distance = se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.computeDistance(te.getTrajectory(), (currentState.getPathIndex() != -1 ? currentState.getPathIndex() : 0), targetPathIndex);
		State state = new State(0.0, currentState.getVelocity());
		double time = 0.0;
		double deltaTime = 0.0001;
		//int delay = this.numControlPeriodsFreeRun*this.CONTROL_PERIOD+this.addtionalDelayInMillis;
		//if (CONTROL_PERIOD != -1) {
		if (this.delay > 0) {
			//while (time*TEMPORAL_RESOLUTION < Math.max(CONTROL_PERIOD*numControlPeriodsFreeRun,TrajectoryEnvelopeCoordinator.EFFECTIVE_CONTROL_PERIOD)) {
			while (time*TEMPORAL_RESOLUTION < this.delay) {
				se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerRK4.integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
				time += deltaTime;
			}
		}
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
		//if (CONTROL_PERIOD != -1) {
		if (this.delay > 0) {
			//while (time*TEMPORAL_RESOLUTION < Math.max(CONTROL_PERIOD*numControlPeriodsFreeRun,TrajectoryEnvelopeCoordinator.EFFECTIVE_CONTROL_PERIOD)) {
			while (time*TEMPORAL_RESOLUTION < this.delay) {
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
