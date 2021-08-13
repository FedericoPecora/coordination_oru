package se.oru.coordination.coordination_oru.tests.testsPedestrians;

import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulationWithPedestrians;

public class PedestrianTrackingCallback extends TrackingCallback {

    protected int robot_id;
    protected TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec;
    double startTime = 0.0;

    public PedestrianTrackingCallback(int robot_id, TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec, double startTime) {
        this.robot_id = robot_id;
        this.tec = tec;
        this.startTime = startTime;
    }

    @Override
    public void beforeTrackingStart() {

    }

    @Override
    public void onTrackingStart() {

    }

    @Override
    public String[] onPositionUpdate() {
        return new String[]{"T: " + tec.getElapsedTrackingTime(robot_id), "S: " + startTime};
    }

    @Override
    public void beforeTrackingFinished() {

    }

    @Override
    public void onTrackingFinished() {

    }

    @Override
    public void onNewGroundEnvelope() {

    }

}
