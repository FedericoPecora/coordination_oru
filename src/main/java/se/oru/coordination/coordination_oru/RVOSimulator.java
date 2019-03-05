package se.oru.coordination.coordination_oru;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import edu.unc.cs.gamma.rvo.Simulator;
import se.oru.coordination.coordination_oru.util.FleetVisualization;

public class RVOSimulator { 
	protected FleetVisualization viz = null;
	protected Simulator sim;
	
	public RVOSimulator(Simulator sim) {
		this.sim = sim;
	}
	
	/**
	 * Sets up a GUI which shows the current status of robots.
	 */
	public void setVisualization(FleetVisualization viz) {
		this.viz = viz;
	}
	
    /**
     * Performs a simulation step and updates the two-dimensional position and
     * two-dimensional velocity of each agent.
     *
     * @return The global time after the simulation step.
     */
    public double doStepAndVisualize() {
    	double time = sim.doStep();
		for (int i = 0; i < sim.getNumAgents(); ++i) {
			Vector2D pos = sim.getAgentPosition(i);	
			double radius = sim.getAgentRadius(i);
			int N = 16;
			Coordinate[] coord = new Coordinate[N];
			for (int j = 0; j < N; ++j) {
				double angle = j * 2 * Math.PI / N;
				coord[j] = new Coordinate(Math.cos(angle) * radius, Math.sin(angle) * radius);
			}
			double vel = sim.getAgentVelocity(i).getNorm();
			String s = String.format("%.2f", vel);
			String extraStatusInfo = new String("vel: " + s);
			Pose pose = new Pose(pos.getX(), pos.getY(), 0);
			RobotReport rr = new RobotReport(i, pose, -2, sim.getAgentVelocity(i).getNorm(), -1, -2);
			viz.displayRobotState(TrajectoryEnvelope.createFootprintPolygon(coord), rr, extraStatusInfo);
		}
		viz.updateVisualization();
        return time;
    }

}
