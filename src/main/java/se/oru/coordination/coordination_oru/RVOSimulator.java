package se.oru.coordination.coordination_oru;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.InputStream;
import java.util.Map;

import javax.imageio.ImageIO;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.jboss.netty.buffer.ChannelBuffer;

import com.google.common.base.Preconditions;
import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.yaml.snakeyaml.Yaml;

import edu.unc.cs.gamma.rvo.Simulator;
import nav_msgs.MapMetaData;
import nav_msgs.OccupancyGrid;
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
    
    public void obstaclesFromYaml(String yamlFile) {
    	viz.setMap(yamlFile);
    	OccupancyGrid map = null;
    	if (viz.getMap(map)) {
		    final ChannelBuffer buffer = map.getData(); 
		    int width = map.getInfo().getWidth();
		    int height = map.getInfo().getHeight();
		    int[] pixels = new int[width * height];
		    for (int i = 0; i < pixels.length; i++) {
		    	//FIXME!! READ THE MAP AND ACCORDING TO THE VALUES SET OBSTACLES IN THE SCENE!!
		    }
    	}
    }
    
    protected void pixelToMeters(double origin_x, double origin_y, double res, int px, int py, double mx, double my) {
    	mx = origin_x + (px+0.5)*res;
    	my = origin_y + (py+0.5)*res;
    }

}
