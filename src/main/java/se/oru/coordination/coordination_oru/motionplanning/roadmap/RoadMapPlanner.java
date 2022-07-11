package se.oru.coordination.coordination_oru.motionplanning.roadmap;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;

/**
 * This class implements a simple planner based on the methods provided by the static class {@link Missions}.
 * The planner uses the locations and paths that are loaded with the method {@link Missions.#loadRoadMap(String)}.
 * The underlying representation is a weighted directed graph, where the nodes are the loaded locations and the weight on the edge
 * connecting two locations is the length of the path between the two locations. A shortest path between the given start and goal locations
 * is computed using Dijkstra's algorithm.     
 * @author fpa
 *
 */
public class RoadMapPlanner extends AbstractMotionPlanner {
	
	@Override
	public AbstractMotionPlanner getCopy(boolean copyObstacles) {
		return this;
	}
	
	@Override
	public boolean doPlanning() {
		String[] wps = new String[this.goalLocationNames.length+1];
		wps[0] = this.startLocationName;
		for (int i = 0; i < this.goalLocationNames.length; i++) wps[i+1] = this.goalLocationNames[i];
		PoseSteering[] path = Missions.getShortestPath(wps);
		if (path == null) return false;
		//TODO: Check if path intersects with known obstacles and return false if it does
		this.pathPS = path;
		return true;
	}

}
