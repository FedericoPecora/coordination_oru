package se.oru.coordination.coordination_oru.motionplanning.roadmap;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;

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
