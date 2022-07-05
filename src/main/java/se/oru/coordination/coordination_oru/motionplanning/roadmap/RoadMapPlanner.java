package se.oru.coordination.coordination_oru.motionplanning.roadmap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;

public class RoadMapPlanner extends AbstractMotionPlanner {
	
	private String startLocationName = null;
	private String[] goalLocationNames = null; 

	@Override
	public AbstractMotionPlanner getCopy(boolean copyObstacles) {
		return this;
	}
	
	public void setStart(String locationName) {
		Pose p = Missions.getLocationPose(locationName);
		if (p == null) throw new Error("Cannot use unknown location " + locationName + " as starting pose!");
		this.setStart(p);
		this.startLocationName = locationName;
	}

	public void setGoals(String ... locationNames) {
		this.goalLocationNames = new String[locationNames.length];
		Pose[] locs = new Pose[locationNames.length];
		for (int i = 0; i < locationNames.length; i++) {
			Pose pi = Missions.getLocationPose(locationNames[i]);
			if (pi == null) {
				this.goalLocationNames = null;
				throw new Error("Cannot use unknown location " + locationNames[i] + " as goal pose!");
			}
			locs[i] = pi;
			this.goalLocationNames[i] = locationNames[i];
		}
		this.setGoals(locs);
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
