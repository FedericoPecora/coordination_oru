package se.oru.coordination.coordination_oru.motionplanning.tests;


import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;

public class TestReedsSheppCarPlanner {
	
	public static void main(String[] args) {
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps/map-partial.png");
		rsp.setMapResolution(0.1);
		rsp.setRobotRadius(0.5);
		rsp.setStart(new Pose(2.0,2.0,0.0));
		rsp.setGoal(new Pose(2.0,38.0,0.0));
		rsp.setNumInterpolationPoints(100);
		rsp.setTurningRadius(2.0);
		if (rsp.plan()) {
			PoseSteering[] pss = rsp.getPath();
			for (PoseSteering ps : pss) {
				System.out.println(ps.getPose());
			}
		}
	}

}
