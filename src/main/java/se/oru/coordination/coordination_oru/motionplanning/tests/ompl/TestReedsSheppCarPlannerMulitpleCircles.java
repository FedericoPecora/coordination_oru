package se.oru.coordination.coordination_oru.motionplanning.tests.ompl;


import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class TestReedsSheppCarPlannerMulitpleCircles {
	
	public static void main(String[] args) {
		double distanceBetweenPathPoints = 0.4;
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps/map-partial.png");
		rsp.setMapResolution(0.1);
		rsp.setRadius(0.5);
		rsp.setStart(new Pose(2.0,2.0,0.0));
		rsp.setGoals(new Pose(2.0,38.0,0.0));
		rsp.setCirclePositions(new Coordinate[] {
				new Coordinate(-1.0,0.0),
				new Coordinate(0.0,0.0),
				new Coordinate(1.0,0.0),
		});
		rsp.setDistanceBetweenPathPoints(distanceBetweenPathPoints);
		rsp.setTurningRadius(2.0);
		if (rsp.plan()) {
			PoseSteering[] pss = rsp.getPath();
			for (PoseSteering ps : pss) {
				System.out.println(ps.getPose());
			}
			double totalDist = 0.0;
			for (int i = 0; i < pss.length-1; i++) {
				totalDist += pss[i].getPose().distanceTo(pss[i+1].getPose());
			}
			System.out.println("- Number of path points: " + pss.length);
			System.out.println("- Total distance: " + totalDist);
			System.out.println("- Max distance between path points: " + distanceBetweenPathPoints);
			System.out.println("- (Number of path points)*(max distance between path points) = " + pss.length*distanceBetweenPathPoints);
		}
	}

}
