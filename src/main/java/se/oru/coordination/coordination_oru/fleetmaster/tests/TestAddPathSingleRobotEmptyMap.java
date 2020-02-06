package se.oru.coordination.coordination_oru.fleetmaster.tests;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class TestAddPathSingleRobotEmptyMap {
	public static void main(String[] args) {
		System.out.print("FleetMasterInterference class");
		FleetMasterInterface flint = new FleetMasterInterface();
		flint.useDefaultGridParams();
		flint.show(true);
		
		//Test 1: using default robot footprint	
		Pose startRobot1 = new Pose(45.0,5.0,0.0);
		Pose goalRobot11 = new Pose(40.0,7.0,0.0);
		Pose goalRobot12 = new Pose(10.0,7.0,0.0);
		Pose goalRobot13 = new Pose(5.0,5.0,0.0);

		//Set up path planner (using empty map)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(flint.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		rsp.setStart(startRobot1);
		rsp.setGoals(goalRobot11,goalRobot12,goalRobot13);
		rsp.plan();
		
		//Add the path to the fleetmaster
		boolean ret = flint.addPath(1, rsp.getPath().hashCode(), rsp.getPath());
		System.out.println("[FleetMasterInterference] addPath: " + ret);
		
		//Show image
		
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//Delete the path
		flint.addPath(1, rsp.getPath().hashCode(), rsp.getPath());
		
		//Show image
		
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
}

