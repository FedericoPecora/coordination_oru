package se.oru.coordination.coordination_oru.tests.clean;

import java.util.HashMap;
import java.util.HashSet;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Coordination on paths obtained from the ReedsSheppCarPlanner for five robots navigating in a random map.")
public class RandomPathsInMap {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 4.0;
		
		//Instantiate a trajectory envelope coordinator.
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(2000, 1000, MAX_VEL,MAX_ACCEL);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(true);
		tec.setBreakDeadlocks(false, true, true);
		//MetaCSPLogging.setLevel(TrajectoryEnvelopeCoordinator.class, Level.FINEST);
		
		//Setup the network parameters
		NetworkConfiguration.setDelays(10, 500);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0;
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 1e-2);

		double xl = 1.0;
		double yl = .5;
		Coordinate footprint1 = new Coordinate(-xl,yl);
		Coordinate footprint2 = new Coordinate(xl,yl);
		Coordinate footprint3 = new Coordinate(xl,-yl);
		Coordinate footprint4 = new Coordinate(-xl,-yl);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = null;
		yamlFile = "maps/map-partial-2.yaml";	
		
		BrowserVisualization viz = new BrowserVisualization();
		viz.setMap(yamlFile);
		viz.setInitialTransform(20.0, 9.0, 2.0);
		tec.setVisualization(viz);
		
		Missions.loadRoadMap("missions/icaps_locations_and_paths_4.txt");

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);
	
		int[] robotIDs = new int[] {1,2,3,4,5,6,7};
		int locationCounter = 0;
		
		//Preload goals and starts. Make them as obstacles for all the other robots
		//in case of testing a valid infrastructure. 
		HashMap<Integer, Pose> startsPoses = new HashMap<Integer, Pose>();
		HashMap<Integer, String> startsNames = new HashMap<Integer, String>();
		HashMap<Integer, Pose> endsPoses = new HashMap<Integer, Pose>();
		HashMap<Integer, String> endsNames = new HashMap<Integer, String>();
		
		for (int robotID : robotIDs) {
			
			String startLocName = "L_"+locationCounter;
			Pose startLoc = Missions.getLocationPose(startLocName);
			startsPoses.put(robotID, startLoc);
			startsNames.put(robotID, startLocName);
			
			String endLocName = "R_"+locationCounter;
			Pose endLoc = Missions.getLocationPose(endLocName);
			endsPoses.put(robotID, endLoc);
			endsNames.put(robotID, endLocName);		
			//obstacles.add(startLoc);
			//obstacles.add(endLoc);
			
			locationCounter += 2;
		}
		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMap(yamlFile);
		rsp.setRadius(0.1);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);
		
		//int[] robotIDs = new int[] {1,2};
		for (int robotID : robotIDs) {

			//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
			tec.setMotionPlanner(robotID, rsp);
			
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));
			String startLocName = startsNames.get(robotID);
			Pose startLoc = startsPoses.get(robotID);
			String endLocName = endsNames.get(robotID);
			Pose endLoc = endsPoses.get(robotID);
			tec.placeRobot(robotID, startLoc);
			System.out.println("Placed Robot" + robotID + " in " + startLocName);

			//If path exists and we have cachePaths flag set to true, load and save computed paths
			PoseSteering[] path = null;
			PoseSteering[] pathInv = null;
			rsp.setStart(startLoc);
			rsp.setGoals(endLoc);
			rsp.plan();
			if (rsp.getPath() == null) throw new Error("No path found.");
			path = rsp.getPath();
			pathInv = rsp.getPathInv();

			//Enqueue forward and backwards mission
			Mission m = new Mission(robotID, path, startLocName, endLocName, Missions.getLocationPose(startLocName), Missions.getLocationPose(endLocName));
			Missions.enqueueMission(m);
			Mission m1 = new Mission(robotID, pathInv, endLocName, startLocName, Missions.getLocationPose(endLocName), Missions.getLocationPose(startLocName));
			Missions.enqueueMission(m1);
		}

		System.out.println("Added missions " + Missions.getMissions());
				
		Missions.startMissionDispatchers(tec, robotIDs);

	}

}
