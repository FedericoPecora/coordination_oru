package se.oru.coordination.coordination_oru.tests.icaps2018.talk;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class ParkingArray {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 4.0;
		
		//Define the radius of the circle
		int numLocations = 8;
		int numRobots = 8;
				
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);
		tec.setBreakDeadlocksByReordering(false);
		tec.setBreakDeadlocksByReplanning(true);
		//Enable checking for collisions
		tec.setCheckCollisions(true);
		
		//Setup the network parameters
		NetworkConfiguration.setDelays(0, 0);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0;
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.1);
		
		//Set the footprint
		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(0.5,0.5);
		Coordinate footprint3 = new Coordinate(0.5,-0.5);
		Coordinate footprint4 = new Coordinate(-0.5,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		// Set the FW models and robotIDs
		final int[] robotIDs = new int[numRobots];
		for (int i = 0; i < numRobots; i++) {
			robotIDs[i] = i+1;
			tec.setForwardModel(i+1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getTrackingPeriod()));
		}
		
		//Set a map
		String yamlFile = "maps/map-empty-circle.yaml";
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		//RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(robotIDs);
		BrowserVisualization viz = new BrowserVisualization();
		//viz.setMap(yamlFile);
//		viz.setInitialTransform(60, 33.58, 13.49);
		viz.setInitialTransform(60, 15, 0.5);
		tec.setVisualization(viz);
		
		//Determine locations around the circle, with random orientation
		long seed = 123123;// Calendar.getInstance().getTimeInMillis();
		System.out.println("Seed random generator: " + seed);
		Random rand = new Random(seed);
		double dx = 1;
		double dy = 10;
		double[] origin = {0,10};
		for (int i = 0; i < numLocations; i++) {
			Missions.setLocation("start_"+i, new Pose(origin[0]+i*dx, origin[1], 0));
			Missions.setLocation("goal_"+i, new Pose(origin[0]+i*dx, origin[1]+dy, 0));
		}
		
		//Set up motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRadius(0.1);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		
		//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
		tec.setMotionPlanner(rsp);
		
		for (String loc1 : Missions.getLocations().keySet()) {
			for (String loc2 : Missions.getLocations().keySet()) {
				if (loc1.contains("start_") && loc1.contains("goal_")) {
					//Compute path in both directions
					rsp.setStart(Missions.getLocation(loc1));
					rsp.setGoals(Missions.getLocation(loc2));
					if (!rsp.doPlanning()) throw new Error("No path between " + loc1 + " and " + loc2);
					Missions.addKnownPath(loc1, loc2, rsp.getPath());
					Missions.addKnownPath(loc2, loc1, rsp.getPathInv());
				}
			}			
		}
		
		// Decide initial poses for robots, place the robots, and create missions
		String[] initialLocations = new String[numRobots];
		for (int i = 0; i < numRobots; i++) {
			String newLocation = "start_"+rand.nextInt(numLocations);
			for (int j = 0; j < i; j++) {
				if (initialLocations[j].equals(newLocation)) {
					j = -1;
					newLocation = "start_"+rand.nextInt(numLocations);
				}
			}
			initialLocations[i] = newLocation;
		}
		System.out.println(initialLocations.toString());
		
		String[] finalLocations = new String[numRobots];
		for (int i = 0; i < numRobots; i++) {
			String newLocation = "goal_"+rand.nextInt(numLocations);
			for (int j = 0; j < i; j++) {
				if (initialLocations[j].equals(newLocation)) {
					j = -1;
					newLocation = "goal_"+rand.nextInt(numLocations);
				}
			}
			finalLocations[i] = newLocation;
		}
		System.out.println(initialLocations.toString());
		

		//Set the goals such that two paths cannot overlap
		for (int i = 0; i < initialLocations.length; i++) {
			int robotID = robotIDs[i];
			String initialLocation = initialLocations[i];
			tec.placeRobot(robotID, Missions.getLocation(initialLocation));
			Mission mFW = new Mission(robotID, Missions.getShortestPath(initialLocation, finalLocations[i]));
			Mission mBW = new Mission(robotID, Missions.getShortestPath(finalLocations[i], initialLocation));
			Missions.enqueueMission(mFW);
			Missions.enqueueMission(mBW);
		}

		for (final int robotID : robotIDs) {
			Thread t = new Thread() {
				public void run() {
					while (true) {
						if (tec.isFree(robotID)) {
							Mission m = Missions.dequeueMission(robotID);
							tec.addMissions(m);
							tec.computeCriticalSectionsAndStartTrackingAddedMission();
							Missions.enqueueMission(m);
						}
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			t.start();
		}
		
	}
}
