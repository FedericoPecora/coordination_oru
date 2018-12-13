package se.oru.coordination.coordination_oru.tests.icaps2018.talk;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination of robots along a circle. Start and goals are placed along the circumference of a circle with a fixed radious, and are continually posted to robots. Paths are computed by using the ReedsSheppCarPlanner.")
public class Circle {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 4.0;
		
		//Define the radius of the circle
		double radius = 15.0;
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
		tec.setBreakDeadlocksByReordering(true);
		tec.setBreakDeadlocksByReplanning(true);
		//Enable checking for collisions
		tec.setCheckCollisions(true);
		
		//Setup the network parameters
		//NetworkConfiguration.setDelays(50, 3000);
		//NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.5;
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
			tec.setForwardModel(i+1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), 2*(tec.getControlPeriod()+tec.getMaxTxDelay()+tec.getTrackingPeriod())));
		}

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		//RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(robotIDs);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(60, 33.58, 13.49);
		tec.setVisualization(viz);
		
		//Determine locations around the circle, with random orientation
		Random rand = new Random(123123);
		double angle = 2*Math.PI/numLocations;
		for (int i = 0; i < numLocations; i++) {
			double theta = 2*Math.PI*rand.nextDouble();
			Missions.setLocation("loc_"+i, new Pose(radius*Math.cos(angle*i), radius*Math.sin(angle*i), theta));
		}
		//System.out.println(">>>>>>>>>>>>>" + Arrays.toString(initialLocations));
		
		//Set up motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename(null);
		rsp.setRadius(0.1);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		
		//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
		tec.setMotionPlanner(rsp);
		
		for (String loc1 : Missions.getLocations().keySet()) {
			for (String loc2 : Missions.getLocations().keySet()) {
				if (!loc1.equals(loc2)) {
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
			String newLocation = "loc_"+rand.nextInt(numLocations);
			for (int j = 0; j < i; j++) {
				if (initialLocations[j].equals(newLocation)) {
					j = -1;
					newLocation = "loc_"+rand.nextInt(numLocations);
				}
			}
			initialLocations[i] = newLocation;
		}
		
		//System.out.println(">>>>>>>>>>>>>" + Arrays.toString(initialLocations));
		
		for (int i = 0; i < initialLocations.length; i++) {
			int robotID = robotIDs[i];
			String initialLocation = initialLocations[i];
			tec.placeRobot(robotID, Missions.getLocation(initialLocation));
			String goalLocation = "loc_"+rand.nextInt(numLocations);
			while (initialLocation.equals(goalLocation)) goalLocation = "loc_"+rand.nextInt(numLocations);
			Mission mFW = new Mission(robotID, Missions.getShortestPath(initialLocation, goalLocation));
			Mission mBW = new Mission(robotID, Missions.getShortestPath(goalLocation, initialLocation));
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
