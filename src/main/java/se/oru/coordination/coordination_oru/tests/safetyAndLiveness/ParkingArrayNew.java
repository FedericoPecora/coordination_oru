package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashSet;
import java.util.Random;
import java.util.logging.Level;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import com.vividsolutions.jts.geom.Coordinate;
import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination of robots vertically. Goals are continually posted to robots. Paths are computed by using the ReedsSheppCarPlanner.")
public class ParkingArrayNew {
	
	private static AbstractMotionPlanner createMotionPlanner(Coordinate[] footprint, String yamlFile) {
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRadius(0.1);
		rsp.setFootprint(footprint);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);
		return rsp;
	}
	
	private static void writeStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), true)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	private static void initStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), false)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 2.5;
		
		//Define the radius of the circle
		int numSlots = 8;
		double deltaX = 5.0;
		double deltaY = 15.0;
		double offsetX = 30.0;
		double offsetY = 30.0;
				
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setCheckEscapePoses(true);
		tec.setYieldIfParking(false);
		tec.setBreakDeadlocksByReordering(false);
		tec.setBreakDeadlocksByReplanning(true);
		tec.setCheckCollisions(true);
		//MetaCSPLogging.setLevel(TrajectoryEnvelopeCoordinator.class, Level.FINEST);
		
		//Setup the network parameters
		NetworkConfiguration.setDelays(10, 2000);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.2;
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.02);
				
		//Set the footprint
		Coordinate[] footprint = new Coordinate[] { 
				new Coordinate(-0.5,0.5),
				new Coordinate(0.5,0.5),
				new Coordinate(0.5,-0.5),
				new Coordinate(-0.5,-0.5),
		};
		tec.setDefaultFootprint(footprint);
		
		// Set the FW models and robotIDs
		final int[] robotIDs = new int[] {1,2,3,4,5,6,7,8,9,10,11,12};
		HashSet<Integer> inactiveRobots = new HashSet<Integer>();
//		inactiveRobots.add(1);
//		inactiveRobots.add(2);
//		inactiveRobots.add(3);
//		inactiveRobots.add(4);
//		inactiveRobots.add(5);
//		inactiveRobots.add(6);
//		inactiveRobots.add(7);
//		inactiveRobots.add(8);
//		inactiveRobots.add(9);
//		inactiveRobots.add(10);
		inactiveRobots.add(11);
		inactiveRobots.add(12);
		
		//Set a map
		String yamlFile = "maps/map-empty-circle.yaml";
		
		for (int i : robotIDs) {
			tec.setForwardModel(i, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
			AbstractMotionPlanner rsp_i = createMotionPlanner(footprint, yamlFile);
			tec.setMotionPlanner(i, rsp_i);
		}
				
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(robotIDs);
		//BrowserVisualization viz = new BrowserVisualization();
		//viz.setMap(yamlFile);
		//viz.setInitialTransform(60, 33.58, 13.49);
		tec.setVisualization(viz);
				
		//Determine locations around the circle, with random orientation
		long seed = 123123;//Calendar.getInstance().getTimeInMillis();
		Random rand = new Random(seed);
		ArrayList<String> locationsUp = new ArrayList<String>();
		ArrayList<String> locationsDown = new ArrayList<String>();
		for (int i = 0; i < numSlots; i++) {
			locationsUp.add(i,"locUp_"+i);
			locationsDown.add(i,"locDown_"+i);
			Missions.setLocation(locationsUp.get(i), new Pose(offsetX+i*deltaX, offsetY+deltaY, -Math.PI/2.0));
			Missions.setLocation(locationsDown.get(i), new Pose(offsetX+i*deltaX, offsetY, -Math.PI/2.0));
		}
		
		final String statFilename = System.getProperty("user.home")+File.separator+"stats.txt";
		String header = "Seed random generator: " + seed + "L.\n" + "#";
		for (int robotID : robotIDs) header += (robotID + "\t");
		initStat(statFilename, header);
		
		AbstractMotionPlanner rsp = createMotionPlanner(footprint, yamlFile);
		
		//Here we pre-compute all paths
		for (int i = 0; i < numSlots; i++) {
			for (int j = 0; j < numSlots; j++) {
				//Compute path in both directions
				rsp.setStart(Missions.getLocation(locationsUp.get(i)));
				rsp.setGoals(Missions.getLocation(locationsDown.get(j)));
				if (!rsp.doPlanning()) throw new Error("No path between " + locationsUp.get(i) + "[" + Missions.getLocation(locationsUp.get(i)) + "] and " + locationsDown.get(j) + "[" + Missions.getLocation(locationsDown.get(j)) + "]");
				Missions.addKnownPath(locationsUp.get(i), locationsDown.get(j), rsp.getPath());
				Missions.addKnownPath(locationsDown.get(j), locationsUp.get(i), rsp.getPathInv());
			}			
		}
		
		// Decide initial poses for robots, place the robots, and create missions
		String[] initialLocations = new String[robotIDs.length];
		for (int i = 0; i < robotIDs.length; i++) {
			String upOrDown = rand.nextBoolean() ? "Up" : "Down"; 
			String newLocation = "loc" + upOrDown + "_"+rand.nextInt(numSlots);
			for (int j = 0; j < i; j++) {
				if (initialLocations[j].equals(newLocation)) {
					j = -1;
					upOrDown = rand.nextBoolean() ? "Up" : "Down"; 
					newLocation = "loc" + upOrDown + "_"+rand.nextInt(numSlots);
				}
			}
			initialLocations[i] = newLocation;
		}
		
		HashSet<Pair<String,String>> assigned = new HashSet<Pair<String,String>>(); 
		
		//Set the goals such that two paths cannot overlap
		for (int i = 0; i < initialLocations.length; i++) {
			int robotID = robotIDs[i];
			String initialLocation = initialLocations[i];
			if (!inactiveRobots.contains(robotID)) tec.placeRobot(robotID, Missions.getLocation(initialLocation));
			String upOrDown = initialLocation.contains("Down") ? "Up" : "Down"; 
			String goalLocation = "loc" + upOrDown + "_"+rand.nextInt(numSlots);
			while (initialLocation.equals(goalLocation) || assigned.contains(new Pair<String,String>(goalLocation,initialLocation))) {
				goalLocation = "loc" + upOrDown + "_"+rand.nextInt(numSlots);
			}
			assigned.add(new Pair<String,String>(initialLocation,goalLocation));
			if (!inactiveRobots.contains(robotID)) {
			Mission mFW = new Mission(robotID, Missions.getShortestPath(initialLocation, goalLocation));
			System.out.println("Robot" + robotID+ ", start: " + initialLocation + " ,goal: " + goalLocation);
			Mission mBW = new Mission(robotID, Missions.getShortestPath(goalLocation, initialLocation));
			Missions.enqueueMission(mFW);
			Missions.enqueueMission(mBW);
			}
		}

		for (final int robotID : robotIDs) {
			if (!inactiveRobots.contains(robotID)) {
				tec.setMotionPlanner(robotID,rsp);
				Thread t = new Thread() {
					public void run() {
//						while (true) {
//							synchronized(tec.getSolver()) {
//								if (tec.isFree(robotID)) {
//									Mission m = Missions.dequeueMission(robotID);
//									tec.addMissions(m);
//									tec.computeCriticalSectionsAndStartTrackingAddedMission();
//									Missions.enqueueMission(m);
//								}
//							}
//							try { Thread.sleep(2000); }
//							catch (InterruptedException e) { e.printStackTrace(); }
//						}
						boolean firstTime = true;
						int totalIterations = 12;
						//if (robotID%2 == 0) totalIterations = 19;
						boolean isFree = false;
						long startTime = Calendar.getInstance().getTimeInMillis();
						while (true && totalIterations > 0) {
							synchronized(tec.getSolver()) {
								if (tec.isFree(robotID)) isFree = true;
							}
							//Sleep for a little
							try { Thread.sleep(2000); }
							catch (InterruptedException e) { e.printStackTrace(); }
							if (isFree) {
								isFree = false;
								synchronized(tec.getSolver()) {
									if (!firstTime) {
										long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
										String stat = "";
										for (int i = 1; i < robotID; i++) stat += "\t";
										stat += elapsed;
										//writeStat(statFilename, stat);
									}
									startTime = Calendar.getInstance().getTimeInMillis();
									firstTime = false;
									Mission m = Missions.dequeueMission(robotID);
									tec.addMissions(m);
									Missions.enqueueMission(m);
									totalIterations--;
								}
							}
							//Sleep for a little
							try { Thread.sleep(2000); }
							catch (InterruptedException e) { e.printStackTrace(); }
						}
						System.out.println("Robot" + robotID + " is done!");
						
					}
				};
				t.start();
				try { Thread.sleep(1000); }
				catch (InterruptedException e) { e.printStackTrace(); }
			}
		}
		
	}

}
