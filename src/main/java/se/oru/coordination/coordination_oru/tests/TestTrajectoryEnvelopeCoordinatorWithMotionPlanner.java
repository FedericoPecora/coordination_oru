package se.oru.coordination.coordination_oru.tests;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;
import java.util.logging.Logger;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public abstract class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner {
	
	private static HashMap<String,Pose> locations = new HashMap<String, Pose>();
	private static HashMap<String,String> paths = new HashMap<String, String>();
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorWithMotionPlanner.class);
	public static HashMap<Integer,ArrayList<Mission>> missions = new HashMap<Integer, ArrayList<Mission>>();

	//Get property from YAML file
	public static String getProperty(String property, String yamlFile) {
		String ret = null;
		try {
			File file = new File(yamlFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String st;
			while((st=br.readLine()) != null){
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals(property)) ret = value;
			}
			br.close();
		}
		catch (IOException e) { e.printStackTrace(); }
		return ret;
	}
	
	//Convenience method to put a mission into a global hashmap
	private static void putMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}
	
	//Convenience method to get the i-th mission for a given robotID from the global hashmap	
	private static Mission getMission(int robotID, int missionNumber) {
		return missions.get(robotID).get(missionNumber);
	}
	
	public static void main(String[] args) throws InterruptedException {
		
		//Some constants used by the trajectory envelope trackers and coordinator:
		
		//Instantiate a trajectory envelope coordinator.
		//This requires providing implementations of:
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker (also abstract)
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		// -- the getOrdering() method, which should return a method for prioritizing robots 
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation();
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map.yaml";
		tec.setupGUI(yamlFile);
		//tec.setupGUI(null);
				
		//Instantiate a simple motion planner
		ReedsSheppPlanner rsp = new ReedsSheppPlanner();
		rsp.setMapFilename("maps"+File.separator+getProperty("image", yamlFile));
		double res = Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		
		Pose startPose1 = new Pose(2.0,28.0,0.0);
		Pose goalPose1 = new Pose(1.0,1.0,0.0);
		Pose startPose2 = new Pose(2.0,38.0,0.0);
		Pose goalPose2 = new Pose(2.0,3.0,0.0);
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPose1, null, "r1p");
		tec.placeRobot(2, startPose2, null, "r2p");

		rsp.setStart(startPose1);
		rsp.setGoal(goalPose1);
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
		rsp.addObstacles(fpGeom, startPose2, goalPose2);
		if (!rsp.plan()) throw new Error ("No path between " + startPose1 + " and " + goalPose1);
		PoseSteering[] pss1 = rsp.getPath();
		rsp.setStart(goalPose1);
		rsp.setGoal(startPose1);
		if (!rsp.plan()) throw new Error ("No path between " + goalPose1 + " and " + startPose1);
		PoseSteering[] pss1Inv = rsp.getPath();

		rsp.setStart(startPose2);
		rsp.setGoal(goalPose2);
		rsp.clearObstacles();
		rsp.addObstacles(fpGeom, startPose1, goalPose1);
		if (!rsp.plan()) throw new Error ("No path between " + startPose2 + " and " + goalPose2);
		PoseSteering[] pss2 = rsp.getPath();
		rsp.setStart(goalPose2);
		rsp.setGoal(startPose2);
		if (!rsp.plan()) throw new Error ("No path between " + goalPose2 + " and " + startPose2);
		PoseSteering[] pss2Inv = rsp.getPath();

		putMission(new Mission(1, pss1, "r1p", "dest1", startPose1, goalPose1));
		putMission(new Mission(1, pss1Inv, "dest1", "r1p", goalPose1, startPose1));

		putMission(new Mission(2, pss2, "r2p", "dest2", startPose2, goalPose2));
		putMission(new Mission(2, pss2Inv, "dest2", "r2p", goalPose2, startPose2));

		metaCSPLogger.info("Added missions " + missions);

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 2; i++) {
			final int robotID = i;
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = getMission(robotID, iteration%2);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
								iteration++;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}
				
	}
	
}
