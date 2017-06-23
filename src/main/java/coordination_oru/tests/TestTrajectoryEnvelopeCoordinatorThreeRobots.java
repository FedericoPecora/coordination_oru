package coordination_oru.tests;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;
import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.utility.logging.MetaCSPLogging;

import coordination_oru.Mission;
import coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public abstract class TestTrajectoryEnvelopeCoordinatorThreeRobots {
	
	private static HashMap<String,Pose> locations = new HashMap<String, Pose>();
	private static HashMap<String,String> paths = new HashMap<String, String>();
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorThreeRobots.class);
	public static HashMap<Integer,ArrayList<Mission>> missions = new HashMap<Integer, ArrayList<Mission>>();

	//Convenience method to put a mission into a global hashmap
	private static void putMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}
	
	//Convenience method to get the i-th mission for a given robotID from the global hashmap	
	private static Mission getMission(int robotID, int missionNumber) {
		return missions.get(robotID).get(missionNumber);
	}
	
	//Load location and path data from a file
	public static void loadLocationAndPathData(String fileName) {
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0 && !line.startsWith("#")) {
					String[] oneline = line.split(" |\t");
					Pose ps = null;
					if (line.contains("->")) {
						paths.put(oneline[0]+oneline[1]+oneline[2], oneline[3]);
						System.out.println("Added to paths paths: " + oneline[0]+oneline[1]+oneline[2] + " --> " + oneline[3]);
					}
					else {
						String locationName = oneline[0];
						ps = new Pose(
							new Double(oneline[1]).doubleValue(),
							new Double(oneline[2]).doubleValue(),
							new Double(oneline[3]).doubleValue());
						locations.put(locationName, ps);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
	}

	//Convenience method to get locations from the data loaded by method loadLocationAndPathData()
	public static Pose getLocation(String name) {
		Pose ret = locations.get(name);
		if (ret == null) throw new Error("Unknown location " + name);
		return ret;
	}
	
	//Convenience method to get path files from the data loaded by method loadLocationAndPathData()
	public static String getPathFile(String fromLocation, String toLocation) {
		String ret = paths.get(fromLocation+"->"+toLocation);
		if (!locations.containsKey(fromLocation)) throw new Error("Unknown location " + fromLocation);
		if (!locations.containsKey(toLocation)) throw new Error("Unknown location " + toLocation);
		if (ret == null) throw new Error("No path between " + fromLocation + " and " + toLocation);
		return ret;
	}
	


	public static void main(String[] args) throws InterruptedException {
		
		//Some constants used by the trajectory envelope trackers and coordinator:
		
		//Instantiate a trajectory envelope coordinator.
		//This requires providing implementations of:
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker (also abstract)
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		// -- the getOrdering() method, which should return a method for prioritizing robots 
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation();
			
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		tec.setupGUI(null);

		//Load data file with locations and pointers to files containing paths between locations
		loadLocationAndPathData("paths/test_poses_and_path_data.txt");
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, getLocation("r1p"), null, "r1p");
		tec.placeRobot(2, getLocation("r2p"), null, "r2p");
		tec.placeRobot(3, getLocation("r3p"), null, "r3p");

		String prefix = "paths/";
		//Make a mission for each robot, and store it in the global hashmap:
		// -- from parking location of robot i (rip)
		// -- to destination location of robot i (desti)
		putMission(new Mission(1, prefix+getPathFile("r1p", "dest1"), "r1p", "dest1", getLocation("r1p"), getLocation("dest1")));
		putMission(new Mission(2, prefix+getPathFile("r2p", "dest2"), "r2p", "dest2", getLocation("r2p"), getLocation("dest2")));
		putMission(new Mission(3, prefix+getPathFile("r3p", "dest3"), "r3p", "dest3", getLocation("r3p"), getLocation("dest3")));

		//Make another mission for each robot, and store it in the global hashmap:
		// -- from destination location of robot i (desti)
		// -- to parking location of robot i (rip)
		putMission(new Mission(1, prefix+getPathFile("dest1", "r1p"), "dest1", "r1p", getLocation("dest1"), getLocation("r1p")));
		putMission(new Mission(2, prefix+getPathFile("dest2", "r2p"), "dest2", "r2p", getLocation("dest2"), getLocation("r2p")));
		putMission(new Mission(3, prefix+getPathFile("dest3", "r3p"), "dest3", "r3p", getLocation("dest3"), getLocation("r3p")));

		metaCSPLogger.info("Added missions " + missions);

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 3; i++) {
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
