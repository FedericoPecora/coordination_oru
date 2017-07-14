package se.oru.coordination.coordination_oru.tests;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;
import java.util.logging.Logger;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public abstract class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner4 {

	public static int MIN_DELAY = 500;
	public static int MAX_DELAY = 0;

	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorWithMotionPlanner4.class);
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
				if (key.equals(property)) {
					ret = value;
					break;
				}
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
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(4.0,1.0);
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map-empty.yaml";
		tec.setupGUI(yamlFile);
		//tec.setupGUI(null);

		tec.setUseInternalCriticalPoints(false);
		
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		String mapFile = "maps"+File.separator+getProperty("image", yamlFile);
		rsp.setMapFilename(mapFile);
		double res = Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setNumInterpolationPoints(100);
		double deltaY = 3;
		double height = deltaY/2;
		double mapHeight = -1;
		
		try {
			BufferedImage img = ImageIO.read(new File(mapFile));
			mapHeight = img.getHeight()*res*0.9;
		}
		catch (IOException e) { e.printStackTrace(); }

		
		int[] robotIDs = new int[] {1,2,3,4};
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			ArrayList<Pose> posesRobot = new ArrayList<Pose>();
			//if (index%2==0) {
			if (robotID%2==0) {
				posesRobot.add(new Pose(2.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(10.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(18.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(26.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(34.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(42.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(50.0,mapHeight-deltaY-height*index,0.0));
			}
			else {
				posesRobot.add(new Pose(50.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(42.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(34.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(26.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(18.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(10.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(2.0,mapHeight-height*(index-1),Math.PI));
			}
			tec.placeRobot(robotID, posesRobot.get(0));
			ArrayList<PoseSteering> pathsRobot = new ArrayList<PoseSteering>();
			for (int i = 0; i < posesRobot.size()-1; i++) {
				rsp.setStart(posesRobot.get(i));
				rsp.setGoal(posesRobot.get(i+1));
				rsp.clearObstacles();
				if (!rsp.plan()) throw new Error ("No path between " + posesRobot.get(i) + " and " + posesRobot.get(i+1));			
				PoseSteering[] path = rsp.getPath();
				if (i == 0) pathsRobot.add(path[0]);
				for (int j = 1; j < path.length; j++) pathsRobot.add(path[j]);
			}
			ArrayList<PoseSteering> pathsRobotInv = new ArrayList<PoseSteering>();
			pathsRobotInv.addAll(pathsRobot);
			Collections.reverse(pathsRobotInv);

			putMission(new Mission(robotID, pathsRobot.toArray(new PoseSteering[pathsRobot.size()])));
			putMission(new Mission(robotID, pathsRobotInv.toArray(new PoseSteering[pathsRobotInv.size()])));
		}

		metaCSPLogger.info("Added missions " + missions);

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
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
								if (MAX_DELAY-MIN_DELAY > 0) {
									long delay = MIN_DELAY+rand.nextInt(MAX_DELAY-MIN_DELAY);
									//Sleep for a random delay in [minDelay,maxDelay]
									try { Thread.sleep(delay); }
									catch (InterruptedException e) { e.printStackTrace(); }
								}
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
