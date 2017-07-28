package se.oru.coordination.coordination_oru.tests;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.logging.Logger;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public abstract class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner5 {

	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorWithMotionPlanner5.class);
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
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker (also abstract)
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to provide the implementation of:
		// -- the getOrdering() method, which should return a method for prioritizing robots 
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(4.0,1.0)
		{
			@Override
			public Comparator<AbstractTrajectoryEnvelopeTracker> getOrdering() {
				Comparator<AbstractTrajectoryEnvelopeTracker> comp = new Comparator<AbstractTrajectoryEnvelopeTracker>() {
					@Override
					public int compare(AbstractTrajectoryEnvelopeTracker o1, AbstractTrajectoryEnvelopeTracker o2) {
						if (o2.getRobotReport().getVelocity() > o1.getRobotReport().getVelocity()) return 1;
						if (o1.getRobotReport().getVelocity() > o2.getRobotReport().getVelocity()) return -1;
						return 0;
					}
				};
				return comp;
			}
		};

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
		rsp.setNumInterpolationPoints(1000);
		double minDim = -1;
		double delta = 3;

		try {
			BufferedImage img = ImageIO.read(new File(mapFile));
			minDim = 0.7*Math.min(img.getHeight()*res, img.getWidth()*res);
		}
		catch (IOException e) { e.printStackTrace(); }

		Pose startPoseRobot1 = new Pose(10.0,3.0,Math.PI);
		Pose goalPoseRobot1 = new Pose(3.0,10.0,Math.PI/2);
		Pose startPoseRobot2 = new Pose(3.0,10.0,Math.PI/2);
		Pose goalPoseRobot2 = new Pose(10.0,17.0,0.0);
		Pose startPoseRobot3 = new Pose(10.0,17.0,0.0);
		Pose goalPoseRobot3 = new Pose(17.0,10.0,-Math.PI/2);
		Pose startPoseRobot4 = new Pose(17.0,10.0,-Math.PI/2);
		Pose goalPoseRobot4 = new Pose(10.0,3.0,Math.PI);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, startPoseRobot3);
		tec.placeRobot(4, startPoseRobot4);

		ArrayList<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();

		rsp.setStart(startPoseRobot1);
		rsp.setGoal(goalPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		PoseSteering[] pss1 = rsp.getPath();
		paths.add(pss1);

		rsp.setStart(startPoseRobot2);
		rsp.setGoal(goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		PoseSteering[] pss2 = rsp.getPath();
		paths.add(pss2);

		rsp.setStart(startPoseRobot3);
		rsp.setGoal(goalPoseRobot3);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot3 + " and " + goalPoseRobot3);
		PoseSteering[] pss3 = rsp.getPath();
		paths.add(pss3);

		rsp.setStart(startPoseRobot4);
		rsp.setGoal(goalPoseRobot4);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot4 + " and " + goalPoseRobot4);
		PoseSteering[] pss4 = rsp.getPath();
		paths.add(pss4);

		//Start a mission dispatching thread for each robot, which will run forever
		int iteration = 0;
		while(true) {
			for (int i = 0; i < 4; i++) {
				while(!tec.addMissions(new Mission(i+1,paths.get((i+iteration)%4)))) {
					//Sleep for a little (2 sec)
					try { Thread.sleep(500); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
			}			
			tec.computeCriticalSections();
			tec.startTrackingAddedMissions();
			iteration++;
		}				
	}

}
