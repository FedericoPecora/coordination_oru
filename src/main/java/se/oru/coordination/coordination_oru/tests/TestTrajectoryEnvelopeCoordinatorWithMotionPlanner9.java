package se.oru.coordination.coordination_oru.tests;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Scanner;
import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

@DemoDescription(desc = "One-shot navigation of 2 robots showing multiple overlapping critical sections.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner9 {

	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorWithMotionPlanner9.class);
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

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker (also abstract)
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add a comparator to determine robot orderings thru critical sections
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		
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
		rsp.setMapFilename("maps"+File.separator+getProperty("image", yamlFile));
		double res = Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setNumInterpolationPoints(1000);

		Pose startPoseRobot1 = new Pose(12.0,2.0,Math.PI/2);
		Pose goalPoseRobot11 = new Pose(12.0,10.0,Math.PI/4);
		Pose goalPoseRobot12 = new Pose(17.0,7.0,-Math.PI/2);
		Pose goalPoseRobot13 = new Pose(14.0,4.0,Math.PI);
		Pose goalPoseRobot14 = new Pose(12.0,10.0,Math.PI/2);
		Pose goalPoseRobot15 = new Pose(13.0,15.0,Math.PI*3/4);
		Pose startPoseRobot2 = new Pose(3.0,3.0,0.0);
		Pose goalPoseRobot2 = new Pose(20.0,15.0,Math.PI/4);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);

		ArrayList<PoseSteering> pathRobot1 = new ArrayList<PoseSteering>();
		ArrayList<PoseSteering> pathRobot2 = new ArrayList<PoseSteering>();
		rsp.setStart(startPoseRobot1);
		rsp.setGoal(goalPoseRobot11);
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
		rsp.addObstacles(fpGeom, startPoseRobot2, goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot11);
		for (PoseSteering ps : rsp.getPath()) pathRobot1.add(ps);
		rsp.setStart(goalPoseRobot11);
		rsp.setGoal(goalPoseRobot12);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot11 + " and " + goalPoseRobot12);
		for (PoseSteering ps : rsp.getPath()) pathRobot1.add(ps);
		rsp.setStart(goalPoseRobot12);
		rsp.setGoal(goalPoseRobot13);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot12 + " and " + goalPoseRobot13);
		for (PoseSteering ps : rsp.getPath()) pathRobot1.add(ps);
		rsp.setStart(goalPoseRobot13);
		rsp.setGoal(goalPoseRobot14);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot13 + " and " + goalPoseRobot14);
		for (PoseSteering ps : rsp.getPath()) pathRobot1.add(ps);
		rsp.setStart(goalPoseRobot14);
		rsp.setGoal(goalPoseRobot15);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot14 + " and " + goalPoseRobot15);
		for (PoseSteering ps : rsp.getPath()) pathRobot1.add(ps);

		rsp.setStart(startPoseRobot2);
		rsp.setGoal(goalPoseRobot2);
		rsp.clearObstacles();
		rsp.addObstacles(fpGeom, startPoseRobot1, goalPoseRobot15);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		for (PoseSteering ps : rsp.getPath()) pathRobot2.add(ps);

		tec.addMissions(new Mission(1, pathRobot1.toArray(new PoseSteering[pathRobot1.size()])), new Mission(2, pathRobot2.toArray(new PoseSteering[pathRobot2.size()])));
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();

		metaCSPLogger.info("Added missions " + missions);

	}

}
