package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "One-shot navigation of 2 robots showing multiple overlapping critical sections.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner9 {
	
	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
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
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

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
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);

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

		rsp.setStart(startPoseRobot1);
		rsp.setGoals(goalPoseRobot11,goalPoseRobot12,goalPoseRobot13,goalPoseRobot14,goalPoseRobot15);
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
		rsp.addObstacles(fpGeom, startPoseRobot2, goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot15);
		PoseSteering[] pathRobot1 = rsp.getPath();
		
		rsp.setStart(startPoseRobot2);
		rsp.setGoals(goalPoseRobot2);
		rsp.clearObstacles();
		rsp.addObstacles(fpGeom, startPoseRobot1, goalPoseRobot15);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		PoseSteering[] pathRobot2 = rsp.getPath();
		
		tec.addMissions(new Mission(1, pathRobot1), new Mission(2, pathRobot2));
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();

	}

}
