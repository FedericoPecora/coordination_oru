package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "One-shot navigation of 3 robots with different footprints coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner12 {

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
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});

		//You can set a footprint that is specific for each robot
		Coordinate[] fp1 = new Coordinate[] {
				new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5)
		};
		Coordinate[] fp2 = new Coordinate[] {
				new Coordinate(0.36, 0.0),
				new Coordinate(0.18, 0.36),
				new Coordinate(-0.18, 0.36),
				new Coordinate(-0.36, 0.0),
				new Coordinate(-0.18, -0.36),
				new Coordinate(0.18, -0.36)
		};
		Coordinate[] fp3 = new Coordinate[] {
				new Coordinate(-2.0,0.9),
				new Coordinate(2.0,0.9),
				new Coordinate(2.0,-0.9),
				new Coordinate(-2.0,-0.9)
		};
		tec.setFootprint(1,fp1);
		tec.setFootprint(2,fp2);
		tec.setFootprint(3,fp3);

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map-empty.yaml";
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		tec.setVisualization(viz);
		viz.setSize(1024, 768);

		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		Pose startPoseRobot1 = new Pose(4.0,6.0,0.0);
		Pose goalPoseRobot1 = new Pose(16.0,15.0,Math.PI/4);
		Pose startPoseRobot2 = new Pose(6.0,16.0,-Math.PI/4);
		Pose goalPoseRobot2 = new Pose(25.0,3.0,-Math.PI/4);
		Pose startPoseRobot3 = new Pose(9.0,6.0,Math.PI/2);
		Pose goalPoseRobot3 = new Pose(21.0,3.0,-Math.PI/2);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, startPoseRobot3);
		
		//Instantiate a motion planner for each robot
		final ReedsSheppCarPlanner rsp1 = new ReedsSheppCarPlanner();
		rsp1.setMap(yamlFile);
		rsp1.setRadius(0.2);
		rsp1.setTurningRadius(4.0);
		rsp1.setDistanceBetweenPathPoints(0.5);
		tec.setMotionPlanner(1, rsp1);

		rsp1.setFootprint(fp1);
		rsp1.setStart(startPoseRobot1);
		rsp1.setGoals(goalPoseRobot1);
		if (!rsp1.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		PoseSteering[] pss1 = rsp1.getPath();

		final ReedsSheppCarPlanner rsp2 = (ReedsSheppCarPlanner) rsp1.getCopy(false);
		tec.setMotionPlanner(2, rsp2);
		rsp2.setFootprint(fp2);
		rsp2.setStart(startPoseRobot2);
		rsp2.setGoals(goalPoseRobot2);
		if (!rsp2.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		PoseSteering[] pss2 = rsp2.getPath();

		final ReedsSheppCarPlanner rsp3 = (ReedsSheppCarPlanner) rsp1.getCopy(false);
		tec.setMotionPlanner(3, rsp3);
		rsp3.setFootprint(fp3);
		rsp3.setStart(startPoseRobot3);
		rsp3.setGoals(goalPoseRobot3);
		if (!rsp3.plan()) throw new Error ("No path between " + startPoseRobot3 + " and " + goalPoseRobot3);
		PoseSteering[] pss3 = rsp3.getPath();

		
		//Add missions
		Mission m1 = new Mission(1,pss1);
		Mission m2 = new Mission(2,pss2);
		Mission m3 = new Mission(3,pss3);
		tec.addMissions(m1,m2,m3);

	}

}
