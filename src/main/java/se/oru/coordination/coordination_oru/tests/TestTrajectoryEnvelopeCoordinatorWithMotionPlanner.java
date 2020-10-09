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
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Coordination on paths obtained from the ReedsSheppCarPlanner for two robots navigating in opposing directions.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner {

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

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map.yaml";
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);
		tec.setBreakDeadlocks(false, true, true);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMap(yamlFile);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.05);
		//rsp.setRobotRadius(0.5);
		//rsp.setCirclePositions(new Coordinate(-1.5,0.0), new Coordinate(0.0,0.0), new Coordinate(1.5,0.0));
		rsp.setRadius(0.5);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		//rsp.setCirclePositions(footprint1, footprint2, footprint3, footprint4);

		Pose startPoseRobot1 = new Pose(2.0,28.0,0.0);
		Pose goalPoseRobot1 = new Pose(5.0,1.0,0.0);
		Pose startPoseRobot2 = new Pose(2.0,38.0,0.0);
		Pose goalPoseRobot2 = new Pose(2.5,3.0,0.0);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);

		rsp.setStart(startPoseRobot1);
		rsp.setGoals(goalPoseRobot1);
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
		rsp.addObstacles(fpGeom, startPoseRobot2, goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		PoseSteering[] pss1 = rsp.getPath();
		rsp.setStart(goalPoseRobot1);
		rsp.setGoals(startPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot1 + " and " + startPoseRobot1);
		PoseSteering[] pss1Inv = rsp.getPath();

		rsp.setStart(startPoseRobot2);
		rsp.setGoals(goalPoseRobot2);
		rsp.clearObstacles();
		//rsp.addObstacles(fpGeom, startPoseRobot1, goalPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		PoseSteering[] pss2 = rsp.getPath();
		rsp.setStart(goalPoseRobot2);
		rsp.setGoals(startPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot2 + " and " + startPoseRobot2);
		PoseSteering[] pss2Inv = rsp.getPath();

		Missions.enqueueMission(new Mission(1, pss1));
		Missions.enqueueMission(new Mission(1, pss1Inv));

		Missions.enqueueMission(new Mission(2, pss2));
		Missions.enqueueMission(new Mission(2, pss2Inv));

		System.out.println("Added missions " + Missions.getMissions());

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 2; i++) {
			final int robotID = i;
			//Set each robot motion planner
			tec.setMotionPlanner(robotID, rsp.getCopy(false));
			
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = Missions.getMission(robotID, iteration%2);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) iteration++;
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
