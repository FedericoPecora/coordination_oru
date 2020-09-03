package se.oru.coordination.coordination_oru.tests.icaps2018.talk;

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
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner.PLANNING_ALGORITHM;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "One-shot navigation of 3 robots with different footprints coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class OneRobotPlacement {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
		//ST-18 Footprint
		Coordinate[] st18footprint = new Coordinate[] {
				new Coordinate(-5.535,-1.450),
				new Coordinate(-5.535,1.450),
				new Coordinate(2.950,1.450),
				new Coordinate(2.950,1.665),
				new Coordinate(5.891,1.665),
				new Coordinate(5.891,-1.665),
				new Coordinate(2.950,-1.665),
				new Coordinate(2.950,-1.450),
				new Coordinate(-5.535,-1.450)
		};
				
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

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		String mapYamlFile = "maps/map-partial-2.yaml";
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		//viz.setInitialTransform(10, 25, 10);
		viz.setInitialTransform(20.0, 9.0, 2.0);
		tec.setVisualization(viz);
		viz.setMap(mapYamlFile);
		
		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner(PLANNING_ALGORITHM.PRMstar);
		rsp.setPlanningTimeInSecs(5);
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);

		Pose startPoseRobot1 = new Pose(5.0,10.0,0.0);
		Pose goalPoseRobot1 = new Pose(74.0,10.0,0.0);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);

		rsp.setFootprint(fp1);
		rsp.setMap(mapYamlFile);
//		rsp.setMapFilename("maps/map-partial-2.png");
//		rsp.setMapResolution(0.1);
		
		rsp.setStart(startPoseRobot1);
		rsp.setGoals(goalPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		PoseSteering[] pss1 = rsp.getPath();

		Mission m1 = new Mission(1,pss1);
		Missions.enqueueMission(m1);
		Missions.startMissionDispatchers(tec, 1);

	}

}
