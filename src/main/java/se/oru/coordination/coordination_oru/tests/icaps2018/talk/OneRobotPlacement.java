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
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;

@DemoDescription(desc = "One-shot navigation of 3 robots with different footprints coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class OneRobotPlacement {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
		//You can set a footprint that is specific for each robot
		Coordinate[] fp1 = new Coordinate[] {
				new Coordinate(0.0,0.0),
				new Coordinate(5.0,0.0),
				new Coordinate(5.0,5.0),
				new Coordinate(0.0,5.0)
		};
		tec.setFootprint(1,fp1);

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(10, 10, 10);
		tec.setVisualization(viz);
		
		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);

		Pose startPoseRobot1 = new Pose(0.0,0.0,0.0);
		Pose goalPoseRobot1 = new Pose(10.0,0.0,0.0);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);

//		rsp.setFootprint(fp1);
//		rsp.setStart(startPoseRobot1);
//		rsp.setGoals(goalPoseRobot1);
//		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
//		PoseSteering[] pss1 = rsp.getPath();
//
//		Mission m1 = new Mission(1,pss1);
//		tec.addMissions(m1);
//		tec.computeCriticalSections();
//		tec.startTrackingAddedMissions();

	}

}
