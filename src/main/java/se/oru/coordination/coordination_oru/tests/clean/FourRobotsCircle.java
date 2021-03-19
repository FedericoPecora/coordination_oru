package se.oru.coordination.coordination_oru.tests.clean;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.sat4j.ExitCode;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class FourRobotsCircle {

		public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;

		//Instantiate a trajectory envelope coordinator
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);

		//Provide a heuristic for determining orderings thru critical sections
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

		//Provide a conservative forward model for each robot
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(2)));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(3)));
		tec.setForwardModel(4, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(4)));

		//Define robot geometries (here, the same for all robots)
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Set up a simple GUI
		final BrowserVisualization viz = new BrowserVisualization();
		//viz.setInitialTransform(43, 11, 1.6);
		tec.setVisualization(viz);

		//If set to true, attempts to make simulated robots slow down at cusps (this is buggy, but only affects
		//the 2D simulation, not the coordination)
		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		//rsp.setMap("maps/map-empty.yaml");

		//Define poses for the scenario
		Pose startPoseRobot1 = new Pose(10.0,3.0,Math.PI);
		Pose goalPoseRobot1 = new Pose(3.0,10.0,Math.PI/2);
		Pose startPoseRobot2 = new Pose(3.0,10.0,Math.PI/2);
		Pose goalPoseRobot2 = new Pose(10.0,17.0,0.0);
		Pose startPoseRobot3 = new Pose(10.0,17.0,0.0);
		Pose goalPoseRobot3 = new Pose(17.0,10.0,-Math.PI/2);
		Pose startPoseRobot4 = new Pose(17.0,10.0,-Math.PI/2);
		Pose goalPoseRobot4 = new Pose(10.0,3.0,Math.PI);		

		viz.guessInitialTransform(2, startPoseRobot1, startPoseRobot2, startPoseRobot3, startPoseRobot4);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, startPoseRobot3);
		tec.placeRobot(4, startPoseRobot4);

		ArrayList<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();

		rsp.setStart(startPoseRobot1);
		rsp.setGoals(goalPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		PoseSteering[] pss1 = rsp.getPath();
		paths.add(pss1);

		rsp.setStart(startPoseRobot2);
		rsp.setGoals(goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		PoseSteering[] pss2 = rsp.getPath();
		paths.add(pss2);

		rsp.setStart(startPoseRobot3);
		rsp.setGoals(goalPoseRobot3);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot3 + " and " + goalPoseRobot3);
		PoseSteering[] pss3 = rsp.getPath();
		paths.add(pss3);

		rsp.setStart(startPoseRobot4);
		rsp.setGoals(goalPoseRobot4);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot4 + " and " + goalPoseRobot4);
		PoseSteering[] pss4 = rsp.getPath();
		paths.add(pss4);
		
		Missions.enqueueMission(new Mission(1, pss1));
		Missions.enqueueMission(new Mission(1, pss2));
		Missions.enqueueMission(new Mission(1, pss3));
		Missions.enqueueMission(new Mission(1, pss4));

		Missions.enqueueMission(new Mission(2, pss2));
		Missions.enqueueMission(new Mission(2, pss3));
		Missions.enqueueMission(new Mission(2, pss4));
		Missions.enqueueMission(new Mission(2, pss1));
		
		Missions.enqueueMission(new Mission(3, pss3));
		Missions.enqueueMission(new Mission(3, pss4));
		Missions.enqueueMission(new Mission(3, pss1));
		Missions.enqueueMission(new Mission(3, pss2));
		
		Missions.enqueueMission(new Mission(4, pss4));
		Missions.enqueueMission(new Mission(4, pss1));
		Missions.enqueueMission(new Mission(4, pss2));
		Missions.enqueueMission(new Mission(4, pss3));
		
		Missions.startMissionDispatchers(tec, 1,2,3,4);

	}

}
