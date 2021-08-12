package se.oru.coordination.coordination_oru.tests.debug;

import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;

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
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Example showing coordination with completely overlapping trajectory envelopes in opposing directions (should lead to blocking).")
public class TwoRobotsTotalOverlapOpposing {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 2.0;
		double MAX_VEL = 3.0;
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
		tec.setUseInternalCriticalPoints(false);
		
//		NetworkConfiguration.setDelays(0, 3000);
//		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.0;
//		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0);
		
		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(-0.5,-0.5);
		Coordinate footprint3 = new Coordinate(0.7,-0.5);
		Coordinate footprint4 = new Coordinate(0.7,0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(2)));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		
		//Error: FIXME! Lost dependency and order cannot be restored! Key value not found.
		tec.setBreakDeadlocks(false, false, false);
		
//		//Error: FIXME! Lost dependency and order cannot be restored! Key value not found.
//		tec.setBreakDeadlocks(false, true, false);

//		//Error: FIXME! Lost dependency and order cannot be restored! Key value not found.
//		tec.setBreakDeadlocks(false, true, true);

//		//Error: FIXME! Lost dependency and order cannot be restored! Key value not found.
//		tec.setBreakDeadlocks(false, false, true);
		
//		//OK! One robot proceeds, but not the other, and the first robot yields
//		tec.setBreakDeadlocks(true, false, false);

		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(40.6, -1.26, 4.5);
		tec.setVisualization(viz);

		Pose pose_45 = new Pose(45.0,0.0,0.0);
		Pose pose_40 = new Pose(40.0,0.0,0.0);
		Pose pose_10 = new Pose(10.0,0.0,0.0);
		Pose pose_5 = new Pose(5.0,0.0,0.0);
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, pose_5);
		tec.placeRobot(2, pose_45);

		//Set up path planner (using empty map)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		
		rsp.setStart(pose_5);
		rsp.setGoals(pose_45);
		rsp.plan();
		Missions.enqueueMission(new Mission(1,rsp.getPath()));

		rsp.setStart(pose_45);
		rsp.setGoals(pose_5);
		rsp.plan();
		Missions.enqueueMission(new Mission(2,rsp.getPath()));

		System.out.println("Added missions " + Missions.getMissions());

		Missions.startMissionDispatchers(tec, false, 1,2);
		
	}

}
