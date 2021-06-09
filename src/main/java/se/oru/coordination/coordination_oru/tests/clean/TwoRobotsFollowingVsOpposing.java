package se.oru.coordination.coordination_oru.tests.clean;

import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Example showing coordination in opposing directions (following should happen here).")
public class TwoRobotsFollowingVsOpposing {

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
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(31)));
		tec.setForwardModel(4, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(4)));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(46, 4.83, 0);
		tec.setVisualization(viz);

		Pose startRobot1 = new Pose(25.0,5.0,0.0);
		Pose goalRobot11 = new Pose(20.0,7.0,0.0);
		Pose goalRobot12 = new Pose(10.0,7.0,0.0);
		Pose goalRobot13 = new Pose(5.0,5.0,0.0);
		
		Pose startRobot2 = new Pose(25.0,9.0,Math.PI);
		Pose goalRobot21 = new Pose(20.0,7.0,Math.PI);
		Pose goalRobot22 = new Pose(10.0,7.0,Math.PI);
		Pose goalRobot23 = new Pose(5.0,9.0,Math.PI);
		
		Pose startRobot3 = new Pose(5.0,12.0,0.0);
		Pose goalRobot31 = new Pose(10.0,14.0,0.0);
		Pose goalRobot32 = new Pose(20.0,14.0,0.0);
		Pose goalRobot33 = new Pose(25.0,12.0,0.0);
		
		Pose startRobot4 = new Pose(25.0,16.0,Math.PI);
		Pose goalRobot41 = new Pose(20.0,14.0,Math.PI);
		Pose goalRobot42 = new Pose(10.0,14.0,Math.PI);
		Pose goalRobot43 = new Pose(5.0,16.0,Math.PI);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startRobot1);
		tec.placeRobot(2, startRobot2);
		tec.placeRobot(3, startRobot3);
		tec.placeRobot(4, startRobot4);

		//Set up path planner (using empty map)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);

		rsp.setStart(startRobot1);
		rsp.setGoals(goalRobot11,goalRobot12,goalRobot13);
		rsp.plan();
		Missions.enqueueMission(new Mission(1,rsp.getPath()));
		Missions.enqueueMission(new Mission(1,rsp.getPathInv()));

		rsp.setStart(startRobot2);
		rsp.setGoals(goalRobot21,goalRobot22,goalRobot23);
		rsp.plan();
		Missions.enqueueMission(new Mission(2,rsp.getPath()));
		Missions.enqueueMission(new Mission(2,rsp.getPathInv()));
		
		rsp.setStart(startRobot3);
		rsp.setGoals(goalRobot31,goalRobot32,goalRobot33);
		rsp.plan();
		Missions.enqueueMission(new Mission(3,rsp.getPath()));
		Missions.enqueueMission(new Mission(3,rsp.getPathInv()));

		rsp.setStart(startRobot4);
		rsp.setGoals(goalRobot41,goalRobot42,goalRobot43);
		rsp.plan();
		Missions.enqueueMission(new Mission(4,rsp.getPath()));
		Missions.enqueueMission(new Mission(4,rsp.getPathInv()));
		
		System.out.println("Added missions " + Missions.getMissions());
		
		Missions.startMissionDispatchers(tec, 1,2,3,4);
		
	}

}
