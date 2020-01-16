package se.oru.coordination.coordination_oru.examples;

import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class FourRobotsPathPlanning {

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
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(4, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//final BrowserVisualization viz = new BrowserVisualization();
		//viz.setInitialTransform(43, 11, 1.6);
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		HashMap<Integer,Pose> posesFrom = new HashMap<Integer, Pose>();
		HashMap<Integer,Pose> posesTo = new HashMap<Integer, Pose>();
		
		posesFrom.put(1,new Pose(10.0,3.0,Math.PI));
		posesFrom.put(2,new Pose(3.0,10.0,Math.PI/2));
		posesFrom.put(3,new Pose(10.0,17.0,0.0));
		posesFrom.put(4,new Pose(17.0,10.0,-Math.PI/2));

		posesTo.put(1,new Pose(7.0,17.0,-Math.PI));
		posesTo.put(2,new Pose(15.0,3.0,-Math.PI/2));
		posesTo.put(3,new Pose(6.0,12.0,0.0));
		posesTo.put(4,new Pose(7.0,10.0,Math.PI/2));


		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(3.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		
		for (int robotID = 1; robotID <= 4; robotID++) {
			rsp.setStart(posesFrom.get(robotID));
			rsp.setGoals(posesTo.get(robotID));
			if (!rsp.plan()) throw new Error("Cannot plan for Robot" + robotID);
			Mission m = new Mission(robotID, rsp.getPath());
			Missions.enqueueMission(m);
			Mission mInv = new Mission(robotID, rsp.getPathInv());
			Missions.enqueueMission(mInv);
		}
		
		//Missions.saveScenario("FourRobotsExampleScenario");
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, posesFrom.get(1));
		tec.placeRobot(2, posesFrom.get(2));
		tec.placeRobot(3, posesFrom.get(3));
		tec.placeRobot(4, posesFrom.get(4));
		
		tec.setBreakDeadlocksByReordering(true);
		tec.setBreakDeadlocksByReplanning(true);
		tec.setDefaultMotionPlanner(rsp);
		
		Missions.startMissionDispatchers(tec, 1,2,3,4);
		
	}

}
