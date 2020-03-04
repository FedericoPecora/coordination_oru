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
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class ReplanningWithoutMap {

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

		double scale = 1.0;
		Coordinate[] footprint = new Coordinate[] {
				new Coordinate(-1.0*scale,0.5*scale),
				new Coordinate(1.0*scale,0.5*scale),
				new Coordinate(1.0*scale,-0.5*scale),
				new Coordinate(-1.0*scale,-0.5*scale),
		};
		tec.setDefaultFootprint(footprint);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		final BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(43, 11, 1.6);
		//viz.setMap("maps/combined3.yaml");
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		HashMap<Integer,Pose> posesFrom = new HashMap<Integer, Pose>();
		HashMap<Integer,Pose> posesTo = new HashMap<Integer, Pose>();

		posesFrom.put(1,new Pose(10.0,3.0,Math.PI));
		posesTo.put(1,new Pose(7.0,16.5,-Math.PI));
		
		posesFrom.put(2,new Pose(12.0,4.5,Math.PI));
		posesTo.put(2,new Pose(9.5,16.5,-Math.PI));

		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint);
		rsp.setTurningRadius(3.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		//rsp.setMap("maps/combined3.yaml");

		for (int robotID : posesFrom.keySet()) {
			rsp.setStart(posesFrom.get(robotID));
			rsp.setGoals(posesTo.get(robotID));
			if (!rsp.plan()) throw new Error("Cannot plan for Robot" + robotID);
			Mission m = new Mission(robotID, rsp.getPath());
			Missions.enqueueMission(m);
			Mission mInv = new Mission(robotID, rsp.getPathInv());
			Missions.enqueueMission(mInv);
			tec.placeRobot(robotID, posesFrom.get(robotID));
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		}

//		tec.setAvoidDeadlocksGlobally(true);
		tec.setBreakDeadlocksByReordering(true);
		tec.setBreakDeadlocksByReplanning(true);
		tec.setDefaultMotionPlanner(rsp);

		int[] robotIDs = new int[posesFrom.size()];
		int i = 0;
		for (Integer robotID : posesFrom.keySet()) robotIDs[i++] = robotID;
		Missions.startMissionDispatchers(tec, robotIDs);

	}

}
