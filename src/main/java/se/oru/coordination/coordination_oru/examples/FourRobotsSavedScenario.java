package se.oru.coordination.coordination_oru.examples;

import java.util.Comparator;
import java.util.HashMap;
import java.util.Set;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class FourRobotsSavedScenario {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;

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

		Coordinate[] footprint = new Coordinate[] {
				new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5),
		};
		tec.setDefaultFootprint(footprint);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		final BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(43, 11, 0.2);
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);

		//Load scenario (obtained from FourRobotsPathPlanning example)
		Missions.loadScenario("FourRobotsExampleScenario");

		//Put robots in their initial poses (= first pose of each robot's first mission)
		HashMap<Integer,Pose> initPoses = Missions.getInitialPoses();
		Set<Integer> robotIDs = initPoses.keySet();
		for (Integer robotID : robotIDs) {
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));			
			tec.placeRobot(robotID, initPoses.get(robotID));
		}

		tec.setBreakDeadlocksByReordering(true);
		
		tec.setBreakDeadlocksByReplanning(true);
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint);
		rsp.setTurningRadius(3.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		tec.setDefaultMotionPlanner(rsp);

		//For each robot, add a callback which will print additional information
		//in the visualization (the number of missions completed)
		for (Integer robotID : robotIDs) {

			final int myRobotID = robotID;

			tec.addTrackingCallback(robotID, new TrackingCallback() {

				private int missionNumber = 0;

				@Override
				public void onTrackingStart() {
					if (tec.isDriving(myRobotID)) missionNumber++;
				}

				@Override
				public void onTrackingFinished() { }

				@Override
				public String[] onPositionUpdate() {				
					//return new String[0];
					return new String[] { "(" + missionNumber + ")" };
				}

				@Override
				public void onNewGroundEnvelope() { }

				@Override
				public void beforeTrackingStart() { }

				@Override
				public void beforeTrackingFinished() { }
			});
		}

		//Start mission dispatchers (loop through all missions forever)
		Missions.startMissionDispatchers(tec, 1,2,3,4);
	
	}

}
