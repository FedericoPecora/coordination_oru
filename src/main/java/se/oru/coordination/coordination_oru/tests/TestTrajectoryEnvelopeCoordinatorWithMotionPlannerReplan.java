package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Dependency;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Example showing coordination in opposing directions (following should not happen here).")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlannerReplan {

	public static void main(String[] args) throws InterruptedException {

		//Robot footprint
		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(-0.5,-0.5);
		Coordinate footprint3 = new Coordinate(0.7,-0.5);
		Coordinate footprint4 = new Coordinate(0.7,0.5);
		
		//Set up path planner (using empty map)
		final ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		String yamlFile = "maps/map-empty.yaml";
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);
				
		double MAX_ACCEL = 2.0;
		double MAX_VEL = 3.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL) {
			@Override
			protected void rePlanPath(final Dependency dep) {
				
				new Thread() {
					public void run() {
						int replannedRobot = -1;
						
						AbstractTrajectoryEnvelopeTracker tetWaiting = dep.getWaitingTracker();
						PoseSteering[] oldPathWaiting = tetWaiting.getTrajectoryEnvelope().getTrajectory().getPoseSteering();
						Pose currentPoseWaiting = oldPathWaiting[tetWaiting.getRobotReport().getPathIndex()].getPose();
						Pose originalGoalWaiting = oldPathWaiting[oldPathWaiting.length-1].getPose();

						AbstractTrajectoryEnvelopeTracker tetDriving = dep.getDrivingTracker();
						PoseSteering[] oldPathDriving = tetDriving.getTrajectoryEnvelope().getTrajectory().getPoseSteering();
						Pose currentPoseDriving = oldPathDriving[tetDriving.getRobotReport().getPathIndex()].getPose();
						Pose originalGoalDriving = oldPathDriving[oldPathDriving.length-1].getPose();

						rsp.setStart(currentPoseWaiting);
						rsp.setGoals(originalGoalWaiting);
						rsp.addObstacles(makeObstacles(dep.getDrivingRobotID(), currentPoseDriving));
						replannedRobot = dep.getWaitingRobotID();

						System.out.println("ATTEMPTING REPLANNING for Robot" + replannedRobot + "...");
						if (!rsp.plan()) {
							
							rsp.clearObstacles();
							rsp.setStart(currentPoseDriving);
							rsp.setGoals(originalGoalDriving);
							rsp.addObstacles(makeObstacles(dep.getWaitingRobotID(), currentPoseWaiting));
							replannedRobot = dep.getDrivingRobotID();
							
							System.out.println("ATTEMPTING REPLANNING for Robot" + replannedRobot + "...");
							if (!rsp.plan()) {
								System.out.println("Could not replan path!");
								replannedRobot = -1;
							}
						}
						if (replannedRobot != -1) {
							System.out.println("REPLANNING for Robot" + replannedRobot + " SUCCEEDED!");
							PoseSteering[] newPath = rsp.getPath();
							System.out.println("NEW PATH IS " + newPath.length + " POSES LONG:");
							replacePath(replannedRobot, newPath);
							spawnedReplanning.remove(new RobotPair(dep.getWaitingRobotID(), dep.getDrivingRobotID()));
						}						
					}
				}.start();
			}
		};
		
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});
		tec.setUseInternalCriticalPoints(false);
		
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		viz.setSize(1800, 450);
		tec.setVisualization(viz);

		Pose startRobot1 = new Pose(5.0,5.0,0.0);
		Pose goalRobot13 = new Pose(25.0,5.0,0.0);
		
		Pose startRobot2 = new Pose(25.0,5.0,Math.PI);
		Pose goalRobot23 = new Pose(5.0,5.0,Math.PI);

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startRobot1);
		tec.placeRobot(2, startRobot2);


		rsp.setStart(startRobot1);
		rsp.setGoals(goalRobot13);
		rsp.plan();
		Missions.pushMission(new Mission(1,rsp.getPath()));

		rsp.setStart(startRobot2);
		rsp.setGoals(goalRobot23);
		rsp.plan();
		Missions.pushMission(new Mission(2,rsp.getPath()));
		
		System.out.println("Added missions " + Missions.getMissions());

		tec.addMissions(Missions.popMission(1), Missions.popMission(2));
		
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();
		
	}

}
