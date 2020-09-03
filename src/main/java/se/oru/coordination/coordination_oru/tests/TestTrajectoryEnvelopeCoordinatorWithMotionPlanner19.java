package se.oru.coordination.coordination_oru.tests;

import java.io.File;
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
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Example showing hazard of deadlock breaking in combination with yielding-for-parking policy.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner19 {
	
	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
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
		String yamlFile = "maps/map-empty.yaml";
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		tec.setVisualization(viz);
		
		tec.setUseInternalCriticalPoints(false);
		//tec.setBreakDeadlocks(false);
		tec.setYieldIfParking(false);
		
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		Pose startRobot11 = new Pose(3.0,15.0,-Math.PI/2.0);
		Pose finalRobot11 = new Pose(8.0,5.0,0.0);
		
		Pose startRobot12 = new Pose(8.0,5.0,0.0);
		Pose finalRobot12 = new Pose(28.0,5.0,0.0);

		Pose startRobot21 = new Pose(10.0,15.0,-Math.PI/2.0);
		Pose finalRobot21 = new Pose(15.0,5.0,0.0);
		
		Pose startRobot22 = new Pose(15.0,5.0,0.0);
		Pose finalRobot22 = new Pose(40.0,5.0,0.0);

		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMap(yamlFile);
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1,footprint2,footprint3,footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);

		rsp.setStart(startRobot11);
		rsp.setGoals(finalRobot11);
		rsp.plan();
		Missions.enqueueMission(new Mission(1,rsp.getPath()));

		rsp.setStart(startRobot12);
		rsp.setGoals(finalRobot12);
		rsp.plan();
		Missions.enqueueMission(new Mission(1,rsp.getPath()));

		rsp.setStart(startRobot21);
		rsp.setGoals(finalRobot21);
		rsp.plan();
		Missions.enqueueMission(new Mission(2,rsp.getPath()));

		rsp.setStart(startRobot22);
		rsp.setGoals(finalRobot22);
		rsp.plan();
		Missions.enqueueMission(new Mission(2,rsp.getPath()));

		tec.placeRobot(1, startRobot11);
		tec.placeRobot(2, startRobot21);

		for (int i = 1; i <= 2; i++) {
			final int rid = i;
			Thread dispatch = new Thread() {
				private int missionCounter = 0;
				public void run() {
					while (true) {
						if (tec.isFree(rid)) {
							if (rid == 2 && missionCounter == 0) {
								try { Thread.sleep(5500); }
								catch (InterruptedException e) { e.printStackTrace(); }
							}
							tec.addMissions(Missions.getMission(rid, missionCounter++));
							if (missionCounter == 2) break;
						}
						try { Thread.sleep(200); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			dispatch.start();
		}
				
	}
	
}
