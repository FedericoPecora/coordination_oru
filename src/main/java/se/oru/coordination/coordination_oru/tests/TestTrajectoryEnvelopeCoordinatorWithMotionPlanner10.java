package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.Comparator;

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

@DemoDescription(desc = "Four robots cycling through rooms in a large environment (paths obtained with the ReedsSheppCarPlanner).")
public abstract class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner10 {

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
		

		Missions.loadRoadMap("paths/test_poses.txt");

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
		String yamlFile = "maps/map1.yaml";
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile);
		tec.setVisualization(viz);

		tec.setUseInternalCriticalPoints(false);
		tec.setBreakDeadlocks(false, false, false);
		tec.setYieldIfParking(false);
		
		Integer[] robotIDs = new Integer[] {1,2,3,4};
		for (Integer robotID : robotIDs) {
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));	
			tec.placeRobot(robotID, Missions.getLocation("a"+robotID));
			
			//Set up private motion planners.
			final ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setMap(yamlFile);
			rsp.setRadius(0.1);
			rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
			rsp.setTurningRadius(4.0);
			rsp.setDistanceBetweenPathPoints(0.05);
			tec.setMotionPlanner(robotID, rsp);
			
			rsp.setStart(Missions.getLocation("a"+robotID));
			rsp.setGoals(Missions.getLocation("b"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));
			
			rsp.setStart(Missions.getLocation("b"+robotID));
			rsp.setGoals(Missions.getLocation("c"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));
			
			rsp.setStart(Missions.getLocation("c"+robotID));
			rsp.setGoals(Missions.getLocation("d"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));
			
			rsp.setStart(Missions.getLocation("d"+robotID));
			rsp.setGoals(Missions.getLocation("g"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));

			rsp.setStart(Missions.getLocation("g"+robotID));
			rsp.setGoals(Missions.getLocation("e"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));

			rsp.setStart(Missions.getLocation("e"+robotID));
			rsp.setGoals(Missions.getLocation("f"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));
			
			rsp.setStart(Missions.getLocation("f"+robotID));
			rsp.setGoals(Missions.getLocation("a"+robotID));
			rsp.plan();
			Missions.enqueueMission(new Mission(robotID,rsp.getPath()));

		}
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (final Integer robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (true) {
						Mission m = Missions.getMission(robotID, iteration%Missions.getMissions(robotID).size());
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								iteration++;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}		

	}

}
