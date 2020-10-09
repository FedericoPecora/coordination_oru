package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.Comparator;

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

@DemoDescription(desc = "Three robots coordinating at high speed (paths obtained with the ReedsSheppCarPlanner).")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner11 {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 10.0;
		double MAX_VEL = 30.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(1000,1000,MAX_VEL,MAX_ACCEL);
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
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL*0.9, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL*0.9, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL*0.9, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		tec.setVisualization(viz);

		//Load data file with locations and pointers to files containing paths between locations
		Missions.loadLocationAndPathData("paths/test_poses2.txt");

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, Missions.getLocation("r1p"));
		tec.placeRobot(2, Missions.getLocation("r2p"));
		tec.placeRobot(3, Missions.getLocation("r3p"));

		String yamlFile = "maps/map-empty.yaml";
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMap(yamlFile);
		rsp.setRadius(0.2);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		
		rsp.setStart(Missions.getLocation("r1p"));
		rsp.setGoals(Missions.getLocation("dest1"));
		rsp.plan();
		Missions.enqueueMission(new Mission(1,rsp.getPath()));
		Missions.enqueueMission(new Mission(1,rsp.getPathInv()));
		
		rsp.setStart(Missions.getLocation("r2p"));
		rsp.setGoals(Missions.getLocation("dest2"));
		rsp.plan();
		Missions.enqueueMission(new Mission(2,rsp.getPath()));
		Missions.enqueueMission(new Mission(2,rsp.getPathInv()));
		
		rsp.setStart(Missions.getLocation("r3p"));
		rsp.setGoals(Missions.getLocation("dest3"));
		rsp.plan();
		Missions.enqueueMission(new Mission(3,rsp.getPath()));
		Missions.enqueueMission(new Mission(3,rsp.getPathInv()));
	
		System.out.println("Added missions " + Missions.getMissions());

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 3; i++) {
			final int robotID = i;
			//Set each robot motion planner
			tec.setMotionPlanner(robotID, rsp.getCopy(false));
			
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
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
