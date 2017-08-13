package se.oru.coordination.coordination_oru.tests;

import java.util.Comparator;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Simple test showing the use of pre-planned paths stored in files.")
public class TestTrajectoryEnvelopeCoordinatorThreeRobots {

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
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		tec.setupGUI(null);

		//Load data file with locations and pointers to files containing paths between locations
		Missions.loadLocationAndPathData("paths/test_poses_and_path_data.txt");

		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, Missions.getLocation("r1p"));
		tec.placeRobot(2, Missions.getLocation("r2p"));
		tec.placeRobot(3, Missions.getLocation("r3p"));

		String prefix = "paths/";
		//Make a mission for each robot, and store it in the global hashmap:
		// -- from parking location of robot i (rip)
		// -- to destination location of robot i (desti)
		Missions.putMission(new Mission(1, prefix+Missions.getPathFile("r1p", "dest1"), Missions.getLocation("r1p"), Missions.getLocation("dest1")));
		Missions.putMission(new Mission(2, prefix+Missions.getPathFile("r2p", "dest2"), Missions.getLocation("r2p"), Missions.getLocation("dest2")));
		Missions.putMission(new Mission(3, prefix+Missions.getPathFile("r3p", "dest3"), Missions.getLocation("r3p"), Missions.getLocation("dest3")));

		//Make another mission for each robot, and store it in the global hashmap:
		// -- from destination location of robot i (desti)
		// -- to parking location of robot i (rip)
		Missions.putMission(new Mission(1, prefix+Missions.getPathFile("dest1", "r1p"), Missions.getLocation("dest1"), Missions.getLocation("r1p")));
		Missions.putMission(new Mission(2, prefix+Missions.getPathFile("dest2", "r2p"), Missions.getLocation("dest2"), Missions.getLocation("r2p")));
		Missions.putMission(new Mission(3, prefix+Missions.getPathFile("dest3", "r3p"), Missions.getLocation("dest3"), Missions.getLocation("r3p")));

		System.out.println("Added missions " + Missions.getMissions());

		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 3; i++) {
			final int robotID = i;
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = Missions.getMission(robotID, iteration%2);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
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
