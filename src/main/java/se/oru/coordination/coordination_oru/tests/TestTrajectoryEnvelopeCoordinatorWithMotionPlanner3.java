package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

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

@DemoDescription(desc = "Coordination of 2 robots along wave-like paths obtained with the ReedsSheppCarPlanner in opposing directions.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner3 {

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

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMap(yamlFile);
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);

		ArrayList<Pose> posesRobot1 = new ArrayList<Pose>();
		posesRobot1.add(new Pose(2.0,10.0,0.0));
		posesRobot1.add(new Pose(10.0,13.0,0.0));
		posesRobot1.add(new Pose(18.0,10.0,0.0));
		posesRobot1.add(new Pose(26.0,13.0,0.0));
		posesRobot1.add(new Pose(34.0,10.0,0.0));
		posesRobot1.add(new Pose(42.0,13.0,0.0));
		posesRobot1.add(new Pose(50.0,10.0,0.0));

		ArrayList<Pose> posesRobot2 = new ArrayList<Pose>();
		//Robot 1 and robot 2 in opposing directions
		posesRobot2.add(new Pose(50.0,13.0,Math.PI));
		posesRobot2.add(new Pose(42.0,10.0,Math.PI));
		posesRobot2.add(new Pose(34.0,13.0,Math.PI));
		posesRobot2.add(new Pose(26.0,10.0,Math.PI));
		posesRobot2.add(new Pose(18.0,13.0,Math.PI));
		posesRobot2.add(new Pose(10.0,10.0,Math.PI));
		posesRobot2.add(new Pose(2.0,13.0,Math.PI));
		
//		//Robot 1 and Robot 2 in same direction
//		posesRobot2.add(new Pose(2.0,13.0,0.0));
//		posesRobot2.add(new Pose(10.0,10.0,0.0));
//		posesRobot2.add(new Pose(18.0,13.0,0.0));
//		posesRobot2.add(new Pose(26.0,10.0,0.0));
//		posesRobot2.add(new Pose(34.0,13.0,0.0));
//		posesRobot2.add(new Pose(42.0,10.0,0.0));
//		posesRobot2.add(new Pose(50.0,13.0,0.0));
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, posesRobot1.get(0));
		tec.placeRobot(2, posesRobot2.get(0));

		rsp.setStart(posesRobot1.get(0));
		rsp.setGoals(posesRobot1.subList(1, posesRobot1.size()).toArray(new Pose[posesRobot1.size()-1]));
		rsp.clearObstacles();
		Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
		rsp.addObstacles(fpGeom, posesRobot2.get(0), posesRobot2.get(posesRobot2.size()-1));
		if (!rsp.plan()) throw new Error ("No path along goals " + posesRobot1);			
		PoseSteering[] robot1path = rsp.getPath();
		PoseSteering[] robot1pathInv = rsp.getPathInv();

		rsp.setStart(posesRobot2.get(0));
		rsp.setGoals(posesRobot2.subList(1, posesRobot2.size()).toArray(new Pose[posesRobot2.size()-1]));
		rsp.clearObstacles();
		rsp.addObstacles(fpGeom, posesRobot1.get(0), posesRobot1.get(posesRobot1.size()-1));
		if (!rsp.plan()) throw new Error ("No path along goals " + posesRobot2);			
		PoseSteering[] robot2path = rsp.getPath();
		PoseSteering[] robot2pathInv = rsp.getPathInv();

		Missions.enqueueMission(new Mission(1, robot1path));
		Missions.enqueueMission(new Mission(1, robot1pathInv));
		Missions.enqueueMission(new Mission(2, robot2path));
		Missions.enqueueMission(new Mission(2, robot2pathInv));

		System.out.println("Added missions " + Missions.getMissions());

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		final int minDelay = 500;
		final int maxDelay = 3000;
		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 2; i++) {
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
								if (minDelay > 0) {
									long delay = minDelay+rand.nextInt(maxDelay-minDelay);
									//Sleep for a random delay in [minDelay,maxDelay]
									try { Thread.sleep(delay); }
									catch (InterruptedException e) { e.printStackTrace(); }
								}
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
