package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination with deadlock-inducing ordering heuristic (paths obtained with the ReedsSheppCarPlanner).")
public class nRobotsDeadlock {
	
	private static void initStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), false)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	static public void writeStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), true)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
		
	protected static final int NUMBER_ROBOTS = 40;
	protected static final int NUMBER_MISSIONS = 10;

	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		double radius = 40;
		
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(4000,1000,MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				int robot1ID = o1.getRobotReport().getRobotID();
				int robot2ID = o2.getRobotReport().getRobotID();
				if (robot2ID == nRobotsDeadlock.NUMBER_ROBOTS && robot1ID == 1) return 1;
				if (robot1ID == nRobotsDeadlock.NUMBER_ROBOTS && robot2ID == 1) return -1;
				return robot1ID > robot2ID ? 1 : -1; 
				}
		});
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		int[] robotIDs =  new int[NUMBER_ROBOTS];
		for (int i = 1; i <= nRobotsDeadlock.NUMBER_ROBOTS; i++) {
			robotIDs[i-1] = i;
			tec.setForwardModel(i, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		}
		
		//comment out following (or set to true) to make the coordinator attempt to break the deadlock
		tec.setBreakDeadlocks(true, false, false);
		tec.setCheckCollisions(true);

		Coordinate footprint1 = new Coordinate(-0.25,0.25);
		Coordinate footprint2 = new Coordinate(0.25,0.25);
		Coordinate footprint3 = new Coordinate(0.25,-0.25);
		Coordinate footprint4 = new Coordinate(-0.25,-0.25);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		//RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(robotIDs);
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(73, 22, 16);
		tec.setVisualization(viz);
		
		//tec.setUseInternalCriticalPoints(false);
		
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
			
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		HashMap<Integer,Pose> startPoses = new HashMap<Integer,Pose>();
		HashMap<Integer,Pose> goalPoses = new HashMap<Integer,Pose>();
		final HashMap<Integer,Boolean> status = new HashMap<Integer,Boolean>();
		
		double theta = 0.0;
		for (int i = 0; i < NUMBER_ROBOTS; i++) {
			//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
			tec.setMotionPlanner(robotIDs[i], rsp.getCopy(false));
			//Place robots.
			double alpha = theta + i*Math.PI/NUMBER_ROBOTS;
			startPoses.put(robotIDs[i], new Pose(radius*Math.cos(alpha), radius*Math.sin(alpha), alpha));
			goalPoses.put(robotIDs[i], new Pose(radius*Math.cos(alpha+Math.PI), radius*Math.sin(alpha+Math.PI), alpha));
			tec.placeRobot(robotIDs[i], startPoses.get(robotIDs[i]));
			rsp.setStart(startPoses.get(robotIDs[i]));
			rsp.setGoals(goalPoses.get(robotIDs[i]));
			if (!rsp.plan()) throw new Error ("No path between " + startPoses.get(robotIDs[i]) + " and " + goalPoses.get(robotIDs[i]));
			for (int j = 0; j < NUMBER_MISSIONS; j++) {
				Missions.enqueueMission(new Mission(robotIDs[i], rsp.getPath()));
				Missions.enqueueMission(new Mission(robotIDs[i], rsp.getPathInv()));
			}
			status.put(robotIDs[i], false);
		}
		
		final String statFilename = System.getProperty("user.home")+File.separator+"stats.txt";
		String header = "#";
		for (int robotID : robotIDs) header += (robotID + "\t");
		initStat(statFilename, header);

		Thread.sleep(5000);
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			
			//For each robot, create a thread that dispatches the "next" mission when the robot is free
			Thread t = new Thread() {
				@Override
				public void run() {
					boolean firstTime = true;
					long startTime = Calendar.getInstance().getTimeInMillis();
					while (true && !Missions.getMissions(robotID).isEmpty()) {
						synchronized(tec.getSolver()) {
							Mission m = Missions.peekMission(robotID);
							if (tec.addMissions(m)) {
								Missions.dequeueMission(robotID);
								if (!firstTime) {
									long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
									String stat = "";
									for (int i = 1; i < robotID; i++) stat += "\t";
									stat += elapsed;
									writeStat(statFilename, stat);
								}
								startTime = Calendar.getInstance().getTimeInMillis();
								firstTime = false;
							}
						}
						//Sleep for a little
						try { Thread.sleep(tec.getControlPeriod()); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					System.out.println("Robot" + robotID + " is done!");
					synchronized(status) {
						status.put(robotID, true);
						boolean allFinished = true;
						for (int robotID : status.keySet()) {
							allFinished &= status.get(robotID);
							if (!allFinished) break;
						}
						if (allFinished && tec.isStartedInference()) {
							System.out.println("All the robots are done. Finishing the simulation.");
							tec.stopInference();
						}
					}
				}
			};
			
			//Start the thread!
			t.start();
			
			//Sleep for a little
			try { Thread.sleep(tec.getControlPeriod()); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}

	}
	
}
