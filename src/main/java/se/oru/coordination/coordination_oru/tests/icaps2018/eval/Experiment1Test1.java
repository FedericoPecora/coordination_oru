package se.oru.coordination.coordination_oru.tests.icaps2018.eval;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination of up to 15 robots along single constriction path obtained with the ReedsSheppCarPlanner in opposing directions.")
public class Experiment1Test1 {

	public static int usedRobots = 0;
	
	public static boolean inUse(int robotID) {
		if (robotID <= usedRobots) return true;
		return false;
	}

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
		final int numRobots = (args != null && args.length > 0) ? Integer.parseInt(args[0]) : 5;
		String logFile = System.getProperty("user.home")+File.separator+"icaps_test_1_"+numRobots+"_robots.log";
		String logHeading = "#Rob\t#DrvR\tT\tCS";
		final ArrayList<Integer> robotsInUse = new ArrayList<Integer>();
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		//final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		final TrajectoryEnvelopeCoordinatorSimulationICAPS tec = new TrajectoryEnvelopeCoordinatorSimulationICAPS(0, 1000.0, MAX_VEL, MAX_ACCEL, logFile, logHeading, robotsInUse);
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
		
		tec.setQuiet(true);

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);
		tec.setBreakDeadlocks(false, false, false);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
//		String yamlFile = "../maps/map-empty.yaml";
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//RVizVisualization viz = new RVizVisualization();
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(25, -25, 15);
		tec.setVisualization(viz);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		double deltaY = 3;
		double height = deltaY/2;
		
		double mapHeight = 2*numRobots;
		double mapWidht = 100+1.5*numRobots;



//		String setup = "Hourglass test with "+numRobots+" robots"+" in a map "+mapHeight+" by "+mapWidht+" at "+LocalDateTime.now();
		//tec.writeSetupLog("info", setup);
		final int[] robotIDs = new int[numRobots];
		for (int i = 0; i < numRobots; i++) robotIDs[i] = i+1;
		RVizVisualization.writeRVizConfigFile(robotIDs);
		
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			//You probably also want to provide a non-trivial forward model
			//(the default assumes that robots can always stop)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			ArrayList<Pose> posesRobot = new ArrayList<Pose>();
			//if (index%2==0) {
			posesRobot.add(new Pose(Math.floor(mapWidht*0.3),mapHeight-(2*robotID),0.0));
			posesRobot.add(new Pose(Math.floor(mapWidht*0.6),Math.floor(mapHeight/2),0.0));
			posesRobot.add(new Pose(Math.floor(mapWidht*0.9),mapHeight-(2*robotID),0.0));

			tec.placeRobot(robotID, posesRobot.get(0));
			
			rsp.setStart(posesRobot.get(0));
			rsp.setGoals(posesRobot.subList(1, posesRobot.size()).toArray(new Pose[posesRobot.size()-1]));
			rsp.clearObstacles();
			if (!rsp.plan()) throw new Error ("No path along goals " + posesRobot);			
			PoseSteering[] robotPath = rsp.getPath();
			PoseSteering[] robotPathInv = rsp.getPathInv();
			
			Missions.enqueueMission(new Mission(robotID, robotPath));
			Missions.enqueueMission(new Mission(robotID, robotPathInv));
		}

		System.out.println("Added missions " + Missions.getMissions());

		final long durationBeforeIncrement = 20000;
		Thread robotCounter = new Thread() {
			public void run() {
				while (usedRobots < numRobots) {
					robotsInUse.add(robotIDs[usedRobots]);
					usedRobots++;
					System.out.println("ROBOTS IN USE: " + usedRobots);
					try { Thread.sleep(durationBeforeIncrement); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
				System.out.println("TEST FINISHED!");
			}
		};
		robotCounter.start();
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					this.setPriority(Thread.MAX_PRIORITY);
					while (usedRobots <= numRobots) {
						if (inUse(robotID)) {
							//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
							Mission m = Missions.getMission(robotID, iteration%2);
							synchronized(tec) {
								//addMission returns true iff the robot was free to accept a new mission
								if (tec.addMissions(m)) {
									//tec.writeSetupLog("progress", "Robot "+robotID+" has completed " +iteration+" missions.");
									iteration++;
								}
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
