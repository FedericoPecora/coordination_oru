package se.oru.coordination.coordination_oru.tests;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.Random;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Coordination of 4 robots along wave-like paths obtained with the ReedsSheppCarPlanner in opposing directions.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner4 {

	public static int MIN_DELAY = 500;
	public static int MAX_DELAY = 0;

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
		double deltaY = 3;
		double height = deltaY/2;
		double mapHeight = -1;
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));

		try {
			BufferedImage img = ImageIO.read(new File(yamlFile.substring(0,yamlFile.lastIndexOf("."))+".png"));
			mapHeight = img.getHeight()*res*0.9;
		}
		catch (IOException e) { e.printStackTrace(); }
		
		int[] robotIDs = new int[] {1,2,3,4,5,6,7,8};
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			//You probably also want to provide a non-trivial forward model
			//(the default assumes that robots can always stop)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			ArrayList<Pose> posesRobot = new ArrayList<Pose>();
			//if (index%2==0) {
			if (robotID%2==0) {
				posesRobot.add(new Pose(2.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(10.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(18.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(26.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(34.0,mapHeight-deltaY-height*index,0.0));
				posesRobot.add(new Pose(42.0,mapHeight-height*index,0.0));
				posesRobot.add(new Pose(50.0,mapHeight-deltaY-height*index,0.0));
			}
			else {
				posesRobot.add(new Pose(50.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(42.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(34.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(26.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(18.0,mapHeight-height*(index-1),Math.PI));
				posesRobot.add(new Pose(10.0,mapHeight-deltaY-height*(index-1),Math.PI));
				posesRobot.add(new Pose(2.0,mapHeight-height*(index-1),Math.PI));
			}
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

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
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
								if (MAX_DELAY-MIN_DELAY > 0) {
									long delay = MIN_DELAY+rand.nextInt(MAX_DELAY-MIN_DELAY);
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
