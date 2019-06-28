package se.oru.coordination.coordination_oru.tests.icaps2018.talk;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination of robots along sine wave paths (obtained without the ReedsSheppCarPlanner) in opposing directions.")
public class Waves {
	
	private static boolean deleteDir(File dir) {
	    if (dir.isDirectory()) {
	        String[] children = dir.list();
	        for (int i=0; i<children.length; i++) {
	            boolean success = deleteDir(new File(dir, children[i]));
	            if (!success) {
	                return false;
	            }
	        }
	    }
	    return dir.delete();
	}

	private static void writeStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), true)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	private static void initStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), false)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}

	private static PoseSteering[] getSinePath(double period, double magnitude, Pose from, Pose to) {
		if (from.getY() != to.getY()) throw new Error("Can only do straight sine waves ;)");
		ArrayList<Coordinate> coords = new ArrayList<Coordinate>();
		double delta = 0.1;
		double t = 0.0;
		while (t < to.getX()-from.getX()) {
			double value = magnitude*Math.sin(((2*Math.PI)/period)*t);
			Coordinate coord = new Coordinate(t+from.getX(),value+from.getY());
			coords.add(coord);
			t += delta;
		}
		PoseSteering[] pss = new PoseSteering[coords.size()];
		pss[0] = new PoseSteering(coords.get(0).x, coords.get(0).y, Math.atan2(coords.get(1).y-coords.get(0).y, coords.get(1).x-coords.get(0).x),0.0);
		pss[coords.size()-1] = new PoseSteering(coords.get(coords.size()-1).x, coords.get(coords.size()-1).y, Math.atan2(coords.get(coords.size()-1).y-coords.get(coords.size()-2).y, coords.get(coords.size()-1).x-coords.get(coords.size()-2).x),0.0);
		for (int i = 1; i < coords.size()-1; i++) {
			pss[i] = new PoseSteering(coords.get(i).x,coords.get(i).y,Math.atan2(coords.get(i+1).y-coords.get(i).y, coords.get(i+1).x-coords.get(i).x),0.0);
		}		
		return pss;
	}
	
	private static PoseSteering[] invertPath(PoseSteering[] path) {
		PoseSteering[] ret = new PoseSteering[path.length];
		for (int i = 0; i < path.length; i++) {
			ret[i] = path[path.length-i-1];
		}
		return ret;
	}
	
	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 2.5;
		final int numRobots = (args != null && args.length > 0) ? Integer.parseInt(args[0]) : 10;
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
		tec.setCheckCollisions(true);
		
		NetworkConfiguration.setDelays(100, 100);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0;
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.02);
		//tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, 0, 0);

		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(0.5,0.5);
		Coordinate footprint3 = new Coordinate(0.5,-0.5);
		Coordinate footprint4 = new Coordinate(-0.5,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);
		tec.setBreakDeadlocksByReordering(false);
		tec.setBreakDeadlocksByReplanning(true);
		
		//Set up motion planner
		String yamlFile = "maps/map-empty.yaml";
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRadius(0.1);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		double deltaY = 1.7;
		
		final int[] robotIDs = new int[numRobots];
		for (int i = 0; i < numRobots; i++) robotIDs[i] = i+1;
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(robotIDs);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(40.0, 8.7, 5.7);
		viz.setMap(yamlFile);
		tec.setVisualization(viz);
		
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			double period = 10;
			double mag = 2*deltaY;

			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getTrackingPeriod()));
			Pose from = new Pose(0.0,index*deltaY,0.0);
			Pose to = new Pose(3*period,index*deltaY,0.0);

			if (index%2 != 0) mag*=(-1);
			PoseSteering[] robotPath = getSinePath(period, mag, from, to);
			PoseSteering[] robotPathInv = invertPath(robotPath);
			if (robotID%2 == 0) {
				Missions.enqueueMission(new Mission(robotID, robotPath));
				Missions.enqueueMission(new Mission(robotID, robotPathInv));
				tec.placeRobot(robotID, robotPath[0].getPose());
			}
			else {
				Missions.enqueueMission(new Mission(robotID, robotPathInv));
				Missions.enqueueMission(new Mission(robotID, robotPath));
				tec.placeRobot(robotID, robotPathInv[0].getPose());				
			}
		}

		System.out.println("Added missions " + Missions.getMissions());
		
//		while (true) {
//			ArrayList<Mission> missionsToAdd = new ArrayList<Mission>();
//			for (int robotID : robotIDs) {
//				if (tec.isFree(robotID)) {
//					Mission m = Missions.dequeueMission(robotID);
//					missionsToAdd.add(m);
//					Missions.enqueueMission(m);
//				}
//			}
//			tec.addMissions(missionsToAdd.toArray(new Mission[missionsToAdd.size()]));
//			tec.computeCriticalSections();
//			tec.startTrackingAddedMissions();
//			Thread.sleep(1000);
//		}
		
		final String statFilename = System.getProperty("user.home")+File.separator+"stats.txt";
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 0; i < robotIDs.length; i++) {
			final int robotID = robotIDs[i];
			//For each robot, create a thread that dispatches the "next" mission when the robot is free
			
			Thread t = new Thread() {
				@Override
				public void run() {
					boolean firstTime = true;
					boolean isFree = false;
					int sequenceNumber = 0;
					int totalIterations = 21;
					//if (robotID%2 == 0) totalIterations = 19;
					long startTime = Calendar.getInstance().getTimeInMillis();
					while (true && totalIterations > 0) {
						synchronized(tec.getSolver()) {
							if (tec.isFree(robotID)) isFree = true;
						}
						//Sleep for a little
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
						
						if (isFree) {
							isFree = false;
							synchronized(tec.getSolver()) {
								if (!firstTime) {
									long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
									String stat = "";
									for (int i = 1; i < robotID; i++) stat += "\t";
									stat += elapsed;
									writeStat(statFilename, stat);
								}
								startTime = Calendar.getInstance().getTimeInMillis();
								firstTime = false;
								Mission m = Missions.getMission(robotID,sequenceNumber);
								tec.addMissions(m);
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
								sequenceNumber = (sequenceNumber+1)%Missions.getMissions(robotID).size();
								totalIterations--;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(100); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					System.out.println("Robot" + robotID + " is done!");
				}
			};
			//Start the thread!
			t.start();
		}

	}

}
