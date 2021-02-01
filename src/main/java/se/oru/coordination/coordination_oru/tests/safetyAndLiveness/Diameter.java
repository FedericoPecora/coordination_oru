package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class Diameter {
	
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

	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		double radius = 40;
		int NUMBER_ROBOTS = 80;
		
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(1000,1000,MAX_VEL,MAX_ACCEL);
		
		//Provide a heuristic (here, closest to critical section goes first)
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		//IDs in case of equality
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return(o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		HashSet<Integer> robotIDs =  new HashSet<Integer>();
		for (int robotID = 0; robotID < NUMBER_ROBOTS; robotID++) {
			robotIDs.add(robotID);
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));
		}
		
		//comment out following (or set to true) to make the coordinator attempt to break the deadlock
		tec.setBreakDeadlocks(false, true, false);
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
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//BrowserVisualization viz = new BrowserVisualization();
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
		
		//Set the file to track elapsed times
		final String statFilename = System.getProperty("user.home")+File.separator+"stats.txt";
		String header = "#";
		for (int robotID : robotIDs) header += (robotID + "\t");
		initStat(statFilename, header);

		
		double theta = 0.0;
		for (final int robotID : robotIDs) {
			//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
			tec.setMotionPlanner(robotID, rsp.getCopy(false));
			
			//Place robots.
			double alpha = theta + robotID*Math.PI/NUMBER_ROBOTS;
			startPoses.put(robotID, new Pose(radius*Math.cos(alpha), radius*Math.sin(alpha), alpha));
			goalPoses.put(robotID, new Pose(radius*Math.cos(alpha+Math.PI), radius*Math.sin(alpha+Math.PI), alpha));
			tec.placeRobot(robotID, startPoses.get(robotID));
			
			//Plan the path and enqueue the mission
			rsp.setStart(startPoses.get(robotID));
			rsp.setGoals(goalPoses.get(robotID));
			if (!rsp.plan()) throw new Error ("No path between " + startPoses.get(robotID) + " and " + goalPoses.get(robotID));
			Missions.enqueueMission(new Mission(robotID, rsp.getPath()));
			tec.addTrackingCallback(robotID, new TrackingCallback() {

				long startTime = -1;
				
				@Override
				public void beforeTrackingStart() {
					// TODO Auto-generated method stub
					
				}

				@Override
				public void onTrackingStart() {
					// TODO Auto-generated method stub
					startTime = Calendar.getInstance().getTimeInMillis();
				}

				@Override
				public String[] onPositionUpdate() {
					// TODO Auto-generated method stub
					return null;
				}

				@Override
				public void beforeTrackingFinished() {
					// TODO Auto-generated method stub
					
				}

				@Override
				public void onTrackingFinished() {
					// TODO Auto-generated method stub
					long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
					String stat = "";
					for (int i = 0; i < robotID; i++) stat += "\t";
					stat += elapsed;
					writeStat(statFilename, stat);
				}

				@Override
				public void onNewGroundEnvelope() {
					// TODO Auto-generated method stub
					
				}
				
			}
			);
		}
		
		//Wait for all robots ...
		Thread.sleep(10000);
		
		//Start a mission dispatching thread for each robot, which will run forever
		Missions.startMissionDispatchers(tec, false, statFilename, robotIDs.stream().mapToInt(Integer::intValue).toArray());
		
		Thread t = new Thread() {
			@Override
			public void run() {
				while (true) {
					boolean allCompleted = true;
					for (int robotID : robotIDs) allCompleted &= (tec.isFree(robotID) && !Missions.hasMissions(robotID));
					if (allCompleted) {
						tec.stopInference();
						break;
					}
					try {
						Thread.sleep(2*tec.getControlPeriod());
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		};
		t.start();
	}
	
}
