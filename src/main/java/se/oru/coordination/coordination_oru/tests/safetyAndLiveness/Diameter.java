package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.util.Comparator;
import java.util.HashMap;

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
	
	protected static final int NUMBER_ROBOTS = 80;

	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		double radius = 40;
		
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
		int[] robotIDs =  new int[NUMBER_ROBOTS];
		for (int i = 1; i <= nRobotsDeadlock.NUMBER_ROBOTS; i++) {
			robotIDs[i-1] = i;
			tec.setForwardModel(i, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(i)));
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
		
		double theta = 0.0;
		for (int i = 0; i < NUMBER_ROBOTS; i++) {
			//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
			tec.setMotionPlanner(robotIDs[i], rsp.getCopy(false));
			
			//Place robots.
			double alpha = theta + i*Math.PI/NUMBER_ROBOTS;
			startPoses.put(robotIDs[i], new Pose(radius*Math.cos(alpha), radius*Math.sin(alpha), alpha));
			goalPoses.put(robotIDs[i], new Pose(radius*Math.cos(alpha+Math.PI), radius*Math.sin(alpha+Math.PI), alpha));
			tec.placeRobot(robotIDs[i], startPoses.get(robotIDs[i]));
			
			//Plan the path and enqueue the mission
			rsp.setStart(startPoses.get(robotIDs[i]));
			rsp.setGoals(goalPoses.get(robotIDs[i]));
			if (!rsp.plan()) throw new Error ("No path between " + startPoses.get(robotIDs[i]) + " and " + goalPoses.get(robotIDs[i]));
			Missions.enqueueMission(new Mission(robotIDs[i], rsp.getPath()));
		}
		
		//Wait for all robots ...
		Thread.sleep(5000);
		
		//Start a mission dispatching thread for each robot, which will run forever
		Missions.startMissionDispatchers(tec, false, robotIDs);
		
		Thread t = new Thread() {
			@Override
			public void run() {
				while (true) {
					boolean allCompleted = true;
					for (int robotID : robotIDs) allCompleted &= !Missions.hasMissions(robotID);
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
