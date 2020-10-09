package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.Pair;

public class Basement {

	private static Random rand = new Random(123213);

	private static ArrayList<Pair<Integer>> placements = new ArrayList<Pair<Integer>>();

	private static Coordinate[] makeRandomFootprint(int centerX, int centerY, int minVerts, int maxVerts, double minRadius, double maxRadius) {
	    // Split a full circle into numVerts step, this is how much to advance each part
		int numVerts = minVerts+rand.nextInt(maxVerts-minVerts);
	    double angleStep = Math.PI * 2 / numVerts;
	    Coordinate[] ret = new Coordinate[numVerts];
	    for(int i = 0; i < numVerts; ++i) {
	        double targetAngle = angleStep * i; // This is the angle we want if all parts are equally spaced
	        double angle = targetAngle + (rand.nextDouble() - 0.5) * angleStep * 0.25; // add a random factor to the angle, which is +- 25% of the angle step
	        double radius = minRadius + rand.nextDouble() * (maxRadius - minRadius); // make the radius random but within minRadius to maxRadius
	        double x = Math.cos(angle) * radius; 
	        double y = Math.sin(angle) * radius;
	        ret[i] = new Coordinate(x, y);
	    }
	    return ret;
	}

	private static Pose[] makeRandomStartGoalPair(int numRobots, double maxRadius, double offsetX, double offsetY) {
		Pose[] ret = new Pose[2];
		Pair<Integer> placement = new Pair<Integer>(rand.nextInt(numRobots), rand.nextInt(numRobots));		
		while (placements.contains(placement)) placement = new Pair<Integer>(rand.nextInt(numRobots),rand.nextInt(numRobots));
		placements.add(placement);
		ret[0] = new Pose(offsetX+placement.getFirst()*maxRadius,offsetY+placement.getSecond()*maxRadius, 2*Math.PI*rand.nextDouble());
		
		placement = new Pair<Integer>(rand.nextInt(numRobots), rand.nextInt(numRobots));		
		while (placements.contains(placement)) placement = new Pair<Integer>(rand.nextInt(numRobots),rand.nextInt(numRobots));
		placements.add(placement);
		ret[1] = new Pose(offsetX+placement.getFirst()*maxRadius,offsetY+placement.getSecond()*maxRadius, 2*Math.PI*rand.nextDouble());

		return ret;
	}

	public static void main(String[] args) {

		//Maximum acceleration/deceleration and speed for all robots
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		
		//Create a coordinator with interfaces to robots
		//in the built-in 2D simulator
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
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

		//Define a network with uncertainties (see Mannucci et al., 2019)
		NetworkConfiguration.setDelays(0, 0);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.0;
		
		//Tell the coordinator
		// (1) what is known about the communication channel, and
		// (2) the accepted probability of constraint violation
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);

		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Start the thread that revises precedences at every period
		tec.startInference();

		//Robot IDs can be non-sequential (but must be unique)
		int[] robotIDs = new int[] {1,2,3,4,5,6,7};
		
		double xl = 1.;
		double yl = .5;
		Coordinate footprint1 = new Coordinate(-xl,yl);
		Coordinate footprint2 = new Coordinate(xl,yl);
		Coordinate footprint3 = new Coordinate(xl,-yl);
		Coordinate footprint4 = new Coordinate(-xl,-yl);		
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//Define start and goal poses for each robot
		HashMap<Integer,Pose> startPose = new HashMap<Integer,Pose>();
		startPose.put(1, new Pose(33.0,6.0,Math.PI));
		startPose.put(2, new Pose(3.0,28.0,0.0));
		startPose.put(3, new Pose(3.0,20.0,0.0));
		startPose.put(4, new Pose(3.0,25.0,0.0));
		startPose.put(5, new Pose(8.0,2.8,Math.PI/2));
		startPose.put(6, new Pose(11.0,2.8,Math.PI/2));
		startPose.put(7, new Pose(20.0,2.8,Math.PI/2));
		
		HashMap<Integer, ArrayList<Pose>> goalPoses = new HashMap<Integer,ArrayList<Pose>>();
		goalPoses.put(1, new ArrayList<Pose>());
		goalPoses.get(1).add(new Pose(7.0,7.0,Math.PI/2));
		goalPoses.get(1).add(new Pose(6.0,15.5,Math.PI/2));
		
		goalPoses.put(2, new ArrayList<Pose>());
		goalPoses.get(2).add(new Pose(13.0,20.0,0.0));
		goalPoses.get(2).add(new Pose(3.0,23,0.0));
		
		goalPoses.put(3, new ArrayList<Pose>());
		goalPoses.get(3).add(new Pose(13.0,24.25,0.0));
		goalPoses.get(3).add(new Pose(20.0,24.25,0.0));
		
		goalPoses.put(4, new ArrayList<Pose>());
		goalPoses.get(4).add(new Pose(25.0,5.0,0.0));
		goalPoses.get(4).add(new Pose(30.5,7.0,Math.PI/2));
		
		goalPoses.put(5, new ArrayList<Pose>());
		goalPoses.get(5).add(new Pose(18.0,15.0,0.0));
		goalPoses.get(5).add(new Pose(25.0,15.0,0.0));
		
		goalPoses.put(6, new ArrayList<Pose>());
		goalPoses.get(6).add(new Pose(11.0,5.0,Math.PI/2));
		goalPoses.get(6).add(new Pose(3.0,23.0,0.0));
		
		goalPoses.put(7, new ArrayList<Pose>());
		goalPoses.get(7).add(new Pose(20.0,11.0,0.0));
		goalPoses.get(7).add(new Pose(24.0,11.0,0.0));
				
		//set the map yalm file
		String yamlFile = new String("maps/basement.yaml");
		
		//Set the default motion planner
		ReedsSheppCarPlanner rsp_ = new ReedsSheppCarPlanner();
		rsp_.setRadius(0.1);
		rsp_.setTurningRadius(4.0);
		rsp_.setDistanceBetweenPathPoints(0.5);
		rsp_.setFootprint(tec.getDefaultFootprint());
		rsp_.setMap(yamlFile);
		rsp_.setPlanningTimeInSecs(10);

		for (int robotID : robotIDs) {
			
			//Use a random polygon as this robot's geometry (footprint)
			tec.setFootprint(robotID, tec.getDefaultFootprint());
			
			//Set a forward model (all robots have the same here)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		
			//Place the robot in the start pose
			tec.placeRobot(robotID, startPose.get(robotID));

			//Path planner for each robot (with empty map)
			ReedsSheppCarPlanner rsp = (ReedsSheppCarPlanner) rsp_.getCopy(false);
			
			//Plan path from start to goal and vice-versa
			rsp.setStart(startPose.get(robotID));
			rsp.setGoals(goalPoses.get(robotID).get(0), goalPoses.get(robotID).get(1));
			if (!rsp.plan()) throw new Error ("No path between " + startPose.get(robotID) + " and " + goalPoses.get(robotID).get(1));
			PoseSteering[] path = rsp.getPath();
			PoseSteering[] pathInv = rsp.getPathInv();
			
			//Define forward and backward missions and enqueue them
			Missions.enqueueMission(new Mission(robotID,path));
			Missions.enqueueMission(new Mission(robotID,pathInv));
			
			//Path planner to use for re-planning if needed
			tec.setMotionPlanner(robotID, rsp);
		}
		
		//Avoid deadlocks via global re-ordering
		tec.setBreakDeadlocks(true, false, false);

		//Start a visualization (will open a new browser tab)
		BrowserVisualization viz = new BrowserVisualization();
		viz.setMap(yamlFile);
		viz.setInitialTransform(13, 6.1, 6.8);
		tec.setVisualization(viz);
		
		//Start dispatching threads for each robot, each of which
		//dispatches the next mission as soon as the robot is idle
		Missions.startMissionDispatchers(tec, false, robotIDs);
	}

	
}
