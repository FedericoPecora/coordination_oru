package se.oru.coordination.coordination_oru.tests;

import java.util.ArrayList;
import java.util.Comparator;
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

public class MinimalExample {

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
		NetworkConfiguration.setDelays(0,3000);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.1;
		
		//Tell the coordinator
		// (1) what is known about the communication channel, and
		// (2) the accepted probability of constraint violation
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);

		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Start the thread that revises precedences at every period
		tec.startInference();

		//Robot IDs can be non-sequential (but must be unique)
		int[] robotIDs = new int[] {22,7,54,13,1,14};
		
		double minRobotRadius = 0.2;
		double maxRobotRadius = 2.0;

		for (int robotID : robotIDs) {
			
			//Use a random polygon as this robot's geometry (footprint)
			Coordinate[] fp = makeRandomFootprint(0, 0, 3, 6, minRobotRadius, maxRobotRadius);
			tec.setFootprint(robotID,fp);
			
			//Set a forward model (all robots have the same here)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));

			//Define start and goal poses for the robot
			Pose[] startAndGoal = makeRandomStartGoalPair(robotIDs.length, 1.5*maxRobotRadius, 1.1*maxRobotRadius, 1.1*maxRobotRadius);
			
			//Place the robot in the start pose
			tec.placeRobot(robotID, startAndGoal[0]);

			//Path planner for each robot (with empty map)
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setRadius(0.2);
			rsp.setTurningRadius(4.0);
			rsp.setDistanceBetweenPathPoints(0.5);
			rsp.setFootprint(fp);
			
			//Plan path from start to goal and vice-versa
			rsp.setStart(startAndGoal[0]);
			rsp.setGoals(startAndGoal[1]);
			if (!rsp.plan()) throw new Error ("No path between " + startAndGoal[0] + " and " + startAndGoal[1]);
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
		viz.setInitialTransform(49, 5, 0);
		tec.setVisualization(viz);
		
		//Start dispatching threads for each robot, each of which
		//dispatches the next mission as soon as the robot is idle
		Missions.startMissionDispatchers(tec, robotIDs);	
		

	}

}