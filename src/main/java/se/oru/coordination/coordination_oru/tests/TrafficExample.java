package se.oru.coordination.coordination_oru.tests;

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
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class TrafficExample {

	private static void setupRobot(Pose start, Pose goal, int robotID, Coordinate[] footprint, double MAX_ACCEL, double MAX_VEL, TrajectoryEnvelopeCoordinatorSimulation tec) {
		//Use a random polygon as this robot's geometry (footprint)
		tec.setFootprint(robotID,footprint);
		
		//Set a forward model (all robots have the same here)
		tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));

		//Place the robot in the start pose
		tec.placeRobot(robotID, start);

		//Path planner for each robot (with empty map)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setFootprint(footprint);
		
		//Plan path from start to goal and vice-versa
		rsp.setStart(start);
		rsp.setGoals(goal);
		if (!rsp.plan()) throw new Error ("No path between " + start + " and " + goal);
		PoseSteering[] path = rsp.getPath();
		PoseSteering[] pathInv = rsp.getPathInv();
		
		//Define forward and backward missions and enqueue them
		Missions.enqueueMission(new Mission(robotID,path));
		Missions.enqueueMission(new Mission(robotID,pathInv));
		
		//Path planner to use for re-planning if needed
		tec.setMotionPlanner(robotID, rsp);
	}
	
	public static void main(String[] args) {

		//Maximum acceleration/deceleration and speed for all robots
		double MAX_ACCEL = 5.0;
		double MAX_VEL = 10.0;
		double dim = 0.5;
		
		//Robot footprint
		Coordinate[] footprint = new Coordinate[] { 
				new Coordinate(-dim,dim),
				new Coordinate(dim,dim),
				new Coordinate(dim,-dim),
				new Coordinate(-dim,-dim),
		};
		
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
		//NetworkConfiguration.setDelays(0,3000);
		//NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0.1;
		
		//Tell the coordinator
		// (1) what is known about the communication channel, and
		// (2) the accepted probability of constraint violation
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);

		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Start the thread that revises precedences at every period
		tec.startInference();

		//Robot IDs can be non-sequential (but must be unique)
		int[] robotIDs = new int[] {1,2,3,4,5,6,7,8,9,10};
		int index = 0;
		
		for (int robotID : robotIDs) {
			
			if (robotID < 100) {
				//Define start and goal poses for the robot
				Pose start = null;
				Pose goal = null;
				
				if (index%2==0) {
					start = new Pose(0, 10+index*2.2*dim, 0);
					goal = new Pose(10, 10+index*2.2*dim, 0);
				}
				else {
					start = new Pose(10, 10+index*2.2*dim, 0);
					goal = new Pose(0, 10+index*2.2*dim, 0);
				}
				
				index++;
	
				setupRobot(start, goal, robotID, footprint, MAX_ACCEL, MAX_VEL, tec);
			}

		}
		
		Pose perpStart = new Pose(5,robotIDs.length*2.2+10,-Math.PI/2);
		Pose perpGoal = new Pose(5,0,-Math.PI/2);
		
		setupRobot(perpStart, perpGoal, 666, footprint, MAX_ACCEL, MAX_VEL, tec);
		
		//Avoid deadlocks via global re-ordering
		tec.setBreakDeadlocks(false, false, false);

		//Start a visualization (will open a new browser tab)
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 40, 7);
		tec.setVisualization(viz);
		
		//Start dispatching threads for each robot, each of which
		//dispatches the next mission as soon as the robot is idle
		Missions.startMissionDispatchers(tec, robotIDs);	
		Missions.startMissionDispatchers(tec, 666);

		

	}

}