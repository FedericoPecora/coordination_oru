package se.oru.coordination.coordination_oru.taskallocation.tests;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.SimpleNonCooperativeTask;
import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator.RobotType;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TimedTrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.taskallocation.MultiRobotTaskAllocator;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;

@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on paths obtained with the ReedsSheppCarPlanner.")
public class ThreeHomogeneousRobotsInALine {
	
	public static void main(String[] args) throws InterruptedException {
		
		//FIXME Add robots is available. Remove!!
		
		//Maximum acceleration/deceleration and speed for all robots
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		
		//Create a coordinator with interfaces to robots
		//in the built-in 2D simulator
		final TimedTrajectoryEnvelopeCoordinatorSimulation tec = new TimedTrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
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
		NetworkConfiguration.setDelays(0,0);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0;
		
		//Tell the coordinator
		// (1) what is known about the communication channel, and
		// (2) the accepted probability of constraint violation
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.01);

		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Need to instantiate the fleetmaster interface
		tec.instantiateFleetMaster(0.1, false);
		
					
		//Avoid deadlocks via global re-ordering
		tec.setBreakDeadlocks(true, false, false);

		//Start a visualization (will open a new browser tab)
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 0, 0);
		tec.setVisualization(viz);

		
		//Define the start poses for each robot
		HashMap<Integer, Pose> robotStarts = new HashMap<Integer, Pose>(); 
		robotStarts.put(1, new Pose(20.0,6.0,0.0));
		robotStarts.put(2, new Pose(16.0,6.0,0.0));
		robotStarts.put(3, new Pose(12.0,6.0,0.0));
		
		//Use a default polygon as robots' geometries (footprint)
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//Define the map 
		String yamlFile = "maps/map-empty.yaml";
		
		for (int robotID : robotStarts.keySet()) {
			//Set a forward model (all robots have the same here)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			
			//Set the robot motion planner 
			ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
			rsp.setRadius(0.2);
			rsp.setTurningRadius(4.0);
			rsp.setDistanceBetweenPathPoints(0.5);
			rsp.setMap(yamlFile);
			rsp.setPlanningTimeInSecs(2);
			rsp.setFootprint(tec.getDefaultFootprint());
			tec.setMotionPlanner(robotID, rsp.getCopy());
			
			//Place the robot in the start pose
			tec.placeRobot(robotID, robotStarts.get(robotID));
		}
		
		//Define the set of tasks
		ArrayList<Pose[]> taskStartAndGoal = new ArrayList<Pose[]>(); 
		taskStartAndGoal.add(new Pose[] {new Pose(24.0,6.0,0.0), new Pose(34.0,6.0,0.0)});
		taskStartAndGoal.add(new Pose[] {new Pose(24.0,6.0,0.0), new Pose(30.0,6.0,0.0)});
		taskStartAndGoal.add(new Pose[] {new Pose(24.0,6.0,0.0), new Pose(26.0,6.0,0.0)});
		
		//Create the task allocator		
		double interferenceWeight = 0.6;
		double pathLengthWeight = 1.0;
		double arrivalTimeWeight = 0;
		double tardinessWeight = 0;
		int maxNumberPathsPerTask = 1;
		double origin_x = tec.getMotionPlanner(1).getOccupancyMap().getMapOrigin().getOrdinate(0);
		double origin_y = tec.getMotionPlanner(1).getOccupancyMap().getMapOrigin().getOrdinate(1);
		double origin_theta = tec.getMotionPlanner(1).getOccupancyMap().getMapOrigin().getOrdinate(2);
		double resolution = tec.getMotionPlanner(1).getOccupancyMap().getResolution();
		long width = tec.getMotionPlanner(1).getOccupancyMap().getPixelWidth();
		long height = tec.getMotionPlanner(1).getOccupancyMap().getPixelHeight();
		boolean dynamic_size = false;
		boolean propagateDelays = true;
		boolean debug = false;
		MultiRobotTaskAllocator mrta = new MultiRobotTaskAllocator(tec, null, interferenceWeight, pathLengthWeight, arrivalTimeWeight, tardinessWeight, 
				maxNumberPathsPerTask, origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, propagateDelays, debug);
				
		//Add the defined tasks
		for (int task = 0; task < taskStartAndGoal.size(); task++) 
			 mrta.addTask(new SimpleNonCooperativeTask(taskStartAndGoal.get(task)[0], taskStartAndGoal.get(task)[1], null, null, -1, RobotType.NOT_SPECIFIED));

		
		//Start the thread that revises precedences at every period
		tec.startInference();
		
		//CONTINUE HERE!!
		//mrta.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		//mrta.setFleetVisualization(viz);
		//mrta.startTaskAssignment(tec);
	}

}
