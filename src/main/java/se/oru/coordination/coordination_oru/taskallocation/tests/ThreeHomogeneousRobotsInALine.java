package se.oru.coordination.coordination_oru.taskallocation.tests;

import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
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
		
		//FIXME Check minimal example coordination and see if compatible.
		//FIXME Add robots is available. Remove!!
		
		//Set the maximum robots' speed and acceleration.
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		
		//Instantiate a timed trajectory envelope coordinator.
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		//final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		final TimedTrajectoryEnvelopeCoordinatorSimulation tec = new TimedTrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
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
		
		//Need to instantiate the fleetmaster interface
		tec.instantiateFleetMaster(0.1, false);
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1024, 768);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(20, 0, 0);
		tec.setVisualization(viz);
		tec.setUseInternalCriticalPoints(false);
		
		String yamlFile = "maps/map-empty.yaml";
		HashMap<Integer, Pose> startingRobotPoses = new HashMap<Integer, Pose>(); 
		startingRobotPoses.put(1, new Pose(20.0,6.0,0.0));
		startingRobotPoses.put(2, new Pose(16.0,6.0,0.0));
		startingRobotPoses.put(3, new Pose(12.0,6.0,0.0));
		
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setMap(yamlFile);
		rsp.setPlanningTimeInSecs(2);
		rsp.setFootprint(tec.getDefaultFootprint());

		
		for (int robotID : tec.getIdleRobots()) {
			//Instantiate a simple motion planner (no map given here, otherwise provide yaml file)
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
			tec.setMotionPlanner(robotID, rsp.getCopy());
			tec.placeRobot(robotID, startingRobotPoses.get(robotID));
		}
		
		Pose startPoseGoal1 = new Pose(24.0,6.0,0.0);
		Pose startPoseGoal2 = new Pose(24.0,6.0,0.0);
		Pose startPoseGoal3 = new Pose(24.0,6.0,0.0);
		Pose goalPoseRobot1 = new Pose(34.0,6.0,0.0);
		Pose goalPoseRobot2 = new Pose(30.0,6.0,0.0);
		Pose goalPoseRobot3 = new Pose(26.0,6.0,0.0);
				
		SimpleNonCooperativeTask task1 = new SimpleNonCooperativeTask(startPoseGoal1, goalPoseRobot1, null, null, -1, RobotType.NOT_SPECIFIED);
		SimpleNonCooperativeTask task2 = new SimpleNonCooperativeTask(startPoseGoal2, goalPoseRobot2, null, null, -1, RobotType.NOT_SPECIFIED);
		SimpleNonCooperativeTask task3 = new SimpleNonCooperativeTask(startPoseGoal3, goalPoseRobot3, null, null, -1, RobotType.NOT_SPECIFIED);
		
	    ///////////////////////////////////////////////////////
		//Solve the problem to find some feasible solution
		
		double interferenceWeight = 0.6;
		double pathLengthWeight = 1.0;
		double arrivalTimeWeight = 0;
		double tardinessWeight = 0;
		int maxNumberPathsPerTask = 1;
		double origin_x = rsp.getOccupancyMap().getMapOrigin().getOrdinate(0);
		double origin_y = rsp.getOccupancyMap().getMapOrigin().getOrdinate(1);
		double origin_theta = rsp.getOccupancyMap().getMapOrigin().getOrdinate(2);
		double resolution = rsp.getOccupancyMap().getResolution();
		long width = rsp.getOccupancyMap().getPixelWidth();
		long height = rsp.getOccupancyMap().getPixelHeight();
		boolean dynamic_size = false;
		boolean propagateDelays = true;
		boolean debug = false;
		MultiRobotTaskAllocator mrta = new MultiRobotTaskAllocator(tec, null, interferenceWeight, pathLengthWeight, arrivalTimeWeight, tardinessWeight, 
				maxNumberPathsPerTask, origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, propagateDelays, debug);
		mrta.addTask(task1);
		mrta.addTask(task2);
		mrta.addTask(task3);
		
		//mrta.setminMaxVelandAccel(MAX_VEL, MAX_ACCEL);
		//mrta.setFleetVisualization(viz);
		//mrta.startTaskAssignment(tec);
	}

}
