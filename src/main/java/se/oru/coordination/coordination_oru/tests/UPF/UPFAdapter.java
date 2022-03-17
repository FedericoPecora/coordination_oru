package se.oru.coordination.coordination_oru.tests.UPF;

import java.util.HashMap;
import java.util.HashSet;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class UPFAdapter {

	// Ideally these parameters should be settable
	public static final double MAX_ACCEL = 3.0;
	public static final double MAX_VEL = 4.0;
	public static final int CONTROL_PERIOD = 2000;
	public static final double TEMPORAL_RESOLUTION = 1000;
	public static final String yamlFile = "maps/map-partial-2.yaml";	
	public static String roadMapFile = "missions/icaps_locations_and_paths_4.txt";
	
	private TrajectoryEnvelopeCoordinatorSimulation tec = null;
	private HashMap<String,Integer> robotsToIDs = new HashMap<String, Integer>();	
	private ReedsSheppCarPlanner motionPlanner = null;
	
	private static String[] getArgs(String action) {
		String[] ret = new String[(int)action.chars().filter(ch -> ch == ',').count()+1];
		ret[0] = action.substring(action.indexOf("(")+1,action.indexOf(","));
		for (int i = 1; i < ret.length-1; i++) {
			action = action.substring(action.indexOf(",")+1);
			ret[i] = action.substring(0,action.indexOf(","));
		}
		ret[ret.length-1] = action.substring(action.indexOf(",")+1,action.length()-1);
		return ret;
	}
	
	private void setupCoordinator() {
		tec = new TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VEL,MAX_ACCEL);
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(true);
		tec.setBreakDeadlocks(false, true, true);
		
		double xl = 1.0;
		double yl = .5;
		Coordinate footprint1 = new Coordinate(-xl,yl);
		Coordinate footprint2 = new Coordinate(xl,yl);
		Coordinate footprint3 = new Coordinate(xl,-yl);
		Coordinate footprint4 = new Coordinate(-xl,-yl);		
		
		for (int r : robotsToIDs.values()) {
			tec.setFootprint(r, footprint1, footprint2, footprint3, footprint4);
			tec.setForwardModel(r, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(1)));
		}

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		
		BrowserVisualization viz = new BrowserVisualization();
		viz.setMap(yamlFile);
		viz.setInitialTransform(20.0, 9.0, 2.0);
		tec.setVisualization(viz);
		
		Missions.loadRoadMap(roadMapFile);
		
		//Instantiate a simple motion planner
		motionPlanner = new ReedsSheppCarPlanner();
		motionPlanner.setMap(yamlFile);
		motionPlanner.setRadius(0.1);
		motionPlanner.setFootprint(footprint1,footprint2,footprint3,footprint4);
		motionPlanner.setTurningRadius(4.0);
		motionPlanner.setDistanceBetweenPathPoints(0.3);

		// Indicate that this motion planner can be used also to replan on the fly
		for (int robotID : robotsToIDs.values()) tec.setMotionPlanner(robotID, motionPlanner);

	}
	
	public UPFAdapter(String[] init, String[] plan, String robotPrefix, String atPredicate, String movementAction) {

		//Gather robotIDs
		for (String action : plan) {
			String[] arguments = getArgs(action);
			for (String arg : arguments) {
				if (arg.startsWith(robotPrefix)) robotsToIDs.put(arg,Integer.parseInt(arg.substring(robotPrefix.length())));
			}
		}
		
		//Set up the coordinator and motion planner
		this.setupCoordinator();
		
		//Place robots as specified in the initial state
		for (String pred : init) {
			if (pred.startsWith(atPredicate)) {
				String[] arguments = getArgs(pred);
				if (arguments[0].startsWith(robotPrefix)) {
					tec.placeRobot(robotsToIDs.get(arguments[0]), Missions.getLocationPose(arguments[1]));
				}
				else if (arguments[1].startsWith(robotPrefix)) {
					tec.placeRobot(robotsToIDs.get(arguments[1]), Missions.getLocationPose(arguments[0]));
				}
			}
		}
		
		//Plan a path and enqueue a mission for each goto action
		for (String action : plan) {
			if (action.startsWith(movementAction)) {
				String[] arguments = getArgs(action);
				//Assume robot is first argument
				motionPlanner.setStart(Missions.getLocationPose(arguments[1]));
				motionPlanner.setGoals(Missions.getLocationPose(arguments[2]));
				if (!motionPlanner.plan()) throw new Error("Could not plan path between " + arguments[1] + " and " + arguments[2] + " for robot_" + robotsToIDs.get(arguments[0]));
				PoseSteering[] path = motionPlanner.getPath();
				Mission m = new Mission(robotsToIDs.get(arguments[0]), path);
				Missions.enqueueMission(m);
			}
		}
		
	}
	
	public void startPlanExecution() {
		int[] robotIDs = new int[robotsToIDs.size()];
		int counter = 0;
		for (Integer robotID : robotsToIDs.values()) robotIDs[counter++] = robotID;
		Missions.startMissionDispatchers(tec, false, robotIDs);
	}
	
	public static void main(String[] args) throws InterruptedException {

		String[] init= new String[] {
				"at(robot_1,L_2)",
				"at(robot_2,L_4)",
				"at(obj_3,R_5)",
				"at(obj_27,R_2)",
		};
		
		String[] plan = new String[] {
				"goto(robot_1,L_2,R_5)",
				"goto(robot_2,L_4,R_2)",
				"pick(robot_1,obj_3)",
				"pick(robot_2,obj_27)",
				"goto(robot_1,R_5,L_2)",
				"goto(robot_2,R_2,L_3)",
				"place(robot_1,obj_3)",				
				"place(robot_2,obj_27)",
		};
				
		UPFAdapter adapter = new UPFAdapter(init, plan, "robot_", "at", "goto");
		adapter.startPlanExecution();
		

	}

}
