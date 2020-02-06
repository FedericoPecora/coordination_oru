package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.jgrapht.alg.util.Pair;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.NativeLong;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.GridParams;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PathPose;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.TrajParams;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;


public class FleetMasterInterface {
	
	private HashMap<Integer, NativeLong> paths = null; //teID (or this pathID), fleetmaster pathID 
	private HashMap<Integer, TrajParams> trajParams = null; //robotID, trajectory parameters
	private GridParams gridParams = null;
	
	/**
	 * The default footprint used for robots if none is specified.
	 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
	 */
	public static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			new Coordinate(-1.7, 0.7),	//back left
			new Coordinate(-1.7, -0.7),	//back right
			new Coordinate(2.7, -0.7),	//front right
			new Coordinate(2.7, 0.7)	//front left
	};
	
	public Coordinate[] getDefaultFootprint() {
		return DEFAULT_FOOTPRINT;
	}
	
	/**
	 * The default trajectory params used for robots if none is specified.
	 * FIXME For debugging purposes (Values are defined according to default values of trajectory_processor.h).
	 */
	public static TrajParams DEFAULT_TRAJ_PARAMS = new TrajParams(1.0, 1.0, true, 1.0, 1.0, 1.0, 0., 0., 
		    0., 0., 1., 1., 1., 0.06, 0.68, 0., true, 5, 0.00001, true, false, true, new String(""), 0., 0., false, 0);
		
	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("fleetmaster", "FleetMasterInterface"); //FIXME How to add the path of the library? Now it is in {$FLEETMASTER_WS}/devel
		INSTANCE = Native.loadLibrary("fleetmaster", FleetMasterInterfaceLib.class);
	}
	
	/**
	 * Constructor.
	 */
	public FleetMasterInterface() {
		this.paths = new HashMap<Integer, NativeLong>();
		this.trajParams = new HashMap<Integer, TrajParams>();
	}
	
	/**
	 * Set the value of the footprint used for robots if none is specified.
	 */
	public boolean setDefaultFootprints(Coordinate ... coordinates) {
		if (coordinates.length == 0) return false;
		DEFAULT_FOOTPRINT = coordinates;
		return true;
	}
	
	/**
	 * Set the fleetmaster gridmap parameters. Calling this function is mandatory.
	 * @param origin_x The x origin of the map
	 * @param origin_y The y origin of the map
	 * @param origin_theta The theta origin of the map
	 * @param resolution The resolution of the map (in meters/cell).
	 * @param width Number of columns of the map (in cells).
	 * @param height Number of rows of the map (in cells).
	 */
	public void setGridMapParams(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height) {
		gridParams = new GridParams(origin_x, origin_y, origin_theta, resolution, new NativeLong(width), new NativeLong(height));
		INSTANCE.init(gridParams);
	}
	
	/**
	 * Use the default fleetmaster gridmap parameters (see grid_opencv.h).
	 * origin: 0., 0., 0.
	 * resolution: .1
	 * width: 100
	 * height: 100
	 */
	public void useDefaultGridParams() {
		gridParams = new GridParams(0., 0., 0., .1, new NativeLong(100), new NativeLong(100));
		INSTANCE.init(gridParams);
	}
	
	/**
	 * Set if showing the content of each GridMaps.
	 * @param enable true if enabled.
	 */
	public void show(boolean enable) {
		INSTANCE.show(enable);
	}
	
	/**
	 * Set the trajectory parameters for a robot (see trajectory_processor.h).
	 * @param robotID The robot ID.
	 */
	public void addTrajParams(int robotID, double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double initVel, double endVel, 
		    double initSteeringAngleVel, double endSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc, double timeStep, double wheelBaseX, double wheelBaseY, boolean useInitialState,
		    int nbZeroVelControlCommands, double minDist, boolean useCoordTimeAccConstraints, boolean useCoordTimeContraintPoints, boolean debug, String debugPrefix, double creepSpeed, double creepDistance, boolean setCreepSpeedAsEndConstraint, int citiTruckNbClearSpeedCommands) {;
		    
		    trajParams.put(robotID, new TrajParams(maxVel, maxVelRev, useSteerDriveVel, maxRotationalVel, maxRotationalVelRev, maxSteeringAngleVel, initVel, endVel, 
				    initSteeringAngleVel, endSteeringAngleVel, maxAcc, maxRotationalAcc, maxSteeringAngleAcc, timeStep, wheelBaseX, wheelBaseY, useInitialState,
				    nbZeroVelControlCommands, minDist, useCoordTimeAccConstraints, useCoordTimeContraintPoints, debug, debugPrefix, creepSpeed, creepDistance, setCreepSpeedAsEndConstraint, citiTruckNbClearSpeedCommands));
	}
	
	/**
	 * Add a trajectory envelope to the fleetmaster gridmap.
	 * @param te The trajectory envelope to add.
	 * @return
	 */
	public boolean addPath(TrajectoryEnvelope te) {
		return addPath(te.getRobotID(), te.getID(), te.getTrajectory().getPoseSteering(), te.getFootprint().getCoordinates());
	}
	
	/**
	 * Add a spatial envelope to the fleetmaster gridmap.
	 * @param robotID ID of the robot for which adding the new path.
	 * @param pathID ID of the path to be added.
	 * @param pathToAdd The path to be added.
	 * @param coordinates The footprint to be swept along the path.
	 * @return true if success.
	 */
	public boolean addPath(int robotID, int pathID, PoseSteering[] pathToAdd, Coordinate ... coordinates) {
		if (gridParams == null || pathToAdd.length == 0) return false;
		if (coordinates.length == 0) coordinates = DEFAULT_FOOTPRINT;
		TrajParams trjParams = DEFAULT_TRAJ_PARAMS;
		if (trajParams.containsKey(pathID)) trjParams = trajParams.get(robotID);
		if (!clearPath(pathID)) return false;
		
		//Parse the Java values for the path and the footprint
		List<PathPose> path = new ArrayList<PathPose>();
		for (int i = 0; i < pathToAdd.length; i++) {
			path.add(new PathPose(pathToAdd[i].getX(),pathToAdd[i].getY(),pathToAdd[i].getTheta()));
		}
		List<Pair<Double,Double>> footprint = new ArrayList<Pair<Double,Double>>();
		for (int i = 0; i < coordinates.length; i++) footprint.add(new Pair<Double,Double>(coordinates[i].x,coordinates[i].y));
		
		//Call the method
		paths.put(pathID, INSTANCE.addPath(path, trjParams, footprint));
		return true;
	}
	
	/**
	 * Remove the path related to the given {@link TrajectoryEnvelope} from the fleetmaster gridmap.
	 * @param teID The ID of the trajectory envelope to be cleared.
	 */
	public boolean clearPath(int teID) {
		if (gridParams != null && paths.containsKey(teID)) return false;
		INSTANCE.removePath(paths.get(teID));
		return true;
	}
	
	/**
	 * Update the current progress along the trajectory envelope (according to the last received robot report).
	 * @param teID The trajectory envelope ID.
	 * @param currentIdx The current path index.
	 * @return true if the path index has been correctly updated.
	 */
	public boolean updateCurrentPathIdx(int teID, int currentIdx) {
		if (gridParams != null && paths.containsKey(teID)) {
			 return INSTANCE.updateCurrentPathIdx(paths.get(teID), new NativeLong(currentIdx));
		 }
		 return false;
	}
	
	/**
	 * Returning the increment of the time of completion of both the robots when giving precedence to the other at a given @{CriticalSection}. Specifically,
	 * the first value is related to the first robot giving precedence to the second one, viceversa the second value.
	 * Negative delays will be returned to indicate how much a robot may be delayed before the nominal temporal profile of the two robots may conflict while assuming the critical point of both to be set to -1.
	 * @param cs The critical section to be queried.
	 * @param te1TCDelays Additional delays to the single-robot time of completion of te1 due to future critical sections along te1 (to be used for propagation).
	 * @param te2TCDelays Additional delays to the single-robot time of completion of te2 due to future critical sections along te2 (to be used for propagation).
	 * @return The time of completion increments.
	 */
	public Pair<Double,Double> queryTimeDelay(CriticalSection cs, ArrayList<Pair<NativeLong, Double>> te1TCDelays, ArrayList<Pair<NativeLong, Double>> te2TCDelays) {
		
		if (gridParams == null || cs == null) return new Pair<Double, Double>(Double.NaN, Double.NaN);
		
		int teID1 = cs.getTe1().getID();
		int teID2 = cs.getTe2().getID();
		if (paths.containsKey(teID1) && (paths.containsKey(teID2))) {
		return INSTANCE.queryTimeDelay(paths.get(teID1), paths.get(teID2), 
				new Pair<NativeLong, NativeLong>(new NativeLong(cs.getTe1Start()), new NativeLong(cs.getTe1End())), new Pair<NativeLong, NativeLong>(new NativeLong(cs.getTe2Start()), new NativeLong(cs.getTe2End())), 
				te1TCDelays, te2TCDelays);
		}
		return new Pair<Double, Double>(Double.NaN, Double.NaN);
	}
}