package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.HashMap;
import java.util.logging.Logger;

import org.jgrapht.alg.util.Pair;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.NativeLong;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.GridParams;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PathPose;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PropagationTCDelays;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.TrajParams;


/**
 * TODO Write down class description. 
 * @author am
 *
 */
public class FleetMasterInterface {
	
	private HashMap<Integer, NativeLong> paths = null; //teID (or this pathID), fleetmaster pathID 
	private HashMap<Integer, TrajParams> trajParams = null; //robotID, trajectory parameters
	private GridParams gridParams = null;
	PointerByReference p = null;
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(FleetMasterInterface.class);
	
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
	
	/**
	 * The default trajectory params used for robots if none is specified.
	 * (Values are defined according to the file trajectory_processor.h).
	 */
	public static TrajParams DEFAULT_TRAJ_PARAMS = null;


	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("fleetmaster", "FleetMasterInterface"); //FIXME How to add the path of the library? Now it is in {$FLEETMASTER_WS}/devel
		INSTANCE = Native.loadLibrary("fleetmaster", FleetMasterInterfaceLib.class);
	}
	
	/**
	 * Class constructor.
	 */
	public FleetMasterInterface() {
		
		DEFAULT_TRAJ_PARAMS = new TrajParams();
		DEFAULT_TRAJ_PARAMS.maxVel = 1.;
	    DEFAULT_TRAJ_PARAMS.maxVelRev = 1.;
	    DEFAULT_TRAJ_PARAMS.useSteerDriveVel = true;
	    DEFAULT_TRAJ_PARAMS.maxRotationalVel = 1.;	
	    DEFAULT_TRAJ_PARAMS.maxRotationalVelRev = 1.;
	    DEFAULT_TRAJ_PARAMS.maxSteeringAngleVel = 1.;
	    DEFAULT_TRAJ_PARAMS.initVel = 0.;
	    DEFAULT_TRAJ_PARAMS.endVel = 0.;
	    DEFAULT_TRAJ_PARAMS.initSteeringAngleVel = 0.;
	    DEFAULT_TRAJ_PARAMS.endSteeringAngleVel = 0.;
	    DEFAULT_TRAJ_PARAMS.maxAcc = 1.;
	    DEFAULT_TRAJ_PARAMS.maxRotationalAcc = 1.;
	    DEFAULT_TRAJ_PARAMS.maxSteeringAngleAcc = 1.;
	    
		gridParams = new GridParams();
		gridParams.origin.x = 0.;
		gridParams.origin.y = 0.;
		gridParams.origin.theta = 0.;
		gridParams.resolution = .1;
		gridParams.width = new NativeLong(100);
		gridParams.height = new NativeLong(100);
	    
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
	 * Return the value of the trajectory parameters used for robots if none is specified.
	 */
	public Coordinate[] getDefaultFootprint() {
		return DEFAULT_FOOTPRINT;
	}
	
	/**
	 * Set the value of the trajectory parameters used for robots if none is specified.
	 */
	public void setDefaultTrajectoryParams(double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double initVel, double endVel, 
		    double initSteeringAngleVel, double endSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc) {
		DEFAULT_TRAJ_PARAMS.maxVel = maxVel;
		DEFAULT_TRAJ_PARAMS.maxVelRev = maxVelRev;
		DEFAULT_TRAJ_PARAMS.useSteerDriveVel = useSteerDriveVel;
		DEFAULT_TRAJ_PARAMS.maxRotationalVel = maxRotationalVel;
		DEFAULT_TRAJ_PARAMS.maxRotationalVelRev = maxRotationalVelRev;
		DEFAULT_TRAJ_PARAMS.maxSteeringAngleVel = maxSteeringAngleVel;
		DEFAULT_TRAJ_PARAMS.initVel = initVel;
		DEFAULT_TRAJ_PARAMS.endVel = endVel;
		DEFAULT_TRAJ_PARAMS.initSteeringAngleVel = initSteeringAngleVel;
		DEFAULT_TRAJ_PARAMS.endSteeringAngleVel = endSteeringAngleVel;
		DEFAULT_TRAJ_PARAMS.maxAcc = maxAcc;
		DEFAULT_TRAJ_PARAMS.maxRotationalAcc = maxRotationalAcc;
		DEFAULT_TRAJ_PARAMS.maxSteeringAngleAcc = maxSteeringAngleAcc;
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
		gridParams.origin.x = origin_x;
		gridParams.origin.y = origin_y;
		gridParams.origin.theta = origin_theta;
		gridParams.resolution = resolution;
		gridParams.width = new NativeLong(width);
		gridParams.height = new NativeLong(height);
	}
	
	/**
	 * Initilize gridmaps.
	 */
	public void init() {
		p = INSTANCE.init(gridParams);
	}
	
	/**
	 * Set if showing the content of each GridMaps.
	 * @param enable <code>true</code> if enabled.
	 */
	public void show(boolean enable) {
		INSTANCE.show(p, enable);
	}
	
	/**
	 * Set the trajectory parameters for a robot (see trajectory_processor.h).
	 * @param robotID The robot ID.
	 */
	public void addTrajParams(int robotID, double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double initVel, double endVel, 
		    double initSteeringAngleVel, double endSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc) {;
		    
		    trajParams.put(robotID, new TrajParams());
		    trajParams.get(robotID).maxVel = maxVel;
		    trajParams.get(robotID).maxVelRev = maxVelRev;
		    trajParams.get(robotID).useSteerDriveVel = useSteerDriveVel;
		    trajParams.get(robotID).maxRotationalVel = maxRotationalVel;
		    trajParams.get(robotID).maxRotationalVelRev = maxRotationalVelRev;
		    trajParams.get(robotID).maxSteeringAngleVel = maxSteeringAngleVel;
		    trajParams.get(robotID).initVel = initVel;
		    trajParams.get(robotID).endVel = endVel;
		    trajParams.get(robotID).initSteeringAngleVel = initSteeringAngleVel;
		    trajParams.get(robotID).endSteeringAngleVel = endSteeringAngleVel;
		    trajParams.get(robotID).maxAcc = maxAcc;
		    trajParams.get(robotID).maxRotationalAcc = maxRotationalAcc;
		    trajParams.get(robotID).maxSteeringAngleAcc = maxSteeringAngleAcc;
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
	 * @return <code>true</code> if success.
	 */
	public boolean addPath(int robotID, int pathID, PoseSteering[] pathToAdd, Coordinate ... coordinates) {
		if (p == null || pathToAdd.length == 0) return false; //FIXME log error
		
		//already added
		if (paths.containsKey(pathID)) {
			metaCSPLogger.warning("Path already stored.");
			return true;
		}
		
		//Add the new path
		clearPath(pathID);
		
		//Parse the Java values for the path and the footprint
		PathPose[] path = (PathPose[])new PathPose().toArray(pathToAdd.length);
		double[] steering = new double[pathToAdd.length];
		for (int i = 0; i < pathToAdd.length; i++) {
			path[i].x = pathToAdd[i].getX();
			path[i].y = pathToAdd[i].getY();
			path[i].theta = pathToAdd[i].getTheta();
			steering[i] = pathToAdd[i].getSteering();
		}
		
		if (coordinates.length == 0) coordinates = DEFAULT_FOOTPRINT;	
		double[] coordinates_x = new double[coordinates.length];
		double[] coordinates_y = new double[coordinates.length];
		for (int i = 0; i < coordinates.length; i++) {
			coordinates_x[i] = coordinates[i].x;
			coordinates_y[i] = coordinates[i].y;
		}
		
		//Call the method
		if (trajParams.containsKey(pathID)) 
			paths.put(pathID, INSTANCE.addPath(p, path, steering, pathToAdd.length, trajParams.get(robotID), coordinates_x, coordinates_y, coordinates.length));
		else
			paths.put(pathID, INSTANCE.addPath(p, path, steering, pathToAdd.length, DEFAULT_TRAJ_PARAMS, coordinates_x, coordinates_y, coordinates.length));

		return true;
	}
	
	/**
	 * Remove the path related to the given {@link TrajectoryEnvelope} from the fleetmaster gridmap.
	 * @param teID The ID of the trajectory envelope to be cleared.
	 */
	public boolean clearPath(int teID) {
		if (p != null && paths.containsKey(teID)) {
			INSTANCE.removePath(p, paths.get(teID));
			paths.remove(teID);
			return true;
		}
		return false;
	}
	
	/**
	 * Update the current progress along the trajectory envelope (according to the last received robot report).
	 * @param teID The trajectory envelope ID.
	 * @param currentIdx The current path index.
	 * @return <code>true</code> if the path index has been correctly updated.
	 */
	public boolean updateCurrentPathIdx(int teID, int currentIdx) {
		if (p != null && paths.containsKey(teID)) {
			 return INSTANCE.updateCurrentPathIdx(p, paths.get(teID), new NativeLong(currentIdx));
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
	public Pair<Double,Double> queryTimeDelay(CriticalSection cs, PropagationTCDelays te1TCDelays, PropagationTCDelays te2TCDelays) {
		
		if (cs == null) return new Pair<Double, Double>(Double.NaN, Double.NaN);
		
		int teID1 = cs.getTe1().getID();
		int teID2 = cs.getTe2().getID();
		if (paths.containsKey(teID1) && (paths.containsKey(teID2))) {
			DoubleByReference delayTe1 = new DoubleByReference();
			DoubleByReference delayTe2 = new DoubleByReference();
			INSTANCE.queryTimeDelay(p, paths.get(teID1), paths.get(teID2), cs.getTe1Start(), cs.getTe1End(), cs.getTe2Start(), cs.getTe2End(), te1TCDelays, te2TCDelays, delayTe1, delayTe2);
			return new Pair<Double, Double>(delayTe1.getValue(), delayTe2.getValue());
		}
		return new Pair<Double, Double>(Double.NaN, Double.NaN);
	}
}