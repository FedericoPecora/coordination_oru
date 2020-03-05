package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.HashMap;
import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.NativeLong;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;

import aima.core.util.datastructure.Pair;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.GridParams;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PathPose;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.CumulatedIndexedDelaysList;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.TrajParams;

public abstract class AbstractFleetMasterInterface {
	protected HashMap<Integer, Long> paths = null; //teID (or this pathID), fleetmaster pathID 
	protected HashMap<Integer, TrajParams> trajParams = null; //robotID, trajectory parameters
	protected GridParams gridParams = null;
	protected PointerByReference p = null;
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
	 * @param origin_x The x coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta origin of the lower-left pixel map (counterclockwise rotation. 
	 * 					   However, many parts of the system currently ignore it).
	 * @param resolution The resolution of the map (in meters/cell). It is assumed this parameter to be global among the fleet.
	 * @param width Number of columns of the map (in cells) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (in cells) if dynamic sizing is not enabled.
	 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
	 * @param debug If <code>true</code>, it enables writing to screen debugging info.
	 */
	public AbstractFleetMasterInterface(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean debug) {
		DEFAULT_TRAJ_PARAMS = new TrajParams();
		DEFAULT_TRAJ_PARAMS.maxVel = 1.;
	    DEFAULT_TRAJ_PARAMS.maxVelRev = 1.;
	    DEFAULT_TRAJ_PARAMS.useSteerDriveVel = true;
	    DEFAULT_TRAJ_PARAMS.maxRotationalVel = 1.;	
	    DEFAULT_TRAJ_PARAMS.maxRotationalVelRev = 1.;
	    DEFAULT_TRAJ_PARAMS.maxSteeringAngleVel = 1.;
	    DEFAULT_TRAJ_PARAMS.maxAcc = 1.;
	    DEFAULT_TRAJ_PARAMS.maxRotationalAcc = 1.;
	    DEFAULT_TRAJ_PARAMS.maxSteeringAngleAcc = 1.;
	    
		gridParams = new GridParams();
		gridParams.origin.x = origin_x;
		gridParams.origin.y = origin_y;
		gridParams.origin.theta = origin_theta;
		gridParams.resolution = resolution;
		gridParams.width = new NativeLong(width);
		gridParams.height = new NativeLong(height);
		gridParams.dynamic_size = dynamic_size;
		gridParams.debug = debug;
	    
		this.paths = new HashMap<Integer, Long>();
		this.trajParams = new HashMap<Integer, TrajParams>();
	};
	
	/**
	 * Abstract class constructor
	 */
	public AbstractFleetMasterInterface() {
		this(0., 0., 0., 0.1, 100, 100, false, false);
	}
	
	/**
	 * Initialize parameters.
	 */
	public boolean init() {
		if (gridParams == null) return false;
		p = INSTANCE.init(gridParams);
		return true;
	}
		
	/**
	 * Set the value of the footprint used for robots if none is specified.
	 */
	public boolean setDefaultFootprint(Coordinate ... coordinates) {
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
	public void setDefaultTrajectoryParams(double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc) {
		DEFAULT_TRAJ_PARAMS.maxVel = maxVel;
		DEFAULT_TRAJ_PARAMS.maxVelRev = maxVelRev;
		DEFAULT_TRAJ_PARAMS.useSteerDriveVel = useSteerDriveVel;
		DEFAULT_TRAJ_PARAMS.maxRotationalVel = maxRotationalVel;
		DEFAULT_TRAJ_PARAMS.maxRotationalVelRev = maxRotationalVelRev;
		DEFAULT_TRAJ_PARAMS.maxSteeringAngleVel = maxSteeringAngleVel;
		DEFAULT_TRAJ_PARAMS.maxAcc = maxAcc;
		DEFAULT_TRAJ_PARAMS.maxRotationalAcc = maxRotationalAcc;
		DEFAULT_TRAJ_PARAMS.maxSteeringAngleAcc = maxSteeringAngleAcc;
	}
	
	/**
	 * Set the trajectory parameters for a robot. Use <code>-1</code> when the parameter is not known,
	 * where maxSteeringAngleVel equal to <code>-1</code> will automatically force useSteerDriveVel to be <code>false</code>.
	 * @param robotID The robot ID.
	 */
	public void setTrajParams(int robotID, double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc) {;
		    
		    trajParams.put(robotID, new TrajParams());
		    trajParams.get(robotID).maxVel = maxVel;
		    trajParams.get(robotID).maxVelRev = maxVelRev;
		    trajParams.get(robotID).useSteerDriveVel = useSteerDriveVel;
		    trajParams.get(robotID).maxRotationalVel = maxRotationalVel;
		    trajParams.get(robotID).maxRotationalVelRev = maxRotationalVelRev;
		    trajParams.get(robotID).maxSteeringAngleVel = maxSteeringAngleVel;
		    trajParams.get(robotID).maxAcc = maxAcc;
		    trajParams.get(robotID).maxRotationalAcc = maxRotationalAcc;
		    trajParams.get(robotID).maxSteeringAngleAcc = maxSteeringAngleAcc;
	}
	
	/**
	 * Add a spatial envelope to the fleetmaster gridmap while giving its bounding box.
	 * @param robotID ID of the robot for which adding the new path.
	 * @param pathID ID of the path to be added.
	 * @param pathToAdd The path to be added.
	 * @param coordinates The polygon of the robot footprint (clockwise or anti clockwise coordinates).
	 * @return <code>true</code> if success.
	 */
	public boolean addPath(int robotID, int pathID, PoseSteering[] pathToAdd, Geometry boundingBox, Coordinate ... coordinates) {
		if (p == null) {
			metaCSPLogger.severe("The fleetmaster has noot been initialized yet. Call init function first!");
			return false;
		}
		
		if (pathToAdd.length == 0) {
				metaCSPLogger.severe("Added paths cannot have have null length!");
				return false;
		}
		
		if (coordinates.length == 0) coordinates = DEFAULT_FOOTPRINT;	
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[coordinates.length+1];
		for (int i = 0; i < coordinates.length; i++) {
			newCoords[i] = coordinates[i];
		}
		newCoords[newCoords.length-1] = coordinates[0];
		gf.createPolygon(newCoords);
		if (gf.createPolygon(newCoords).getArea() == 0) {
			metaCSPLogger.severe("Robots' footprint cannot have null area!");
			throw new Error("Robots' footprint cannot have null area!");
		}
			
		//already added
		if (paths.containsKey(pathID) && paths.get(pathID) != new Long(0)) {
			metaCSPLogger.warning("Path already stored.");
			return true;
		}
		
		//Add the new path
		clearPath(pathID);
		
		//Parse the Java values for the path and the footprint
		double[] coordinates_x = new double[coordinates.length];
		double[] coordinates_y = new double[coordinates.length];
		for (int i = 0; i < coordinates.length; i++) {
			coordinates_x[i] = coordinates[i].x;
			coordinates_y[i] = coordinates[i].y;
		}
		
		PathPose[] path = (PathPose[])new PathPose().toArray(pathToAdd.length);
		double[] steering = new double[pathToAdd.length];
		for (int i = 0; i < pathToAdd.length; i++) {
			path[i].x = pathToAdd[i].getX();
			path[i].y = pathToAdd[i].getY();
			path[i].theta = pathToAdd[i].getTheta();
			steering[i] = pathToAdd[i].getSteering();
		}
		
		
		Coordinate[] bbx = null;
		if (boundingBox != null) bbx = boundingBox.getCoordinates();
		else if (gridParams.dynamic_size) {
			SpatialEnvelope se = TrajectoryEnvelope.createSpatialEnvelope(pathToAdd, coordinates);
			boundingBox = se.getPolygon();
		}
				
		//Call the method. -1 is used as special value to indicate that the path was not correctly added.
		Long pathCode = new Long(0);
		TrajParams trjp = trajParams.containsKey(robotID) ? trajParams.get(robotID) : DEFAULT_TRAJ_PARAMS;
		double bottom_left_x = (bbx == null) ? Double.MAX_VALUE : bbx[0].x;
		double bottom_left_y = (bbx == null) ? Double.MAX_VALUE : bbx[0].y;
		double top_right_x = (bbx == null) ? Double.MAX_VALUE : bbx[2].x;
		double top_right_y = (bbx == null) ? Double.MAX_VALUE : bbx[2].y;
		pathCode = new Long(INSTANCE.addPath(p, path, steering, pathToAdd.length, trjp, coordinates_x, coordinates_y, coordinates.length, bottom_left_x, bottom_left_y, top_right_x, top_right_y));
		paths.put(pathID, pathCode);

		metaCSPLogger.info("Adding path RobotID: " + robotID + ", pathID: " + pathID + ", fleetmaster pathID: " + pathCode + ".");
		
		return !pathCode.equals(new Long(0));
	}
	
	/**
	 * Add a trajectory envelope to the fleetmaster gridmap while giving its bounding box.
	 * @param te The trajectory envelope to add.
	 * @return <code>true</code> if success.
	 */
	public boolean addPath(TrajectoryEnvelope te) {
		if (te.getEnvelopeBoundingBox() == null) {
			metaCSPLogger.severe("The given TrajectoryEnvelope is empty!");
			throw new Error("The given TrajectoryEnvelope is empty!");
		}
		if (te.getEnvelopeBoundingBox().getArea() <= 0) {
			metaCSPLogger.severe("Robots' footprint cannot have null area!");
			throw new Error("Robots' footprint cannot have null area!");
		}
		return addPath(te.getRobotID(), te.getID(), te.getTrajectory().getPoseSteering(), te.getEnvelopeBoundingBox(), te.getFootprint().getCoordinates());
	}

	
	/**
	 * Remove the path from the fleetmaster GridMaps.
	 * @param pathID The ID of the path to be cleared.
	 */
	public boolean clearPath(int pathID) {
		if (p != null && paths.containsKey(pathID) && paths.get(pathID) != new Long(0)) {
			INSTANCE.removePath(p, paths.get(pathID));
			paths.remove(pathID);
			metaCSPLogger.info("Clearing path pathID: " + pathID);
			return true;
		}
		return false;
	}
	
	/**
	 * Update the current progress along the path (according to the last received robot report).
	 * @param pathID The path ID.
	 * @param currentIdx The current path index.
	 * @return <code>true</code> if the path index has been correctly updated.
	 */
	public boolean updateCurrentPathIdx(int pathID, int currentIdx) {
		if (p != null && paths.containsKey(pathID) && paths.get(pathID) != new Long(0)) {
			metaCSPLogger.info("Updating path pathID: " + pathID + ", current path index: " + currentIdx + ".");
			return INSTANCE.updateCurrentPathIdx(p, paths.get(pathID), new Long(Math.max(currentIdx, 0)));
		 }
		 return false;
	}
	
	/**
	 * Returning the increment of the time of completion of both the robots when giving precedence to the other at a given @{CriticalSection}. Specifically,
	 * the first value is the increment of the time of completion of the first robot due to the queried critical section when giving precedence to the second one, viceversa the second value.
	 * Negative delays will be returned to indicate how much a robot may be delayed before the nominal temporal profile of the two robots may conflict while assuming the critical point of both to be set to -1.
	 * @param cs The critical section to be queried.
	 * @param te1TCDelays Additional delays to the single-robot time of completion of te1 due to future critical sections along te1 (to be used for propagation).
	 * @param te2TCDelays Additional delays to the single-robot time of completion of te2 due to future critical sections along te2 (to be used for propagation).
	 * @return The time of completion increments.
	 */	
	public Pair<Double,Double> queryTimeDelay(CriticalSection cs, CumulatedIndexedDelaysList te1TCDelays, CumulatedIndexedDelaysList te2TCDelays) {
		Pair<Double, Double> ret = new Pair<Double, Double>(Double.NaN, Double.NaN);
		if (cs != null) {
			int teID1 = cs.getTe1().getID();
			int teID2 = cs.getTe2().getID();
			if (paths.containsKey(teID1) && (paths.containsKey(teID2))) {
				DoubleByReference delayTe1 = new DoubleByReference();
				DoubleByReference delayTe2 = new DoubleByReference();
				INSTANCE.queryTimeDelay(p, paths.get(teID1), paths.get(teID2), Math.max(0, cs.getTe1Start()-1), Math.min(cs.getTe1().getPathLength()-1, cs.getTe1End()+1), 
						Math.max(0, cs.getTe2Start()-1), Math.min(cs.getTe2().getPathLength()-1, cs.getTe2End()+1), te1TCDelays.indices, te1TCDelays.values, te1TCDelays.size, te2TCDelays.indices, te2TCDelays.values, te2TCDelays.size, delayTe1, delayTe2);
				ret = new Pair<Double, Double>(delayTe1.getValue(), delayTe2.getValue());
			}
		}
		metaCSPLogger.info("queryTimeDelay at cs: " + cs + " return: " + ret + ".");
		return ret;
	}
	
	/**
	 * Check if a path has been correctly added.
	 * @param pathID The pathID.
	 * @return <code>true</code> if correctly added.
	 */
	public boolean checkPathHasBeenAdded(int pathID) {
		if (paths.containsKey(pathID) && !paths.get(pathID).equals(new Long(0)))	
			return true;
		return false;
	}
	
	/**
	 * Check if a {@link TrajectoryEnvelope} has been correctly added.
	 * @param te The trajectory envelope.
	 * @return <code>true</code> if correctly added.
	 */
	public boolean checkTrajectoryEnvelopeHasBeenAdded(TrajectoryEnvelope te) {
		return checkPathHasBeenAdded(te.getID());
	}

}
