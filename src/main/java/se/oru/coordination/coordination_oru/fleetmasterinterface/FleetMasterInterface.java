package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.jgrapht.alg.util.Pair;
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


public class FleetMasterInterface {
	
	private HashMap<Integer, NativeLong> paths = new HashMap<Integer, NativeLong>(); //teID, pathID
	private HashMap<Integer, TrajParams> trajParams = new HashMap<Integer, TrajParams>(); //robotID, trajectory parameters
	
	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("fleetmaster", "FleetMasterInterface"); //FIXME How to add the path of the library? Now it is in {$FLEETMASTER_WS}/devel
		INSTANCE = Native.loadLibrary("fleetmaster", FleetMasterInterfaceLib.class);
	}
	
	/**
	 * Set the fleetmaster gridmap parameters.
	 * @param origin_x The x origin of the map
	 * @param origin_y The y origin of the map
	 * @param origin_theta The theta origin of the map
	 * @param resolution The resolution of the map (in meters/cell).
	 * @param width Number of columns of the map (in cells).
	 * @param height Number of rows of the map (in cells).
	 */
	void setGridMapParams(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height) {
		INSTANCE.init(new GridParams(origin_x, origin_y, origin_theta, resolution, new NativeLong(width), new NativeLong(height)));
	}
	
	/**
	 * Set the trajectory parameters for a robot
	 * @param robotID The robot ID.
	 * @param maxVel
	 * @param maxVelRev
	 * @param useSteerDriveVel
	 * @param maxRotationalVel
	 * @param maxRotationalVelRev
	 * @param maxSteeringAngleVel
	 * @param initVel
	 * @param endVel
	 * @param initSteeringAngleVel
	 * @param endSteeringAngleVel
	 * @param maxAcc
	 * @param maxRotationalAcc
	 * @param maxSteeringAngleAcc
	 * @param timeStep
	 * @param wheelBaseX
	 * @param wheelBaseY
	 * @param useInitialState
	 * @param nbZeroVelControlCommands
	 * @param minDist
	 * @param useCoordTimeAccConstraints
	 * @param useCoordTimeContraintPoints
	 * @param debug
	 * @param debugPrefix
	 * @param creepSpeed
	 * @param creepDistance
	 * @param setCreepSpeedAsEndConstraint
	 * @param citiTruckNbClearSpeedCommands
	 */
	void addTrajParams(int robotID, double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double initVel, double endVel, 
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
	boolean addPath(TrajectoryEnvelope te) {
		PoseSteering[] pathToAdd = te.getTrajectory().getPoseSteering();
		Coordinate[] coordinates = te.getFootprint().getCoordinates();
		if (pathToAdd.length == 0 || coordinates.length == 0 || !trajParams.containsKey(te.getRobotID())) return false;
		clearPath(te.getID());
		
		//Parse the Java values for the path and the footprint
		List<PathPose> path = new ArrayList<PathPose>();
		for (int i = 0; i < pathToAdd.length; i++) {
			path.add(new PathPose(pathToAdd[i].getX(),pathToAdd[i].getY(),pathToAdd[i].getTheta()));
		}
		List<Pair<Double,Double>> footprint = new ArrayList<Pair<Double,Double>>();
		for (int i = 0; i < coordinates.length; i++) footprint.add(new Pair<Double,Double>(coordinates[i].x,coordinates[i].y));
		
		//Call the method
		paths.put(te.getID(), INSTANCE.addPath(path, trajParams.get(te.getRobotID()), footprint));
		return true;
	}
	
	/**
	 * Remove the path related to the given {@link TrajectoryEnvelope} from the fleetmaster gridmap.
	 * @param teID The ID of the trajectory envelope to be cleared.
	 */
	void clearPath(int teID) {
		if (paths.containsKey(teID)) {
			INSTANCE.removePath(paths.get(teID));
		}
	}
	
	/**
	 * Update the current progress along the trajectory envelope (according to the last received robot report).
	 * @param teID The trajectory envelope ID.
	 * @param currentIdx The current path index.
	 * @return true if the path index has been correctly updated.
	 */
	boolean updateCurrentPathIdx(int teID, int currentIdx) {
		if (paths.containsKey(teID)) {
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
	Pair<Double,Double> queryTimeDelay(CriticalSection cs, ArrayList<Pair<NativeLong, Double>> te1TCDelays, ArrayList<Pair<NativeLong, Double>> te2TCDelays) {
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
