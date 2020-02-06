package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.jgrapht.alg.util.Pair;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.NativeLong;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.GridParams;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PathPose;
import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.TrajParams;


public class FleetMasterInterface {
	
	private HashMap<Integer, NativeLong> paths = null; //teID, pathID
	private HashMap<Integer, TrajParams> trajParams = null; //FIXME teID, trajectory parameters
	
	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("fleetmaster", "FleetMasterInterface");
		INSTANCE = Native.loadLibrary("fleetmaster", FleetMasterInterfaceLib.class);
	}
	
	/**
	 * 
	 * @param origin_x
	 * @param origin_y
	 * @param origin_theta
	 * @param resolution
	 * @param width
	 * @param height
	 */
	void init(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height) {
		INSTANCE.init(new GridParams(origin_x, origin_y, origin_theta, resolution, new NativeLong(width), new NativeLong(height)));
	}
	
	/**
	 * 
	 * @param teID
	 * @param pathToAdd
	 * @param coordinates
	 * @return
	 */
	boolean addPath(int teID, List<PoseSteering> pathToAdd, Coordinate ... coordinates) {
		if (pathToAdd.size() == 0 || coordinates.length == 0) return false;
		clearPath(teID);
		
		//Parse the Java values for the path and the footprint
		List<PathPose> path = new ArrayList<PathPose>();
		for (int i = 0; i < pathToAdd.size(); i++) {
			path.add(new PathPose(pathToAdd.get(i).getX(),pathToAdd.get(i).getY(),pathToAdd.get(i).getTheta()));
		}
		List<Pair<Double,Double>> footprint = new ArrayList<Pair<Double,Double>>();
		for (int i = 0; i < coordinates.length; i++) footprint.add(new Pair<Double,Double>(coordinates[i].x,coordinates[i].y));
		
		//Call the method
		paths.put(teID, INSTANCE.addPath(path, trajParams.get(teID), footprint));
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
	 * Function to update the current progress along the path according to the last received robot report.
	 * @param robotID The robot ID.
	 * @param currentIdx The path index.
	 * @return true if the path index has been correctly updated.
	 */
	boolean updateCurrentPathIdx(int teID, int currentIdx) {
		if (paths.containsKey(teID)) {
			 return INSTANCE.updateCurrentPathIdx(paths.get(teID), new NativeLong(currentIdx));
		 }
		 return false;
	}
	
	/**
	 * 
	 * @param cs
	 * @param pathId1TTCDelays
	 * @param pathId2TTCDelays
	 * @return
	 */
	Pair<Double,Double> queryTimeDelay(CriticalSection cs, ArrayList<Pair<NativeLong, Double>> pathId1TTCDelays, ArrayList<Pair<NativeLong, Double>> pathId2TTCDelays) {
		int teID1 = cs.getTe1().getID();
		int teID2 = cs.getTe2().getID();
		if (paths.containsKey(teID1) && (paths.containsKey(teID2))) {
		return INSTANCE.queryTimeDelay(paths.get(teID1), paths.get(teID2), 
				new Pair<NativeLong, NativeLong>(new NativeLong(cs.getTe1Start()), new NativeLong(cs.getTe1End())), new Pair<NativeLong, NativeLong>(new NativeLong(cs.getTe2Start()), new NativeLong(cs.getTe2End())), 
				pathId1TTCDelays, pathId2TTCDelays);
		}
		return new Pair<Double, Double>(Double.NaN, Double.NaN);
	}
}
