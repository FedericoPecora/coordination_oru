package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.jgrapht.alg.util.Pair;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.NativeLong;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.fleetmasterinterface.FleetMasterInterfaceLib.PathPose;


public class FleetMasterInterface {
	
	private HashMap<Integer,NativeLong> paths = null; //robot, pathID
	private HashMap<Integer,PointerByReference> trajParams = null; //FIXME robotID, trajectory parameters
	private PointerByReference gridParams = null; //FIXME
	
	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("fleetmaster", "FleetMasterInterface");
		INSTANCE = Native.loadLibrary("fleetmaster", FleetMasterInterfaceLib.class);
	}
	
	boolean addPath(int robotID, List<PoseSteering> pathToAdd, Coordinate ... coordinates) {
		if (pathToAdd.size() == 0 || coordinates.length == 0) return false;
		clearPath(robotID);
		
		//Parse the Java values for the path and the footprint
		List<PathPose> path = new ArrayList<PathPose>();
		for (int i = 0; i < pathToAdd.size(); i++) {
			path.add(new PathPose(pathToAdd.get(i).getX(),pathToAdd.get(i).getY(),pathToAdd.get(i).getTheta()));
		}
		List<Pair<Double,Double>> footprint = new ArrayList<Pair<Double,Double>>();
		for (int i = 0; i < coordinates.length; i++) footprint.add(new Pair<Double,Double>(coordinates[i].x,coordinates[i].x));
		
		//Call the method
		paths.put(robotID, INSTANCE.addPath(path, trajParams.get(robotID), footprint));
		return true;
	}
	
	void clearPath(int robotID) {
		if (paths.containsKey(robotID)) {
			INSTANCE.removePath(paths.get(robotID));
		}
	}
}
