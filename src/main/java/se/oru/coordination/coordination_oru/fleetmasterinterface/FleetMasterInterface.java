package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.HashMap;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;


public class FleetMasterInterface {
	
	private HashMap<Integer,Long> paths = null; //robot, pathID
	private HashMap<Integer,PointerByReference> trajParams = null; //robotID, trajectory parameters
	private PointerByReference gridParams = null;
	
	public static FleetMasterInterfaceLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("fleetmaster", "FleetMasterInterface");
		INSTANCE = Native.loadLibrary("fleetmaster", FleetMasterInterfaceLib.class);
	}
	
	void addPath(PointerByReference path, int robotID, PointerByReference coordinates) {
		clearPath(robotID);
		//controlli vari di variabili?
		paths.put(robotID, INSTANCE.addPath(path, trajParams.get(robotID), coordinates));
	}
	
	void clearPath(int robotID) {
		if (paths.containsKey(robotID)) {
			INSTANCE.removePath(paths.get(robotID));
		}
	}
}
