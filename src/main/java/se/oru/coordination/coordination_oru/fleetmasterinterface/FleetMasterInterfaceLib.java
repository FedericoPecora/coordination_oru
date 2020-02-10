package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.jgrapht.alg.util.Pair;

import com.sun.jna.Library;
import com.sun.jna.NativeLong;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.PointerByReference;


public interface FleetMasterInterfaceLib extends Library {
			
	PointerByReference init(GridParams gridParams);
	
	void show(PointerByReference p, boolean enable);
	
	NativeLong addPath(PointerByReference p, PathPose[] path, double[] steering, int pathLength, TrajParams trajParams, double[] coordinates_x, double[] coordinates_y, int num_coordinate);
	
    void removePath(PointerByReference p, NativeLong id);

    boolean updateCurrentPathIdx(PointerByReference p, NativeLong pathId, NativeLong currentIdx);
    
    //FIXME remove dynamic vectors
    Pair<Double,Double> queryTimeDelay(PointerByReference p, NativeLong pathId1, NativeLong pathId2, Pair<NativeLong, NativeLong> indexRangePath1, Pair<NativeLong, NativeLong> indexRangePath2, ArrayList<Pair<NativeLong, Double>> pathId1TTCDelays, ArrayList<Pair<NativeLong, Double>> pathId2TTCDelays);
	
	public static class PathPose extends Structure {
		public static class ByReference extends PathPose implements Structure.ByReference {}

		public double x;
		public double y;
		public double theta;
		
		public PathPose() {}
		public PathPose(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"x", "y", "theta"});
		}
	}
		
	public static class GridParams extends Structure {
		public static class ByReference extends GridParams implements Structure.ByReference {}

		public PathPose origin;
		public double resolution;
		public NativeLong width;
		public NativeLong height;
		
		public GridParams() {}
		public GridParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"origin", "resolution", "width", "height"});
		}
	}
	
	public static class TrajParams extends Structure {
		public static class ByReference extends TrajParams implements Structure.ByReference {}

		public double maxVel;
		public double maxVelRev;
		public boolean useSteerDriveVel;
	    public double maxRotationalVel;
	    public double maxRotationalVelRev;
	    public double maxSteeringAngleVel;
	    public double initVel;
	    public double endVel;
	    public double initSteeringAngleVel;
	    public double endSteeringAngleVel;
	    public double maxAcc;
	    public double maxRotationalAcc;
	    public double maxSteeringAngleAcc;
	    public double timeStep;
	    public double wheelBaseX;
	    public double wheelBaseY;
	    public boolean useInitialState;
	    public int nbZeroVelControlCommands;
	    public double minDist;
	    public boolean useCoordTimeAccConstraints;
	    public boolean useCoordTimeContraintPoints;
	    public boolean debug;
	    public String debugPrefix;
	    public double creepSpeed;
	    public double creepDistance;
	    public boolean setCreepSpeedAsEndConstraint;
	    public int citiTruckNbClearSpeedCommands;
		
		public TrajParams() {}
		public TrajParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {
			"maxVel", "maxVelRev", "useSteerDriveVel", "maxRotationalVel", "maxRotationalVelRev", "maxSteeringAngleVel", "initVel", "endVel",
			"initSteeringAngleVel", "endSteeringAngleVel", "maxAcc", "maxRotationalAcc", "maxSteeringAngleAcc", "timeStep", "wheelBaseX", "wheelBaseY", "useInitialState", "nbZeroVelControlCommands", "minDist", 
			"useCoordTimeAccConstraints", "useCoordTimeContraintPoints", "debug", "debugPrefix", "creepSpeed", "creepDistance", "setCreepSpeedAsEndConstraint", "citiTruckNbClearSpeedCommands"
			});
		}
	}
}
