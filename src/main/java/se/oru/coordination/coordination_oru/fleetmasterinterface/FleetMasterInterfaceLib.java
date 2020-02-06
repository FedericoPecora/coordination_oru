package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.jgrapht.alg.util.Pair;

import com.sun.jna.NativeLong;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.PointerByReference;

public interface FleetMasterInterfaceLib {
	
	void init(PointerByReference gridParams);
	
	NativeLong addPath(PointerByReference path, PointerByReference trajParams, PointerByReference footPrint);
	
    void removePath(NativeLong id);

    boolean updateCurrentPathIdx(NativeLong pathId, NativeLong currentIdx);
    
    double[] queryTimeDelay(NativeLong pathId1, NativeLong pathId2, Pair<NativeLong, NativeLong> indexRangePath1, Pair<NativeLong, NativeLong> indexRangePath2, ArrayList<Pair<NativeLong, Double>> pathId1TTCDelays, ArrayList<Pair<NativeLong, Double>> pathId2TTCDelays);
	
	public static class Pose2d extends Structure {
		public static class ByReference extends Pose2d implements Structure.ByReference {}

		public double x;
		public double y;
		public double theta;
		
		public Pose2d() {}
		public Pose2d(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"x", "y", "theta"});
		}
	}
	
	public static class GridParams extends Structure {
		public static class ByReference extends GridParams implements Structure.ByReference {}

		public Pose2d origin;
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

	    double maxVel;
	    double maxVelRev;
	    boolean useSteerDriveVel;
	    double maxRotationalVel;
	    double maxRotationalVelRev;
	    double maxSteeringAngleVel;
	    double initVel;
	    double endVel;
	    double initSteeringAngleVel;
	    double endSteeringAngleVel;
	    double maxAcc;
	    double maxRotationalAcc;
	    double maxSteeringAngleAcc;
	    double timeStep;
	    double wheelBaseX;
	    double wheelBaseY;
	    boolean useInitialState;
	    int nbZeroVelControlCommands;
	    double minDist;
	    boolean useCoordTimeAccConstraints;
	    boolean useCoordTimeContraintPoints;
	    boolean debug;
	    String debugPrefix;
	    double creepSpeed;
	    double creepDistance;
	    boolean setCreepSpeedAsEndConstraint;
	    int citiTruckNbClearSpeedCommands;
		
		public TrajParams() {}
		public TrajParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"maxVel", "maxVelRev", "useSteerDriveVel", "maxRotationalVel", "maxRotationalVelRev", "maxSteeringAngleVel", "initVel", "endVel",
			"initSteeringAngleVel", "endSteeringAngleVel", "maxAcc", "maxRotationalAcc", "maxSteeringAngleAcc", "timeStep", "wheelBaseX", "wheelBaseY", "useInitialState", "nbZeroVelControlCommands", "minDist", 
			"useCoordTimeAccConstraints", "useCoordTimeContraintPoints", "debug", "debugPrefix", "creepSpeed", "creepDistance", "setCreepSpeedAsEndConstraint", "citiTruckNbClearSpeedCommands"});
		}
	}
	
	public static class Polygon extends Structure {
		public static class ByReference extends Polygon implements Structure.ByReference {}

		public Pose2d[] points;
		
		public Polygon() {}
		public Polygon(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"points"});
		}
	}
}
