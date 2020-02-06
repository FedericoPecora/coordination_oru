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
	
	NativeLong addPath(List<PathPose> path, PointerByReference trajParams, List<Pair<Double,Double>> footPrint);
	
    void removePath(NativeLong id);

    boolean updateCurrentPathIdx(NativeLong pathId, NativeLong currentIdx);
    
    Pair<Double,Double> queryTimeDelay(NativeLong pathId1, NativeLong pathId2, Pair<NativeLong, NativeLong> indexRangePath1, Pair<NativeLong, NativeLong> indexRangePath2, ArrayList<Pair<NativeLong, Double>> pathId1TTCDelays, ArrayList<Pair<NativeLong, Double>> pathId2TTCDelays);
	
	public static class PathPose extends Structure {
		public static class ByReference extends PathPose implements Structure.ByReference {

			public ByReference(Pointer p) {
				super(p);
				// TODO Auto-generated constructor stub
			}}

		public double x;
		public double y;
		public double theta;
		
		public PathPose(double x, double y, double theta) {
			this.x = x;
			this.y = y;
			this.theta = theta;
		}
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
}
