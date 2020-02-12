package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.Arrays;
import java.util.List;

import com.sun.jna.Library;
import com.sun.jna.NativeLong;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.PointerByReference;


public interface FleetMasterInterfaceLib extends Library {
			
	PointerByReference init(GridParams gridParams);
	
	NativeLong addPath(PointerByReference p, PathPose[] path, double[] steering, int pathLength, TrajParams trajParams, double[] coordinates_x, double[] coordinates_y, int num_coordinate);
	
    void removePath(PointerByReference p, NativeLong id);

    boolean updateCurrentPathIdx(PointerByReference p, NativeLong pathId, NativeLong currentIdx);
    
    void queryTimeDelay(PointerByReference p, NativeLong pathId1, NativeLong pathId2, int csStart1, int csEnd1, int csStart2, int csEnd2,
    				PropagationTCDelays pTC1, PropagationTCDelays pTC2, DoubleByReference delay1, DoubleByReference delay2);

    
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
		public boolean debug;
		
		public GridParams() {}
		public GridParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"origin", "resolution", "width", "height", "debug"});
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
	    public double maxAcc;
	    public double maxRotationalAcc;
	    public double maxSteeringAngleAcc;

		public TrajParams() {}
		public TrajParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {
			"maxVel", "maxVelRev", "useSteerDriveVel", "maxRotationalVel", "maxRotationalVelRev", "maxSteeringAngleVel", "maxAcc", "maxRotationalAcc", "maxSteeringAngleAcc"});
		}
	}
	
	public static class PropagationTCDelays extends Structure {
		public static class ByReference extends PropagationTCDelays implements Structure.ByReference {}

        public NativeLong[] indices = new NativeLong[1];
        public double[] values = new double[1];
        public int size = 0;
		
		public PropagationTCDelays() {}
		public PropagationTCDelays(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"indices", "values", "size"});
		}
	}
}
