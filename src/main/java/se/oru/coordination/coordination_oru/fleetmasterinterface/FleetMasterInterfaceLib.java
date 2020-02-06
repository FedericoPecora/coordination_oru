package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.jgrapht.alg.util.Pair;

import com.sun.jna.NativeLong;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;

public interface FleetMasterInterfaceLib {
	
	void show(boolean enable);
	
	void init(GridParams gridParams);
	
	NativeLong addPath(List<PathPose> path, TrajParams trajParams, List<Pair<Double,Double>> footPrint);
	
    void removePath(NativeLong id);

    boolean updateCurrentPathIdx(NativeLong pathId, NativeLong currentIdx);
    
    Pair<Double,Double> queryTimeDelay(NativeLong pathId1, NativeLong pathId2, Pair<NativeLong, NativeLong> indexRangePath1, Pair<NativeLong, NativeLong> indexRangePath2, ArrayList<Pair<NativeLong, Double>> pathId1TTCDelays, ArrayList<Pair<NativeLong, Double>> pathId2TTCDelays);
	
	public static class PathPose extends Structure {
		public static class ByReference extends PathPose implements Structure.ByReference {
			public ByReference(double x, double y, double theta) {
				super(x, y, theta);
			}
			public ByReference(Pointer p) {
				super(p);
			}
			}

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
		public static class ByReference extends GridParams implements Structure.ByReference {

			public ByReference(double origin_x, double origin_y, double origin_theta, double resolution,
					NativeLong width, NativeLong height) {
				super(origin_x, origin_y, origin_theta, resolution, width, height);
			}
			public ByReference(Pointer p) {
				super(p);
			}
		}

		public PathPose origin;
		public double resolution;
		public NativeLong width;
		public NativeLong height;
		
		public GridParams(double origin_x, double origin_y, double origin_theta, double resolution, NativeLong width, NativeLong height) {
			this.origin = new PathPose(origin_x, origin_y, origin_theta);
			this.resolution = resolution;
			this.width = width;
			this.height = height;
		}
		public GridParams(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"origin", "resolution", "width", "height"});
		}
	}
	
	public static class TrajParams extends Structure {
		public static class ByReference extends TrajParams implements Structure.ByReference {

			public ByReference(double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel,
					double maxRotationalVelRev, double maxSteeringAngleVel, double initVel, double endVel,
					double initSteeringAngleVel, double endSteeringAngleVel, double maxAcc, double maxRotationalAcc,
					double maxSteeringAngleAcc, double timeStep, double wheelBaseX, double wheelBaseY,
					boolean useInitialState, int nbZeroVelControlCommands, double minDist,
					boolean useCoordTimeAccConstraints, boolean useCoordTimeContraintPoints, boolean debug,
					String debugPrefix, double creepSpeed, double creepDistance, boolean setCreepSpeedAsEndConstraint,
					int citiTruckNbClearSpeedCommands) {
				super(maxVel, maxVelRev, useSteerDriveVel, maxRotationalVel, maxRotationalVelRev, maxSteeringAngleVel, initVel, endVel,
						initSteeringAngleVel, endSteeringAngleVel, maxAcc, maxRotationalAcc, maxSteeringAngleAcc, timeStep, wheelBaseX,
						wheelBaseY, useInitialState, nbZeroVelControlCommands, minDist, useCoordTimeAccConstraints,
						useCoordTimeContraintPoints, debug, debugPrefix, creepSpeed, creepDistance, setCreepSpeedAsEndConstraint,
						citiTruckNbClearSpeedCommands);
			}
			public ByReference(Pointer p) {
				super(p);
			}
		}

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
		
		public TrajParams(double maxVel, double maxVelRev, boolean useSteerDriveVel, double maxRotationalVel, double maxRotationalVelRev, double maxSteeringAngleVel, double initVel, double endVel, 
	    double initSteeringAngleVel, double endSteeringAngleVel, double maxAcc, double maxRotationalAcc, double maxSteeringAngleAcc, double timeStep, double wheelBaseX, double wheelBaseY, boolean useInitialState,
	    int nbZeroVelControlCommands, double minDist, boolean useCoordTimeAccConstraints, boolean useCoordTimeContraintPoints, boolean debug, String debugPrefix, double creepSpeed, double creepDistance, boolean setCreepSpeedAsEndConstraint, int citiTruckNbClearSpeedCommands) {
		    this.maxVel = maxVel;
		    this.maxVelRev = maxVelRev;
		    this.useSteerDriveVel = useSteerDriveVel;
		    this.maxRotationalVel = maxRotationalVel;
		    this.maxRotationalVelRev = maxRotationalVelRev;
		    this.maxSteeringAngleVel = maxSteeringAngleVel;
		    this.initVel = initVel;
		    this.endVel = endVel;
		    this.initSteeringAngleVel = initSteeringAngleVel;
		    this.endSteeringAngleVel = endSteeringAngleVel;
		    this.maxAcc = maxAcc;
		    this.maxRotationalAcc = maxRotationalAcc;
		    this.maxSteeringAngleAcc = maxSteeringAngleAcc;
		    this.timeStep = timeStep;
		    this.wheelBaseX = wheelBaseX;
		    this.wheelBaseY = wheelBaseY;
		    this.useInitialState = useInitialState;
		    this.nbZeroVelControlCommands = nbZeroVelControlCommands;
		    this.minDist = minDist;
		    this.useCoordTimeAccConstraints = useCoordTimeAccConstraints;
		    this.useCoordTimeContraintPoints = useCoordTimeContraintPoints;
		    this.debug = debug;
		    this.debugPrefix = debugPrefix;
		    this.creepSpeed = creepSpeed;
		    this.creepDistance = creepDistance;
		    this.setCreepSpeedAsEndConstraint = setCreepSpeedAsEndConstraint;
		    this.citiTruckNbClearSpeedCommands = citiTruckNbClearSpeedCommands;
		}
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
