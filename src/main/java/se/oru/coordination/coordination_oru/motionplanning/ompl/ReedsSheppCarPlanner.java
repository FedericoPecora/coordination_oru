package se.oru.coordination.coordination_oru.motionplanning.ompl;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.OccupancyMap;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlannerLib.PathPose;
import se.oru.coordination.coordination_oru.util.GeometrySmoother;
import se.oru.coordination.coordination_oru.util.GeometrySmoother.SmootherControl;

public class ReedsSheppCarPlanner extends AbstractMotionPlanner {
	
	public static enum PLANNING_ALGORITHM { RRTConnect, RRTstar, TRRT, SST, LBTRRT, PRMstar, SPARS, pRRT, LazyRRT }; 

	private double robotRadius = 1.0;
	private PointerByReference path = null;
	private IntByReference pathLength = null;
	private double distanceBetweenPathPoints = 0.5;
	private double turningRadius = 1.0;
	private double planningTimeInSecs = 30.0;
	private Coordinate[] collisionCircleCenters = null;
	private PLANNING_ALGORITHM algo = PLANNING_ALGORITHM.RRTConnect;
	
	public static ReedsSheppCarPlannerLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("simplereedssheppcarplanner", "SimpleReedsSheppCarPlanner");
		INSTANCE = Native.loadLibrary("simplereedssheppcarplanner", ReedsSheppCarPlannerLib.class);
	}

	@Override
	public AbstractMotionPlanner getCopy(boolean copyObstacles) {
		ReedsSheppCarPlanner ret = new ReedsSheppCarPlanner(this.algo);
		ret.setRadius(this.robotRadius);
		ret.setDistanceBetweenPathPoints(this.distanceBetweenPathPoints);
		ret.setTurningRadius(this.turningRadius);
		ret.setFootprint(this.footprintCoords);
		ret.setPlanningTimeInSecs(planningTimeInSecs);
		if (this.om != null) ret.om = new OccupancyMap(this.om, copyObstacles);
		return ret;
	}
	
	@Override
	public void setFootprint(Coordinate ... coords) {
		super.setFootprint(coords);
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[coords.length+1];
		for (int i = 0; i < coords.length; i++) {
			newCoords[i] = coords[i];
		}
		newCoords[newCoords.length-1] = coords[0];
		Polygon footprint = gf.createPolygon(newCoords);
		GeometrySmoother gs = new GeometrySmoother(gf);
		SmootherControl sc = new SmootherControl() {
	        public double getMinLength() {
	            return robotRadius;
	        }	        
	        public int getNumVertices(double length) {
	            return (int)(length/(2*robotRadius))+2;
	        }
	    };
	    gs.setControl(sc);
	    Polygon smoothFootprint = gs.smooth(footprint, 1.0);
		collisionCircleCenters = smoothFootprint.getCoordinates();
	}
	
	public Coordinate[] getCollisionCircleCenters() {
		return collisionCircleCenters;
	}
	
	public ReedsSheppCarPlanner() {
		this.algo = PLANNING_ALGORITHM.RRTConnect;
	}
	
	public ReedsSheppCarPlanner(PLANNING_ALGORITHM algo) {
		this.algo = algo;
	}

	public void setCirclePositions(Coordinate ... circlePositions) {
		this.collisionCircleCenters = circlePositions;
	}

	public void setRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setDistanceBetweenPathPoints(double maxDistance) {
		this.distanceBetweenPathPoints = maxDistance;
	}

	public void setTurningRadius(double rad) {
		this.turningRadius = rad;
	}
	
	public void setPlanningTimeInSecs(double planningTimeInSecs) {
		this.planningTimeInSecs = planningTimeInSecs;
	}
	
	public Pose getStart() {
		return this.start;
	}
	
	public Pose[] getGoals() {
		return this.goal;
	}
	
	public double getPlanningTimeInSecs() {
		return this.planningTimeInSecs;
	}
	
	@Override
	public boolean doPlanning() {
		ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();  
		for (int i = 0; i < this.goal.length; i++) {
			Pose start_ = null;
			Pose goal_ = this.goal[i];
			if (i == 0) start_ = this.start;
			else start_ = this.goal[i-1];
			path = new PointerByReference();
			pathLength = new IntByReference();
			double[] xCoords = new double[collisionCircleCenters.length];
			double[] yCoords = new double[collisionCircleCenters.length];
			int numCoords = collisionCircleCenters.length;
			for (int j = 0; j < collisionCircleCenters.length; j++) {
				xCoords[j] = collisionCircleCenters[j].x;
				yCoords[j] = collisionCircleCenters[j].y;
			}
			metaCSPLogger.info("Path planning with " + collisionCircleCenters.length + " circle positions");
			if (this.om != null) {
				byte[] occ = om.asByteArray();
				int w = om.getPixelWidth();
				int h = om.getPixelHeight();
				double res = om.getResolution();
				double mapOriginX = om.getMapOrigin().x;
				double mapOriginY = om.getMapOrigin().y;
				if (!INSTANCE.plan_multiple_circles(occ, w, h, res, mapOriginX, mapOriginY, robotRadius, xCoords, yCoords, numCoords, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius, planningTimeInSecs, algo.ordinal())) return false;
			}
			else {
				if (!INSTANCE.plan_multiple_circles_nomap(xCoords, yCoords, numCoords, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius, planningTimeInSecs, algo.ordinal())) return false;					
			}
			final Pointer pathVals = path.getValue();
			final PathPose valsRef = new PathPose(pathVals);
			valsRef.read();
			int numVals = pathLength.getValue();
			if (numVals == 0) return false;
			PathPose[] pathPoses = (PathPose[])valsRef.toArray(numVals);
			if (i == 0) finalPath.add(new PoseSteering(pathPoses[0].x, pathPoses[0].y, pathPoses[0].theta, 0.0));
			for (int j = 1; j < pathPoses.length; j++) finalPath.add(new PoseSteering(pathPoses[j].x, pathPoses[j].y, pathPoses[j].theta, 0.0));
			INSTANCE.cleanupPath(pathVals);
		}
		this.pathPS = finalPath.toArray(new PoseSteering[finalPath.size()]);
		return true;
	}

}
