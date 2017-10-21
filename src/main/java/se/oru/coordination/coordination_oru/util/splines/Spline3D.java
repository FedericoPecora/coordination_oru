package se.oru.coordination.coordination_oru.util.splines;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

public class Spline3D {

	public static enum Type {SPLINE_BEZIER, SPLINE_CUBIC, SPLINE_CATMULL_ROM}

	private Coordinate[] coords = null;
	private Coordinate[] controlPoints = null;
	private Type type = null; 
	
	public Spline3D(double[] points, Coordinate[] controlPoints, Type type) {
		this.type = type;
		this.controlPoints = controlPoints;
		this.coords = new Coordinate[points.length/3];
		for (int i = 0; i < points.length; i+=3) this.coords[i/3] = new Coordinate(points[i], points[i+1], points[i+2]);
	}
	
	public Type getType() {
		return this.type;
	}
	
	public Coordinate[] getControlPoints() {
		return this.controlPoints;
	}
	
	public Coordinate[] getCoords() {
		return this.coords;
	}
	
	public Pose[] asPoses() {
		Pose[] poses = new Pose[coords.length];
		if (this.type.equals(Type.SPLINE_BEZIER)) {
			poses[0] = new Pose(controlPoints[0].x, controlPoints[0].y, Math.atan2(controlPoints[1].y-controlPoints[0].y, controlPoints[1].x-controlPoints[0].x));
			poses[coords.length-1] = new Pose(controlPoints[controlPoints.length-1].x, controlPoints[controlPoints.length-1].y, Math.atan2(controlPoints[controlPoints.length-1].y-controlPoints[controlPoints.length-2].y, controlPoints[controlPoints.length-1].x-controlPoints[controlPoints.length-2].x));
		}
		else {
			poses[0] = new Pose(coords[0].x, coords[0].y, Math.atan2(coords[1].y-coords[0].y, coords[1].x-coords[0].x));
			poses[coords.length-1] = new Pose(coords[coords.length-1].x, coords[coords.length-1].y, Math.atan2(coords[coords.length-1].y-coords[coords.length-2].y, coords[coords.length-1].x-coords[coords.length-2].x));
		}
		for (int i = 1; i < coords.length-1; i++) {
			poses[i] = new Pose(coords[i].x,coords[i].y,Math.atan2(coords[i+1].y-coords[i].y, coords[i+1].x-coords[i].x));
		}
		return poses;
	}

	public PoseSteering[] asPoseSteerings() {
		Pose[] poses = asPoses();
		PoseSteering[] pss = new PoseSteering[coords.length];
		for (int i = 0; i < poses.length; i++) pss[i] = new PoseSteering(poses[i].getX(),poses[i].getY(),poses[i].getTheta(),0.0);
		return pss;
	}

	public String toString() {
		String ret = "";
		for (Coordinate coord : coords) ret += (coord+"\n");
		return ret;
	}
	
	public double[] computeDistances() {
		double[] dist = new double[coords.length-1];
		for (int i = 0; i < coords.length-1; i++) dist[i] = coords[i].distance(coords[i+1]);
		return dist;
	}
	
	public double[] computeMinMaxDistances() {
		double[] ret = new double[] {Double.MAX_VALUE, Double.MIN_VALUE};
		double[] dist = computeDistances();
		for (int i = 0; i < dist.length; i++) {
			if (dist[i] < ret[0]) ret[0] = dist[i];
			if (dist[i] > ret[1]) ret[1] = dist[i];
		}
		return ret;
	}

}
