package se.oru.coordination.coordination_oru.util.splines;

import java.util.ArrayList;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

public class BezierSplineFactory {
	
	/**
	
	 * A cubic bezier method to calculate the point at t along the Bezier Curve give	
	 * the parameter points.
	 * @param p1
	 * @param p2
	 * @param p3
	 * @param p4
	 * @param t A value between 0 and 1, inclusive. 
	 * @return
	 */	
	public static Coordinate cubicBezier(Coordinate p1, Coordinate p2, Coordinate p3, Coordinate p4, double t){
		return new Coordinate(
				cubicBezierPoint(p1.x, p2.x, p3.x, p4.x, t), 
				cubicBezierPoint(p1.y, p2.y, p3.y, p4.y, t), 
				cubicBezierPoint(p1.z, p2.z, p3.z, p4.z, t));
	}
	
	/**
	 * A quadratic Bezier method to calculate the point at t along the Bezier Curve give
	 * the parameter points.
	 * @param p1
	 * @param p2
	 * @param p3
	 * @param t A value between 0 and 1, inclusive. 
	 * @return
	 */
	public static Coordinate quadBezier(Coordinate p1, Coordinate p2, Coordinate p3, double t){
		return new Coordinate(
				quadBezierPoint(p1.x, p2.x, p3.x, t), 
				quadBezierPoint(p1.y, p2.y, p3.y, t), 
				quadBezierPoint(p1.z, p2.z, p3.z, t));
	}
	
	/**
	 * The cubic Bezier equation. 
	 * @param a0
	 * @param a1
	 * @param a2
	 * @param a3
	 * @param t
	 * @return
	 */
	private static double cubicBezierPoint(double a0, double a1, double a2, double a3, double t){
		return Math.pow(1-t, 3) * a0 + 3* Math.pow(1-t, 2) * t * a1 + 3*(1-t) * Math.pow(t, 2) * a2 + Math.pow(t, 3) * a3;
	}
	
	/**
	 * The quadratic Bezier equation,
	 * @param a0
	 * @param a1
	 * @param a2
	 * @param t
	 * @return
	 */
	private static double quadBezierPoint(double a0, double a1, double a2, double t){
		return Math.pow(1-t, 2) * a0 + 2* (1-t) * t * a1 + Math.pow(t, 2) * a2;
	}
	
	public static Pose[] asPoses(Coordinate[] spline) {
		Pose[] poses = new Pose[spline.length];
		poses[0] = new Pose(spline[0].x, spline[0].y, Math.atan2(spline[1].y-spline[0].y, spline[1].x-spline[0].x));
		poses[spline.length-1] = new Pose(spline[spline.length-1].x, spline[spline.length-1].y, Math.atan2(spline[spline.length-1].y-spline[spline.length-2].y, spline[spline.length-1].x-spline[spline.length-2].x));
		for (int i = 1; i < spline.length-1; i++) {
			poses[i] = new Pose(spline[i].x,spline[i].y,Math.atan2(spline[i+1].y-spline[i].y, spline[i+1].x-spline[i].x));
		}
		return poses;
	}
	
	public static PoseSteering[] asPoseSteering(Coordinate[] spline) {
		Pose[] poses = asPoses(spline);
		PoseSteering[] ret = new PoseSteering[poses.length];
		for (int i = 0; i < ret.length; i++) ret[i] = new PoseSteering(poses[i].getX(),poses[i].getY(),poses[i].getTheta(),0.0);
		return ret;
	}
	
	public static Coordinate[] createBezierSpline(Coordinate p1, Coordinate p2, Coordinate p3, Coordinate p4, double dist) {
		ArrayList<Coordinate> ret = new ArrayList<Coordinate>();
		double t = 0.0;
		Coordinate prevCoord = p1;
		ret.add(prevCoord);
		do {
			Coordinate coord = cubicBezier(p1, p2, p3, p4, t);
			if (prevCoord.distance(coord) >= dist) {
				ret.add(coord);
				prevCoord = coord;
			}
			t += 0.01;
		}
		while (t < 1.0);
		if (ret.get(ret.size()-1).distance(p4) > dist/4.0) ret.add(p4);
		return ret.toArray(new Coordinate[ret.size()]);
	}

	public static void main(String[] args) {
		
		Coordinate[] coords = new Coordinate[] {
				new Coordinate(0.0,0.0),
				new Coordinate(1.0,1.0),
				new Coordinate(1.0,2.0),
				new Coordinate(3.0,0.0)
		};
				
		double t = 0.0;
		do {
			Coordinate coord = cubicBezier(coords[0], coords[1], coords[2], coords[3], t);
			System.out.println(coord);
			t += 0.1;
		}
		while (t < 1.0);
				
	}
	
}

