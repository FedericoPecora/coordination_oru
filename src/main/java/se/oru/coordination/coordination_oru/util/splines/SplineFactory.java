package se.oru.coordination.coordination_oru.util.splines;
import java.util.Arrays;

import com.vividsolutions.jts.geom.Coordinate;

/*
 * This code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this program; if not, write to the Free 
 * Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, 
 * MA  02111-1307, USA.
 */
//package no.geosoft.cc.geometry.spline;



/**
 * A spline factory instance.
 * 
 * @author <a href="mailto:jacob.dreyer@geosoft.no">Jacob Dreyer</a>
 */   
public class SplineFactory {

	/**
	 * Create a Bezier spline based on the given control points.
	 * The generated curve starts in the first control point and ends
	 * in the last control point.
	 * 
	 * @param controlPoints Control points of spline.
	 * @param maxDist Maximum distance between points along the spline.
	 * @return a Bezier spline based on the given control points.
	 */
	public static Spline3D createBezier (Coordinate[] controlPoints, double maxDist) {
		double[] cPoints = new double[controlPoints.length*3];
		int counter = 0;
		for (int i = 0; i < controlPoints.length; i++) {
			cPoints[counter] = controlPoints[i].x;
			cPoints[counter+1] = controlPoints[i].y;
			cPoints[counter+2] = controlPoints[i].z;
			counter+=3;
		}
		int numParts = 3;
		Spline3D ret = null;
		do {
			ret = new Spline3D(new BezierSpline (cPoints, numParts++).generate(), controlPoints, Spline3D.Type.SPLINE_BEZIER);
		}
		while(ret.computeMinMaxDistances()[1] > maxDist);
		return ret;
	}

	/**
	 * Create a Bezier spline based on the given control points.
	 * The generated curve starts in the first control point and ends
	 * in the last control point.
	 * 
	 * @param controlPoints Control points of spline.
	 * @param nParts Number of parts to divide each leg into.
	 * @return A Bezier spline based on the given control points.
	 */	
	public static Spline3D createBezier (Coordinate[] controlPoints, int nParts) {
		double[] cPoints = new double[controlPoints.length*3];
		int counter = 0;
		for (int i = 0; i < controlPoints.length; i++) {
			cPoints[counter] = controlPoints[i].x;
			cPoints[counter+1] = controlPoints[i].y;
			cPoints[counter+2] = controlPoints[i].z;
			counter+=3;
		}
		return new Spline3D(new BezierSpline (cPoints, nParts).generate(), controlPoints, Spline3D.Type.SPLINE_BEZIER);
	}



	/**
	 * Create a cubic spline based on the given control points.
	 * The generated curve starts in the first control point and ends
	 * in the last control point.
	 * 
	 * @param controlPoints Control points of spline.
	 * @param maxDist Maximum distance between points along the spline.
	 * @return A cubic spline based on the given control points.
	 */
	public static Spline3D createCubic (Coordinate[] controlPoints, double maxDist) {
		double[] cPoints = new double[controlPoints.length*3];
		int counter = 0;
		for (int i = 0; i < controlPoints.length; i++) {
			cPoints[counter] = controlPoints[i].x;
			cPoints[counter+1] = controlPoints[i].y;
			cPoints[counter+2] = controlPoints[i].z;
			counter+=3;
		}
		int numParts = 3;
		Spline3D ret = null;
		do {
			ret = new Spline3D(new CubicSpline (cPoints, numParts++).generate(), controlPoints, Spline3D.Type.SPLINE_CUBIC);
		}
		while(ret.computeMinMaxDistances()[1] > maxDist);
		return ret;

	}

	/**
	 * Create a cubic spline based on the given control points.
	 * The generated curve starts in the first control point and ends
	 * in the last control point.
	 * 
	 * @param controlPoints Control points of spline.
	 * @param nParts Number of parts to divide each leg into.
	 * @return A cubic spline based on the given control points.
	 */
	public static Spline3D createCubic (Coordinate[] controlPoints, int nParts) {
		double[] cPoints = new double[controlPoints.length*3];
		int counter = 0;
		for (int i = 0; i < controlPoints.length; i++) {
			cPoints[counter] = controlPoints[i].x;
			cPoints[counter+1] = controlPoints[i].y;
			cPoints[counter+2] = controlPoints[i].z;
			counter+=3;
		}
		return new Spline3D(new CubicSpline (cPoints, nParts++).generate(), controlPoints, Spline3D.Type.SPLINE_CUBIC);
	}


	/**
	 * Create a Catmull-Rom spline based on the given control points.
	 * The generated curve starts in the first control point and ends
	 * in the last control point.
	 * In addition, the curve intersects all the control points.
	 * 
	 * @param controlPoints  Control points of spline.
	 * @param maxDist Maximum distance between points along the spline.
	 * @return A Catmull-Rom spline based on the given control points.
	 */
	public static Spline3D createCatmullRom (Coordinate[] controlPoints, double maxDist) {
		double[] cPoints = new double[controlPoints.length*3];
		int counter = 0;
		for (int i = 0; i < controlPoints.length; i++) {
			cPoints[counter] = controlPoints[i].x;
			cPoints[counter+1] = controlPoints[i].y;
			cPoints[counter+2] = controlPoints[i].z;
			counter+=3;
		}
		int numParts = 3;
		Spline3D ret = null;
		do {
			ret = new Spline3D(new CatmullRomSpline (cPoints, numParts++).generate(), controlPoints, Spline3D.Type.SPLINE_CATMULL_ROM);
		}
		while(ret.computeMinMaxDistances()[1] > maxDist);
		return ret;

	}

	/**
	 * Create a Catmull-Rom spline based on the given control points.
	 * The generated curve starts in the first control point and ends
	 * in the last control point.
	 * In addition, the curve intersects all the control points.
	 * 
	 * @param controlPoints  Control points of spline.
	 * @param nParts Number of parts to divide each leg into.
	 * @return A Catmull-Rom spline based on the given control points.
	 */
	public static Spline3D createCatmullRom (Coordinate[] controlPoints, int nParts) {
		double[] cPoints = new double[controlPoints.length*3];
		int counter = 0;
		for (int i = 0; i < controlPoints.length; i++) {
			cPoints[counter] = controlPoints[i].x;
			cPoints[counter+1] = controlPoints[i].y;
			cPoints[counter+2] = controlPoints[i].z;
			counter+=3;
		}
		return new Spline3D(new CatmullRomSpline (cPoints, nParts++).generate(), controlPoints, Spline3D.Type.SPLINE_CATMULL_ROM);
	}


	public static void main (String[] args) {
		Coordinate[] c = new Coordinate[] {
				new Coordinate(0.0,0.0,0.0),
				new Coordinate(1.0,1.0,0.0),
				new Coordinate(2.0,-1.0,0.0),
				new Coordinate(10.0,0.0,0.0)
		};
		
		Spline3D spline1 = SplineFactory.createBezier (c, 0.5);
		Spline3D spline2 = SplineFactory.createCubic (c, 0.5);
		Spline3D spline3 = SplineFactory.createCatmullRom (c, 0.5);        

		System.out.println ("-- Bezier (length = "+spline1.getCoords().length+", min/max dist = "+Arrays.toString(spline1.computeMinMaxDistances())+")\n" + Arrays.toString(spline1.asPoses()));

		System.out.println ("-- Cubic (length = "+spline2.getCoords().length+", min/max dist = "+Arrays.toString(spline2.computeMinMaxDistances())+")\n" + Arrays.toString(spline2.asPoses()));

		System.out.println ("-- Catmull-Rom (length = "+spline3.getCoords().length+", min/max dist = "+Arrays.toString(spline3.computeMinMaxDistances())+")\n" + Arrays.toString(spline3.asPoses()));
	}
}

















