package se.oru.coordination.coordination_oru.util.splines;
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
 * A Catmull-Rom spline object. Use the SplineFactory class to create
 * splines of this type.
 * 
 * @author <a href="mailto:jacob.dreyer@geosoft.no">Jacob Dreyer</a>
 */   
public class CatmullRomSpline extends CubicSpline {

	/**
	 * Construct a Catmull-Rom spline. Package local; Use the SplineFactory
	 * to create splines of this type. The control points are used according
	 * to the definition of Catmull-Rom splines.
	 * 
	 * @param controlPoints  Control points of spline (x0,y0,z0,x1,y1,z1,...)
	 * @param nParts         Number of parts in generated spline.
	 */
	public CatmullRomSpline (double controlPoints[], int nParts) {
		super (controlPoints, nParts);
	}



	protected void initialize (double controlPoints[], int nParts) {
		nParts_ = nParts;

		// Endpoints are added twice to force in the generated array
		controlPoints_ = new double[controlPoints.length + 6];
		System.arraycopy (controlPoints, 0, controlPoints_, 3,
				controlPoints.length);

		controlPoints_[0] = controlPoints_[3];
		controlPoints_[1] = controlPoints_[4];
		controlPoints_[2] = controlPoints_[5];

		controlPoints_[controlPoints_.length - 3] = controlPoints_[controlPoints_.length - 6];
		controlPoints_[controlPoints_.length - 2] = controlPoints_[controlPoints_.length - 5];
		controlPoints_[controlPoints_.length - 1] = controlPoints_[controlPoints_.length - 4];
	}



	protected double blend (int i, double t) {
		if      (i == -2) return ((-t + 2) * t - 1) * t / 2;
		else if (i == -1) return (((3 * t - 5) * t) * t + 2) / 2;
		else if (i ==  0) return ((-3 * t + 4) * t + 1) * t / 2;
		else              return ((t - 1) * t * t) / 2;
	}
}