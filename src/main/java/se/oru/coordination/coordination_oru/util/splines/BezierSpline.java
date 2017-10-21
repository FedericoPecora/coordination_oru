package se.oru.coordination.coordination_oru.util.splines;

/*
 * (C) 2004 - Geotechnical Software Services
 * 
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
 * A Bezier spline object. Use the SplineFactory class to create
 * splines of this type.
 * 
 * @author <a href="mailto:jacob.dreyer@geosoft.no">Jacob Dreyer</a>
 */   
public class BezierSpline extends Spline {
	/**
	 * Construct a bezier spline. Package local; Use the SplineFactory
	 * to create splines of this type. The control points are used according
	 * to the definition of Bezier splines.
	 * 
	 * @param controlPoints  Control points of spline (x0,y0,z0,x1,y1,z1,...)
	 * @param nParts         Number of parts in generated spline.
	 */
	public BezierSpline (double controlPoints[], int nParts) {
		controlPoints_ = controlPoints;
		nParts_ = nParts;
	}



	/**
	 * Generate this spline.
	 * 
	 * @return  Coordinates of the spline (x0,y0,z0,x1,y1,z1,...)
	 */
	public double[] generate() {
		if (controlPoints_.length < 9) {
			double[] copy = new double[controlPoints_.length];
			System.arraycopy (controlPoints_, 0, copy, 0, controlPoints_.length);
			return copy;
		}

		int n = controlPoints_.length / 3;
		int length = (n - 3) * nParts_ + 1;
		
		double spline[] = new double[length * 3];

		p (0, 0, controlPoints_, spline, 0);

		int index = 3;
		for (int i = 0; i < n - 3; i += 3) {
			for (int j = 1; j <= nParts_; j++) {
				p (i, j / (double) nParts_, controlPoints_, spline, index);
				index += 3;
			}
		}

		return spline;      
	}



	private void p (int i, double t, double cp[], double spline[], int index) {
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;

		int k = i;
		for (int j = 0; j <= 3; j++) {
			double b = blend (j, t);

			x += b * cp[k++];
			y += b * cp[k++];
			z += b * cp[k++];
		}

		spline[index + 0] = x;
		spline[index + 1] = y;
		spline[index + 2] = z;    
	}



	private double blend (int i, double t) {
		if      (i == 0) return (1 - t) * (1 - t) * (1 - t);
		else if (i == 1) return 3 * t * (1 - t) * (1 - t);
		else if (i == 2) return 3 * t * t * (1 - t);
		else             return t * t * t;
	}
}
