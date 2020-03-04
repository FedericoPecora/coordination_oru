/*******************************************************************************
 * Copyright (c) 2010-2013 Federico Pecora <federico.pecora@oru.se>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************/
package temporalpropagation;

import java.util.logging.Level;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.SimpleDistanceConstraint;
import org.metacsp.utility.logging.MetaCSPLogging;


public class TestAPSPSolverSimple {
	
	public static void main(String[] args) throws InterruptedException {
				
		APSPSolver solver = new APSPSolver(100, 500);
		
		MetaCSPLogging.setLevel(solver.getClass(), Level.FINE);

		//solver.setOptions(framework.ConstraintSolver.OPTIONS.AUTO_PROPAGATE);
		Variable[] vars = solver.createVariables(2);
		Variable one = vars[0];
		Variable two = vars[1];

		ConstraintNetwork.draw(solver.getConstraintNetwork());

		SimpleDistanceConstraint con1 = new SimpleDistanceConstraint();
		con1.setFrom(one);
		con1.setTo(two);
		con1.setMinimum(-APSPSolver.INF);
		con1.setMaximum(-10);
//		con1.setMaximum(APSPSolver.INF);
//		con1.setMinimum(10);
		
		System.out.println("Adding constraint " + con1 + "...");
		System.out.println(solver.addConstraint(con1));
		
		Thread.sleep(3000);
		
		solver.removeConstraint(con1);
		
		
	}

}
