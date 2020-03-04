package temporalpropagation;

import java.util.Arrays;
import java.util.Calendar;
import java.util.Random;
import java.util.logging.Level;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.meta.TCSP.MostConstrainedFirstVarOH;
import org.metacsp.meta.TCSP.TCSPLabeling;
import org.metacsp.meta.TCSP.TCSPSolver;
import org.metacsp.meta.TCSP.WidestIntervalFirstValOH;
import org.metacsp.multi.TCSP.DistanceConstraint;
import org.metacsp.multi.TCSP.DistanceConstraintSolver;
import org.metacsp.multi.TCSP.MultiTimePoint;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.time.SimpleDistanceConstraint;
import org.metacsp.utility.logging.MetaCSPLogging;

public class TestTCSPSolver {
	
	public static Random rand = new Random(Calendar.getInstance().getTimeInMillis());
	
	public static long[] fakeHenrik(int robot1, int robot2, int criticalSection) {
		return new long[] {rand.nextInt(10), rand.nextInt(10)};
	}
	
	public static void main(String[] args) {

		TCSPSolver metaSolver = new TCSPSolver(0, 1000, 0);
		DistanceConstraintSolver groundSolver = (DistanceConstraintSolver)metaSolver.getConstraintSolvers()[0];		
		APSPSolver groundGroundSolver = (APSPSolver)groundSolver.getConstraintSolvers()[0];
		
		//MetaCSPLogging.setLevel(metaSolver.getClass(), Level.FINEST);
		
		/*                  5
		 *                -------
		 *      8     C1 /  17   \ C2   8
		 * R1 -->-------/---------\------------
		 *       ------/           \  7
		 *  4   /   7               \------<-- R2
		 * ----/---------<-- R3
		 *    / C3    8
		 *   /
		 *  /  4
		 *	
		 *
		 */
		
		//Variables:
		//  rX_init = robot RX at initial pose
		//  rX_goal = robot RX at goal
		//  rXcY_start = robot RX enters critical section CY
	    //  rXcY_end = robot RX exits critical section CY
		//  Durations of paths outside critical sections: written above
		//  Durations of traversal of critical sections: we assume 3 for all critical sections
		//  Delay that depends on who goes first thru a critical section: returned by fakeHenrik() function

		MultiTimePoint r1_init = (MultiTimePoint)groundSolver.createVariable("R1 starts");
		MultiTimePoint r1_goal = (MultiTimePoint)groundSolver.createVariable("R1 arrives at goal");
		MultiTimePoint r2_init = (MultiTimePoint)groundSolver.createVariable("R2 starts");
		MultiTimePoint r2_goal = (MultiTimePoint)groundSolver.createVariable("R2 arrives at goal");
		MultiTimePoint r3_init = (MultiTimePoint)groundSolver.createVariable("R3 starts");
		MultiTimePoint r3_goal = (MultiTimePoint)groundSolver.createVariable("R3 arrives at goal");

		MultiTimePoint r1c1_start = (MultiTimePoint)groundSolver.createVariable("R1 enters C1");
		MultiTimePoint r1c1_end = (MultiTimePoint)groundSolver.createVariable("R1 exits C1");
		MultiTimePoint r1c2_start = (MultiTimePoint)groundSolver.createVariable("R1 enters C2");
		MultiTimePoint r1c2_end = (MultiTimePoint)groundSolver.createVariable("R1 exits C2");
		MultiTimePoint r2c1_start = (MultiTimePoint)groundSolver.createVariable("R2 enters C1");
		MultiTimePoint r2c1_end = (MultiTimePoint)groundSolver.createVariable("R2 exits C1");
		MultiTimePoint r2c2_start = (MultiTimePoint)groundSolver.createVariable("R2 enters C2");
		MultiTimePoint r2c2_end = (MultiTimePoint)groundSolver.createVariable("R2 exits C2");
		MultiTimePoint r2c3_start = (MultiTimePoint)groundSolver.createVariable("R2 enters C3");
		MultiTimePoint r2c3_end = (MultiTimePoint)groundSolver.createVariable("R2 exits C3");
		MultiTimePoint r3c3_start = (MultiTimePoint)groundSolver.createVariable("R3 enters C3");
		MultiTimePoint r3c3_end = (MultiTimePoint)groundSolver.createVariable("R3 exits C3");

//		ConstraintNetwork.draw(groundSolver.getConstraintNetwork(), "TCSP");
//		ConstraintNetwork.draw(groundGroundSolver.getConstraintNetwork(), "STP");
				
		//Robot 1
		DistanceConstraint duration_r1_init_c1 = new DistanceConstraint(new Bounds(8,APSPSolver.INF));
		duration_r1_init_c1.setFrom(r1_init);
		duration_r1_init_c1.setTo(r1c1_start);

		DistanceConstraint duration_r1_c1 = new DistanceConstraint(new Bounds(3,APSPSolver.INF));
		duration_r1_c1.setFrom(r1c1_start);
		duration_r1_c1.setTo(r1c1_end);

		DistanceConstraint duration_r1_c1_c2 = new DistanceConstraint(new Bounds(17,APSPSolver.INF));
		duration_r1_c1_c2.setFrom(r1c1_end);
		duration_r1_c1_c2.setTo(r1c2_start);
		
		DistanceConstraint duration_r1_c2 = new DistanceConstraint(new Bounds(3,APSPSolver.INF));
		duration_r1_c2.setFrom(r1c2_start);
		duration_r1_c2.setTo(r1c2_end);
		
		DistanceConstraint duration_r1_c2_goal = new DistanceConstraint(new Bounds(8,APSPSolver.INF));
		duration_r1_c2_goal.setFrom(r1c2_end);
		duration_r1_c2_goal.setTo(r1_goal);

		//Robot 2
		DistanceConstraint duration_r2_init_c2 = new DistanceConstraint(new Bounds(7,APSPSolver.INF));
		duration_r2_init_c2.setFrom(r2_init);
		duration_r2_init_c2.setTo(r2c2_start);

		DistanceConstraint duration_r2_c2 = new DistanceConstraint(new Bounds(3,APSPSolver.INF));
		duration_r2_c2.setFrom(r2c2_start);
		duration_r2_c2.setTo(r2c2_end);

		DistanceConstraint duration_r2_c2_c1 = new DistanceConstraint(new Bounds(5,APSPSolver.INF));
		duration_r2_c2_c1.setFrom(r2c2_end);
		duration_r2_c2_c1.setTo(r2c1_start);
		
		DistanceConstraint duration_r2_c1 = new DistanceConstraint(new Bounds(3,APSPSolver.INF));
		duration_r2_c1.setFrom(r2c1_start);
		duration_r2_c1.setTo(r2c1_end);
		
		DistanceConstraint duration_r2_c1_c3 = new DistanceConstraint(new Bounds(7,APSPSolver.INF));
		duration_r2_c1_c3.setFrom(r2c1_end);
		duration_r2_c1_c3.setTo(r2c3_start);

		DistanceConstraint duration_r2_c3 = new DistanceConstraint(new Bounds(3,APSPSolver.INF));
		duration_r2_c3.setFrom(r2c3_start);
		duration_r2_c3.setTo(r2c3_end);
		
		DistanceConstraint duration_r2_c3_goal = new DistanceConstraint(new Bounds(4,APSPSolver.INF));
		duration_r2_c3_goal.setFrom(r2c3_end);
		duration_r2_c3_goal.setTo(r2_goal);

		//Robot 3
		DistanceConstraint duration_r3_init_c3 = new DistanceConstraint(new Bounds(8,APSPSolver.INF));
		duration_r3_init_c3.setFrom(r3_init);
		duration_r3_init_c3.setTo(r3c3_start);
		
		DistanceConstraint duration_r3_c3 = new DistanceConstraint(new Bounds(3, APSPSolver.INF));
		duration_r3_c3.setFrom(r3c3_start);
		duration_r3_c3.setTo(r3c3_end);
				
		DistanceConstraint duration_r3_c3_goal = new DistanceConstraint(new Bounds(4,APSPSolver.INF));
		duration_r3_c3_goal.setFrom(r3c3_end);
		duration_r3_c3_goal.setTo(r3_goal);
		
		// Possible orderings of robots through critical sections (with delays provided by Henrik's function)
		//
		// t1 before t2 iff t1 --[d1,INF)--> t2, which means
		//                  t2-t1 >= d1 and t2-t1 < INF
		//
		// OR:
		//
		// t2 before t1 iff t2 --[d2,INF)--> t1, which means
		//                  t1-t2 >= d2  and t1-t2 < INF, hence
		//                  t2-t1 > -INF and t2-t1 <= -d, hence
		//                  t1 --(-INF,-d2]--> t2
		//
		// So, overall, a disjunctive constraint:
		//
		//           t1 --[d1,INF) v (-INF,-d2]--> t2
		
		
		// Robots R1 and R2 access critical section C1
		long[] delays = fakeHenrik(1,2,1);
		DistanceConstraint r1_r2_c1 = new DistanceConstraint(new Bounds(delays[0],APSPSolver.INF),new Bounds(-APSPSolver.INF,-delays[1]));
		r1_r2_c1.setFrom(r1c1_start);
		r1_r2_c1.setTo(r2c1_start);

		// Robots R1 and R2 access critical section C2
		delays = fakeHenrik(1,2,2);
		DistanceConstraint r1_r2_c2 = new DistanceConstraint(new Bounds(delays[0],APSPSolver.INF), new Bounds(-APSPSolver.INF,-delays[1]));
		r1_r2_c2.setFrom(r1c2_start);
		r1_r2_c2.setTo(r2c2_start);

		// Robots R2 and R3 access critical section C3
		delays = fakeHenrik(2,3,3);
		DistanceConstraint r2_r3_c3 = new DistanceConstraint(new Bounds(delays[0],APSPSolver.INF), new Bounds(-APSPSolver.INF,-delays[1]));
		r2_r3_c3.setFrom(r2c3_start);
		r2_r3_c3.setTo(r3c3_start);

		//Add all the constraints
		groundSolver.addConstraints(
				duration_r1_init_c1,
				duration_r1_c1,
				duration_r1_c1_c2,
				duration_r1_c2,
				duration_r1_c2_goal,
				duration_r2_init_c2,
				duration_r2_c2,
				duration_r2_c2_c1,
				duration_r2_c1,
				duration_r2_c1_c3,
				duration_r2_c3,
				duration_r2_c3_goal,
				duration_r3_init_c3,
				duration_r3_c3,
				duration_r3_c3_goal,
				r1_r2_c1,
				r1_r2_c2,
				r2_r3_c3
			);
		
//		VariableOrderingH varOH = new MostConstrainedFirstVarOH();
//		ValueOrderingH valOH = new WidestIntervalFirstValOH();

//		ValueOrderingH valOH = new ValueOrderingH() {
//			@Override
//			public int compare(ConstraintNetwork o1, ConstraintNetwork o2) {
//				return 1-rand.nextInt(3);
//			}
//		};		

		ValueOrderingH valOH = new ValueOrderingH() {
			@Override
			public int compare(ConstraintNetwork o1, ConstraintNetwork o2) {
				Bounds b1 = ((DistanceConstraint)o1.getConstraints()[0]).getBounds()[0];
				Bounds b2 = ((DistanceConstraint)o2.getConstraints()[0]).getBounds()[0];
				long d1 = b1.min < 0 ? -b1.max : b1.min;
				long d2 = b2.min < 0 ? -b2.max : b2.min;
				return (int)(d1-d2);
			}
		};		

		TCSPLabeling metaCons = new TCSPLabeling(null,valOH);
		metaSolver.addMetaConstraint(metaCons);

		ConstraintNetwork[] resolvers = new ConstraintNetwork[0];
		long max = -APSPSolver.INF;
		
		while (metaSolver.backtrack()) {
			resolvers = metaSolver.getAddedResolvers();
			
			for (Variable var : groundSolver.getVariables()) {
				MultiTimePoint tp = (MultiTimePoint)var;
//				System.out.println(var.getComponent() + ": " + var.getDomain());
				//System.out.println(var.getComponent() + ": " + tp.getLowerBound());
				if (tp.getComponent() != null && max < tp.getLowerBound()) max = tp.getLowerBound();				
			}

			System.out.println("\nFound solution with makespan " + max);
			
			System.out.println("Ordering decisions made:");
			for (ConstraintNetwork cn : resolvers) {
				DistanceConstraint dc = (DistanceConstraint)cn.getConstraints()[0];
				System.out.println(dc.getFrom().getComponent() + (dc.getBounds()[0].min < 0 ? " after " : " before ") + dc.getTo().getComponent());
			}
			
			metaSolver.retractResolvers();

			max--;
			DistanceConstraint maxR1 = new DistanceConstraint(new Bounds(0,max));
			maxR1.setFrom(groundSolver.getSource());
			maxR1.setTo(r1_goal);
			DistanceConstraint maxR2 = new DistanceConstraint(new Bounds(0,max));
			maxR2.setFrom(groundSolver.getSource());
			maxR2.setTo(r2_goal);
			DistanceConstraint maxR3 = new DistanceConstraint(new Bounds(0,max));
			maxR3.setFrom(groundSolver.getSource());
			maxR3.setTo(r3_goal);
			
			if (!groundSolver.addConstraints(maxR1,maxR2,maxR3)) break;
		}

	}

}
