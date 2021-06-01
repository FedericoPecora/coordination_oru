package se.oru.coordination.coordination_oru.tests.clean;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Random;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination of robots along sine wave paths (obtained without the ReedsSheppCarPlanner) in opposing directions.")
public class Waves {
	
	private static PoseSteering[] getSinePath(double period, double magnitude, Pose from, Pose to) {
		if (from.getY() != to.getY()) throw new Error("Can only do straight sine waves ;)");
		ArrayList<Coordinate> coords = new ArrayList<Coordinate>();
		double delta = 0.1;
		double t = 0.0;
		while (t < to.getX()-from.getX()) {
			double value = magnitude*Math.sin(((2*Math.PI)/period)*t);
			Coordinate coord = new Coordinate(t+from.getX(),value+from.getY());
			coords.add(coord);
			t += delta;
		}
		PoseSteering[] pss = new PoseSteering[coords.size()];
		pss[0] = new PoseSteering(coords.get(0).x, coords.get(0).y, Math.atan2(coords.get(1).y-coords.get(0).y, coords.get(1).x-coords.get(0).x),0.0);
		pss[coords.size()-1] = new PoseSteering(coords.get(coords.size()-1).x, coords.get(coords.size()-1).y, Math.atan2(coords.get(coords.size()-1).y-coords.get(coords.size()-2).y, coords.get(coords.size()-1).x-coords.get(coords.size()-2).x),0.0);
		for (int i = 1; i < coords.size()-1; i++) {
			pss[i] = new PoseSteering(coords.get(i).x,coords.get(i).y,Math.atan2(coords.get(i+1).y-coords.get(i).y, coords.get(i+1).x-coords.get(i).x),0.0);
		}		
		return pss;
	}
	
	private static PoseSteering[] invertPath(PoseSteering[] path) {
		PoseSteering[] ret = new PoseSteering[path.length];
		for (int i = 0; i < path.length; i++) {
			ret[i] = path[path.length-i-1];
		}
		return ret;
	}
	
	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 3.0;
		double MAX_VEL = 10;
		final int numRobots = (args != null && args.length > 0) ? Integer.parseInt(args[0]) : 10;
		
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(500, 1000, MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return(o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});
		
		// Prob. of packet loss, min, and max delays
		tec.setNetworkParameters(0, 0, 0);

		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(0.5,0.5);
		Coordinate footprint3 = new Coordinate(0.5,-0.5);
		Coordinate footprint4 = new Coordinate(-0.5,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);
		tec.setBreakDeadlocks(false, true, false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		double deltaY = 2.0; //1.7
		
		final int[] robotIDs = new int[numRobots];
		for (int i = 0; i < numRobots; i++) robotIDs[i] = i+1;
		
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(31, 12, 6); //viz.setInitialTransform(20.0, 45, 20);
		tec.setVisualization(viz);
		
		for (int index = 0; index < robotIDs.length; index++) {
			int robotID = robotIDs[index];
			double period = 18; //10
			double mag = deltaY; //2*deltaY

			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));
			Pose from = new Pose(0.0,index*deltaY,0.0);
			Pose to = new Pose(2*period,index*deltaY,0.0);
			if (index%2 != 0) mag*=(-1);
			PoseSteering[] robotPath = getSinePath(period, mag, from, to);
			PoseSteering[] robotPathInv = invertPath(robotPath);
			if (index%2 != 0) {
				tec.placeRobot(robotID, to);
				Missions.enqueueMission(new Mission(robotID, robotPathInv));
				Missions.enqueueMission(new Mission(robotID, robotPath));				
			}
			else {
				tec.placeRobot(robotID, from);
				Missions.enqueueMission(new Mission(robotID, robotPath));
				Missions.enqueueMission(new Mission(robotID, robotPathInv));				
			}
		}

		System.out.println("Added missions " + Missions.getMissions());
		
		Missions.startMissionDispatchers(tec, robotIDs);
	}

}
