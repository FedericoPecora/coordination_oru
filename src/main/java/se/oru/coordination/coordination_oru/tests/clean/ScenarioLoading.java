package se.oru.coordination.coordination_oru.tests.clean;

import java.awt.image.BufferedImage;
import java.util.Comparator;
import java.util.Map.Entry;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

public class ScenarioLoading {

	public static void main(String[] args) {

		//Maximum acceleration/deceleration and speed for all robots
		double MAX_ACCEL = 5.0;
		double MAX_VEL = 15.0;
		
		//Create a coordinator with interfaces to robots in the built-in 2D simulator
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		
		//Provide a heuristic (here, closest to critical section goes first)
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getRobotReport();
				RobotReport robotReport2 = o2.getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});

		//Set up infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Start the thread that revises precedences at every period
		tec.startInference();
			
		//Avoid deadlocks via global re-ordering
		tec.setBreakDeadlocks(true, false, false);
		tec.setUseInternalCriticalPoints(false);

		//Load the scenario saved in "testS.zip"
		Missions.loadScenario("testS");

		//Define robot footprints
		double xl = 1.0;
		double yl = .5;
		Coordinate footprint1 = new Coordinate(-xl,yl);
		Coordinate footprint2 = new Coordinate(xl,yl);
		Coordinate footprint3 = new Coordinate(xl,-yl);
		Coordinate footprint4 = new Coordinate(-xl,-yl);
		
		//Place robots in their starting poses (initial poses of their first missions)
		//Also define the forward model and footprint of each robot
		for (Entry<Integer,Pose> e : Missions.getInitialPoses().entrySet()) {
			int robotID = e.getKey();
			tec.setFootprint(robotID, footprint1, footprint2, footprint3, footprint4);
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getRobotTrackingPeriodInMillis(robotID)));
			tec.placeRobot(robotID, e.getValue());
		}
		
		BrowserVisualization viz = new BrowserVisualization();
		if (Missions.getMap() != null) viz.setMap(Missions.getMap(), Missions.getMapResolution(), Missions.getMapOrigin());
		viz.setInitialTransform(20.0, 9.0, 2.0);
		tec.setVisualization(viz);

		Missions.startMissionDispatchers(tec, Missions.getIDsOfRobotsWithMissions());	
		
		//Missions.saveMap("fuzz");
		

	}

}