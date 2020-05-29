package se.oru.coordination.coordination_oru.tests;

import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "One-shot navigation of 3 robots coordinating on static paths that overlap in a straight portion.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner18 {
	
	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
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
				return (o2.getRobotReport().getRobotID()-o1.getRobotReport().getRobotID());
			}
		});

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		//Start the thread that checks and enforces dependencies at every clock tick
		tec.startInference();
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		tec.setVisualization(viz);
		
		tec.setUseInternalCriticalPoints(false);
		
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		PoseSteering[] path1 = Missions.loadPathFromFile("paths/debug1.path");
		PoseSteering[] path2 = Missions.loadPathFromFile("paths/debug2.path");
		PoseSteering[] path3 = Missions.loadPathFromFile("paths/debug3.path");
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, path1[0].getPose());
		tec.placeRobot(2, path2[0].getPose());
		tec.placeRobot(3, path3[0].getPose());

		Mission m1 = new Mission(1,path1);
		Mission m2 = new Mission(2,path2);
		Mission m3 = new Mission(3,path3);
		
		Missions.startMissionDispatchers(tec, false, 1);
		
		Thread.sleep(2000);
		
		Missions.startMissionDispatchers(tec, false, 2);
		
		Thread.sleep(2000);
		
		Missions.startMissionDispatchers(tec, false, 3);
		
	}
	
}
