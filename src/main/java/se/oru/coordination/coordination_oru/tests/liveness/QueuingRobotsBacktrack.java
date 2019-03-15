package se.oru.coordination.coordination_oru.tests.liveness;

import java.util.Comparator;
import org.metacsp.multi.spatioTemporal.paths.Pose;
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

@DemoDescription(desc = "Example showing coordination in opposing directions. Deadlock is induced by choosing the same goal foe both the robots."
		+ "Using backtrack to recover from deadlock.")

public class QueuingRobotsBacktrack {
	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 2.0;
		double MAX_VEL = 3.0;
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
		tec.setYieldIfParking(true);
		tec.setUseInternalCriticalPoints(false);
		tec.setBreakDeadlocksByReordering(true);
		tec.setBreakDeadlocksByReplanning(false);
		tec.setBreakDeadlocksByBackTracking(true);
		tec.setCheckCollisions(true);
		
		Coordinate footprint1 = new Coordinate(-0.5,0.5);
		Coordinate footprint2 = new Coordinate(-0.5,-0.5);
		Coordinate footprint3 = new Coordinate(0.7,-0.5);
		Coordinate footprint4 = new Coordinate(0.7,0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//Setup the network parameters
		NetworkConfiguration.setDelays(0, 0);
		NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS = 0;
		tec.setNetworkParameters(NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS, NetworkConfiguration.getMaximumTxDelay(), 0.02);
		
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//viz.setSize(1800, 450);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(40, 0, 0);
		tec.setVisualization(viz);
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		double lenghtX = 10.0; //something strange happen with 5.0!!
		double displX = 5.0;
		double displY = 2.0;
		final int[] robotIDs = new int[] {1,2}; 
		Pose startsCenter = new Pose(25.0,7.0,0.0);
		Pose goalRobotsFinal = new Pose(startsCenter.getX()-lenghtX,startsCenter.getY(),startsCenter.getTheta());
		Pose goalRobot1 = new Pose(startsCenter.getX()-displX,startsCenter.getY(),startsCenter.getTheta());
		
		//Set up path planner (using empty map)
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(footprint1, footprint2, footprint3, footprint4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.1);
		
		for (int i = 0; i < robotIDs.length; i++) {
			tec.setForwardModel(robotIDs[i], new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getTrackingPeriod()));
			double y = -0.5*displY*(double)(robotIDs.length-1)+i*displY;
			Pose start = new Pose(startsCenter.getX(),startsCenter.getY()+y,startsCenter.getTheta());
			tec.placeRobot(robotIDs[i], start);
			rsp.setStart(start);
			rsp.setGoals(goalRobot1,goalRobotsFinal);
			rsp.plan();
			Missions.enqueueMission(new Mission(robotIDs[i],rsp.getPath()));
			Missions.enqueueMission(new Mission(robotIDs[i],rsp.getPathInv()));
		}

		System.out.println("Added missions " + Missions.getMissions());
		
		for (final int robotID : robotIDs ) {
			Thread t = new Thread() {
				@Override
				public void run() {
					boolean isFree = false;
					int sequenceNumber = 0;
					int totalIterations = 2;
					while (true && totalIterations > 0) {
						synchronized(tec.getSolver()) {
							if (tec.isFree(robotID)) isFree = true;
						}
						
						//Sleep for a little
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
						
						if (isFree) {
							isFree = false;
							synchronized(tec.getSolver()) {
								Mission m = Missions.getMission(robotID,sequenceNumber);
								tec.addMissions(m);
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
								sequenceNumber = (sequenceNumber+1)%Missions.getMissions(robotID).size();
								totalIterations--;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(100); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					System.out.println("Robot" + robotID + " is done!");
				}
			};
			//Start the thread!
			t.start();
	}
		
	}

}
