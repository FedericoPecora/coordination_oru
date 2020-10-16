package se.oru.coordination.coordination_oru.simulation2D;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.util.ColorPrint;

public class TrajectoryEnvelopeTrackerPedestrian extends AbstractTrajectoryEnvelopeTracker implements Runnable {

	protected static final long WAIT_AMOUNT_AT_END = 3000;
	protected static final double EPSILON = 0.01;

	protected double totalDistance = 0.0;

	protected double elapsedTrackingTime = 0.0;

	protected Pose currentPose;
	protected int currentPathIndex = 0;
	protected double currentSpeed;

	protected double stoppageTime = 0.0;
	protected int stops = 0;
	protected double positionToStop = -1.0;

	protected PedestrianTrajectory pedestrianTraj;

	private Thread th = null;
	private int maxDelayInMilis = 0;
	private Random rand = new Random();

	public TrajectoryEnvelopeTrackerPedestrian(TrajectoryEnvelope te, int timeStep, double temporalResolution, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb, PedestrianTrajectory pedestrianTraj) {
		super(te, temporalResolution, tec, timeStep, cb);
		this.pedestrianTraj = pedestrianTraj;
		this.currentPose = pedestrianTraj.getPoses().get(0);
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.positionToStop = this.totalDistance;
		this.th = new Thread(this, "Pedestrian tracker " + te.getComponent());
		this.th.setPriority(Thread.MAX_PRIORITY);
	}

	@Override
	public void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te) {
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
	}

	@Override
	public void startTracking() {
		while (this.th == null) {
			try { Thread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		this.th.start();
	}

	public static double computeDistance(Trajectory traj, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < endIndex; i++) {
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	public double computeCurrentDistanceFromStart() {
		double ret = 0.0;
		for (int i = 0; i < currentPathIndex; i++) {
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	public double computeDistance(int startIndex, int endIndex) {
		if(startIndex == -1) startIndex = 0;
		return computeDistance(this.traj, startIndex, endIndex);
	}

	@Override
	public void setCriticalPoint(int criticalPointToSet) {
		metaCSPLogger.warning("%%%%%%%% setCriticalPoint called for Pedestrian %%%%%%%%");
		if (this.criticalPoint != criticalPointToSet) {

			//A new intermediate index to stop at has been given
			if (criticalPointToSet != -1 && criticalPointToSet > getRobotReport().getPathIndex()) {
				//Store backups in case we are too late for critical point
				double totalDistanceBKP = this.totalDistance;
				int criticalPointBKP = this.criticalPoint;
				double positionToStopBKP = this.positionToStop;

				this.criticalPoint = criticalPointToSet;
				this.positionToStop = computeDistance(0, criticalPointToSet);

				if (this.positionToStop < computeCurrentDistanceFromStart()) {
					metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because stop distance (" + this.positionToStop +") < current distance (" + this.computeCurrentDistanceFromStart() + ")");
					this.criticalPoint = criticalPointBKP;
					this.totalDistance = totalDistanceBKP;
					this.positionToStop = positionToStopBKP;
				}
				else {
					metaCSPLogger.warning("Set critical point (" + te.getComponent() + "): " + criticalPointToSet + ", currently at point " + this.getRobotReport().getPathIndex() + ", distance " + this.computeCurrentDistanceFromStart() + ", will slow down at distance " + this.positionToStop);
				}
			}

			//Critical point <= current position, ignore -- WHY??
			else if (criticalPointToSet != -1 && criticalPointToSet <= getRobotReport().getPathIndex()) {
				metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because robot is already at " + getRobotReport().getPathIndex() + " (and current CP is " + this.criticalPoint + ")");
			}

			//The critical point has been reset, go to the end
			else if (criticalPointToSet == -1) {
				this.criticalPoint = criticalPointToSet;
				this.positionToStop = computeDistance(0, traj.getPose().length-1);
				metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet);
			}
		}

		//Same critical point was already set
		else {
			metaCSPLogger.warning("Critical point (" + te.getComponent() + ") " + criticalPointToSet + " was already set!");
		}

	}

	@Override
	public RobotReport getRobotReport() {
		if (this.currentPose == null) return null;
		if (!this.th.isAlive()) return new RobotReport(te.getRobotID(), traj.getPose()[0], -1, 0.0, 0.0, -1);
		return new RobotReport(te.getRobotID(), this.currentPose, this.currentPathIndex, this.currentSpeed, this.computeCurrentDistanceFromStart(), this.criticalPoint);
	}

	@Override
	public double getStoppageTime() {
		return stoppageTime;
	}

	@Override
	public int getStops() { return stops; }

	public void delayIntegrationThread(int maxDelayInmillis) {
		this.maxDelayInMilis = maxDelayInmillis;
	}

	protected void updatePedestrianState(boolean stopping) {

		// Time that has passed for the pedestrian is the tracking time that has elapsed minus the stoppageTime.
		double timePassed = this.elapsedTrackingTime - this.stoppageTime;

		if(timePassed < this.pedestrianTraj.getTimeStamp(0) || stopping) {
			currentSpeed = 0.0;
			return;
		}

		int index = 0;
		while(timePassed > this.pedestrianTraj.getTimeStamp(index)) {
			index++;
			if(index == pedestrianTraj.size()) break;
		}

		if(index != 0) {
			index = index - 1;
		}

		this.currentPathIndex = index;
		this.currentSpeed = this.pedestrianTraj.getSpeed(index);

		// Motion is over.
		if(index == this.pedestrianTraj.size() - 1) {
			this.currentPose = this.pedestrianTraj.getPose(index);
			return;
		}

		// Interpolate the pose. TODO: Make use of the velocities in each direction as well

		double diffTime = timePassed - this.pedestrianTraj.getTimeStamp(index);
		double deltaTime = this.pedestrianTraj.getTimeStamp(index + 1) - this.pedestrianTraj.getTimeStamp(index);
		double ratio = diffTime / deltaTime;
		this.currentPose = this.pedestrianTraj.getPose(index).interpolate(this.pedestrianTraj.getPose(index + 1), ratio);
	}

	@Override
	public void run() {
		this.elapsedTrackingTime = this.pedestrianTraj.getTimeStamp(0);
		double deltaTime = 0.0;
		boolean atCP = false;
		int myRobotID = te.getRobotID();
		int myTEID = te.getID();

		ColorPrint.positive("Pedestrian " + this.getRobotReport().getRobotID() + " is being tracked...");

		while (true) {

			//End condition: passed the middle AND velocity < 0 AND no criticalPoint
			boolean skipIntegration = false;

			if (computeCurrentDistanceFromStart() >= this.positionToStop && this.currentSpeed == 0.0) {
				if (criticalPoint == -1 && !atCP) {
					//set state to final position, just in case it didn't quite get there (it's certainly close enough)
					this.currentPose = this.pedestrianTraj.getPoses().get(this.pedestrianTraj.getPoses().size()-1);
					onPositionUpdate();
					break;
				}

				//Vel < 0 hence we are at CP, thus we need to skip integration
				if (!atCP /*&& getRobotReport().getPathIndex() == criticalPoint*/) {
					metaCSPLogger.info("At critical point (" + te.getComponent() + "): " + criticalPoint + " (" + getRobotReport().getPathIndex() + ")");
					atCP = true;
				}
				skipIntegration = true;
			}

			//Compute deltaTime
			long timeStart = Calendar.getInstance().getTimeInMillis();

			//Update the robot's state via RK4 numerical integration
			if (!skipIntegration) {
				if (atCP) {
					metaCSPLogger.info("Resuming from critical point (" + te.getComponent() + ")");
					stops = stops + 1;
					atCP = false;
				}
				boolean stopping = false;
				if(computeCurrentDistanceFromStart() >= this.positionToStop) {stopping = true;}
				updatePedestrianState(stopping);
			}

			//Do some user function on position update
			onPositionUpdate();

			//Sleep for tracking period
			int delay = trackingPeriodInMillis;
			if (maxDelayInMilis > 0) delay += rand.nextInt(maxDelayInMilis);
			try { Thread.sleep(delay); }
			catch (InterruptedException e) { e.printStackTrace(); }

			//Advance time to reflect how much we have slept (~ trackingPeriod)
			long deltaTimeInMillis = Calendar.getInstance().getTimeInMillis()-timeStart;
			deltaTime = deltaTimeInMillis/this.temporalResolution;
			this.elapsedTrackingTime += deltaTime;
			// If we have skipped integration, add this to stoppage time.
			if(skipIntegration)	this.stoppageTime += deltaTime;
		}

		//persevere with last path point in case listeners didn't catch it!
		long timerStart = getCurrentTimeInMillis();
		while (getCurrentTimeInMillis()-timerStart < WAIT_AMOUNT_AT_END) {
			//System.out.println("Waiting " + te.getComponent());
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		metaCSPLogger.info("Pedestrian tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + myTEID + ")");
	}

	@Override
	public long getCurrentTimeInMillis() {
		return this.tec.getCurrentTimeInMillis();
	}

}
