package se.oru.coordination.coordination_oru.simulation2D;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;
import java.util.TreeMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.NetworkConfiguration;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.util.Missions;

public abstract class TrajectoryEnvelopeTrackerRK4 extends AbstractTrajectoryEnvelopeTracker implements Runnable {

	protected static final long WAIT_AMOUNT_AT_END = 3000;
	protected static final double EPSILON = 0.01;
	protected final double MAX_VELOCITY;
	protected final double MAX_ACCELERATION;
	protected double overallDistance = 0.0;
	protected double totalDistance = 0.0;
	protected double positionToSlowDown = -1.0;
	protected double elapsedTrackingTime = 0.0;
	private Thread th = null;
	protected State state = null;
	protected double[] curvatureDampening = null;
	private ArrayList<Integer> internalCriticalPoints = new ArrayList<Integer>();
	private int maxDelayInMillis = 0;
	private Random rand = new Random(Calendar.getInstance().getTimeInMillis()); 
	private TreeMap<Double,Double> slowDownProfile = null;
	private boolean slowingDown = false;
	private boolean useInternalCPs = true;
	protected ArrayList<RobotReport> reportsList = new ArrayList<RobotReport>();
	protected ArrayList<Long> reportTimeLists = new ArrayList<Long>();
	
	private HashMap<Integer,Integer> userCPReplacements = null;

	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}
	
	public TrajectoryEnvelopeTrackerRK4(TrajectoryEnvelope te, int timeStep, double temporalResolution, TrajectoryEnvelopeCoordinatorSimulation tec, TrackingCallback cb) {
		this(te, timeStep, temporalResolution, 1.0, 0.1, tec, cb);
	}
	
	private void computeInternalCriticalPoints() {
		this.curvatureDampening = new double[te.getTrajectory().getPose().length];
		this.curvatureDampening[0] = 1.0;
		Pose[] poses = this.traj.getPose();
		double prevTheta = poses[0].getTheta();
		if (poses.length > 1) prevTheta = Math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX());
		for (int i = 0; i < poses.length-1; i++) {
			double theta = Math.atan2(poses[i+1].getY() - poses[i].getY(), poses[i+1].getX() - poses[i].getX());
			double deltaTheta = (theta-prevTheta);
			prevTheta = theta;
			if (Math.abs(deltaTheta) > Math.PI/2 && Math.abs(deltaTheta) < 1.9*Math.PI) {
				internalCriticalPoints.add(i);
				metaCSPLogger.info("Found internal critical point (" + te.getComponent() + "): " + (i));
			}
			this.curvatureDampening[i+1] = 1.0;
		}
	}
	
	public void setCurvatureDampening(int index, double dampening) {
		this.curvatureDampening[index] = dampening;
	}

	public void setCurvatureDampening(int indexFrom, int indexTo, double dampening) {
		for (int i = indexFrom; i < indexTo; i++) curvatureDampening[i] = dampening;
	}

	public void resetCurvatureDampening() {
		for (int i  = 0; i < curvatureDampening.length; i++) curvatureDampening[i] = 1.0;
	}
	
	public double[] getCurvatureDampening() {
		return this.curvatureDampening;
	}private void computeCurvatureDampening() {
		PoseSteering[] path = this.traj.getPoseSteering();
		double deltaSinTheta = 0;
		double sinThetaPrev = Math.sin(Missions.wrapAngle180b(path[0].getTheta()));
		for (int i = 1; i < path.length; i++) {
			double sinTheta = Math.sin(Missions.wrapAngle180b(path[i].getTheta()));
			double deltaSinThetaNew = sinTheta-sinThetaPrev;
			if (deltaSinThetaNew*deltaSinTheta < 0 && i != 1) {
				System.out.println("Direction change for Robot" + this.te.getRobotID() + " in " + i);
				this.curvatureDampening[i] = 0.2;
			}
			deltaSinTheta = deltaSinThetaNew;
			sinThetaPrev = deltaSinThetaNew;
		}
	}

	public double getCurvatureDampening(int index, boolean backwards) {
		if (!backwards) return curvatureDampening[index];
		return curvatureDampening[this.traj.getPose().length-1-index];
	}
	
	public TrajectoryEnvelopeTrackerRK4(TrajectoryEnvelope te, int timeStep, double temporalResolution, double maxVelocity, double maxAcceleration, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb) {
		super(te, temporalResolution, tec, timeStep, cb);
		this.MAX_VELOCITY = maxVelocity;
		this.MAX_ACCELERATION = maxAcceleration;
		this.state = new State(0.0, 0.0);
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
		this.computeInternalCriticalPoints();
		this.slowDownProfile = this.getSlowdownProfile();
		this.positionToSlowDown = this.computePositionToSlowDown();
		this.th = new Thread(this, "RK4 tracker " + te.getComponent());
		this.th.setPriority(Thread.MAX_PRIORITY);
	}
	
	@Override
	protected void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te) {
		synchronized(reportsList) { //FIXME not ok, all the mutex should be changed
			this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
			this.overallDistance = totalDistance;
			this.internalCriticalPoints.clear();
			this.computeInternalCriticalPoints();
			reportsList.clear();
			reportTimeLists.clear(); //semplify to avoid discontinuities ... to be fixed.
		}
	}
	
	@Override
	public void startTracking() {		
		while (this.th == null) {
			try { Thread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		this.th.start();
		if (useInternalCPs) this.startInternalCPThread();	
	}

	public static double computeDistance(Trajectory traj, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,traj.getPoseSteering().length-1); i++) {
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	private double computeDistance(int startIndex, int endIndex) {
		return computeDistance(this.traj, startIndex, endIndex);
	}
		
	private void enqueueOneReport() {
			
		synchronized (reportsList) {
			
			//Before start, initialize the position
			if (reportsList.isEmpty()) {
				if (getRobotReport() != null) {
					reportsList.add(0, getRobotReport());
					reportTimeLists.add(0, Calendar.getInstance().getTimeInMillis());
				}
				return;
			}
			
			long timeNow = Calendar.getInstance().getTimeInMillis();
			final int numberOfReplicasReceiving = Math.max(1, (int)Math.ceil(tec.getNumberOfReplicas()*(double)trackingPeriodInMillis/tec.getControlPeriod()));
				
			timeNow = Calendar.getInstance().getTimeInMillis();
			long timeOfArrival = timeNow;
			if (NetworkConfiguration.getMaximumTxDelay() > 0) {
				//the real delay
				int delay = (NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay() > 0) ? rand.nextInt(NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay()) : 0;
				timeOfArrival = timeOfArrival + NetworkConfiguration.getMinimumTxDelay() + delay;
			}
				
			//Get the message according to packet loss probability (numberOfReplicas trials)
			boolean received = (NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS > 0) ? false : true;
			int trial = 0;
			while(!received && trial < numberOfReplicasReceiving) {
				if (rand.nextDouble() < (1-NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS)) //the real packet loss probability
					received = true;
				trial++;
			}
			if (received) {		
				//Delete old messages that, due to the communication delay, will arrive after this one.			
				ArrayList<Long> reportTimeToRemove = new ArrayList<Long>();
				ArrayList<RobotReport> reportToRemove = new ArrayList<RobotReport>();
				
				for (int index = 0; index < reportTimeLists.size(); index++) {
					if (reportTimeLists.get(index) < timeOfArrival) break;
					if (reportTimeLists.get(index) >= timeOfArrival) {
						reportToRemove.add(reportsList.get(index));
						reportTimeToRemove.add(reportTimeLists.get(index));
					}	
				}
				
				for (Long time : reportTimeToRemove) reportTimeLists.remove(time);
				for (RobotReport report : reportToRemove) reportsList.remove(report);
				
				reportsList.add(0, getRobotReport()); //The new one is the one that will arrive later and is added in front of the queue.
				reportTimeLists.add(0, timeOfArrival); //The oldest is in the end.
			}
			
			//Keep alive just the most recent message before now.
			if (reportTimeLists.get(reportTimeLists.size()-1) > timeNow) {
				metaCSPLogger.severe("* ERROR * Unknown status Robot"+te.getRobotID());
				//FIXME add a function for stopping pausing the fleet and eventually restart
			}
			else {
				ArrayList<Long> reportTimeToRemove = new ArrayList<Long>();
				ArrayList<RobotReport> reportToRemove = new ArrayList<RobotReport>();
				
				for (int index = reportTimeLists.size()-1; index > 0; index--) {
					if (reportTimeLists.get(index) > timeNow) break; //the first in the future
					if (reportTimeLists.get(index) < timeNow && reportTimeLists.get(index-1) <= timeNow) {
						reportToRemove.add(reportsList.get(index));
						reportTimeToRemove.add(reportTimeLists.get(index));
					}
				}
	
				for (Long time : reportTimeToRemove) reportTimeLists.remove(time);
				for (RobotReport report : reportToRemove) reportsList.remove(report);
			}
			
			//Check if the current status message is too old.
			if (timeNow - reportTimeLists.get(reportTimeLists.size()-1) > tec.getControlPeriod() + TrajectoryEnvelopeCoordinator.MAX_TX_DELAY) { //the known delay
				metaCSPLogger.severe("* ERROR * Status of Robot"+ te.getRobotID() + " is too old.");
				//FIXME add a function for stopping pausing the fleet and eventually restart
				}
		}
	}
	
	@Override
	public RobotReport getLastRobotReport() {
		synchronized (reportsList) {
			if (reportsList.isEmpty()) return getRobotReport();
			return reportsList.get(reportsList.size()-1);
		}
	}

	private void startInternalCPThread() {
		Thread t = new Thread() {
			@Override
			public void run() {
				userCPReplacements = new HashMap<Integer, Integer>();
				
				while (th.isAlive()) {
					ArrayList<Integer> toRemove = new ArrayList<Integer>();
					for (Integer i : internalCriticalPoints) {
						if (getRobotReport().getPathIndex() >= i) {
							toRemove.add(i);
							setCriticalPoint(userCPReplacements.get(i));
							metaCSPLogger.info("Restored critical point (" + te.getComponent() + "): " + userCPReplacements.get(i) + " which was masked by internal critical point " + i);
							break;
						}
						else {
							if (criticalPoint == -1 || criticalPoint > i) {
								userCPReplacements.put(i, criticalPoint);
								metaCSPLogger.info("Set internal critical point (" + te.getComponent() + "): " + i + " replacing critical point " + criticalPoint);
								setCriticalPoint(i);
								break;					
							}
						}
					}
					for (Integer i : toRemove) {
						internalCriticalPoints.remove(i);
					}
					
					try { Thread.sleep(trackingPeriodInMillis); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
			}
		};
		t.start();
	}
	
	
	private TreeMap<Double,Double> getSlowdownProfile() {
		TreeMap<Double,Double> ret = new TreeMap<Double, Double>(Collections.reverseOrder());
		State tempStateBW = new State(0.0, 0.0);
		ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());

		double time = 0.0;
		double deltaTime = 0.5*(this.trackingPeriodInMillis/this.temporalResolution);
		//Compute where to slow down (can do forward here for both states...)
		while (tempStateBW.getVelocity() < MAX_VELOCITY*1.1) {
			double dampeningBW = getCurvatureDampening(getRobotReport(tempStateBW).getPathIndex(), true);
			//Use slightly conservative max deceleration (which is positive acceleration since we simulate FW dynamics)
			integrateRK4(tempStateBW, time, deltaTime, false, MAX_VELOCITY*1.1, dampeningBW, MAX_ACCELERATION);
			time += deltaTime;
			ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());
		}
		
		//for (Double speed : ret.keySet()) System.out.println("@speed " + speed + " --> " + ret.get(speed));
		return ret;
	}
	
	private double computePositionToSlowDown() {
		State tempStateFW = new State(state.getPosition(), state.getVelocity());
		double time = 0.0;
		double deltaTime = 0.5*(this.trackingPeriodInMillis/this.temporalResolution);

		//Compute where to slow down (can do forward here, we have the slowdown profile...)
		while (tempStateFW.getPosition() < this.totalDistance) {
			double prevSpeed = -1.0;
			boolean firstTime = true;
			for (Double speed : this.slowDownProfile.keySet()) {
				//Find your speed in the table (table is ordered w/ highest speed first)...
				if (tempStateFW.getVelocity() > speed) {
					//If this speed lands you after total dist you are OK (you've checked at lower speeds and either returned or breaked...) 
					double landingPosition = tempStateFW.getPosition() + (firstTime ? 0.0 : slowDownProfile.get(prevSpeed));
					if (landingPosition > totalDistance) {
						//System.out.println("Found: speed = " + tempStateFW.getVelocity() + " space needed = " + slowDownProfile.get(prevSpeed) + " (delta = " + Math.abs(totalDistance-landingPosition) + ")");
						//System.out.println("Position to slow down = " + tempStateFW.getPosition());
						return tempStateFW.getPosition();
					}
					//System.out.println("Found: speed = " + tempStateFW.getVelocity() + " space needed = " + slowDownProfile.get(speed) + " (undershoot by " + (totalDistance-tempStateFW.getPosition()+slowDownProfile.get(speed)) + ")");
					break;
				}
				firstTime = false;
				prevSpeed = speed;
			}
			
			double dampeningFW = getCurvatureDampening(getRobotReport(tempStateFW).getPathIndex(), true);
			integrateRK4(tempStateFW, time, deltaTime, false, MAX_VELOCITY, dampeningFW, MAX_ACCELERATION);

			time += deltaTime;
		}
		return -this.totalDistance;
	}
	
	public static void integrateRK4(State state, double time, double deltaTime, boolean slowDown, double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION) {
		synchronized(state) {
			Derivative a = Derivative.evaluate(state, time, 0.0, new Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative b = Derivative.evaluate(state, time, deltaTime/2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative c = Derivative.evaluate(state, time, deltaTime/2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR,MAX_ACCELERATION);
			Derivative d = Derivative.evaluate(state, time, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
	
			double dxdt = (1.0f / 6.0f) * ( a.getVelocity() + 2.0f*(b.getVelocity() + c.getVelocity()) + d.getVelocity() ); 
		    double dvdt = (1.0f / 6.0f) * ( a.getAcceleration() + 2.0f*(b.getAcceleration() + c.getAcceleration()) + d.getAcceleration() );
			
		    state.setPosition(state.getPosition()+dxdt*deltaTime);
		    state.setVelocity(state.getVelocity()+dvdt*deltaTime);
		}
	}
	

	@Override
	public void setCriticalPoint(int criticalPointToSet, int extCPCounter) {

		final int criticalPoint = criticalPointToSet;
		final int externalCPCount = extCPCounter;
		final int numberOfReplicas = tec.getNumberOfReplicas();
		
		//Define a thread that will send the information
		Thread waitToTXThread = new Thread("Wait to TX thread for robot " + te.getRobotID()) {
			public void run() {

				int delayTx = 0;
				if (NetworkConfiguration.getMaximumTxDelay() > 0) {
					//the real delay
					int delay = (NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay() > 0) ? rand.nextInt(NetworkConfiguration.getMaximumTxDelay()-NetworkConfiguration.getMinimumTxDelay()) : 0;
					delayTx = NetworkConfiguration.getMinimumTxDelay() + delay;
				}
				
				//Sleep for delay in communication
				try { Thread.sleep(delayTx); }
				catch (InterruptedException e) { e.printStackTrace(); }
				
				//if possible (according to packet loss, send
				synchronized (externalCPCounter)
				{
					boolean send = false;
					int trial = 0;
					//while(!send && trial < numberOfReplicas) {
					while(trial < numberOfReplicas) {
						if (rand.nextDouble() < (1-NetworkConfiguration.PROBABILITY_OF_PACKET_LOSS)) //the real one
							send = true;
						else {
							TrajectoryEnvelopeCoordinatorSimulation tc = (TrajectoryEnvelopeCoordinatorSimulation)tec;
							tc.incrementLostPacketsCounter();
						}
						trial++;
					}
					if (send) {
						//metaCSPLogger.info("PACKET to Robot" + te.getRobotID() + " SENT, criticalPoint: " + criticalPoint + ", externalCPCounter: " + externalCPCount);
						if (
								(externalCPCount < externalCPCounter && externalCPCount-externalCPCounter > Integer.MAX_VALUE/2.0) ||
								(externalCPCounter > externalCPCount && externalCPCounter-externalCPCount < Integer.MAX_VALUE/2.0)) {
							metaCSPLogger.info("Ignored critical point " + criticalPoint + " related to counter " + externalCPCount + " because counter is already at " + externalCPCounter + ".");
						}
						else {
							setCriticalPoint(criticalPoint);
							externalCPCounter = externalCPCount;
						}
											
						if (!canStartTracking()) {
							setCanStartTracking();
						}
					}
					else {
						TrajectoryEnvelopeCoordinatorSimulation tc = (TrajectoryEnvelopeCoordinatorSimulation)tec;
						tc.incrementLostMsgsCounter();
						metaCSPLogger.info("PACKET to Robot" + te.getRobotID() + " LOST, criticalPoint: " + criticalPoint + ", externalCPCounter: " + externalCPCount);
					}
				}
			}
		};
		//let's start the thread
		waitToTXThread.start();
		
	}
			
	
	@Override
	protected void setCriticalPoint(int criticalPointToSet) {
				
		if (this.criticalPoint != criticalPointToSet) {
			
			//A new intermediate index to stop at has been given
			if (criticalPointToSet != -1 && criticalPointToSet > getRobotReport().getPathIndex()) {			
				//Store backups in case we are too late for critical point
				double totalDistanceBKP = this.totalDistance;
				int criticalPointBKP = this.criticalPoint;
				double positionToSlowDownBKP = this.positionToSlowDown;
	
				this.criticalPoint = criticalPointToSet;
				//TOTDIST: ---(state.getPosition)--->x--(computeDist)--->CP
				this.totalDistance = computeDistance(0, criticalPointToSet);
				this.positionToSlowDown = computePositionToSlowDown();
				
				//We are too late for critical point, restore everything
				if (this.positionToSlowDown < state.getPosition()) {
					metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because slowdown distance (" + this.positionToSlowDown +") < current distance (" + state.getPosition() + ")");
					this.criticalPoint = criticalPointBKP;
					this.totalDistance = totalDistanceBKP;
					this.positionToSlowDown = positionToSlowDownBKP;
				}
				else {
					metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet + ", currently at point " + this.getRobotReport().getPathIndex() + ", distance " + state.getPosition() + ", will slow down at distance " + this.positionToSlowDown);
				}
			}

			//Critical point <= current position, ignore -- WHY??
			else if (criticalPointToSet != -1 && criticalPointToSet <= getRobotReport().getPathIndex()) {
				metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because robot is already at " + getRobotReport().getPathIndex() + " (and current CP is " + this.criticalPoint + ")");
			}
			
			//The critical point has been reset, go to the end
			else if (criticalPointToSet == -1) {
				this.criticalPoint = criticalPointToSet;
				this.totalDistance = computeDistance(0, traj.getPose().length-1);
				this.positionToSlowDown = computePositionToSlowDown();
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
		if (state == null) return null;
		if (!this.th.isAlive()) return new RobotReport(te.getRobotID(), traj.getPose()[0], -1, 0.0, 0.0, -1);
		synchronized(state) {
			Pose pose = null;
			int currentPathIndex = -1;
			double accumulatedDist = 0.0;
			Pose[] poses = traj.getPose();
			for (int i = 0; i < poses.length-1; i++) {
				double deltaS = poses[i].distanceTo(poses[i+1]);
				accumulatedDist += deltaS;
				if (accumulatedDist > state.getPosition()) {
					double ratio = 1.0-(accumulatedDist-state.getPosition())/deltaS;
					pose = poses[i].interpolate(poses[i+1], ratio);
					currentPathIndex = i;
					break;
				}
			}
			if (currentPathIndex == -1) {
				currentPathIndex = poses.length-1;
				pose = poses[currentPathIndex];
			}
			return new RobotReport(te.getRobotID(), pose, currentPathIndex, state.getVelocity(), state.getPosition(), this.criticalPoint);
		}
	}

	private static RobotReport getRobotReport(Trajectory traj, State auxState) {
		if (auxState == null) return null;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return new RobotReport(-1, pose, currentPathIndex, auxState.getVelocity(), auxState.getPosition(), -1);
	}

	public RobotReport getRobotReport(State auxState) {
		if (auxState == null) return null;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return new RobotReport(te.getRobotID(), pose, currentPathIndex, auxState.getVelocity(), auxState.getPosition(), -1);
	}

	public void delayIntegrationThread(int maxDelayInMillis) {
		this.maxDelayInMillis = maxDelayInMillis;
	}
	
	@Override
	public void run() {
		this.elapsedTrackingTime = 0.0;
		double deltaTime = 0.0;
		boolean atCP = false;
		int myRobotID = te.getRobotID();
		int myTEID = te.getID();
		
		
		while (true) {
						
			//End condition: passed the middle AND velocity < 0 AND no criticalPoint 			
			boolean skipIntegration = false;
			//if (state.getPosition() >= totalDistance/2.0 && state.getVelocity() < 0.0) {
			if (state.getPosition() >= this.positionToSlowDown && state.getVelocity() < 0.0) {
				if (criticalPoint == -1 && !atCP) {
					//set state to final position, just in case it didn't quite get there (it's certainly close enough)
					state = new State(totalDistance, 0.0);
					onPositionUpdate();
					break;
				}
								
				//Vel < 0 hence we are at CP, thus we need to skip integration
				if (!atCP /*&& getRobotReport().getPathIndex() == criticalPoint*/) {
					int pathIndex = getRobotReport().getPathIndex();
					metaCSPLogger.info("At critical point (" + te.getComponent() + "): " + criticalPoint + " (" + pathIndex + ")");
					if (pathIndex > criticalPoint) metaCSPLogger.severe("* ATTENTION! STOPPED AFTER!! *");
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
					atCP = false;
				}
				slowingDown = false;
				if (state.getPosition() >= positionToSlowDown) slowingDown = true;
				double dampening = getCurvatureDampening(getRobotReport().getPathIndex(), false);
				integrateRK4(state, elapsedTrackingTime, deltaTime, slowingDown, MAX_VELOCITY, dampening, MAX_ACCELERATION);

			}
			
			//Do some user function on position update
			onPositionUpdate();
			enqueueOneReport();
						
			//Sleep for tracking period
			int delay = trackingPeriodInMillis;
			if (maxDelayInMillis > 0) delay += rand.nextInt(maxDelayInMillis);
			try { Thread.sleep(delay); }
			catch (InterruptedException e) { e.printStackTrace(); }
			
			//Advance time to reflect how much we have slept (~ trackingPeriod)
			long deltaTimeInMillis = Calendar.getInstance().getTimeInMillis()-timeStart;
			deltaTime = deltaTimeInMillis/this.temporalResolution;
			elapsedTrackingTime += deltaTime;
		}
		
		//continue transmitting until the coordinator will be informed of having reached the last position.
		while (tec.getRobotReport(te.getRobotID()).getPathIndex() != -1)
		{
			enqueueOneReport();
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		
		//persevere with last path point in case listeners didn't catch it!
		long timerStart = getCurrentTimeInMillis();
		while (getCurrentTimeInMillis()-timerStart < WAIT_AMOUNT_AT_END) {
			//System.out.println("Waiting " + te.getComponent());
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		metaCSPLogger.info("RK4 tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + myTEID + ")");
	}

	public static double[] computeDTs(Trajectory traj, double maxVel, double maxAccel) {
		double distance = computeDistance(traj, 0, traj.getPose().length-1);
		State state = new State(0.0, 0.0);
		double time = 0.0;
		double deltaTime = 0.0001;
		
		ArrayList<Double> dts = new ArrayList<Double>();
		HashMap<Integer,Double> times = new HashMap<Integer, Double>();
		dts.add(0.0);
		times.put(0, 0.0);
		
		//First compute time to stop (can do FW here...)
		while (state.getPosition() < distance/2.0 && state.getVelocity() < maxVel) {
			integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
			time += deltaTime;
		}
		double positionToSlowDown = distance-state.getPosition();
		//System.out.println("Position to slow down is: " + MetaCSPLogging.printDouble(positionToSlowDown,4));

		state = new State(0.0, 0.0);
		time = 0.0;
		while (true) {
			if (state.getPosition() >= distance/2.0 && state.getVelocity() < 0.0) break;
			if (state.getPosition() >= positionToSlowDown) {
				integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			}
			else {
				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);				
			}
			//System.out.println("Time: " + time + " " + rr);
			//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
			time += deltaTime;
			RobotReport rr = getRobotReport(traj, state);
			if (!times.containsKey(rr.getPathIndex())) {
				times.put(rr.getPathIndex(), time);
				dts.add(time-times.get(rr.getPathIndex()-1));
			}
		}
		if (dts.size() < traj.getPose().length) {
			times.put(traj.getPose().length-1, time);		
			dts.add(time-times.get(traj.getPose().length-2));
		}
		
		//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
		
		double[] ret = new double[dts.size()];
		for (int i = 0; i < dts.size(); i++) ret[i] = dts.get(i);
		return ret;

	}

	

}
