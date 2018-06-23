package se.oru.coordination.coordination_oru.tests.icaps2018.eval;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;

import org.metacsp.utility.UI.Callback;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeTrackerDummy;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public class TrajectoryEnvelopeCoordinatorSimulationICAPS extends TrajectoryEnvelopeCoordinatorSimulation {
	
	private ArrayList<Integer> robotsInUse = null;
	public TrajectoryEnvelopeCoordinatorSimulationICAPS(int CONTROL_PERIOD, double TEMPORAL_RESOLUTION, double MAX_VELOCITY, double MAX_ACCELERATION, String logFile, String logHeading, ArrayList<Integer> robotsInUse) {
		super(CONTROL_PERIOD, TEMPORAL_RESOLUTION, MAX_VELOCITY, MAX_ACCELERATION, 30);
		this.robotsInUse = robotsInUse;
		final FileWriter writer = writeSetupLog(logFile, logHeading);
		Callback cb = new Callback() {
			private long lastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			public void performOperation() {
				long timeNow = Calendar.getInstance().getTimeInMillis();
				if (timeNow-lastUpdate > 1000) {
					lastUpdate = timeNow;
					writeEvaluationLog(writer);
				}
			}
		};
		this.setInferenceCallback(cb);
	}
		
	public void writeEvaluationLog(FileWriter writer) {
		if (robotsInUse.size() > 0) {
			try {
				int numDrivingRobots = 0;
				for (AbstractTrajectoryEnvelopeTracker et : trackers.values()) {
					if (!(et instanceof TrajectoryEnvelopeTrackerDummy)) numDrivingRobots++;
				}
				writer.write(robotsInUse.size()+"\t"+numDrivingRobots+"\t"+EFFECTIVE_CONTROL_PERIOD+"\t"+allCriticalSections.size()+"\n");
				writer.flush();
			}
			catch (IOException e) { e.printStackTrace(); }
		}
	}
	
	public FileWriter writeSetupLog(String filename, String heading) {
		FileWriter writer = null;
		try {
			File log = new File(filename);
			writer = new FileWriter(log, false);
			heading+="\n";
			writer.write(heading);
		}
		catch (IOException e) { e.printStackTrace(); }
		return writer;
	}
	

	

}
