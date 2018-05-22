package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.util.FleetVisualization;

/**
 * Implement this class to provide a callback to pass on to an {@link AbstractTrajectoryEnvelopeTracker} which
 * will be called at important points during tracking.
 * 
 * @author fpa
 *
 */
public abstract class TrackingCallback {

	protected TrajectoryEnvelope myTE = null;
	
	public void updateTrajectoryEnvelope(TrajectoryEnvelope newTE) {
		this.myTE = newTE;
	}
	
	public TrackingCallback() {
		this.myTE = null;
	}
	
	public TrackingCallback(TrajectoryEnvelope te) {
		this.myTE = te;
	}
	
	/**
	 * Will be called once before tracking actually starts.
	 */
	public abstract void beforeTrackingStart();

	/**
	 * Will be called once right after tracking has started.
	 */
	public abstract void onTrackingStart();


	/**
	 * Will be called on position update.
	 * @return Can return an array of strings that will be displayed if there is a {@link FleetVisualization} that supports it.
	 */
	public abstract String[] onPositionUpdate();
	
	/**
	 * Will be called once just before tracking ends.
	 */
	public abstract void beforeTrackingFinished();

	/**
	 * Will be called once right after tracking has ended.
	 */
	public abstract void onTrackingFinished();

	/**
	 * Will be called every time a new ground envelope is entered.
	 */
	public abstract void onNewGroundEnvelope();

}
