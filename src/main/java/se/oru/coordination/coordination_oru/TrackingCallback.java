package se.oru.coordination.coordination_oru;

import se.oru.coordination.coordination_oru.util.FleetVisualization;

/**
 * Implement this class to provide a callback to pass on to an {@link AbstractTrajectoryEnvelopeTracker} which
 * will be called at important points during tracking.
 * 
 * @author fpa
 *
 */
public interface TrackingCallback {

	/**
	 * Will be called once before tracking actually starts.
	 */
	public void beforeTrackingStart();

	/**
	 * Will be called once right after tracking has started.
	 */
	public void onTrackingStart();


	/**
	 * Will be called on position update.
	 * @return Can return an array of strings that will be displayed if there is a {@link FleetVisualization} that supports it.
	 */
	public String[] onPositionUpdate();
	
	/**
	 * Will be called once just before tracking ends.
	 */
	public void beforeTrackingFinished();

	/**
	 * Will be called once right after tracking has ended.
	 */
	public void onTrackingFinished();

	/**
	 * Will be called every time a new ground envelope is entered.
	 */
	public void onNewGroundEnvelope();

}
