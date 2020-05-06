package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TaskData {
	
	protected String fromLocation = null;
	protected String toLocation = null;
	protected Pose fromPose = null;
	protected Pose toPose = null;
	protected ArrayList<Pose> stoppingPoints = new ArrayList<Pose>();
	protected ArrayList<Integer> stoppingPointDurations = new ArrayList<Integer>();
	
	/**
	 * Instantiate the data to navigate between two locations.
	 * @param fromLocation The identifier of the source location.
	 * @param toLocation The identifier of the destination location.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public TaskData(String fromLocation, String toLocation, Pose fromPose, Pose toPose) {
		this.fromLocation = fromLocation;
		this.toLocation = toLocation;
		this.fromPose = fromPose;
		this.toPose = toPose;		
	}
		
	/**
	 * Instantiate the data to navigate between two locations.
	 * @param fromPose The pose of the source location.
	 * @param toPose The pose of the destination location.
	 */
	public TaskData(Pose fromPose, Pose toPose) {
		this(fromPose.toString(), toPose.toString(), fromPose, toPose);		
	}
	
	/**
	 * Get the name of the source location of this {@link TaskData}.
	 * @return The name of the source location of this {@link TaskData}.
	 */
	public String getFromLocation() {
		return fromLocation;
	}
	
	/**
	 * Get the name of the destination location of this {@link TaskData}.
	 * @return The name of the destination location of this {@link TaskData}.
	 */
	public String getToLocation() {
		return toLocation;
	}
	/**
	 * Get the {@link Pose} of source location of this {@link TaskData}.
	 * @return The {@link Pose} of source location of this {@link TaskData}.
	 */
	public Pose getFromPose() {
		return fromPose;
	}

	/**
	 * Set the {@link Pose} of source location of this {@link TaskData}.
	 * @param fromPose The {@link Pose} of source location of this {@link TaskData}.
	 */
	public void setFromPose(Pose fromPose) {
		this.fromPose = fromPose;
	}


	/**
	 * Get the {@link Pose} of destination location of this {@link TaskData}.
	 * @return The {@link Pose} of destination location of this {@link TaskData}.
	 */
	public Pose getToPose() {
		return toPose;
	}

	/**
	 * Get the {@link Pose} of destination location of this {@link TaskData}.
	 * @param toPose The {@link Pose} of destination location of this {@link TaskData}.
	 */
	public void setToPose(Pose toPose) {
		this.toPose = toPose;
	}
	
	/**
	 * Make the robot stop at the nearest location to a given pose for a given duration.
	 * @param pose A pose to stop at.
	 * @param duration Stopping time in milliseconds. 
	 */
	public void setStoppingPoint(Pose pose, int duration) {
		this.stoppingPoints.add(pose);
		this.stoppingPointDurations.add(duration);
		//this.stoppingPoints.put(pose, duration);
	}
	
	/**
	 * Clear the stopping points of this {@link TaskData}.
	 */
	public void clearStoppingPoints() {
		this.stoppingPoints.clear();
		this.stoppingPointDurations.clear();
	}
	
	/**
	 * Get the stopping points along this {@link Mission}'s trajectory along with their durations.
	 * @return The stopping points along this {@link Mission}'s trajectory along with their durations.
	 */
	public HashMap<Pose,Integer> getStoppingPoints() {
		HashMap<Pose,Integer> ret = new HashMap<Pose, Integer>();
		for (int i = 0; i < this.stoppingPoints.size(); i++) {
			ret.put(this.stoppingPoints.get(i), this.stoppingPointDurations.get(i));
		}
		return ret;
		//return this.stoppingPoints;
	}
	
	/**
	 * Set the destination location for this {@link TaskData}.
	 * @param location The destination of this {@link TaskData}.
	 */
	public void setToLocation(String location) {
		this.toLocation = location;
	}

	/**
	 * Set the source location for this {@link TaskData}.
	 * @param location The source of this {@link TaskData}.
	 */
	public void setFromLocation(String location) {
		this.fromLocation = location;
	}
}
