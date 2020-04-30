package se.oru.coordination.coordination_oru;

import com.vividsolutions.jts.geom.Coordinate;

public class Robot {

	 protected int ID;
	 protected int type;
	 protected Coordinate[] footprint;
	 protected ForwardModel fm = null;
	 
	 /**
	 * Create a new {@link Robot} 
	 * @param ID The ID of the robot. It is assumed to be unique among the fleet.
	 * @param type The robot's type.
	 * @param footprint The robot's footprint. 
	 * @param fm The forward model of the robot.
	 */
	 public Robot(int ID, int type, Coordinate[] footprint, ForwardModel fm) {
		 this.ID = ID;
		 this.type = type;
		 this.footprint = footprint;
		 this.fm = fm;
	 }

	 public int getID() {
		 return this.ID;
	 }

	 public int getType() {
		 return this.type;
	 }
	 
	 public Coordinate[] getFootprint() {
		 return this.footprint;
	 }
	 
	 public ForwardModel getForwardModel() {
		 return this.fm;
	 }

}
