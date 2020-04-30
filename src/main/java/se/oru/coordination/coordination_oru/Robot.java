package se.oru.coordination.coordination_oru;

import com.vividsolutions.jts.geom.Coordinate;

public class Robot {

	 protected int ID;
	 protected int type;
	 protected Coordinate[] footprint;
	 protected ForwardModel fm = null;
	 
	 /**
	 * Create a new {@link Robot} 
	 * @param ID The robot ID. It is assumed to be unique among the fleet.
	 * @param type The robot type.
	 * @param footprint The robot footprint. 
	 * @param fm The forward model of the robot.
	 */
	 public Robot(int robotID, int robotType, Coordinate[] footprint, ForwardModel fm) {
		 this.ID = robotID;
		 this.type = robotType;
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
