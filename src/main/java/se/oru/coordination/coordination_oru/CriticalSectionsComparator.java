package se.oru.coordination.coordination_oru;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Class to order critical sections related to the same robotID and trajectory envelopes with increasing starting indices. 
 */
public class CriticalSectionsComparator implements Comparator<CriticalSection> { 
	protected int robotID = -1;
	protected int teID = -1;
	protected int parkingTeID = -1;
	
	public CriticalSectionsComparator(int robotID, int teID, int parkingTeID) {
		this.robotID = robotID;
		this.teID = teID;
		this.parkingTeID = parkingTeID;
	};
	
	public void setRobotID(int robotID) {
		this.robotID = robotID;
	}
	
	public void setTrajectoryEnvelopeID(int teID) {
		this.teID = teID;
	}
	
	public void setParkingTrajectoryEnvelopeID(int parkingTeID) {
		this.parkingTeID = parkingTeID;
	}
	
	public int getRobotID() {
		return this.robotID;
	}
	
	public int getTrajectoryEnvelopeID() {
		return this.teID;
	}
	
	public int getParkingTrajectoryEnvelopeID() {
		return this.parkingTeID;
	}
	
    @Override
	public int compare(CriticalSection cs1, CriticalSection cs2) {
    	List<Boolean> cs1Te = new ArrayList<Boolean>();
    	cs1Te.add(cs1.getTe1().getRobotID() == this.robotID && cs1.getTe1().getID() == this.parkingTeID);
    	cs1Te.add(cs1.getTe1().getRobotID() == this.robotID && cs1.getTe1().getID() == this.teID);
    	cs1Te.add(cs1.getTe2().getRobotID() == this.robotID && cs1.getTe2().getID() == this.parkingTeID);
    	cs1Te.add(cs1.getTe2().getRobotID() == this.robotID && cs1.getTe2().getID() == this.teID);
    	    	
    	List<Boolean> cs2Te = new ArrayList<Boolean>();
    	cs2Te.add(cs2.getTe1().getRobotID() == this.robotID && cs2.getTe1().getID() == this.parkingTeID);
    	cs2Te.add(cs2.getTe1().getRobotID() == this.robotID && cs2.getTe1().getID() == this.teID);
    	cs2Te.add(cs2.getTe2().getRobotID() == this.robotID && cs2.getTe2().getID() == this.parkingTeID);
    	cs2Te.add(cs2.getTe2().getRobotID() == this.robotID && cs2.getTe2().getID() == this.teID);
    	
    	//check if the critical sections are comparable (related to robot robotID and to its current trajectory envelopes).
    	assert( ( cs1Te.get(0)|| cs1Te.get(1) || cs1Te.get(2) || cs1Te.get(3) ) && ( cs2Te.get(0) || cs2Te.get(1) || cs2Te.get(2) || cs2Te.get(3) ) &&
    			((cs1Te.get(0) ? 1 : 0) + (cs1Te.get(1) ? 1 : 0) + (cs1Te.get(2) ? 1 : 0) + (cs1Te.get(3) ? 1 : 0) + 
    					(cs2Te.get(0) ? 1 : 0) + (cs2Te.get(1) ? 1 : 0) + (cs2Te.get(2) ? 1 : 0) + (cs2Te.get(3) ? 1 : 0) == 2));

    	
    	//find the starting poses 
    	int startInCS1 = (cs1Te.get(0)||cs1Te.get(1)) ? cs1.getTe1Start() : cs1.getTe2Start();
    	int endInCS1 = (cs1Te.get(0)||cs1Te.get(1)) ? cs1.getTe1End() : cs1.getTe2End();
    	int otherIdInCS1 = (cs1Te.get(0)||cs1Te.get(1)) ? cs1.getTe2().getRobotID() : cs1.getTe1().getRobotID();
    	int startInCS2 = (cs2Te.get(0)||cs2Te.get(1)) ? cs2.getTe1Start() : cs2.getTe2Start();
    	int endInCS2 = (cs2Te.get(0)||cs2Te.get(1)) ? cs2.getTe1End() : cs2.getTe2End();
    	int otherIdInCS2 = (cs2Te.get(0)||cs2Te.get(1)) ? cs2.getTe2().getRobotID() : cs2.getTe1().getRobotID();
    	
    	// Return a negative integer, zero, or a positive integer as the first argument is less than, 
    	//equal to, or greater than the second.
    	//if (startInCS1 + 0.1*otherIdInCS1 > startInCS2 + 0.1*otherIdInCS2) return 1;
    	//if (startInCS1 + 0.1*otherIdInCS1 < startInCS2 + 0.1*otherIdInCS2) return -1;
    	//return 0;
    	
    	if (otherIdInCS1 == otherIdInCS2 && startInCS1 == startInCS2 && endInCS1 == endInCS2) return 0;
    	if ( startInCS1 > startInCS2 ||
    			startInCS1 == startInCS2 && ( endInCS1 > endInCS2 || endInCS1 == endInCS2 && otherIdInCS1 > otherIdInCS2 ) )
    		return 1;
    	return -1; 	
   } 

}
