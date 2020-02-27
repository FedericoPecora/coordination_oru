package se.oru.coordination.coordination_oru;

import java.util.Comparator;

/**
 * Class to order critical sections related to one robot with increasing indices for starts,
 * The class checks the consistency of provided data throwing an error if not. 
 */
public class CriticalSectionsComparator implements Comparator<CriticalSection> { 
	protected int robotID = -1;
	
	public CriticalSectionsComparator(int robotID) {
		this.robotID = robotID;
	};
	
	public int getRobotID() {
		return this.robotID;
	}
	
	/** Assess if two critical sections are comparable with respect to one robot (the one with ID equal to robotID).
	 *  Two critical sections are not comparable
	 *  - if they do not involve the one common robot.
	 *  - if the trajectory envelopes of the common robot are not consistent, that is, they are related to a consistent pair of 
	 *    driving and parking envelope. 
	 *  @param cs1 One critical section to be compared.
	 *  @param cs2 The other critical section to be compared.
	 *  @return If the critical sections are comparable.
	 */
	public boolean comparable(CriticalSection cs1, CriticalSection cs2) {
		boolean[] flags = new boolean[4];
		flags[0] = cs1.getTe1().getRobotID() == this.robotID;
		flags[1] = cs1.getTe2().getRobotID() == this.robotID;
		flags[2] = cs2.getTe1().getRobotID() == this.robotID;
		flags[3] = cs2.getTe2().getRobotID() == this.robotID;
		
		int sum = 0;
		for (int i = 0; i < flags.length; i++) sum += (flags[i] ? 1 : 0);
    	
    	//Check if the critical sections are comparable:
    	// - check if cs1 and cs2 are related to the same robot.
    	if (!((flags[0] || flags[1]) && (flags[2] || flags[3]) && sum == 2)) {
    		return false;
    	};
    	// - if the same pair of robots is involved, check correctness of trajectory envelopes.
    	if (flags[0] && flags[2] && (
    			((cs1.getTe1Start() != cs1.getTe1End() && cs2.getTe1Start() != cs2.getTe1End()) || //Either both are driving envelopes
    					(cs1.getTe1Start() == cs1.getTe1End() && cs2.getTe1Start() == cs2.getTe1End()))    //or both are parking envelopes,
    			&& !cs1.getTe1().equals(cs2.getTe1())                                              //but they are different.
    			)) return false;
    	if (flags[0] && flags[3] && (
    			((cs1.getTe1Start() != cs1.getTe1End() && cs2.getTe2Start() != cs2.getTe2End()) ||
    					(cs1.getTe1Start() == cs1.getTe1End() && cs2.getTe2Start() == cs2.getTe2End()))
    			&& !cs1.getTe1().equals(cs2.getTe2())
    			)) return false;
    	if (flags[1] && flags[2] && (
    			((cs1.getTe2Start() != cs1.getTe2End() && cs2.getTe1Start() != cs2.getTe1End()) ||
    					(cs1.getTe2Start() == cs1.getTe2End() && cs2.getTe1Start() == cs2.getTe1End()))
    			&& !cs1.getTe2().equals(cs2.getTe1())
    			)) return false;
    	if (flags[1] && flags[3] && (
    			((cs1.getTe2Start() != cs1.getTe2End() && cs2.getTe2Start() != cs2.getTe2End()) ||
    			(cs1.getTe2Start() == cs1.getTe2End() && cs2.getTe2Start() == cs2.getTe2End()))
    			&& !cs1.getTe2().equals(cs2.getTe2())
    			)) return false;
		return true;
	}
	
	@Override
	public int compare(CriticalSection cs1, CriticalSection cs2) {   	
    	//Check if the critical sections are comparable.
    	assert(comparable(cs1,cs2));

		boolean[] flags = new boolean[4];
		flags[0] = cs1.getTe1().getRobotID() == this.robotID;
		flags[1] = cs1.getTe2().getRobotID() == this.robotID;
		flags[2] = cs2.getTe1().getRobotID() == this.robotID;
		flags[3] = cs2.getTe2().getRobotID() == this.robotID;
		
    	//find the starting poses 
    	int startInCS1 = flags[0] ? cs1.getTe1Start() : cs1.getTe2Start();
    	int endInCS1 = flags[0] ? cs1.getTe1End() : cs1.getTe2End();
    	int otherIdInCS1 = flags[0] ? cs1.getTe2().getRobotID() : cs1.getTe1().getRobotID();
    	int startInCS2 = flags[2] ? cs2.getTe1Start() : cs2.getTe2Start();
    	int endInCS2 = flags[2] ? cs2.getTe1End() : cs2.getTe2End();
    	int otherIdInCS2 = flags[2] ? cs2.getTe2().getRobotID() : cs2.getTe1().getRobotID();
    	
    	// Return a negative integer, zero, or a positive integer as the first argument is less than, 
    	//equal to, or greater than the second.
    	
    	if (cs1.equals(cs2)) return 0;
    	if (startInCS1 > startInCS2 ||
    			startInCS1 == startInCS2 && (endInCS1 > endInCS2 || endInCS1 == endInCS2 && otherIdInCS1 > otherIdInCS2))
    		return 1;
    	return -1; 	
   } 

}
