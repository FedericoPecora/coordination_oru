package se.oru.coordination.coordination_oru;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.metacsp.utility.logging.MetaCSPLogging;

/**
 * This class provides an ordered collection of all the current active critical sections computed by the {@link TrajectoryEnvelopeCoordinator}. 
 * TODO An instantiatable {@link CriticalSectionsTreeSets} must provide a criteria with which robots are to be prioritized (FIXME the closest is used for now.).
 * FIXME The best idea is to have this map of comparators inside the trajectory envelope coordinator to ensure consistency.
 * @author anna
 */
public class CriticalSectionsTreeSets {
	protected TrajectoryEnvelopeCoordinator tec;
	protected HashMap<Integer, HashMap<Integer,TreeSet<CriticalSection>>> map = new HashMap<Integer, HashMap<Integer,TreeSet<CriticalSection>>>();
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(CriticalSectionsTreeSets.class);
	protected String logDirName = null;
	/*protected Comparator<CriticalSection> defaultComparator = new Comparator<CriticalSection>() {
			@Override
			public int compare(CriticalSection cs1, CriticalSection cs2) {	    	
		    	// Return a negative integer, zero, or a positive integer as the first argument is less than, 
		    	//equal to, or greater than the second.
			}
	};*/
	//protected HashMap<Integer,CriticalSectionComparator> comparators = new HashMap<Integer,CriticalSectionComparator>;
	
	protected void setupLogging() {
		logDirName = "logs";
		File dir = new File(logDirName);
		dir.mkdir();
		MetaCSPLogging.setLogDir(logDirName);
	}
	
	/** TODO
	 * Add a criterion for determining the order of robots through critical sections
	 * (comparator of {@link AbstractTrajectoryEnvelopeTracker}s). 
	 * Comparators are considered in the order in which they are added.
	 * @param c A new comparator for determining robot ordering through critical sections.
	 *
	public void addComparator(Comparator<CriticalSection> c) {
		this.comparators.addComparator(c);
	}
	*/
	
	public CriticalSectionsTreeSets(TrajectoryEnvelopeCoordinator tec) {
		this.tec = tec;
	}
	
	boolean check(CriticalSection cs) {
		int firstID = cs.getTe1().getRobotID();
		int secondID = cs.getTe2().getRobotID();
		if (!(cs.getTe1().getRobotID() == tec.getCurrentTrajectoryEnvelope(firstID).getID() ||
				cs.getTe1().getRobotID() == tec.getCurrentParkingEnvelope(firstID).getID()) || 
				//and for the second robot
				!(cs.getTe2().getRobotID() == tec.getCurrentTrajectoryEnvelope(secondID).getID() ||
				cs.getTe2().getRobotID() == tec.getCurrentParkingEnvelope(secondID).getID())	
				) {
			metaCSPLogger.severe("Critical section " + cs.toString() + " cannot be added since it is not coherent with current trajectry envelopes.");
			return false;
		}
		return true;
	}
	
	/**
	 * Add a critical section to the ordered map.
	 * @param cs The critical section to be add.
	 */
	public void add(CriticalSection cs) {
		//FIXME Add more checks.
		//Decide when cleaning up.
		
		int firstID = cs.getTe1().getRobotID();
		int secondID = cs.getTe2().getRobotID();
		
		//Check trajectory envelopes 
		if (!check(cs)) return;
			
		//Add the critical section to the first robot, ...
		if (!map.containsKey(firstID)) {
			map.put(firstID, new HashMap<Integer, TreeSet<CriticalSection>>());
			CriticalSectionsComparator comparator = new CriticalSectionsComparator(firstID,tec.getCurrentTrajectoryEnvelope(firstID).getID(),tec.getCurrentParkingEnvelope(firstID).getID());
			map.get(firstID).put(secondID, new TreeSet<CriticalSection>(comparator));
		}	
		if (!map.get(firstID).get(secondID).add(cs)) metaCSPLogger.severe("Failed adding critical section " + cs.toString() + ".");
		
		//and to the second.
		if (!map.containsKey(secondID)) {
			map.put(secondID, new HashMap<Integer, TreeSet<CriticalSection>>());
			CriticalSectionsComparator comparator = new CriticalSectionsComparator(secondID,tec.getCurrentTrajectoryEnvelope(secondID).getID(),tec.getCurrentParkingEnvelope(secondID).getID());
			map.get(secondID).put(firstID, new TreeSet<CriticalSection>(comparator));
		}
		if (!map.get(secondID).get(firstID).add(cs)) metaCSPLogger.severe("Failed adding critical section " + cs.toString() + ".");
	}
	
	/**
	 * Remove a critical section if currently stored.
	 * @param cs The critical section to be deleted.
	 */
	public void remove(CriticalSection cs) {
		int firstID = cs.getTe1().getRobotID();
		int secondID = cs.getTe2().getRobotID();
		if (map.containsKey(firstID) && map.get(firstID).containsKey(secondID)) map.get(firstID).get(secondID).remove(cs);
		if (map.containsKey(secondID) && map.get(secondID).containsKey(firstID)) map.get(secondID).get(firstID).remove(cs);		
	}
	
	/**
	 * Method to be called before adding new critical sections whenever the {@link TrajectoryEnvelopeCoordinator} of a robot is updated.
	 * @param robotID The ID of the robot which {@link TrajectoryEnvelopeCoordinator} has been updated.
	 */
	public void clearRobot(int robotID) {
		if (!map.containsKey(robotID)) return;
		for (Integer otherID : map.get(robotID).keySet()) {
			if (map.containsKey(otherID)) map.get(otherID).remove(robotID);
		}
		map.remove(robotID);
	}
	
	/**
	 * Return the first critical section for one robot (firstID) with respect to another one (secondID).
	 * @NOTE If old critical sections are deleted while the robot is in motion, the closest critical 
	 * section closest to the first robot involving the second robot is returned.
	 * @param firstID ID of the first robot.
	 * @param second ID id of the second robot.
	 */
	public CriticalSection next(int firstID, int secondID) {
		if (!map.containsKey(firstID) || !map.containsKey(secondID) || 
				!(map.containsKey(firstID) && (!map.get(firstID).containsKey(secondID) || map.get(firstID).containsKey(secondID) && map.get(firstID).get(secondID).isEmpty())) ||
				!(map.containsKey(secondID) && (!map.get(secondID).containsKey(firstID) || map.get(secondID).containsKey(firstID) && map.get(secondID).get(firstID).isEmpty()))) {
			metaCSPLogger.info("No critical section between Robot" + firstID + " and " + secondID + ".");
			return null; 
		}
		return map.get(firstID).get(secondID).first();
	}	
}
