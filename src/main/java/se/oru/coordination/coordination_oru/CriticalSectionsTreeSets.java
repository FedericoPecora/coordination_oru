package se.oru.coordination.coordination_oru;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
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
 * @author anna
 */
public class CriticalSectionsTreeSets {
	protected AbstractTrajectoryEnvelopeCoordinator tec;
	protected HashMap<Integer, HashMap<Integer,TreeSet<CriticalSection>>> map = new HashMap<Integer, HashMap<Integer,TreeSet<CriticalSection>>>();
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(CriticalSectionsTreeSets.class);
	protected String logDirName = null;
		
	public CriticalSectionsTreeSets(AbstractTrajectoryEnvelopeCoordinator tec) {
		this.tec = tec;
	}
	
	/**
	 * Setup the file to be used for logging data.
	 */
	protected void setupLogging() {
		logDirName = "logs";
		File dir = new File(logDirName);
		dir.mkdir();
		MetaCSPLogging.setLogDir(logDirName);
	}
	
	/** Check if the critical section is related to up-to-date trajectory envelope.
	 * @return False if the two parking and the two driving envelopes involved are not up-to-date. Otherwise, true.
	 */
	protected boolean check(CriticalSection cs) {
		if (!(checkOneRobot(cs, true) && checkOneRobot(cs, false))) {
			metaCSPLogger.severe("Critical section " + cs.toString() + " can't be added (not coherent with current trajectory envelopes).");
			return false;
		}
		return true;
	}
	
	/** Check if the critical section is related to up-to-date trajectory envelopes for one of the robot.
	 * @param firstRobot True if the first robot is checked, otherwise false.
	 * @param cs The critical section to be checked.
	 * @return False if the parking and the driving envelopes of the robot are not up-to-date. Otherwise, true.
	 */
	protected boolean checkOneRobot(CriticalSection cs, boolean firstRobot) {
		int robotID = firstRobot ? cs.getTe1().getRobotID() : cs.getTe2().getRobotID();
		int teID = firstRobot ? cs.getTe1().getID() : cs.getTe2().getID();
		if (!(teID == tec.getCurrentTrajectoryEnvelope(robotID).getID() || //check if its driving envelope is up to date 
				teID == tec.getCurrentParkingEnvelope(robotID).getID())    //and if its parking envelope is up to date.
			) {
			return false;
		}
		return true;
	}
	
	/**
	 * Check it the TreeSets involving the pair of robots (firstID, secondID) are up-to-date. Clear them if not.
	 * @param firstID ID of the first robot.
	 * @param secondID ID of the second robot.
	 */
	protected void updateTreeSets(int firstID, int secondID) {
		if (map.containsKey(firstID) && map.containsKey(secondID)) {
			if (map.get(firstID).containsKey(secondID) && map.get(secondID).containsKey(firstID)) {
				CriticalSection firstCS = map.get(firstID).get(secondID).first();
				if (firstCS != null && !check(firstCS)) {
					metaCSPLogger.info("Clearing the TreeSets for robots ("+ firstID +", " + secondID + ").");
					map.get(firstID).remove(secondID);
					if (map.get(firstID).isEmpty()) map.remove(firstID);
					map.get(secondID).remove(firstID);
					if (map.get(secondID).isEmpty()) map.remove(secondID);
				}
			}
		}
	}
	
	/**
	 * Add a critical section to the ordered map.
	 * @param cs The critical section to be add.
	 */
	public void add(CriticalSection cs) {
		int firstID = cs.getTe1().getRobotID();
		int secondID = cs.getTe2().getRobotID();
		
		//Check trajectory envelopes 
		if (!check(cs)) return;
		
		//Clear old information
		updateTreeSets(firstID, secondID);
			
		//Add the critical section to the first robot, ...
		if (!map.containsKey(firstID)) map.put(firstID, new HashMap<Integer, TreeSet<CriticalSection>>());
		if (!map.get(firstID).containsKey(secondID)) {
			CriticalSectionsComparator comparator = new CriticalSectionsComparator(firstID);
			map.get(firstID).put(secondID, new TreeSet<CriticalSection>(comparator));
		}
		if (!map.get(firstID).get(secondID).add(cs)) metaCSPLogger.severe("Failed adding critical section " + cs.toString() + ".");
		
		//and to the second.
		if (!map.containsKey(secondID)) map.put(secondID, new HashMap<Integer, TreeSet<CriticalSection>>());
		if (!map.get(secondID).containsKey(firstID)) {
			CriticalSectionsComparator comparator = new CriticalSectionsComparator(secondID);
			map.get(secondID).put(firstID, new TreeSet<CriticalSection>(comparator));
		}
		if (!map.get(secondID).get(firstID).add(cs)) metaCSPLogger.severe("Failed adding critical section " + cs.toString() + ".");
	}
	
	/** Add all the critical sections in the given set.
	 * @param allCS The critical sections to be added.
	 */
	public void addAll(HashSet<CriticalSection> allCS) {
		metaCSPLogger.finest("Adding " + allCS.size() + " critical sections.");
		for (CriticalSection cs : allCS) add(cs);
	}
	
	/**
	 * Remove a critical section if currently stored.
	 * @param cs The critical section to be deleted.
	 */
	public void remove(CriticalSection cs) {
		remove(cs, true);
		remove(cs, false);
	}
	
	/**
	 * Remove a critical section for a robot if currently stored.
	 * @param cs The critical section to be deleted.
	 * @param firstRobot True if the robot for which the critical section should be deleted is the first one (related to te1), otherwise false.
	 */
	public void remove(CriticalSection cs, boolean firstRobot) {
		int robotID = firstRobot ? cs.getTe1().getRobotID() : cs.getTe2().getRobotID();
		int otherID = firstRobot ? cs.getTe2().getRobotID() : cs.getTe1().getRobotID();
		if (map.containsKey(robotID) && map.get(robotID).containsKey(otherID)) map.get(robotID).get(otherID).remove(cs);
		if (map.get(robotID).get(otherID).isEmpty()) map.get(robotID).remove(otherID);	
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
	 * @NOTE If old critical sections are deleted while the robot is in motion, the critical section closest to the first robot involving the second robot is returned.
	 * If also critical sections are deleted when the robot can't stop anymore before entering it, the returned critical section is the closest one for which the precedence can be decided according to the heuristic.
	 * @param firstID ID of the first robot.
	 * @param secondID ID of the second robot.
	 * @return The first critical section according to the ordering criteria.
	 */
	public CriticalSection next(int firstID, int secondID) {
		//Clear old information
		updateTreeSets(firstID, secondID);
		
		if (!map.containsKey(firstID) || !map.containsKey(secondID) || 
				(map.containsKey(firstID) && (!map.get(firstID).containsKey(secondID) || map.get(firstID).containsKey(secondID) && map.get(firstID).get(secondID).isEmpty())) ||
				(map.containsKey(secondID) && (!map.get(secondID).containsKey(firstID) || map.get(secondID).containsKey(firstID) && map.get(secondID).get(firstID).isEmpty()))) {
			metaCSPLogger.info("No critical section between Robot" + firstID + " and " + secondID + ".");
			return null; 
		}
		return map.get(firstID).get(secondID).first();
	}

	/**
	 * Clear the current map.
	 */
	public void clear() {
		map.clear();		
	}	
}
