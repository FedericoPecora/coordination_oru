package se.oru.coordination.coordination_oru;

import java.io.File;
import java.util.Comparator;
import java.util.HashMap;
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
	
	public void add(CriticalSection cs) {
		int firstID = cs.getTe1().getRobotID();
		int secondID = cs.getTe2().getRobotID();
		
		//Add for robot 1, ...
		if (!map.containsKey(firstID)) {
			map.put(firstID, new HashMap<Integer, TreeSet<CriticalSection>>());
			CriticalSectionsComparator comparator = new CriticalSectionsComparator(firstID,tec.getCurrentTrajectoryEnvelope(firstID).getID(),tec.getCurrentParkingEnvelope(firstID).getID());
			map.get(firstID).put(secondID, new TreeSet<CriticalSection>(comparator));
		}
		if (!map.get(firstID).get(secondID).add(cs)) metaCSPLogger.severe("Failed adding critical section " + cs.toString() + ".");
		
		//and for robot 2
		if (!map.containsKey(secondID)) {
			map.put(secondID, new HashMap<Integer, TreeSet<CriticalSection>>());
			CriticalSectionsComparator comparator = new CriticalSectionsComparator(secondID,tec.getCurrentTrajectoryEnvelope(secondID).getID(),tec.getCurrentParkingEnvelope(secondID).getID());
			map.get(secondID).put(firstID, new TreeSet<CriticalSection>(comparator));
		}
		if (!map.get(secondID).get(firstID).add(cs)) metaCSPLogger.severe("Failed adding critical section " + cs.toString() + ".");
	}
	
	public void remove(CriticalSection cs) {
		int firstID = cs.getTe1().getRobotID();
		int secondID = cs.getTe2().getRobotID();
		if (map.containsKey(firstID) && map.get(firstID).containsKey(secondID)) map.get(firstID).get(secondID).remove(cs);
		if (map.containsKey(secondID) && map.get(secondID).containsKey(firstID)) map.get(secondID).get(firstID).remove(cs);		
	}
	
	public CriticalSection next(int firstID, int secondID) {
		if (!map.containsKey(firstID) || !map.containsKey(secondID) || 
				!(map.containsKey(firstID) && (!map.get(firstID).containsKey(secondID) || map.get(firstID).containsKey(secondID) && map.get(firstID).get(secondID).isEmpty())) ||
				!(map.containsKey(secondID) && (!map.get(secondID).containsKey(firstID) || map.get(secondID).containsKey(firstID) && map.get(secondID).get(firstID).isEmpty()))) {
			metaCSPLogger.info("No critical section between Robot" + firstID + " and " + secondID + ".");
			return null; 
		}
		return map.get(firstID).get(secondID).first();
	}
	
	public CriticalSection next(int robotID) {
		if (!map.containsKey(robotID) || map.containsKey(robotID) && map.get(robotID).isEmpty()) {
			metaCSPLogger.info("The current trajectory envelope of Robot" + robotID + " is not intersecting any other trajectory envelope.");
			return null; 
		}
		CriticalSectionsComparator comparator = new CriticalSectionsComparator(robotID,tec.getCurrentTrajectoryEnvelope(robotID).getID(),tec.getCurrentParkingEnvelope(robotID).getID());
		CriticalSection cs = map.get(robotID).get(0).first();
		for (int other = 1; other < map.get(robotID).keySet().size(); other++) {
			CriticalSection otherCS = map.get(robotID).get(other).first();
			if (comparator.compare(cs, otherCS) < 0) cs = otherCS;
			//if (this.comparators.get(robotID).compare(cs, otherCS) < 0) cs = otherCS;
		}
		return cs;
	}
	
}
