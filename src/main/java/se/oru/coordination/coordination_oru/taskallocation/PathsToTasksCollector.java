package se.oru.coordination.coordination_oru.taskallocation;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

public class PathsToTasksCollector {
	HashMap<Integer, PoseSteering[]> paths = new HashMap<Integer, PoseSteering[]>();
	HashMap<PoseSteering[], Double> pathsToPathLength = new HashMap<PoseSteering[], Double>();
	HashMap<PoseSteering[], Double> pathsToArrivalTimes = new HashMap<PoseSteering[], Double>();
	HashMap<PoseSteering[], Double> pathsToTardiness = new HashMap<PoseSteering[], Double>();
	
	public boolean put(int key, PoseSteering[] path, double arrivalTime, double tardiness) {
		if (arrivalTime < 0) {
			System.out.println("Invalid estimated arrival time (should be >= 0)");
			return false;
		}
		if (path == null) {
			System.out.println("Invalid path (null).");
			return false;
		}
		paths.put(key, path);
		double pathLength = 0;
		for (int i = 1; i < path.length; i++) pathLength += path[i].getPose().distanceTo(path[i-1].getPose());
		pathsToPathLength.put(path, pathLength);
		pathsToArrivalTimes.put(path, arrivalTime);
		pathsToTardiness.put(path, tardiness);
		return true;
	}
	
	public void remove(int key) {
		if (!paths.containsKey(key)) return;
		pathsToPathLength.remove(paths.get(key));
		pathsToArrivalTimes.remove(paths.get(key));
		pathsToTardiness.remove(paths.get(key));
		paths.remove(key);
	}
	
	public double getCumulativePathLength() {
		double ret = 0;
		for (PoseSteering[] path : pathsToPathLength.keySet()) ret += pathsToPathLength.get(path);
		return ret;
	}
	
	public double getCumulativeArrivalTimes() {
		double ret = 0;
		for (PoseSteering[] path : pathsToArrivalTimes.keySet()) ret += pathsToArrivalTimes.get(path);
		return ret;
	}
	
	public double getCumulativeTardiness() {
		double ret = 0;
		for (PoseSteering[] path : pathsToTardiness.keySet()) ret += pathsToTardiness.get(path);
		return ret;
	}
	
	public HashMap<Integer, PoseSteering[]> getPaths() {
		return paths;
	}
}
