package SimpleRoadMapPlanner;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.motionplanning.OccupancyMap;
import se.oru.coordination.coordination_oru.util.Missions;

public class SimpleRoadMapPlanner extends AbstractMotionPlanner {
	protected SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph_original = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class); 
	protected SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
	protected HashMap<String,Pose> locations = new HashMap<String,Pose>();
	protected HashMap<String,PoseSteering[]> paths = new HashMap<String, PoseSteering[]>();
	private double distanceBetweenPathPoints = 0.5;
	//private double planningTimeInSecs = 30.0;
	//private Coordinate[] collisionCircleCenters = null;
	//private PLANNING_ALGORITHM algo = PLANNING_ALGORITHM.RRTConnect;
	
	//FIXME: the function is equal to the loadRoadMap in the Missions class.
	public void loadRoadMap(String path) {
		try {
			String fileOnly;
			String pathOnly;
			File f = new File(path);
			if (f.isDirectory()) {
				pathOnly = path;
				if (!pathOnly.endsWith(File.separator)) pathOnly += File.separator;
				fileOnly = "roadmap.txt";
			}
			else {
				pathOnly = f.getAbsolutePath().substring(0,f.getAbsolutePath().lastIndexOf(File.separator))+File.separator;
				fileOnly = f.getAbsolutePath().substring(f.getAbsolutePath().lastIndexOf(File.separator)+1);
			}
			Scanner in = new Scanner(new FileReader(pathOnly+fileOnly));
			
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0 && !line.startsWith("#")) {
					String[] oneline = line.split(" |\t");
					Pose ps = null;
					if (line.contains("->")) {
						PoseSteering[] knownPath = Missions.loadPathFromFile(pathOnly+oneline[3]);
						paths.put(oneline[0]+oneline[1]+oneline[2], knownPath);
					}
					else {
						ArrayList<String> newLineContents = new ArrayList<String>();
						for (int i = 0; i < oneline.length; i++) {
							if (!oneline[i].trim().equals("")) newLineContents.add(oneline[i].trim()); 
						}
						String locationName = newLineContents.get(0);
						ps = new Pose(
								Double.parseDouble(newLineContents.get(1)),
								Double.parseDouble(newLineContents.get(2)),
								Double.parseDouble(newLineContents.get(3)));
						locations.put(locationName, ps);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { 
			e.printStackTrace(); 
			throw new Error("Unable to load the required scenario: " + e.toString());
		}
		//Build the graph
		buildGraphs();
	}
	
	//FIXME: the function is equal to the buildGraphs in the Missions class.
	public void buildGraphs() {
		this.graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		this.graph_original = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		
		metaCSPLogger.info("Updating the roadmap with the given graph ...");
		
		for (String v : locations.keySet()) {
			this.graph.addVertex(v);
			this.graph_original.addVertex(v);
			metaCSPLogger.info("Added vertex " + v + ".");
		}
		
		for (String from : locations.keySet()) {
			for (String to : locations.keySet()) {
				if (!from.equals(to)) {
					if (paths.containsKey(from+"->"+to)) {
						PoseSteering[] path = paths.get(from+"->"+to);
						DefaultWeightedEdge e = addEdge(graph, from, to, (double)path.length);
						addEdge(graph_original, from, to, (double)path.length);
						metaCSPLogger.info("Added edge " + e);
					}
				}
			}	
		}
	}
		
	public void setLocationsPathsAndGraphs(HashMap<String, Pose> locations, HashMap<String,PoseSteering[]> paths) {
		this.locations.putAll(locations);
		this.paths.putAll(paths);
		buildGraphs();
	}
	
	public void setDistanceBetweenPathPoints(double distanceBetweenPathPoints) {
		this.distanceBetweenPathPoints = distanceBetweenPathPoints;
	}
	
	protected DefaultWeightedEdge addEdge(SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph2, String source, String target, double weight) {
		graph2.addVertex(source);
		graph2.addVertex(target);
		DefaultWeightedEdge e = graph2.addEdge(source,target);
		graph2.setEdgeWeight(e, weight);
		return e;
	}
		
	/**
	 * Add a path to the current roadmap.
	 * @param source The starting location of the path.
	 * @param target The goal location of the path.
	 * @param path The path to add.
	 */
	public void addPathToRoadMap(String source, String target, PoseSteering[] path) {
		if (!locations.containsKey(source) || !locations.containsKey(target) || path == null || path.length == 0) throw new Error("Locations unknown or path is invalid (" + (source+"->"+target) + ")!");
		paths.put(source+"->"+goal, path);
		addEdge(graph, source, target, (double)path.length);
		addEdge(graph_original, source, target, (double)path.length);
	}
	
	/**
	 * Remove a named location from the roadmap.
	 * @param locationName The name of the location to remove.
	 */
	public void removeLocationFromRoadMap(String locationName) {
		locations.remove(locationName);
		ArrayList<String> toRemove = new ArrayList<String>();
		for (String key : paths.keySet()) {
			if (key.substring(0,key.indexOf("->")).equals(locationName)) toRemove.add(key);
			else if (key.substring(key.indexOf("->")).equals(locationName)) toRemove.add(key);
		}
		for (String key : toRemove) paths.remove(key);
		if (graph != null) graph.removeVertex(locationName);
		if (graph_original != null) graph_original.removeVertex(locationName);
	}
	
	/**
	 * Add a location to the roadmap.
	 * @param locationName The name of the location.
	 * @param pose The pose of the location.
	 */
	public void addLocationToRoadMap(String locationName, Pose pose) {
		locations.put(locationName, pose);
		graph.addVertex(locationName);
		graph_original.addVertex(locationName);
	}
		
	/**
	 * Get the pose of a given named location in the roadmap.
	 * @param name The name of the location to get the pose of.
	 * @return The {@link Pose} of the location
	 */
	public Pose getLocationPose(String name) {
		Pose ret = locations.get(name);
		if (ret == null) throw new Error("Unknown location " + name);
		return ret;
	}
	
	//TODO
	public boolean doPlanning() {
		return false;
	}

	@Override
	public AbstractMotionPlanner getCopy() {
		//FIXME
		SimpleRoadMapPlanner ret = new SimpleRoadMapPlanner();
		ret.setLocationsPathsAndGraphs(this.locations,this.paths);
		ret.setFootprint(this.footprintCoords);
		ret.om = this.om;
		return ret;
	}
	
	@Override
	public synchronized void addObstacles(Geometry geom, Pose ... poses) {
		//TODO
		if (this.om == null) this.om = new OccupancyMap(1000, 1000, 0.01);
		this.om.addObstacles(geom, poses);
	}

}
