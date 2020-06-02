package se.oru.coordination.coordination_oru.motionplanning;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Scanner;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.util.Missions;

public class SimpleRoadMapPlanner extends AbstractMotionPlanner {
	
	//FIXME: constructor more generic
	class ScopedDefaultWeightedEdge {
		private String source = null;
		private String target = null;
		private double weight = -1;
		
		public ScopedDefaultWeightedEdge(String source, String target, double weight) {
			this.source = source;
			this.target = target;
			this.weight = weight;
		}
		
		public String getSource() {
			return this.source;
		}
		
		public String getTarget() {
			return this.target;
		}
		
		public double getWeight() {
			return this.weight;
		}
	}
	
	//Graph which vertex set corresponds to the set of locations.
	private SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
	private HashMap<String,Pose> locations = new HashMap<String,Pose>();
	private HashMap<String,PoseSteering[]> paths = new HashMap<String, PoseSteering[]>();
	
	//Sets of removed vertices and edges from the original graph after calling  {@link #addObstacles(Geometry geom, Pose ... poses)}. 
	private HashSet<String> removedVertices = new HashSet<String>();
	private HashMap<DefaultWeightedEdge, ScopedDefaultWeightedEdge> removedEdges = new HashMap<DefaultWeightedEdge, ScopedDefaultWeightedEdge>();

	public SimpleRoadMapPlanner(){
		super();
	};
	
	/**
	 * Class constructor with path re-sampling.
	 * @param fileName The directory or file to load the roadmap from (see {@link #loadRoadMap}).
	 * @param distanceBetweenPathPoints The minimum acceptable distance between path poses or -1 if any value is ok 
	 * 									(re-sampling is not performed in this case); see {@link #loadRoadMap}.
	 * FIXME: Maximum accettable distance?
	 * @param footprint The robot footprint.
	 */
	public SimpleRoadMapPlanner(String fileName, double distanceBetweenPathPoints, Coordinate ... footprint) {
		if (footprint == null) throw new Error("Provide the robot footprint!!"); //FIXME inscribed/circunscribed radius
		this.footprintCoords = footprint;
		loadRoadMap(fileName, distanceBetweenPathPoints);
	}
	
	/**
	 * Class constructor without path re-sampling.
	 * @param fileName The directory or file to load the roadmap from (see {@link #loadRoadMap}).
	 * @param footprint The robot footprint.
	 */
	public SimpleRoadMapPlanner(String fileName, Coordinate ... footprint) {
		this(fileName,-1,footprint);
	}
	
	/**
	 * Load a roadmap stored in a give directory or file, eventually re-sampling paths at a given step).
	 * @param fileName The directory or file to load the roadmap from.
	 * @param distanceBetweenPathPoints The minimum acceptable distance between path poses or -1 if any value is ok 
	 * 									(re-sampling is not performed in this case).
	 * If a directory is given, the filename is assumed to he "roadmap.txt".
	 */
	public void loadRoadMap(String fileName, double distanceBetweenPathPoints) {
		try {
			String fileOnly;
			String pathOnly;
			File f = new File(fileName);
			if (f.isDirectory()) {
				pathOnly = fileName;
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
						if (distanceBetweenPathPoints > 0) knownPath = resamplePath(knownPath,distanceBetweenPathPoints);
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
		
	//Build the graph
	private void buildGraphs() {
		this.graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		
		metaCSPLogger.info("Updating the roadmap with the given graph ...");
		
		for (String v : locations.keySet()) {
			this.graph.addVertex(v);
			metaCSPLogger.info("Added vertex " + v + ".");
		}
		
		for (String from : locations.keySet()) {
			for (String to : locations.keySet()) {
				if (!from.equals(to)) {
					if (paths.containsKey(from+"->"+to)) {
						PoseSteering[] path = paths.get(from+"->"+to);
						DefaultWeightedEdge e = addEdge(graph, from, to, (double)path.length);
						metaCSPLogger.info("Added edge " + e);
					}
				}
			}	
		}
	}
		
	/**
	 * Re-sample all paths so that the minimum distance between path poses is the given value.
	 * @param distanceBetweenPathPoints The minimum acceptable distance between path poses.
	 */
	public void resamplePathsInRoadMap(double distanceBetweenPathPoints) {
		for (String pathname : paths.keySet()) {
			paths.put(pathname, resamplePath(paths.get(pathname),distanceBetweenPathPoints));
		}
	}
	
	private PoseSteering[] resamplePath(PoseSteering[] path, double distanceBetweenPathPoints) {
		if (distanceBetweenPathPoints < 0) return path;
		ArrayList<PoseSteering> ret = new ArrayList<PoseSteering>();
		PoseSteering lastAdded = path[0];
		ret.add(lastAdded);
		for (int i = 1; i < path.length; i++) {
			Coordinate p1 = lastAdded.getPose().getPosition();
			Coordinate p2 = path[i].getPose().getPosition();
			//FIXME The function does not consider the rotation!!
			if (p2.distance(p1) > distanceBetweenPathPoints) {
				lastAdded = path[i];
				ret.add(path[i]);
			}
		}
		//Add the goal if not already added
		if (!ret.get(ret.size()-1).equals(path[path.length-1])) ret.add(path[path.length-1]);
		return ret.toArray(new PoseSteering[ret.size()]);
	}
		
	private DefaultWeightedEdge addEdge(SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph, String source, String target, double weight) {
		graph.addVertex(source);
		graph.addVertex(target);
		DefaultWeightedEdge e = graph.addEdge(source,target);
		graph.setEdgeWeight(e, weight);
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
	}
	
	/**
	 * Add a location to the roadmap.
	 * @param locationName The name of the location.
	 * @param pose The pose of the location.
	 */
	public void addLocationToRoadMap(String locationName, Pose pose) {
		locations.put(locationName, pose);
		graph.addVertex(locationName);
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
	
	//FIXME this is the simplest but the silliest way to do planning.
	public boolean doPlanning() {
		//Check that at least one start and one goal have been provided
		if (this.start == null) throw new Error("Start is not specified!");
		if (this.goal.length == 0) throw new Error("Goal is not specified!");
		
		//Compute the shortest path between start and goals.
		String[] locations = new String[1+this.goal.length];
		
		//FIXME More then one path may contain the given starting location, so it should be given also the old path name.
		locations[0] = computeLocationOnRoadmap(this.start); //FIXME here we should do rewiring
		
		//FIXME at least the last goal should be already a location in the roadmap (under the assubption that robot follows the path).
		//However, it may be that the coordinator does not keep track of intermediate goals (CHECK AND EVENTUALLY ADD!!).
		for (int i = 0; i < this.goal.length; i++) locations[i] = computeLocationOnRoadmap(this.goal[i]);
		
		this.pathPS = getShortestPath(locations);
		return this.pathPS != null;
	}
	
	/**
	 * Return the name of the location in the roadmap which is closest to the given pose.
	 * @param pose The pose of interest.
	 */
	public String computeLocationOnRoadmap(Pose pose) {
		//FIXME The closest location should be accessible. Add a check.
		String closestLocation = null;
		double minDistance = Double.MAX_VALUE;
		for (String location : locations.keySet()) {
			double distance = pose.distanceTo(locations.get(location));
			if (distance < minDistance) {
				minDistance = distance;
				closestLocation = location;
			}
		}
		return closestLocation;
	}
	
	
	/**
	 * Return the name of the locations in the roadmap which is closest to the given poses.
	 * @param poses The interesting poses.
	 */
	public String[] computeLocationsOnRoadmap(Pose ... poses) {
		String[] ret = new String[poses.length];
		for (int i = 0; i < poses.length; i++) ret[i] = computeLocationOnRoadmap(poses[i]);
		return ret;
	}
	
	
	/**
	 * Get the shortest path connecting given locations (two or more). The path between successive pairs of locations
	 * is computed with Dijkstra's algorithm, where edge weights are path lengths.
	 * @param locations At least two location names.
	 * @return The shortest path connecting given locations.
	 */
	public PoseSteering[] getShortestPath(String[] locations) {
		if (locations.length < 2) throw new Error("Please provide at least two locations for path extraction!");
		DijkstraShortestPath<String, DefaultWeightedEdge> dijkstraShortestPath = new DijkstraShortestPath<String, DefaultWeightedEdge>(graph);
		ArrayList<PoseSteering> overallShortestPath = new ArrayList<PoseSteering>();
		for (int k = 0; k < locations.length-1; k++) {
		    GraphPath<String, DefaultWeightedEdge> gp = dijkstraShortestPath.getPath(locations[k], locations[k+1]);			
		    if (gp == null) return null;
		    List<String> oneShortestPath = gp.getVertexList();
		    ArrayList<PoseSteering> allPoses = new ArrayList<PoseSteering>();
		    for (int i = 0; i < oneShortestPath.size()-1; i++) {
		    	//PoseSteering[] onePath = loadKnownPath(oneShortestPath.get(i),oneShortestPath.get(i+1));
		    	PoseSteering[] onePath = paths.get(oneShortestPath.get(i)+"->"+oneShortestPath.get(i+1));
		    	if (i == 0) allPoses.add(onePath[0]);
		    	for (int j = 1; j < onePath.length-1; j++) {
		    		allPoses.add(onePath[j]);
		    	}
		    	if (i == oneShortestPath.size()-2) allPoses.add(onePath[onePath.length-1]);
		    }
		    if (k == 0) overallShortestPath.add(allPoses.get(0));
		    for (int i = 1; i < allPoses.size(); i++) {
		    	overallShortestPath.add(allPoses.get(i));
		    }
		}
		return overallShortestPath.toArray(new PoseSteering[overallShortestPath.size()]);
	}

	@Override
	public AbstractMotionPlanner getCopy() {
		SimpleRoadMapPlanner ret = new SimpleRoadMapPlanner();
		ret.setFootprint(this.footprintCoords);
		ret.om = this.om;
		ret.locations.putAll(this.locations);
		ret.paths.putAll(this.paths);
		ret.removedVertices.addAll(this.removedVertices);
		ret.removedEdges.putAll(this.removedEdges);
		ret.graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		for (String v : this.graph.vertexSet()) ret.graph.addVertex(v);
		for (DefaultWeightedEdge e : this.graph.edgeSet()) {
			DefaultWeightedEdge e1 = ret.graph.addEdge(this.graph.getEdgeSource(e),this.graph.getEdgeTarget(e));
			ret.graph.setEdgeWeight(e1, this.graph.getEdgeWeight(e));
		}
		return ret;
	}
	
	@Override
	public synchronized void addObstacles(Geometry geom, Pose ... poses) {
		if (this.footprintCoords.length == 0) throw new Error("Robot footprint is not set!");
		
		//Add the obstacles to the occupancy grid
		if (this.om == null) this.om = new OccupancyMap(1000, 1000, 0.01);
		ArrayList<Geometry> obstacles = this.om.addObstacles(geom, poses);
		
		//Find and remove all vertices which are not collision free.
		for (String loc : locations.keySet()) {
			Geometry robotInPose = makeObstacle(locations.get(loc));
			for (Geometry obstacle : obstacles) {
				if (robotInPose.intersects(obstacle)) removedVertices.add(loc);
			}
		}
		this.graph.removeAllVertices(removedVertices);
		
		//Find and remove all edges which are not collision free.
		for (String pathName : paths.keySet()) {
			String startLoc = pathName.substring(0,pathName.indexOf("-"));
			String targetLoc = pathName.substring(pathName.indexOf(">")+1,pathName.length());
			if (this.graph.containsVertex(startLoc) && this.graph.containsVertex(targetLoc)) {
				PoseSteering[] path = paths.get(pathName);
				for (PoseSteering pose : path) {
					Geometry robotInPose = makeObstacle(new Pose(pose.getX(),pose.getY(),pose.getTheta()));
					for (Geometry obstacle : obstacles) {
						if (robotInPose.intersects(obstacle)) {
							for (DefaultWeightedEdge e : graph.getAllEdges(startLoc, targetLoc))
								removedEdges.put(e, new ScopedDefaultWeightedEdge(startLoc, targetLoc, graph.getEdgeWeight(e)));
							break;
						}
					}
				}
			}
		}
		this.graph.removeAllEdges(removedEdges.keySet());
		
	}
	
	private Geometry makeObstacle(Pose pose) {
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newFoot = new Coordinate[this.footprintCoords.length+1];
		for (int j = 0; j < this.footprintCoords.length; j++) {
			newFoot[j] = this.footprintCoords[j];
		}
		newFoot[this.footprintCoords.length] = this.footprintCoords[0];
		Geometry obstacle = gf.createPolygon(newFoot);
		AffineTransformation at = new AffineTransformation();
		at.rotate(pose.getTheta());
		at.translate(pose.getX(), pose.getY());
		return at.transform(obstacle);
	}
	
	@Override
	public synchronized void clearObstacles() {
		
		if (this.noMap) this.om = null;
		else this.om.clearObstacles();
		
		//Restore the original graph
		if (removedVertices.isEmpty() && removedEdges.isEmpty()) return;
		this.graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		
		//Tracking changes
		for (String v : removedVertices) this.graph.addVertex(v);
		for (DefaultWeightedEdge e : removedEdges.keySet()) {
			DefaultWeightedEdge e1 = this.graph.addEdge(removedEdges.get(e).getSource(),removedEdges.get(e).getTarget());
			this.graph.setEdgeWeight(e1, removedEdges.get(e).getWeight());
		}
		removedVertices.clear();
		removedEdges.clear();
	}

}
