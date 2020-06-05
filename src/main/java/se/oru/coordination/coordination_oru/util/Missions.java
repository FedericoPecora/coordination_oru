package se.oru.coordination.coordination_oru.util;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Scanner;
import java.util.Set;
import java.util.logging.Logger;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

import javax.imageio.ImageIO;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;

/**
 * This class collects utility methods for storing {@link Mission}s, regulating their dispatch, maintaining locations
 * and paths (i.e., a roadmap), finding shortest paths through the roadmap, and extracting information from YAML files.
 * 
 * @author fpa
 *
 */
public class Missions {

	protected static HashMap<String,Pose> locations = new HashMap<String, Pose>();
	protected static HashMap<String,PoseSteering[]> paths = new HashMap<String, PoseSteering[]>();
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(Missions.class);
	protected static HashMap<Integer,ArrayList<Mission>> missions = new HashMap<Integer, ArrayList<Mission>>();
	protected static HashMap<Integer,Boolean> missionDispatcherFlags = new HashMap<Integer,Boolean>();
	protected static HashMap<Integer,MissionDispatchingCallback> mdcs = new HashMap<Integer, MissionDispatchingCallback>();
	protected static HashMap<Mission,ArrayList<Mission>> concatenatedMissions = new HashMap<Mission, ArrayList<Mission>>();
	//protected static String pathPrefix = "";
	protected static SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class); 
	protected static String mapYAML = null;
	protected static String mapImageFilename = null;
	protected static BufferedImage map = null;
	protected static double mapResolution = -1;
	protected static Coordinate mapOrigin = null;

	protected static double minPathDistance = -1;
	
	/**
	 * Set the minimum acceptable distance between path poses. This is used to re-sample paths
	 * when they are loaded from file or when the method {@link #resamplePathsInRoadMap()} is called.
	 * @param minPathDist The minimum acceptable distance between path poses.
	 */
	public static void setMinPathDistance(double minPathDist) {
		minPathDistance = minPathDist;
	}
	
	/**
	 * Re-sample all paths so that the minimum distance between path poses is the value
	 * set by {@link #setMinPathDistance(double)}.
	 */
	public static void resamplePathsInRoadMap() {
		for (String pathname : paths.keySet()) {
			paths.put(pathname, resamplePath(paths.get(pathname)));
		}
	}
	
	private static PoseSteering[] resamplePath(PoseSteering[] path) {
		if (minPathDistance < 0) return path;
		ArrayList<PoseSteering> ret = new ArrayList<PoseSteering>();
		PoseSteering lastAdded = path[0];
		ret.add(lastAdded);
		for (int i = 1; i < path.length; i++) {
			Coordinate p1 = lastAdded.getPose().getPosition();
			Coordinate p2 = path[i].getPose().getPosition();
			if (p2.distance(p1) > minPathDistance) {
				lastAdded = path[i];
				ret.add(path[i]);
			}
		}
		return ret.toArray(new PoseSteering[ret.size()]);
	}

	/**
	 * Get the length in meters of a path. This is the sum of distances between path poses.
	 * @return The length in meters of a path. This is the sum of distances between path poses.
	 */
	public static double getPathLength(PoseSteering[] path) {
		double length = 0.0;
		for (int i = 0; i < path.length-1; i++) {
			length += path[i].getPose().distanceTo(path[i+1].getPose());
		}
		return length;
	}


	/**
	 * Get the length in meters of a path. This is the sum of distances between path poses.
	 * @return The length in meters of a path. This is the sum of distances between path poses.
	 */
	public static double getPathLength(Pose[] path) {
		double length = 0.0;
		for (int i = 0; i < path.length-1; i++) {
			length += path[i].distanceTo(path[i+1]);
		}
		return length;
	}

	/**
	 * Get the image of the current map, if set.
	 * @return The image of the current map, <code>null</code> if no map is known.
	 */
	public static BufferedImage getMap() {
		return Missions.map;
	}
	
	/**
	 * Get the YAML file of the current map, if set.
	 * @return The YAML file of the current map, <code>null</code> if no map is known.
	 */
	public static String getMapYAML() {
		return Missions.mapYAML;
	}

	private static String extractZipFile(String fileName) {
		String json = "";
		try {
	        byte[] buffer = new byte[1024];
	        ZipInputStream zis = new ZipInputStream(new FileInputStream(fileName));
	        ZipEntry zipEntry = zis.getNextEntry();
	        while(zipEntry != null){
	            String oneFileName = zipEntry.getName();
	            int len;
	            ArrayList<Byte> bytes = new ArrayList<Byte>();
	            while ((len = zis.read(buffer)) > 0) {
	                for (int i = 0; i < len; i++) bytes.add(buffer[i]);
	            }
            	byte[] dataBytes = new byte[bytes.size()];
            	for (int i = 0; i < bytes.size(); i++) dataBytes[i] = bytes.get(i);
	            if (oneFileName.endsWith(".json")) json = new String(dataBytes);
	            else {
	            	ByteArrayInputStream bais = new ByteArrayInputStream(dataBytes);
	            	Missions.map = ImageIO.read(bais);
	            	//Missions.mapImageFilename = oneFileName;
	            }
	            zipEntry = zis.getNextEntry();
	        }
	        zis.closeEntry();
	        zis.close();
		}
		catch (IOException e) { 
			e.printStackTrace(); 
			throw new Error("Unable to estract the ZIP file: " + e.toString());
		}
        return json;
	}
	
	private static void makeZipFile(String ... fileNames) {
		try {
			File f = new File(fileNames[fileNames.length-1]);
			ZipOutputStream out = new ZipOutputStream(new FileOutputStream(f));
			for (int i = 0; i < fileNames.length-1; i++) {
				ZipEntry e = new ZipEntry(fileNames[i]);
				out.putNextEntry(e);
				File entryFile = new File(fileNames[i]);
				byte[] data = Files.readAllBytes(entryFile.toPath());
				out.write(data, 0, data.length);
				out.closeEntry();
			}
			out.close();
		}
		catch (IOException e) { e.printStackTrace(); }
	}
	
	private static class ScenarioContainer {
		private String locationsJSON;
		private String pathsJSON;
		private String missionsJSON;
		private String mapYAMLJSON;
		private String mapImageFilenameJSON;
		private String mapResolutionJSON;
		private String mapOriginJSON;
		private ScenarioContainer() {
			this.missionsJSON = Missions.getJSONString(Missions.missions);
            this.pathsJSON = Missions.getJSONString(Missions.paths);
            this.locationsJSON = Missions.getJSONString(Missions.locations);
            this.mapImageFilenameJSON = Missions.getJSONString(Missions.mapImageFilename);
            this.mapYAMLJSON = Missions.getJSONString(Missions.mapYAML);
            this.mapOriginJSON = Missions.getJSONString(Missions.mapOrigin);
            this.mapResolutionJSON = Missions.getJSONString(Missions.mapResolution);
		}
	}
	
	/**
	 * Save the current scenario. A scenario consists of all known locations and paths,
	 * all known missions, and a map of the environment if it is known. This is stored in a
	 * ZIP file with the given name, and can be re-loaded via a call to {@link Missions#loadScenario(String)}.
	 * @param scenarioName The name of the archive in which to save the current scenario.
	 */
	public static void saveScenario(String scenarioName) {
        try {
        	String zipFilename = scenarioName+".zip";
            System.out.println("Saving scenario in ZIP file: " + zipFilename);
            String jsonFilename = scenarioName + ".json";
            PrintWriter writer = new PrintWriter(jsonFilename);
            ScenarioContainer sc = new ScenarioContainer();
            String scenarioJSON = Missions.getJSONString(sc);
            writer.println(scenarioJSON);
            writer.close();
            if (Missions.mapImageFilename != null) makeZipFile(Missions.mapImageFilename,jsonFilename,zipFilename);
            else makeZipFile(jsonFilename,zipFilename);
        }
        catch (IOException e) { 
        	e.printStackTrace(); 
        	throw new Error("Unable to save the scenario: " + e.toString()); 
        }		
	}
	
	/**
	 * Load a scenario. A scenario contains one or more of the following:
	 * locations, paths, missions, and a map of the environment.
	 * Scenarios are stored in ZIP files, which can be saved via a call
	 * to {@link Missions#saveScenario(String)}.
	 * @param scenarioName The name of the scenario to load.
	 */
	@SuppressWarnings("unchecked")
	public static void loadScenario(String scenarioName) {
		String zipFilename = scenarioName+".zip";
		System.out.println("Loading scenario from ZIP file: " + zipFilename);
		String json = Missions.extractZipFile(zipFilename);
		Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().serializeNulls().create();
		ScenarioContainer sc = gson.fromJson(json, ScenarioContainer.class);
		Type collectionType = new TypeToken<HashMap<Integer,ArrayList<Mission>>>(){}.getType();
		Missions.missions = (HashMap<Integer, ArrayList<Mission>>) parseJSONString(collectionType, sc.missionsJSON);
		collectionType = new TypeToken<HashMap<String,Pose>>(){}.getType();
		Missions.locations = (HashMap<String,Pose>) parseJSONString(collectionType, sc.locationsJSON);
		collectionType = new TypeToken<HashMap<String,PoseSteering[]>>(){}.getType();
		Missions.paths = (HashMap<String,PoseSteering[]>) parseJSONString(collectionType, sc.pathsJSON);
		Missions.mapImageFilename = (String)parseJSONString(String.class, sc.mapImageFilenameJSON);
		//Missions.map = ImageIO.read(new File(Missions.mapImageFilename));
		Missions.mapYAML = (String)parseJSONString(String.class, sc.mapYAMLJSON);
		Missions.mapResolution = (Double)parseJSONString(Double.TYPE, sc.mapResolutionJSON);
		Missions.mapOrigin = (Coordinate)parseJSONString(Coordinate.class, sc.mapOriginJSON);
		Missions.buildGraph();
	}
	
	private static String getJSONString(Object o) {
		Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().serializeNulls().create();
		String json = gson.toJson(o);
		return json;
	}
	
	private static Object parseJSONString(Type t, String json) {
		Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().serializeNulls().create();
		return gson.fromJson(json, t);
	}
	
	/**
	 * Set the current map.
	 * @param mapYAMLFile A file containing the description of the current map and a pointer to its image.
	 */
	public static void setMap(String mapYAMLFile) {
		try {
			File file = new File(mapYAMLFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String st;
			Missions.mapYAML = "";
			while((st=br.readLine()) != null){
				Missions.mapYAML += st;
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals("image")) Missions.mapImageFilename = file.getParentFile()+File.separator+value;
				else if (key.equals("resolution")) Missions.mapResolution = Double.parseDouble(value);
				else if (key.equals("origin")) {
					String x = value.substring(1, value.indexOf(",")).trim();
					String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
					Missions.mapOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
				}
			}
			br.close();
			Missions.map = ImageIO.read(new File(Missions.mapImageFilename));
		}
		catch (IOException e) { e.printStackTrace(); }
	}
	
	/**
	 * Get the name of the initial location of each robot's first {@link Mission}.
	 * The pose corresponding to each location name can be looked up via the method {@link Missions#getLocation(String)}.
	 * @return The name of the initial location of each robot's first {@link Mission}.
	 */
	public static HashMap<Integer,String> getInitialLocations() {
		HashMap<Integer,String> ret = new HashMap<Integer, String>();
		for (Integer robotID : missions.keySet()) {
			Mission m = Missions.peekMission(robotID);
			if (m != null) ret.put(robotID, m.getFromLocation());
		}
		return ret;
	}
	
	/**
	 * Get the pose of the initial location of each robot's first {@link Mission}.
	 * @return The pose of the initial location of each robot's first {@link Mission}.
	 */
	public static HashMap<Integer,Pose> getInitialPoses() {
		HashMap<Integer,Pose> ret = new HashMap<Integer, Pose>();
		for (Integer robotID : missions.keySet()) {
			Mission m = Missions.peekMission(robotID);
			if (m != null) ret.put(robotID, m.getFromPose());
		}
		return ret;
	}
	
	private static void buildGraph() {
		
		graph = new SimpleDirectedWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		
		metaCSPLogger.info("Updating the roadmap...");
		
		for (String oneLoc : locations.keySet()) {
			graph.addVertex(oneLoc);
			metaCSPLogger.info("Added vertex " + oneLoc);
		}
		
		for (String from : locations.keySet()) {
			for (String to : locations.keySet()) {
				if (!from.equals(to)) {
					if (isKnownPath(from, to)) {
						DefaultWeightedEdge e = graph.addEdge(from, to);
						//PoseSteering[] path = loadKnownPath(from, to);
						PoseSteering[] path = paths.get(from+"->"+to);
						graph.setEdgeWeight(e, path.length);
						metaCSPLogger.info("Added edge " + e);
					}
				}
			}	
		}
	}

	/**
	 * Get the shortest path connecting given locations (two or more). The path between successive pairs of locations
	 * is computed with Dijkstra's algorithm, where edge weights are path lengths.
	 * @param locations At least two location names.
	 * @return The shortest path connecting given locations.
	 */
	public static PoseSteering[] getShortestPath(String ... locations) {
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
	
	/**
	 * Get all the {@link Mission}s currently known for one robot
	 * @param robotID A robot identifier
	 * @return All the {@link Mission}s currently known for one robot
	 */
	public static ArrayList<Mission> getMissions(int robotID) {
		return missions.get(robotID);
	}
	
	/**
	 * Ascertain whether there is at least one {@link Mission} for a given robot. 
	 * @param robotID The ID of a robot.
	 * @return <code>true</code> iff the robot with the given ID has a mission.
	 */
	public static boolean hasMissions(int robotID) {
		return missions.containsKey(robotID) && !missions.get(robotID).isEmpty();
	}
	
	/**
	 * Remove one or more {@link Mission}s.
	 * @param m The mission(s) to remove.
	 */
	public static void removeMissions(Mission ... m) {
		for (Mission mis : m) missions.get(mis.getRobotID()).remove(mis);
	}
	
	/**
	 * Set the pose of a location.
	 * @param locationName The name of the location.
	 * @param p The pose of the location.
	 */
	@Deprecated
	public static void setLocation(String locationName, Pose p) {
		locations.put(locationName, p);
	}
	
	/**
	 * Get all the {@link Mission}s currently currently known
	 * @return All the {@link Mission}s currently currently known
	 */
	public static HashMap<Integer,ArrayList<Mission>> getMissions() {
		return missions;
	}

	/**
	 * Add a new {@link Mission} for a robot.
	 * @param m The mission to push.
	 */
	@Deprecated
	public static void putMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}
	
	/**
	 * Enqueue a new {@link Mission} for a robot.
	 * @param m The mission to enqueue.
	 */
	public static void enqueueMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}

	/**
	 * Push a new {@link Mission} for a robot on the mission queue.
	 * @param m The mission to push
	 */
	@Deprecated
	public static void pushMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}

	/**
	 * Get the i-th mission for a given robot.	
	 * @param robotID A robot identifier.
	 * @param missionNumber The mission to get.
	 * @return The i-th mission for a given robot.
	 */
	public static Mission getMission(int robotID, int missionNumber) {
		return missions.get(robotID).get(missionNumber);
	}
	
	/**
	 * Dequeue the first mission from the queue of a given robot. 
	 * @param robotID The ID of the robot.
	 * @return The first mission from the queue of a given robot.
	 */
	public static Mission dequeueMission(int robotID) {
		if (!missions.get(robotID).isEmpty()) {
			Mission m = missions.get(robotID).get(0);
			missions.get(robotID).remove(0);
			return m;
		}
		return null;
	}
	
	/**
	 * Pop the first mission from the queue of a given robot. 
	 * @param robotID The ID of the robot.
	 * @return The first mission from the queue of a given robot.
	 */
	@Deprecated
	public static Mission popMission(int robotID) {
		if (!missions.get(robotID).isEmpty()) {
			Mission m = missions.get(robotID).get(0);
			missions.get(robotID).remove(0);
			return m;
		}
		return null;
	}
	
	/**
	 * Pop the first mission from the queue of a given robot, but do NOT remove the mission from the queue. 
	 * @param robotID The ID of the robot.
	 * @return The first mission from the queue of a given robot.
	 */
	public static Mission peekMission(int robotID) {
		if (!missions.get(robotID).isEmpty()) {
			Mission m = missions.get(robotID).get(0);
			return m;
		}
		return null;
	}

	/**
	 * Normalize an angle to be within (-PI,PI).
	 * @param th The angle to normalize
	 * @return A value within (-PI,PI)
	 */
	public static double wrapAngle180b(double th) {
		double ret = Math.atan2(Math.sin(th), Math.cos(th));
		if (Math.abs(ret-Math.PI) < 0.00001) return Math.PI-0.01;
		return ret;
	}

	/**
	 * Normalize an angle to be within (-PI,PI].
	 * @param th The angle to normalize
	 * @return A value within (-PI,PI]
	 */
	public static double wrapAngle180a(double th) {
		double ret = Math.atan2(Math.sin(th), Math.cos(th));
		if (Math.abs(ret-Math.PI) < 0.00001) return Math.PI;
		return ret;
	}
	
	/**
	 * Normalize an angle to be within [-PI,PI).
	 * @param th The angle to normalize
	 * @return A value within [-PI,PI)
	 */
	public static double wrapAngle180(double th) {
		double ret = Math.atan2(Math.sin(th), Math.cos(th));
		if (Math.abs(ret-Math.PI) < 0.00001) return -Math.PI;
		return ret;
	}

	/**
	 * Normalize an angle to be within [0,2*PI).
	 * @param th The angle to normalize
	 * @return A value within [0,2*PI)
	 */
	public static double wrapAngle360(double th) {
		double ret = Math.atan2(Math.sin(th), Math.cos(th));
		if (ret < 0) return ret+2*Math.PI;
		return ret;
	}

	/**
	 * Get the last placement along the {@link Trajectory} of a {@link TrajectoryEnvelope} that does
	 * not overlap with the final pose of the robot.
	 * @param te The trajectory envelope to search on
	 * @return The last placement along the {@link Trajectory} of a {@link TrajectoryEnvelope} that does
	 * not overlap with the final pose of the robot.
	 */
	public static Geometry getBackBlockingObstacle(TrajectoryEnvelope te) {
		Trajectory traj = te.getTrajectory();
		PoseSteering[] path = traj.getPoseSteering();
		Geometry placementLast = te.makeFootprint(path[path.length-1]);
		for (int i = path.length-2; i >= 0; i--) {
			Geometry placement = te.makeFootprint(path[i]);
			if (!placement.intersects(placementLast)) return placement;
		}
		return null;
	}

	/**
	 * Get the known locations and their poses.
	 * @return All known locations and their poses.
	 */
	@Deprecated
	public static HashMap<String,Pose> getLocations() {
		return locations;
	}

	/**
	 * Get the known locations and their poses.
	 * @return All known locations and their poses.
	 */
	public static HashMap<String,Pose> getLocationsAndPoses() {
		return locations;
	}

	
	/**
	 * Load location and path data from a file.
	 * @param fileName The file to load the data from.
	 */
	@Deprecated
	public static void loadLocationAndPathData(String fileName) {
		Missions.loadRoadMap(fileName);
	}
	
	/**
	 * Load a roadmap stored in a give directory or file.
	 * @param fileName The directory or file to load the roadmap from.
	 * If a directory is given, the filename is assumed to he "roadmap.txt"
	 * (the same convention used in saving, see {@link #saveRoadMap(String)}).
	 */
	public static void loadRoadMap(String path) {
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
						PoseSteering[] knownPath = loadPathFromFile(pathOnly+oneline[3]);
						paths.put(oneline[0]+oneline[1]+oneline[2], knownPath);
						//paths.put(oneline[0]+oneline[1]+oneline[2], oneline[3]);
						//metaCSPLogger.info("Loaded path: " + oneline[0]+oneline[1]+oneline[2] + " --> " + oneline[3]);
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
		Missions.buildGraph();
	}
	
	/**
	 * Save all locations and paths into a given directory.
	 * @param pathname The name of a directory in which to store the data.
	 */
	@Deprecated
	public static void saveLocationAndPathData(String pathname) {
		Missions.saveRoadMap(pathname);
	}
	
	/**
	 * Save the current roadmap into a given directory. All known paths referred to
	 * in the roadmap are saved in the given directory. The roadmap itself is saved
	 * in a file named "roadmap.txt" within the given directory.
	 * @param pathname The name of a directory in which to store the roadmap data.
	 */
	public static void saveRoadMap(String pathname) {
		if (new File(pathname).exists()) throw new Error("Directory \"" + pathname + "\" exists, will abort saving");
		if (!new File(pathname).mkdir()) throw new Error("Could not make path \"" + pathname + "\"");
		if (!pathname.endsWith(File.separator)) pathname += File.separator;
		String st = "#Locations\n";
		for (Entry<String,Pose> entry : locations.entrySet()) {
			st += entry.getKey() + "\t" + entry.getValue().getX() + "\t" + entry.getValue().getY() + "\t" + entry.getValue().getTheta() + "\n";
		}
		st+="\n#Paths\n";
		for (Entry<String,PoseSteering[]> entry : paths.entrySet()) {
			String pathFilename = entry.getKey().replaceAll("->", "-")+".path";
			st += entry.getKey().replaceAll("->", " -> ") + "\t" + pathFilename + "\n";
			writePath(pathname+pathFilename, entry.getValue());
		}
        try {
        	String newFilename = pathname+"roadmap.txt";
            File file = new File(newFilename);
            PrintWriter writer = new PrintWriter(file);
            writer.write(st);
            writer.close();
            System.out.println("Saved roadmap " + newFilename);
        }
        catch (Exception ex) { ex.printStackTrace(); }
	}
	

	/**
	 * Add a path to the set of known paths.
	 * @param start The starting location of the path.
	 * @param goal The goal location of the path.
	 * @param path The path to add.
	 */
	@Deprecated
	public static void addKnownPath(String start, String goal, PoseSteering[] path) {
		Missions.addPathToRoadMap(start, goal, path);
	}
		
	/**
	 * Add a path to the current roadmap.
	 * @param start The starting location of the path.
	 * @param goal The goal location of the path.
	 * @param path The path to add.
	 */
	public static void addPathToRoadMap(String start, String goal, PoseSteering[] path) {
		if (!locations.containsKey(start) || !locations.containsKey(goal) || path == null || path.length == 0) throw new Error("Locations unknown or path is invalid (" + (start+"->"+goal) + ")!");
		paths.put(start+"->"+goal, path);
		Missions.buildGraph();
	}

	/**
	 * Remove a named location.
	 * @param locationName The name of the location to remove.
	 */
	@Deprecated
	public static void removeLocation(String locationName) {
		Missions.removeLocationFromRoadMap(locationName);
	}
	
	/**
	 * Remove a named location from the roadmap.
	 * @param locationName The name of the location to remove.
	 */
	public static void removeLocationFromRoadMap(String locationName) {
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
	public static void addLocationToRoadMap(String locationName, Pose pose) {
		locations.put(locationName, pose);
	}
	
	/**
	 * Get locations from the data loaded by method loadLocationAndPathData()
	 * @param name The name of the location to get
	 * @return The {@link Pose} of the location
	 */
	@Deprecated
	public static Pose getLocation(String name) {
		Pose ret = locations.get(name);
		if (ret == null) throw new Error("Unknown location " + name);
		return ret;
	}
	
	/**
	 * Get the pose of a given named location in the roadmap.
	 * @param name The name of the location to get the pose of.
	 * @return The {@link Pose} of the location
	 */
	public static Pose getLocationPose(String name) {
		Pose ret = locations.get(name);
		if (ret == null) throw new Error("Unknown location " + name);
		return ret;
	}

	
	/**
	 * Get the mission following a given mission.
	 * @param m A mission.
	 * @return The mission following the given mission.
	 */
	public static Mission getNextMission(Mission m) {
		for (int i = 0; i < missions.get(m.getRobotID()).size(); i++) {
			if (missions.get(m.getRobotID()).get(i).equals(m)) return missions.get(m.getRobotID()).get((i+1)%missions.get(m.getRobotID()).size()); 
		}
		return null;
	}

	/**
	 * Get the mission preceding a given mission.
	 * @param m A mission.
	 * @return The mission preceding the given mission.
	 */
	public static Mission getPreviousMission(Mission m) {
		for (int i = 0; i < missions.get(m.getRobotID()).size(); i++) {
			if (missions.get(m.getRobotID()).get(i).equals(m)) {
				if (i == 0) return missions.get(m.getRobotID()).get(missions.get(m.getRobotID()).size()-1);
				return missions.get(m.getRobotID()).get((i-1)); 
			}
		}
		return null;
	}

	/**
	 * Get path files from the data loaded by method {@link Missions#loadLocationAndPathData(String)}.
	 * @param fromLocation The name of the source location.
	 * @param toLocation The name of the destination location.
	 * @return The name of the file where the path is stored. 
	 */
	@Deprecated
	public static String getPathFile(String fromLocation, String toLocation) {
		//String ret = paths.get(fromLocation+"->"+toLocation);
		String ret = fromLocation+"->"+toLocation;
		if (!locations.containsKey(fromLocation)) throw new Error("Unknown location " + fromLocation);
		if (!locations.containsKey(toLocation)) throw new Error("Unknown location " + toLocation);
		File f = new File(ret);
		if(!f.exists() || f.isDirectory()) throw new Error("No path between " + fromLocation + " and " + toLocation);
		//if (ret == null) throw new Error("No path between " + fromLocation + " and " + toLocation);
		return ret;
	}
	
	/**
	 * Queries whether a path between two named locations is known.
	 * @param fromLocation The source location.
	 * @param toLocation The goal location.
	 * @return <code>true</code> iff a path between two locations is known.
	 */
	public static boolean isKnownPath(String fromLocation, String toLocation) {
		return paths.containsKey(fromLocation+"->"+toLocation);
	}
	
	/**
	 * Return a path between locations if available (throws error if locations and/or path are now known)
	 * @param fromLocation The source location
	 * @param toLocation The goal location
	 * @return The path between the two (known) locations
	 */
//	public static PoseSteering[] loadKnownPath(String fromLocation, String toLocation) {
//		if (!isKnownPath(fromLocation, toLocation)) throw new Error("No path between " + fromLocation + " and " + toLocation);
//		return loadPathFromFile(pathPrefix+getPathFile(fromLocation, toLocation));
//	}

	/**
	 * Get a property from a YAML file
	 * @param property The name of the property
	 * @param yamlFile The YAML file from which to get the property
	 * @return The value of the property in the YAML file
	 */
	public static String getProperty(String property, String yamlFile) {
		String ret = null;
		try {
			File file = new File(yamlFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String st;
			while((st=br.readLine()) != null){
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals(property)) {
					ret = value;
					break;
				}
			}
			br.close();
		}
		catch (IOException e) { e.printStackTrace(); }
		return ret;
	}

	/**
	 * Kill the dispatching threads for a given set of robots.
	 * @param robotIDs The robots for which the dispatching thread should be killed.
	 */
	public static void stopMissionDispatchers(int ... robotIDs) {
		for (int robotID : robotIDs) {
			missionDispatcherFlags.put(robotID, false);
			mdcs.remove(robotID);
		}
	}
	
	/**
	 * Save a path to a file.
	 * @param fileName The name of the file.
	 * @param path The path to save.
	 */
	public static void writePath(String fileName, ArrayList<PoseSteering> path) {
        try {
            File file = new File(fileName);
            System.out.println("Saved path file: " + file.getAbsolutePath());
            PrintWriter writer = new PrintWriter(file);
            for (PoseSteering ps : path) {
            	writer.println(ps.getPose().getX() + "\t" + ps.getPose().getY() + "\t" + ps.getPose().getTheta() + "\t" + ps.getSteering());
            }
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}

	/**
	 * Save a path to a file.
	 * @param fileName The name of the file.
	 * @param path The path to save.
	 */
	public static void writePath(String fileName, PoseSteering[] path) {
        try {
            File file = new File(fileName);
            System.out.println("Saved path file " + file.getAbsolutePath());
            PrintWriter writer = new PrintWriter(file);
            for (PoseSteering ps : path) {
            	writer.println(ps.getPose().getX() + "\t" + ps.getPose().getY() + "\t" + ps.getPose().getTheta() + "\t" + ps.getSteering());
            }
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}

	/**
	 * Tag a set of {@link Mission}s as concatenated, meaning that they should be executed in sequence.
	 * This marking will be accounted for by the dispatchers started via methods {@link Missions#startMissionDispatchers(TrajectoryEnvelopeCoordinator, int...)}
	 * and {@link Missions#startMissionDispatchers(TrajectoryEnvelopeCoordinator, boolean, int...)}.
	 * @param m The {@link Mission}s that should be considered concatenated.
	 */
	public static void concatenateMissions(Mission ... m) {
		ArrayList<Mission> toAdd = new ArrayList<Mission>();
		for (Mission oneM : m) toAdd.add(oneM);
		concatenatedMissions.put(m[0], toAdd);
	}
		
	/**
	 * Add a {@link MissionDispatchingCallback} which defines methods to be called before and after mission dispatching.
	 * @param robotID The callback functions will be invoked whenever a mission for this robot is dispatched. 
	 * @param cb The {@link MissionDispatchingCallback} to attach.
	 */
	public static void addMissionDispatchingCallback(int robotID, MissionDispatchingCallback cb) {
		mdcs.put(robotID, cb);
	}
	
	/**
	 * Start a thread for each robot that cycles through the known missions for that
	 * robot and dispatches them when the robot is free. This method will loop through all missions forever.
	 * @param tec The {@link TrajectoryEnvelopeCoordinator} that coordinates the missions.
	 * @param robotIDs The robot IDs for which a dispatching thread should be started.
	 */
	public static void startMissionDispatchers(final TrajectoryEnvelopeCoordinator tec, int ... robotIDs) {
		startMissionDispatchers(tec, true, robotIDs);
	}
	
	/**
	 * Start a thread for each robot that cycles through the known missions for that
	 * robot and dispatches them when the robot is free.
	 * @param tec The {@link TrajectoryEnvelopeCoordinator} that coordinates the missions.
	 * @param loop Set to <code>false</code> if missions should be de-queued once dispatched. 
	 * @param robotIDs The robot IDs for which a dispatching thread should be started.
	 */
	public static void startMissionDispatchers(final TrajectoryEnvelopeCoordinator tec, final boolean loop, int ... robotIDs) {
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				@Override
				public void run() {
					while (missionDispatcherFlags.get(robotID)) {
						if (Missions.hasMissions(robotID)) {
							Mission m = Missions.peekMission(robotID);
							if (m != null) {								
								synchronized(tec) {
									if (tec.isFree(m.getRobotID())) {
										//cat with future missions if necessary
										if (concatenatedMissions.containsKey(m)) {
											ArrayList<Mission> catMissions = concatenatedMissions.get(m);
											m = new Mission(m.getRobotID(), m.getFromLocation(), catMissions.get(catMissions.size()-1).getToLocation(), m.getFromPose(), catMissions.get(catMissions.size()-1).getToPose());
											ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
											for (int i = 0; i < catMissions.size(); i++) {
												Mission oneMission = catMissions.get(i);
												if (mdcs.containsKey(robotID)) mdcs.get(robotID).beforeMissionDispatch(oneMission);
												if (i == 0) path.add(oneMission.getPath()[0]);
												for (int j = 1; j < oneMission.getPath().length-1; j++) {
													path.add(oneMission.getPath()[j]);
												}
												if (i == catMissions.size()-1) path.add(oneMission.getPath()[oneMission.getPath().length-1]);
											}
											m.setPath(path.toArray(new PoseSteering[path.size()]));
										}
										else if (mdcs.containsKey(robotID)) mdcs.get(robotID).beforeMissionDispatch(m);							
									}
		
									//addMission returns true iff the robot was free to accept a new mission
									if (tec.addMissions(m)) {
										//tec.computeCriticalSectionsAndStartTrackingAddedMission();
										if (mdcs.containsKey(robotID)) mdcs.get(robotID).afterMissionDispatch(m);
										if (!loop) {
											Missions.removeMissions(m);
											System.out.println("Removed mission " + m);
											if (concatenatedMissions.get(m) != null) {
												for (Mission cm : concatenatedMissions.get(m)) {
													Missions.removeMissions(cm);
												}
											}
										}
										else {
											Missions.dequeueMission(m.getRobotID());
											Missions.enqueueMission(m);
										}
									}
								}
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					missionDispatcherFlags.remove(robotID);
				}
			};
			missionDispatcherFlags.put(robotID, true);
			//Start the thread!
			t.start();
		}
	}


	/**
	 * Read a path from a file.
	 * @param fileName The name of the file containing the path
	 * @return The path read from the file
	 */
	public static PoseSteering[] loadPathFromFile(String fileName) {
		ArrayList<PoseSteering> ret = new ArrayList<PoseSteering>();
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			String prevLine = "Gibberish";
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (!line.equals(prevLine)) {
					prevLine = line;
					if (line.length() != 0) {
						String[] oneline = line.split(" |\t");
						PoseSteering ps = null;
						if (oneline.length == 4) {
						ps = new PoseSteering(
								Double.parseDouble(oneline[0]),
								Double.parseDouble(oneline[1]),
								Double.parseDouble(oneline[2]),
								Double.parseDouble(oneline[3]));
						}
						else {
							ps = new PoseSteering(
									Double.parseDouble(oneline[0]),
									Double.parseDouble(oneline[1]),
									Double.parseDouble(oneline[2]),
									0.0);					
						}
						ret.add(ps);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
		PoseSteering[] retArray = ret.toArray(new PoseSteering[ret.size()]);
		return resamplePath(retArray);
	}
	

	
	/**
	 * Create a mission following a given mission. The mission will make the follower robot navigate from its
	 * current pose to the start pose of the leader's mission, after which the follower robot will follow the
	 * leader's mission.
	 * @param leaderMission The mission to follow.
	 * @param followerID The ID of the follower robot.
	 * @param followerStartingPose The current pose of the follower robot.
	 * @param mp The motion planner that should be used to compute the path from the follower robot's
	 * current pose to goal pose of the leader's mission, passing through the start pose of the leader's mission.
	 * @param computePathToLeaderGoal Set this to <code>true</code> iff the follower's path to the goal of the 
	 * leader's mission should be recomputed (otherwise the leader's path will be re-used).  
	 * @return
	 */
	public static Mission followMission(Mission leaderMission, int followerID, Pose followerStartingPose, AbstractMotionPlanner mp, boolean computePathToLeaderGoal) {
		mp.setStart(followerStartingPose);
		if (computePathToLeaderGoal) mp.setGoals(leaderMission.getFromPose(), leaderMission.getToPose());
		else mp.setGoals(leaderMission.getFromPose());
		if (!mp.plan()) return null;
		PoseSteering[] followerPath = mp.getPath();
		if (!computePathToLeaderGoal) {
			PoseSteering[] newPath = new PoseSteering[followerPath.length+leaderMission.getPath().length-1];
			int counter = 0;
			for (PoseSteering ps : followerPath) newPath[counter++] = ps;
			for (int i = 1; i < leaderMission.getPath().length; i++) newPath[counter++] = leaderMission.getPath()[i];
			followerPath = newPath;
		}
		Mission followerMission = new Mission(followerID, followerPath);
		return followerMission;
	}
	
	public static Set<String> getAllGraphVertices() {
		return graph.vertexSet();
	}
	
	public static Set<DefaultWeightedEdge> getAllGraphEdges() {
		return graph.edgeSet();
	}

}
