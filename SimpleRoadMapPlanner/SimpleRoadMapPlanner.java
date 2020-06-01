package SimpleRoadMapPlanner;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

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
						//Update both the vertix and edge sets of the two graphs.
						graph.addVertex(oneline[0]);
						graph.addVertex(oneline[2]);		
						DefaultWeightedEdge e = graph.addEdge(oneline[0], oneline[2]); 
			            graph.setEdgeWeight(e, knownPath.length);		
						graph_original.addVertex(oneline[0]);
						graph_original.addVertex(oneline[2]);		
						DefaultWeightedEdge eo = graph_original.addEdge(oneline[0], oneline[2]); 
			            graph_original.setEdgeWeight(eo, knownPath.length);
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
						graph.addVertex(locationName);
						graph_original.addVertex(locationName);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { 
			e.printStackTrace(); 
			throw new Error("Unable to load the required scenario: " + e.toString());
		}
	}
		
	public void setGraph(SimpleDirectedWeightedGraph<String, DefaultWeightedEdge> graph) {
		metaCSPLogger.info("Updating the roadmap with the given graph ...");
		
		for (String v : graph.vertexSet()) {
			this.graph.addVertex(v);
			this.graph_original.addVertex(v);
			metaCSPLogger.info("Added vertex " + v + ".");
		}
		
		for (DefaultWeightedEdge e : graph.edgeSet()) {
			DefaultWeightedEdge e1 = this.graph.addEdge(graph.getEdgeSource(e),graph.getEdgeTarget(e));
			this.graph.setEdgeWeight(e1, graph.getEdgeWeight(e));
			DefaultWeightedEdge e2 = this.graph_original.addEdge(graph.getEdgeSource(e),graph.getEdgeTarget(e));
			this.graph_original.setEdgeWeight(e2, graph.getEdgeWeight(e));
			metaCSPLogger.info("Added edge from " + graph.getEdgeSource(e) + " to " + graph.getEdgeTarget(e) + " (weight " + graph.getEdgeWeight(e) + ").");
		}
	}
	
	public boolean doPlanning() {
		return false;
	}

	@Override
	public AbstractMotionPlanner getCopy() {
		//TODO
		SimpleRoadMapPlanner ret = new SimpleRoadMapPlanner();
		return ret;
	}
	
	@Override
	public synchronized void addObstacles(Geometry geom, Pose ... poses) {
		//TODO
		if (this.om == null) this.om = new OccupancyMap(1000, 1000, 0.01);
		this.om.addObstacles(geom, poses);
	}

}
