package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import org.jgrapht.alg.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.alg.util.Pair;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.DirectedSubgraph;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;
import org.metacsp.utility.Permutation;

public class TestUpdateGraph {
	
	private static void writeStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), true)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	private static void initStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), false)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	static int NUMBER_ROBOTS = 1000;
	static String searchType = "LC";//otherwise it assumes DF
	static HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> currentCyclesList = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
	static SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge> currentOrdersGraph = new SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge>(DefaultWeightedEdge.class);
	static long nameTime = Calendar.getInstance().getTimeInMillis();
	static String fileName1 = System.getProperty("user.home")+File.separator+"T-RO-ORU/update Graph/"+NUMBER_ROBOTS+"/"+searchType+"/add-"+nameTime+".txt";
	static String stat1 = "";
	static String fileName2 = System.getProperty("user.home")+File.separator+"T-RO-ORU/update Graph/"+NUMBER_ROBOTS+"/"+searchType+"/delete-"+nameTime+".txt";
	static String stat2 = "";
	static String fileName3 = System.getProperty("user.home")+File.separator+"T-RO-ORU/update Graph/"+NUMBER_ROBOTS+"/"+searchType+"/degrees-"+nameTime+".txt";
	static String stat3 = "";
	
	

	static void deleteEdges(HashMap<Pair<Integer,Integer>, Integer> edgesToDelete) {

		if (edgesToDelete == null || edgesToDelete.isEmpty()) return;
		
		for (Pair<Integer,Integer> edge : edgesToDelete.keySet()) {
			Integer occurrence = edgesToDelete.get(edge);
			if (occurrence == 0 || occurrence == null) {
				System.out.println("<<<<<<<<<< Found edge " + edge.toString() + " with invalid weigth. Skipping deletion." );
				continue;
			}
			DefaultWeightedEdge e = currentOrdersGraph.getEdge(edge.getFirst(), edge.getSecond());
			if (e != null) {
				int numEdge = (int) currentOrdersGraph.getEdgeWeight(e);
				if (numEdge > occurrence) currentOrdersGraph.setEdgeWeight(e, numEdge-occurrence);
				else {
					//FIXME Waste of time is probably here
					System.out.println("Removing edge " + edge.toString());
					long startTime = Calendar.getInstance().getTimeInMillis();
					currentOrdersGraph.removeEdge(edge.getFirst(), edge.getSecond());
					stat2 = stat2 + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
					startTime = Calendar.getInstance().getTimeInMillis();
					/*if (currentCyclesList.containsKey(edge)) {
						HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> toRemove = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
						for (ArrayList<Integer> cycle : currentCyclesList.get(edge)) {
							for (int i = 0; i < cycle.size(); i++) {
								Pair<Integer,Integer> key = new Pair<Integer,Integer>(cycle.get(i), cycle.get(i < cycle.size()-1 ? i+1 : 0));
								if (!toRemove.containsKey(key)) toRemove.put(key, new HashSet<ArrayList<Integer>>());
								toRemove.get(key).add(cycle);
							}
						}
						for (Pair<Integer, Integer> key : toRemove.keySet()) {
							currentCyclesList.get(key).removeAll(toRemove.get(key));
							if (currentCyclesList.get(key).isEmpty()) currentCyclesList.remove(key);
						}
					}*/
					stat2 = stat2 + "\t" + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
				}
			}
			else 
				stat2 += stat2 + "0\t 0";
		}
	}
	
	static void addEdge(int source, int target) {
		HashMap<Pair<Integer,Integer>, Integer> edgesToAdd = new HashMap<Pair<Integer,Integer>, Integer>();
		edgesToAdd.put(new Pair<Integer,Integer>(source,target), 1);
		long startTime = Calendar.getInstance().getTimeInMillis();
		addEdges(edgesToAdd);
		stat1 = stat1 + "\t" + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
		stat3 = stat3 + currentOrdersGraph.edgeSet().size();
		for (int i = 0; i < NUMBER_ROBOTS; i++) {
			int in_degree = 0;
			int out_degree = 0;
			if (currentOrdersGraph.containsVertex(i)) {
				in_degree = currentOrdersGraph.inDegreeOf(i);
				out_degree = currentOrdersGraph.outDegreeOf(i);
			}
			stat3 = stat3 + "\t[" + in_degree + "," + out_degree + "]";
		}
	}
	
	static void addEdgeLocal(int source, int target) {
		HashMap<Pair<Integer,Integer>, Integer> edgesToAdd = new HashMap<Pair<Integer,Integer>, Integer>();
		edgesToAdd.put(new Pair<Integer,Integer>(source,target), 1);
		addEdgesLocal(edgesToAdd);
		stat3 = stat3 + currentOrdersGraph.edgeSet().size();
		for (int i = 0; i < NUMBER_ROBOTS; i++) {
			int in_degree = 0;
			int out_degree = 0;
			if (currentOrdersGraph.containsVertex(i)) {
				in_degree = currentOrdersGraph.inDegreeOf(i);
				out_degree = currentOrdersGraph.outDegreeOf(i);
			}
			stat3 = stat3 + "\t[" + in_degree + "," + out_degree + "]";
		}
	}
	
	static void deleteEdge(int source, int target) {
		HashMap<Pair<Integer,Integer>, Integer> edgesToDelete = new HashMap<Pair<Integer,Integer>, Integer>();
		edgesToDelete.put(new Pair<Integer,Integer>(source,target), 1);
		long startTime = Calendar.getInstance().getTimeInMillis();
		deleteEdges(edgesToDelete);
		stat2 = stat2 + "\t" + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
	}

	static void addEdges(HashMap<Pair<Integer,Integer>, Integer> edgesToAdd) {

		if (edgesToAdd == null || edgesToAdd.isEmpty()) return;

		HashSet<Pair<Integer,Integer>> toAdd = new HashSet<Pair<Integer,Integer>>();

		//add the edges if not already in the graph
		for (Pair<Integer,Integer> edge : edgesToAdd.keySet()) {
			Integer occurrence = edgesToAdd.get(edge);
			if (occurrence == 0 || occurrence == null) {
				System.out.println("<<<<<<<<<< Found edge " + edge.toString() + " with invalid weigth. Skipping addition." );
				continue;
			}
			if (!currentOrdersGraph.containsEdge(edge.getFirst(),edge.getSecond())) {
				toAdd.add(edge);
				currentOrdersGraph.addVertex(edge.getFirst());
				currentOrdersGraph.addVertex(edge.getSecond());
				DefaultWeightedEdge e = currentOrdersGraph.addEdge(edge.getFirst(),edge.getSecond());
				if (e == null) System.out.println("<<<<<<<<< Add dependency order fails (12). Edge: " + edge.getFirst() + "->" + edge.getSecond());				
				currentOrdersGraph.setEdgeWeight(e, occurrence);
				//System.out.println("Add " + occurrence + " edges:" + edge.toString());
			}
			else {
				DefaultWeightedEdge e = currentOrdersGraph.getEdge(edge.getFirst(),edge.getSecond());
				currentOrdersGraph.setEdgeWeight(e,currentOrdersGraph.getEdgeWeight(e)+occurrence);
			}
		}
		if (toAdd.isEmpty()) {
			stat1 = stat1 + "\t\t";
			return;
		}

		//compute strongly connected components
		long startTime = Calendar.getInstance().getTimeInMillis();
		KosarajuStrongConnectivityInspector<Integer,DefaultWeightedEdge> ksccFinder = new KosarajuStrongConnectivityInspector<Integer,DefaultWeightedEdge>(currentOrdersGraph);
		List<DirectedSubgraph<Integer,DefaultWeightedEdge>> sccs = ksccFinder.stronglyConnectedSubgraphs();
		stat1 = stat1 + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
		//System.out.println("Connected components: " + sccs.toString());

		//update the cycle list. Use a map to avoid recomputing cycles of each connected component.
		for (Pair<Integer,Integer> pair : toAdd) {
			System.out.println("Adding edge " + pair.toString());
			//search the strongly connected components containing the two vertices
			for (DirectedSubgraph<Integer,DefaultWeightedEdge> scc : sccs) {
				boolean got = false;
				if (scc.containsVertex(pair.getFirst()) ||	scc.containsVertex(pair.getSecond())) {
					if (scc.containsVertex(pair.getFirst()) && scc.containsVertex(pair.getSecond())) {
						got = true;
						//get cycles in this strongly connected components
						startTime = Calendar.getInstance().getTimeInMillis();
						JohnsonSimpleCycles<Integer,DefaultWeightedEdge> cycleFinder = new JohnsonSimpleCycles<Integer,DefaultWeightedEdge>(scc);
						List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
						stat1 = stat1 + "\t" + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
						//System.out.println("Reversed cycles: " + cycles);
						startTime = Calendar.getInstance().getTimeInMillis();
						if (!cycles.isEmpty()) {
							for(List<Integer> cycle : cycles) {
								Collections.reverse(cycle);
								//update the list of cycles for each edge
								for (int i = 0; i < cycle.size(); i++) {
									int j = i < cycle.size()-1 ? i+1 : 0;
									Pair<Integer,Integer> edge = new Pair<Integer,Integer>(cycle.get(i), cycle.get(j));
									if (!currentCyclesList.containsKey(edge)) currentCyclesList.put(edge, new HashSet<ArrayList<Integer>>());
									currentCyclesList.get(edge).add((ArrayList<Integer>)cycle);
									//System.out.println("edge: " + edge.toString() + "currentCyclesList:" + currentCyclesList.get(edge));
								}
							}
						}
						stat1 = stat1 + "\t" + cycles.size() + "\t" + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
					}
					if (!got) stat1 = stat1 + "\t 0\t\t";
					break;
				}
			}
		}
	}
	
	static void addEdgesLocal(HashMap<Pair<Integer,Integer>, Integer> edgesToAdd) {

		if (edgesToAdd == null || edgesToAdd.isEmpty()) return;

		HashSet<Pair<Integer,Integer>> toAdd = new HashSet<Pair<Integer,Integer>>();

		//add the edges if not already in the graph
		for (Pair<Integer,Integer> edge : edgesToAdd.keySet()) {
			Integer occurrence = edgesToAdd.get(edge);
			if (occurrence == 0 || occurrence == null) {
				System.out.println("<<<<<<<<<< Found edge " + edge.toString() + " with invalid weigth. Skipping addition." );
				continue;
			}
			if (!currentOrdersGraph.containsEdge(edge.getFirst(),edge.getSecond())) {
				toAdd.add(edge);
				currentOrdersGraph.addVertex(edge.getFirst());
				currentOrdersGraph.addVertex(edge.getSecond());
				DefaultWeightedEdge e = currentOrdersGraph.addEdge(edge.getFirst(),edge.getSecond());
				if (e == null) System.out.println("<<<<<<<<< Add dependency order fails (12). Edge: " + edge.getFirst() + "->" + edge.getSecond());				
				currentOrdersGraph.setEdgeWeight(e, occurrence);
				System.out.println("Add " + occurrence + " edges:" + edge.toString());
			}
			else {
				DefaultWeightedEdge e = currentOrdersGraph.getEdge(edge.getFirst(),edge.getSecond());
				currentOrdersGraph.setEdgeWeight(e,currentOrdersGraph.getEdgeWeight(e)+occurrence);
			}
		}
		if (toAdd.isEmpty()) {
			stat1 = stat1 + "\t 0 \t 0";
			return;
		}

		//compute strongly connected components
		long startTime = Calendar.getInstance().getTimeInMillis();
		JohnsonSimpleCycles<Integer,DefaultWeightedEdge> cycleFinder = new JohnsonSimpleCycles<Integer,DefaultWeightedEdge>(currentOrdersGraph);
		List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
		stat1 = stat1 + "\t" + cycles.size() + "\t" + Long.toString(Calendar.getInstance().getTimeInMillis()-startTime);
	}
	
	//map edge, counter
	static void updateGraph(HashMap<Pair<Integer,Integer>, Integer> edgesToDelete, HashMap<Pair<Integer,Integer>, Integer> edgesToAdd) {

			HashMap<Pair<Integer,Integer>, Integer> toDelete = null;
			if (edgesToDelete != null) {
				toDelete = new HashMap<Pair<Integer,Integer>, Integer>();
				for (Pair<Integer,Integer> edge : edgesToDelete.keySet()) {
					if (edgesToAdd.containsKey(edge) && edgesToAdd.get(edge) != null && edgesToAdd.get(edge) < edgesToDelete.get(edge)) 
						toDelete.put(edge, edgesToDelete.get(edge)-edgesToAdd.get(edge));
					else toDelete.put(edge, edgesToDelete.get(edge));
				}
			}
			System.out.println(toDelete.toString());
			
			HashMap<Pair<Integer,Integer>, Integer> toAdd = null;
			if (edgesToAdd != null) {
				toAdd = new HashMap<Pair<Integer,Integer>, Integer>();
				for (Pair<Integer,Integer> edge : edgesToAdd.keySet()) {
					if (edgesToDelete.containsKey(edge) && edgesToDelete.get(edge) != null && edgesToDelete.get(edge) < edgesToAdd.get(edge)) 
						toAdd.put(edge, edgesToAdd.get(edge)-edgesToDelete.get(edge));
					else toAdd.put(edge, edgesToAdd.get(edge));
				}
			}
			System.out.println(toAdd.toString());

			deleteEdges(toDelete);			
			addEdges(toAdd);
	}
	
	
	
	public static void main(String[] args) {
		
		boolean DEBUGGING = false;
				
		if (DEBUGGING) {
	        //Basic test
	        HashMap<Pair<Integer,Integer>,Integer> edgesToDelete = new HashMap<Pair<Integer,Integer>,Integer>();
	        HashMap<Pair<Integer,Integer>,Integer> edgesToAdd = new HashMap<Pair<Integer,Integer>,Integer>();
	        edgesToAdd.put(new Pair<Integer,Integer>(1,2), 2);
	        edgesToAdd.put(new Pair<Integer,Integer>(2,3), 1);
	        edgesToAdd.put(new Pair<Integer,Integer>(3,1), 1);
	        updateGraph(edgesToDelete,edgesToAdd);
	        System.out.println("currentCyclesList: " + currentCyclesList.toString());
	        System.out.println("graph: " + currentOrdersGraph.toString());
	        
	        HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> backupcurrentCyclesList = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
			for (Pair<Integer,Integer> key : currentCyclesList.keySet())
				backupcurrentCyclesList.put(key, new HashSet<ArrayList<Integer>>(currentCyclesList.get(key)));
	        
	        SimpleDirectedWeightedGraph<Integer, DefaultWeightedEdge> backupGraph = new SimpleDirectedWeightedGraph<Integer, DefaultWeightedEdge>(DefaultWeightedEdge.class);
			for (int v : currentOrdersGraph.vertexSet()) backupGraph.addVertex(v);
			for (DefaultWeightedEdge e : currentOrdersGraph.edgeSet()) {
				DefaultWeightedEdge e_ = backupGraph.addEdge(currentOrdersGraph.getEdgeSource(e), currentOrdersGraph.getEdgeTarget(e));
				if (e_ == null) System.out.println("<<<<<<<<< Add egde fails (7). Edge: " + e.toString());
				backupGraph.setEdgeWeight(e_, currentOrdersGraph.getEdgeWeight(e));
			}
	        //check delete edge
	        edgesToDelete.put(new Pair<Integer,Integer>(1,2), 2);
	        edgesToAdd.clear();
	        edgesToDelete.put(new Pair<Integer,Integer>(2,3), 1);
	        updateGraph(edgesToDelete,edgesToAdd);
	        System.out.println("backupcurrentCyclesList: " + backupcurrentCyclesList.toString());
	        System.out.println("currentCyclesList: " + currentCyclesList.toString());
	        System.out.println("backupGraph: " + backupGraph.toString());
	        System.out.println("graph: " + currentOrdersGraph.toString());
	        
	        Pair<Integer,Integer> pair1 = new Pair<Integer,Integer>(1,2);
	        Pair<Integer,Integer> pair2 = new Pair<Integer,Integer>(1,2);
	        Pair<Integer,Integer> pair3 = new Pair<Integer,Integer>(2,1);
	        System.out.println(pair1.equals(pair2) + ", " + pair1.equals(pair3));
	        
	        return;
		}
		
		//initStat(fileName1,"Number robots: " + NUMBER_ROBOTS + "\n" + "Connected components\t Number cycles found \t Compute cycles\t Update list\t Tot time");
		initStat(fileName1,"Number robots: " + NUMBER_ROBOTS + "\n" + "Number cycles \t Compute cycles");
		initStat(fileName2,"Number robots: " + NUMBER_ROBOTS + "\n" + "Update graph\t Update list\t Tot");
		initStat(fileName3,"Number robots: " + NUMBER_ROBOTS + "\n" + "Number edges\t [In degree, Out degree]");
		
		currentCyclesList = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
		currentOrdersGraph = new SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge>(DefaultWeightedEdge.class);
		
					
		//Add and remove all the edges one by one
		/*Permutation p = new Permutation(NUMBER_ROBOTS, 2);
		while (p.hasNext()) {
		      int[] a = p.next();
		      System.out.println(Arrays.toString(a));
		      stat1 = "";
		      addEdge(a[0],a[1]);   
		      writeStat(fileName1, stat1);
		      stat2 = "";
		      deleteEdge(a[0],a[1]);
		      writeStat(fileName2, stat2);
		      addEdge(a[0],a[1]);   
		}

	    System.out.println("Done!");*/
	    
	    /////////// entering edges ///////
	    /*for (int i = 0; i < NUMBER_ROBOTS; i++) {
	    	for (int j = 0; j < NUMBER_ROBOTS; j++) {
	    		if (j != i) {
		    		stat1 = "";
		    		stat3 = "";
				    addEdge(i,j);   
				    writeStat(fileName1, stat1);
				    writeStat(fileName3, stat3);
				    stat2 = "";
				    deleteEdge(i,j);
				    writeStat(fileName2, stat2);
			    	//if (s == 1 && t == 10) break;
				    addEdge(s,t);
	    		}
	    	}
	    }
	    System.out.println("Done!");*/
	    
	    /////////// entering edges ///////
	    for (int i = 0; i < .5*NUMBER_ROBOTS; i++) {
	    	stat1 = "";
		    stat3 = "";
		    int s = 2*i;
		    int t = i == 0 ? NUMBER_ROBOTS : (2*i)+1;
			addEdgeLocal(s,t);   
			writeStat(fileName1, stat1);
			writeStat(fileName3, stat3);
			addEdgeLocal(t,s);
			writeStat(fileName1, stat1);
			writeStat(fileName3, stat3);

	    }
	    System.out.println("Done!");

	}

}
