package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import org.jgrapht.alg.KosarajuStrongConnectivityInspector;
import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.alg.util.Pair;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.DirectedSubgraph;
import org.jgrapht.graph.SimpleDirectedWeightedGraph;

public class TestUpdateGraph {
	
	static HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>> currentCyclesList = new HashMap<Pair<Integer, Integer>, HashSet<ArrayList<Integer>>>();
	static SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge> currentOrdersGraph = new SimpleDirectedWeightedGraph<Integer,DefaultWeightedEdge>(DefaultWeightedEdge.class);
	

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
					System.out.println("Removing the edge: " + edge.toString());
					currentOrdersGraph.removeEdge(edge.getFirst(), edge.getSecond());
					if (currentCyclesList.containsKey(edge)) {
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
					}
				}
			}
		}
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
				System.out.println("Add " + occurrence + " edges:" + edge.toString());
			}
			else {
				DefaultWeightedEdge e = currentOrdersGraph.getEdge(edge.getFirst(),edge.getSecond());
				currentOrdersGraph.setEdgeWeight(e,currentOrdersGraph.getEdgeWeight(e)+occurrence);
			}
		}
		if (toAdd.isEmpty()) return;

		//compute strongly connected components
		KosarajuStrongConnectivityInspector<Integer,DefaultWeightedEdge> ksccFinder = new KosarajuStrongConnectivityInspector<Integer,DefaultWeightedEdge>(currentOrdersGraph);
		List<DirectedSubgraph<Integer,DefaultWeightedEdge>> sccs = ksccFinder.stronglyConnectedSubgraphs();
		System.out.println("Connected components: " + sccs.toString());

		//update the cycle list. Use a map to avoid recomputing cycles of each connected component.
		for (Pair<Integer,Integer> pair : toAdd) {
			//search the strongly connected components containing the two vertices
			for (DirectedSubgraph<Integer,DefaultWeightedEdge> scc : sccs) {
				if (scc.containsVertex(pair.getFirst()) ||	scc.containsVertex(pair.getSecond())) {
					if (scc.containsVertex(pair.getFirst()) && scc.containsVertex(pair.getSecond())) {
						//get cycles in this strongly connected components
						JohnsonSimpleCycles<Integer,DefaultWeightedEdge> cycleFinder = new JohnsonSimpleCycles<Integer,DefaultWeightedEdge>(scc);
						List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
						System.out.println("Reversed cycles: " + cycles);
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
					}
					break;
				}
			}
		}
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
		/*ArrayList<Integer> cycle = new ArrayList<Integer>(Arrays.asList(1,2,3));
		for (int i = 0; i < cycle.size(); i++) {
			int j = i < cycle.size()-1 ? i+1 : 0;
			Pair<Integer,Integer> edge = new Pair<Integer,Integer>(cycle.get(i), cycle.get(j));
			if (!currentCyclesList.containsKey(edge)) currentCyclesList.put(edge, new HashSet<ArrayList<Integer>>());
			currentCyclesList.get(edge).add((ArrayList<Integer>)cycle);
		}*/
        
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

	}

}
