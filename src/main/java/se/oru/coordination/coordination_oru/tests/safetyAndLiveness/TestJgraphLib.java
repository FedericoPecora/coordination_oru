package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.util.List;

import org.jgrapht.alg.cycle.JohnsonSimpleCycles;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleDirectedGraph;

import aima.core.util.datastructure.Pair;

public class TestJgraphLib {

	public static void main(String[] args) throws InterruptedException {
		SimpleDirectedGraph<Integer, DefaultWeightedEdge> g = new SimpleDirectedGraph<Integer, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		g.addVertex(1);
		g.addVertex(2);
		g.addVertex(3);
		g.addEdge(1, 2);
		g.addEdge(2, 3);
		g.addEdge(3, 1);
		JohnsonSimpleCycles<Integer, DefaultWeightedEdge> cycleFinder = new JohnsonSimpleCycles<Integer, DefaultWeightedEdge>(g);
		List<List<Integer>> cycles = cycleFinder.findSimpleCycles();
		System.out.println("Cycles: " + cycles.toString());
		if (!cycles.isEmpty()) {
			for(List<Integer> cycle : cycles) {
				//update the list of cycles for each edge
				for (int i = 0; i < cycle.size(); i++) {
					int j = i < cycle.size()-1 ? i+1 : 0;
					Pair<Integer, Integer> edge = new Pair<Integer, Integer>(cycle.get(i), cycle.get(j));
					System.out.println("edge: " + edge.toString());
				}
			}
		}
	}
}
