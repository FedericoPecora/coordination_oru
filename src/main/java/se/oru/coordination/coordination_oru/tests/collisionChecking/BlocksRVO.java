package se.oru.coordination.coordination_oru.tests.collisionChecking;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import edu.unc.cs.gamma.rvo.Simulator;

public class BlocksRVO {
	
	//Specify the goals
	static ArrayList<Vector2D> goals = new ArrayList<Vector2D>();
	static long seed = 123123;// Calendar.getInstance().getTimeInMillis();
	static Random rand = new Random(seed);
	
	private static void setupScenario(Simulator sim)
	{
		/* Specify the global time step of the simulation. */
		sim.setTimeStep(0.25f);

		/* Specify the default parameters for agents that are subsequently added. */
		//sim.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
		sim.setAgentDefaults(15.0, 10, 5.0, 5.0, 5.0, 2.0, new Vector2D(2.0,0.0)); //(neighborDistance, maxNeighbors, timeHorizonAgents, timeHorizonObstacles, radius, maxSpeed, velocity)

		/*
		 * Add agents, specifying their start position, and store their goals on the
		 * opposite side of the environment.
		 */
		sim.addAgent(new Vector2D(55.0, 55.0 + 10.0));
		goals.add(new Vector2D(-75.0, -75.0));

		sim.addAgent(new Vector2D(-55.0, 55.0 + 10.0));
		goals.add(new Vector2D(75.0, -75.0));

		/*
		 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
		 * order.
		 */
		ArrayList<Vector2D> obstacle1 = new ArrayList<Vector2D>();
		ArrayList<Vector2D> obstacle2 = new ArrayList<Vector2D>();
		ArrayList<Vector2D> obstacle3 = new ArrayList<Vector2D>();
		ArrayList<Vector2D> obstacle4 = new ArrayList<Vector2D>();

		obstacle1.add(new Vector2D(-10.0f, 40.0f));
		obstacle1.add(new Vector2D(-40.0f, 40.0f));
		obstacle1.add(new Vector2D(-40.0f, 10.0f));
		obstacle1.add(new Vector2D(-10.0f, 10.0f));

		obstacle2.add(new Vector2D(10.0f, 40.0f));
		obstacle2.add(new Vector2D(10.0f, 10.0f));
		obstacle2.add(new Vector2D(40.0f, 10.0f));
		obstacle2.add(new Vector2D(40.0f, 40.0f));

		obstacle3.add(new Vector2D(10.0f, -40.0f));
		obstacle3.add(new Vector2D(40.0f, -40.0f));
		obstacle3.add(new Vector2D(40.0f, -10.0f));
		obstacle3.add(new Vector2D(10.0f, -10.0f));

		obstacle4.add(new Vector2D(-10.0f, -40.0f));
		obstacle4.add(new Vector2D(-10.0f, -10.0f));
		obstacle4.add(new Vector2D(-40.0f, -10.0f));
		obstacle4.add(new Vector2D(-40.0f, -40.0f));

		sim.addObstacle(obstacle1);
		sim.addObstacle(obstacle2);
		sim.addObstacle(obstacle3);
		sim.addObstacle(obstacle4);

		/* Process the obstacles so that they are accounted for in the simulation. */
		sim.processObstacles();
	}
	
	static void updateVisualization(Simulator sim)
	{
		/* Output the current global time. */
		System.out.println("Time: " + sim.getGlobalTime());

		/* Output the current position of all the agents. */
		for (int i = 0; i < sim.getNumAgents(); ++i) {
			System.out.println("p: " + sim.getAgentPosition(i));
			System.out.println("v: " + sim.getAgentPreferredVelocity(i));
		}
	}
	
	static void setPreferredVelocities(Simulator sim)
	{
		/*
		 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
		 * direction of the goal.
		 */
		for (int i = 0; i < sim.getNumAgents(); ++i) {
			Vector2D goalVector = new Vector2D(goals.get(i).getX() - sim.getAgentPosition(i).getX(),goals.get(i).getY() - sim.getAgentPosition(i).getY());

			if (goalVector.getNorm() > 1.0f) {
				goalVector.normalize();
			}

			sim.setAgentPreferredVelocity(i, goalVector);

			/*
			 * Perturb a little to avoid deadlocks due to perfect symmetry.
			 */
			double angle = rand.nextDouble() * 2.0 * Math.PI / Double.MAX_VALUE;
			double dist = rand.nextDouble() * 0.0001 / Double.MAX_VALUE;

			Vector2D preferredVelocity = new Vector2D(sim.getAgentPreferredVelocity(i).getX()+dist*Math.cos(angle),sim.getAgentPreferredVelocity(i).getY()+dist*Math.sin(angle));
			sim.setAgentPreferredVelocity(i, preferredVelocity);
		}
	}
	
	static boolean reachedGoal(Simulator sim)
	{
		/* Check if all agents have reached their goals. */
		for (int i = 0; i < sim.getNumAgents(); ++i) {
			if (sim.getAgentPosition(i).distance(goals.get(i)) > 1.0) {
				return false;
			}
		}

		return true;
	}
 
	
	public static void main(String[] args) throws InterruptedException {
		
		//Create a new Simulator
		Simulator sim = Simulator.instance;

		/* Set up the scenario. */
		setupScenario(sim);

		/* Perform (and manipulate) the simulation. */
		do {
			updateVisualization(sim);
			setPreferredVelocities(sim);
			sim.doStep();
		}
		while (!reachedGoal(sim));
		
		System.out.println("Done!");
	}
}
