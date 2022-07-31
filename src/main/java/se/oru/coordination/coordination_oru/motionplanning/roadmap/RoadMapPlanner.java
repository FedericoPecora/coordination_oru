package se.oru.coordination.coordination_oru.motionplanning.roadmap;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

/**
 * This class implements a simple planner based on the methods provided by the static class {@link Missions}.
 * The planner uses the locations and paths that are loaded with the method {@link Missions.#loadRoadMap(String)}.
 * The underlying representation is a weighted directed graph, where the nodes are the loaded locations and the weight on the edge
 * connecting two locations is the length of the path between the two locations. A shortest path between the given start and goal locations
 * is computed using Dijkstra's algorithm.     
 * @author fpa
 *
 */
public class RoadMapPlanner extends AbstractMotionPlanner {

	/**
	 * Instantiate a new {@link RoadMapPlanner}. This constructor sets the <code>escapePoseMode</code> flag to false,
	 * meaning that planning fails if paths are obstructed by obstacles.
	 */
	public RoadMapPlanner() {
		this(false);
	}

	/**
	 * Instantiate a new {@link RoadMapPlanner}. This constructor allows to set the behavior of the planner
	 * in case paths are obstructed by obstacles.
	 * @param escapePoseMode If this flag is set to <code>true</code> the planner seeks a path through the nearest
	 * node to the goal in case the goal is obstructed by an obstacle. This allows to obtain paths with a "waiting pose"
	 * that is obstacle free which a robot could use to wait for the obstructing ribit to be assigned a new task.
	 */
	public RoadMapPlanner(boolean escapePoseMode) {
		super();
		if (escapePoseMode) {
			this.checkGoalPose = false;
			this.verifyPlanning = false;
		}
	}
	
	@Override
	public AbstractMotionPlanner getCopy(boolean copyObstacles) {
		return this;
	}
	
	@Override
	public boolean doPlanning() {
		this.addEscapePoseIfNeeded();
		String[] wps = new String[this.goalLocationNames.length+1];
		wps[0] = this.startLocationName;
		for (int i = 0; i < this.goalLocationNames.length; i++) wps[i+1] = this.goalLocationNames[i];
		PoseSteering[] path = Missions.getShortestPath(wps);
		if (path == null) return false;
		this.pathPS = path;
		return true;
	}
	
	private void addEscapePoseIfNeeded() {
		Geometry footprintInGoal = this.getFootprintInPose(this.goal[this.goal.length-1]);
		boolean intersects = false;
		Geometry[] obstacles = this.getObstacles();
		if (obstacles != null) {
			for (Geometry geom : this.getObstacles()) {
				if (geom.intersects(footprintInGoal)) {
					intersects = true;
					break;
				}
			}
		}
		if (intersects) {
			String[] nearby = Missions.getNearLocations(this.goal[this.goal.length-1]);
			String freeLocation = null;
			for (String oneLoc : nearby) {
				Geometry footprint = getFootprintInPose(Missions.getLocationPose(oneLoc));
				boolean isFree = true;
				for (Geometry geom : this.getObstacles()) {
					if (geom.intersects(footprint)) {
						isFree = false;
						break;
					}
				}
				if (isFree) {
					freeLocation = oneLoc;
					break;
				}
			}
			if (freeLocation != null) {
				String[] newGoalLocationNames = new String[this.goalLocationNames.length+1];
				for (int i = 0; i < this.goalLocationNames.length-1; i++) newGoalLocationNames[i] = this.goalLocationNames[i];
				newGoalLocationNames[newGoalLocationNames.length-2] = freeLocation;
				newGoalLocationNames[newGoalLocationNames.length-1] = this.goalLocationNames[this.goalLocationNames.length-1];
				this.setGoals(newGoalLocationNames);
			}
		}
	}
	
	public static void main(String[] args) {
		Missions.loadRoadMap("/home/fpa/gitroot.gitlab/atlas_copco/coordination_oru_epiroc/output-good-depot/roadmap.txt");
		
//		String[] nearby = Missions.getNearLocations(Missions.getLocationPose("depot_1"));
//		for (String loc : nearby) System.out.println(loc);

		BrowserVisualization viz = new BrowserVisualization();
		String mapFile = "/home/fpa/gitroot.gitlab/atlas_copco/coordination_oru_epiroc/maps/plan_4650_obstacles_scaled_depot.yaml";
		viz.setMap(mapFile);
		viz.setInitialTransform(2.75, 148.0, -142.31);
		
		//ST-18 Footprint
		Coordinate[] st18footprint = new Coordinate[] {
				new Coordinate(-5.535,-1.450),
				new Coordinate(-5.535,1.450),
				new Coordinate(2.950,1.450),
				new Coordinate(2.950,1.665),
				new Coordinate(5.891,1.665),
				new Coordinate(5.891,-1.665),
				new Coordinate(2.950,-1.665),
				new Coordinate(2.950,-1.450),
				new Coordinate(-5.535,-1.450)
		};

		RoadMapPlanner rmp = new RoadMapPlanner(true);
		rmp.setFootprint(st18footprint);
		
		String start = "depot_1";
		String goal1 = "dump_4";
		String goal2 = "up_0_3";

		rmp.setStart(start);
		rmp.setGoals(goal1,goal2);

		try { Thread.sleep(2000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		if (!rmp.plan()) throw new Error("No plan found!");
		PoseSteering[] path = rmp.getPath();

		viz.addPath("path", path, 5, "#ff0000");

		try { Thread.sleep(2000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		viz.removePath("path", path);
		
		Geometry obstacle = rmp.getFootprintInPose(Missions.getLocationPose(goal2));
		rmp.addObstacles(new Geometry[] {obstacle});

		if (!rmp.plan()) throw new Error("No plan found!");
		path = rmp.getPath();
		
		viz.addPath("newPath", path, 5, "#0000ff");
		
	}

}
