package se.oru.coordination.coordination_oru.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Scanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Geometry;

import aima.core.search.nondeterministic.Path;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;

/**
 * This class collects utility methods for storing missions and extracting information from YAML files.
 * 
 * @author fpa
 *
 */
public class Missions {

	protected static HashMap<String,Pose> locations = new HashMap<String, Pose>();
	protected static HashMap<String,String> paths = new HashMap<String, String>();
	//private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorThreeRobots.class);
	protected static HashMap<Integer,ArrayList<Mission>> missions = new HashMap<Integer, ArrayList<Mission>>();
	protected static HashMap<Integer,Boolean> missionDispatcherFlags = new HashMap<Integer,Boolean>();
	protected static HashMap<Integer,MissionDispatchingCallback> mdcs = new HashMap<Integer, MissionDispatchingCallback>();
	protected static HashMap<Mission,ArrayList<Mission>> concatenatedMissions = new HashMap<Mission, ArrayList<Mission>>();
	protected static String pathPrefix = "";
	
	/**
	 * Get all the {@link Mission}s currently known for one robot
	 * @param robotID A robot identifier
	 * @return All the {@link Mission}s currently known for one robot
	 */
	public static ArrayList<Mission> getMissions(int robotID) {
		return missions.get(robotID);
	}
	
	/**
	 * Set the pose of a location.
	 * @param locationName The name of the location.
	 * @param p The pose of the location.
	 */
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
	 * Add a new {@link Mission} for a robot
	 * @param m The mission to push
	 */
	public static void putMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}

	/**
	 * Get the i-th mission for a given robot	
	 * @param robotID A robot identifier
	 * @param missionNumber The mission to get
	 * @return The i-th mission for a given robot
	 */
	public static Mission getMission(int robotID, int missionNumber) {
		return missions.get(robotID).get(missionNumber);
	}
	
	/**
	 * Pop the first mission from the queue of a given robot. 
	 * @param robotID The ID of the robot.
	 * @return The first mission from the queue of a given robot.
	 */
	public static Mission popMission(int robotID) {
		if (!missions.get(robotID).isEmpty()) {
			Mission m = missions.get(robotID).get(0);
			missions.get(robotID).remove(0);
			return m;
		}
		return null;
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
	 * Normalize an angle to be within [0,PI].
	 * @param th The angle to normalize
	 * @return A value within [0,PI]
	 */
	public static double wrapAngle360(double th) {
		return th-Math.PI*2.0*Math.floor(th/(Math.PI*2.0));
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
	public static HashMap<String,Pose> getLocations() {
		return locations;
	}
	
	/**
	 * Load location and path data from a file
	 * @param fileName The file to load the data from
	 */
	public static void loadLocationAndPathData(String fileName) {
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			File f = new File(fileName);
			pathPrefix = f.getAbsolutePath().substring(0,f.getAbsolutePath().lastIndexOf(File.separator))+File.separator;
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0 && !line.startsWith("#")) {
					String[] oneline = line.split(" |\t");
					Pose ps = null;
					if (line.contains("->")) {
						paths.put(oneline[0]+oneline[1]+oneline[2], oneline[3]);
						System.out.println("Added to paths: " + oneline[0]+oneline[1]+oneline[2] + " --> " + oneline[3]);
					}
					else {
						ArrayList<String> newLineContents = new ArrayList<String>();
						for (int i = 0; i < oneline.length; i++) {
							if (!oneline[i].trim().equals("")) newLineContents.add(oneline[i].trim()); 
						}
						String locationName = newLineContents.get(0);
						ps = new Pose(
								new Double(newLineContents.get(1)).doubleValue(),
								new Double(newLineContents.get(2)).doubleValue(),
								new Double(newLineContents.get(3)).doubleValue());
						locations.put(locationName, ps);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
	}

	/**
	 * Remove a location.
	 * @param locationName The name of the location to remove.
	 */
	public static void removeLocation(String locationName) {
		locations.remove(locationName);
	}
	
	/**
	 * Get locations from the data loaded by method loadLocationAndPathData()
	 * @param name The name of the location to get
	 * @return The {@link Pose} of the location
	 */
	public static Pose getLocation(String name) {
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
	 * Get path files from the data loaded by method loadLocationAndPathData()
	 * @param fromLocation The name of the source location
	 * @param toLocation The name of the destination location
	 * @return The name of the file where the path is stored 
	 */
	public static String getPathFile(String fromLocation, String toLocation) {
		String ret = paths.get(fromLocation+"->"+toLocation);
		if (!locations.containsKey(fromLocation)) throw new Error("Unknown location " + fromLocation);
		if (!locations.containsKey(toLocation)) throw new Error("Unknown location " + toLocation);
		if (ret == null) throw new Error("No path between " + fromLocation + " and " + toLocation);
		return ret;
	}
	
	/**
	 * Queries whether a path between two locations is known
	 * @param fromLocation The source location
	 * @param toLocation The goal location
	 * @return <code>true</code> iff a path between two locations is known
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
	public static PoseSteering[] loadKnownPath(String fromLocation, String toLocation) {
		if (!isKnownPath(fromLocation, toLocation)) throw new Error("No path between " + fromLocation + " and " + toLocation);
		return loadPathFromFile(pathPrefix+getPathFile(fromLocation, toLocation));
	}

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
            System.out.println("Saved path file: " + file.getAbsolutePath());
            PrintWriter writer = new PrintWriter(file);
            for (PoseSteering ps : path) {
            	writer.println(ps.getPose().getX() + "\t" + ps.getPose().getY() + "\t" + ps.getPose().getTheta() + "\t" + ps.getSteering());
            }
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}

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
	 * robot and dispatches them when the robot is free.
	 * @param tec The {@link TrajectoryEnvelopeCoordinator} that coordinates the missions.
	 * @param robotIDs The robot IDs for which a dispatching thread should be started.
	 */
	public static void startMissionDispatchers(final TrajectoryEnvelopeCoordinator tec, int ... robotIDs) {
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				@Override
				public void run() {
					while (missionDispatcherFlags.get(robotID)) {
						Mission m = Missions.getMission(robotID, iteration%Missions.getMissions(robotID).size());
						if (iteration > 0 && iteration%Missions.getMissions(robotID).size() == 0) {
							iteration++;
							continue;
						}
						synchronized(tec) {
							int numCats = 1;
							if (tec.isFree(m.getRobotID())) {
								//cat with future missions if necessary
								if (concatenatedMissions.containsKey(m)) {
									ArrayList<Mission> catMissions = concatenatedMissions.get(m);
									numCats = catMissions.size();
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
//								tec.computeCriticalSections();
//								tec.startTrackingAddedMissions();
								tec.computeCriticalSectionsAndStartTrackingAddedMission();
								iteration = iteration + numCats;
								if (mdcs.containsKey(robotID)) mdcs.get(robotID).afterMissionDispatch(m);
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
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0) {
					String[] oneline = line.split(" |\t");
					PoseSteering ps = null;
					if (oneline.length == 4) {
					ps = new PoseSteering(
							new Double(oneline[0]).doubleValue(),
							new Double(oneline[1]).doubleValue(),
							new Double(oneline[2]).doubleValue(),
							new Double(oneline[3]).doubleValue());
					}
					else {
						ps = new PoseSteering(
								new Double(oneline[0]).doubleValue(),
								new Double(oneline[1]).doubleValue(),
								new Double(oneline[2]).doubleValue(),
								0.0);					
					}
					ret.add(ps);
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
		return ret.toArray(new PoseSteering[ret.size()]);
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
	
	/**
	 * Splice a new path into an existing {@link Mission} at a given path point. 
	 * @param currentMission The {@link Mission} to splice the given path into.
	 * @param splice The path to splice into the given {@link Mission}.
	 * @param currentPathPoint The path point where the given path should be splices into the {@link Mission}.
	 */
	public static void spliceMission(Mission currentMission, PoseSteering[] splice, int currentPathPoint) {
		PoseSteering[] oldPath = currentMission.getPath();
		PoseSteering[] newPath = new PoseSteering[oldPath.length+splice.length-1];
		int counter = 0;
		for (int i = 0; i < currentPathPoint; i++) newPath[counter++] = oldPath[i];
		for (int i = 0; i < splice.length; i++) newPath[counter++] = splice[i];
		for (int i = currentPathPoint+1; i < oldPath.length; i++) newPath[counter++] = oldPath[i];
		currentMission.setPath(newPath);
	}

}
