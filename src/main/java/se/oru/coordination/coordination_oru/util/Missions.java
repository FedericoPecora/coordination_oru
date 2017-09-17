package se.oru.coordination.coordination_oru.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Scanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.Mission;

/**
 * This class collects utility methods for storing missions and extracting information from YAML files.
 * 
 * @author fpa
 *
 */
public class Missions {

	private static HashMap<String,Pose> locations = new HashMap<String, Pose>();
	private static HashMap<String,String> paths = new HashMap<String, String>();
	//private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorThreeRobots.class);
	private static HashMap<Integer,ArrayList<Mission>> missions = new HashMap<Integer, ArrayList<Mission>>();

	/**
	 * Get all the {@link Mission}s currently known for one robot
	 * @param robotID A robot identifier
	 * @return All the {@link Mission}s currently known for one robot
	 */
	public static ArrayList<Mission> getMissions(int robotID) {
		return missions.get(robotID);
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
	 * Normalize an angle to be within [-PI,PI).
	 * @param th The angle to normalize
	 * @return A value within [-PI,PI)
	 */
	public static double wrapAngle180(double th) {
	    return Math.atan2(Math.sin(th), Math.cos(th));
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
	 * Load location and path data from a file
	 * @param fileName The file to load the data from
	 */
	public static void loadLocationAndPathData(String fileName) {
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0 && !line.startsWith("#")) {
					String[] oneline = line.split(" |\t");
					Pose ps = null;
					if (line.contains("->")) {
						paths.put(oneline[0]+oneline[1]+oneline[2], oneline[3]);
						System.out.println("Added to paths paths: " + oneline[0]+oneline[1]+oneline[2] + " --> " + oneline[3]);
					}
					else {
						String locationName = oneline[0];
						ps = new Pose(
								new Double(oneline[1]).doubleValue(),
								new Double(oneline[2]).doubleValue(),
								new Double(oneline[3]).doubleValue());
						locations.put(locationName, ps);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
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

}
