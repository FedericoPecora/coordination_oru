package se.oru.coordination.coordination_oru.fleetmasterinterface;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import javax.imageio.ImageIO;
import com.sun.jna.NativeLong;

/**
 * TODO Write down class description. 
 * @author am
 *
 */
public class FleetMasterInterface extends AbstractFleetMasterInterface {
	
	/**
	 * Class constructor.
	 * ATTENTION: If dynamic_size is <code>false</code>, then the user should check that all the paths will lay in the given area.
	 * @param origin_x The x coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta origin of the lower-left pixel map (counterclockwise rotation. 
	 * 					   However, many parts of the system currently ignore it).
	 * @param resolution The resolution of the map (in meters/cell). It is assumed this parameter to be global among the fleet.
	 * @param width Number of columns of the map (in cells) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (in cells) if dynamic sizing is not enabled.
	 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
	 * @param debug If <code>true</code>, it enables writing to screen debugging info.
	 */
	public FleetMasterInterface(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean debug) {
		super(origin_x, origin_y, origin_theta, resolution, width,  height, dynamic_size, debug);
		init();
	}
	
	/**
	 * Class constructor.
	 * ATTENTION: If dynamic_size is <code>false</code>, then the user should check that all the paths will lay in the given area.
	 * @param origin_x The x coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_y The y coordinate (in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
	 * @param origin_theta The theta origin of the lower-left pixel map (counterclockwise rotation. 
	 * 					   However, many parts of the system currently ignore it).
	 * @param resolution The resolution of the map (in meters/cell). It is assumed this parameter to be global among the fleet.
	 * @param width Number of columns of the map (in cells) if dynamic sizing is not enabled.
	 * @param height Number of rows of the map (in cells) if dynamic sizing is not enabled.
	 */
	public FleetMasterInterface(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size) {
		this(origin_x, origin_y, origin_theta, resolution, width,  height, dynamic_size, false);
	}
	
	/**
	 * Class constructor when not using dynamic sizing (e.g., for small maps). All the grid parameters are set according to the yaml file.
	 * @param mapYAMLFile file to the global map of the environment (path specified as "/.../...").
	 * Possible: absolute paths or stored in the maps directory of the coordination_oru package (e.g., "maps/map-empty");
	 * Note: it is required the map to contain all the possible paths.
	 */
	@Deprecated
	public FleetMasterInterface(String mapYAMLFile, boolean debug) {
		super();
		
		double origin_x = Double.NaN;
		double origin_y = Double.NaN;
		double origin_theta = Double.NaN;
		double resolution = Double.NaN;
		long width = -1;
		long height = -1;
		int keys_found = 0;
		
		try {
			File file = new File(mapYAMLFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String st;
			while((st=br.readLine()) != null){
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals("image")) {
					String filename = value.substring(0).trim();
					if (!value.contains("home")) {
						filename = new String("maps/").concat(filename);
					}
					File image = new File(filename);
					BufferedImage bimg = ImageIO.read(image);
					width = bimg.getWidth();
					height = bimg.getHeight();
					keys_found += 2;
				}
				else if (key.equals("resolution")) {
					resolution = Double.parseDouble(value);
					keys_found += 1;
				}
				else if (key.equals("origin")) {
					int first_comma = value.indexOf(",");
					int second_comma = first_comma + 1 + value.substring(first_comma + 1).indexOf(",");
					origin_x = Double.parseDouble(value.substring(1, first_comma));
					origin_y = Double.parseDouble(value.substring(first_comma + 1, second_comma));
					int rbrake = value.indexOf("]");
					origin_theta = Double.parseDouble(value.substring(second_comma + 1, rbrake));
					keys_found += 3;
				}
			}
			br.close();
		}
		catch (IOException e) { e.printStackTrace(); }
		
		if (keys_found != 6) throw new Error("Error in parsing the yaml file");
		
		//Set the grid parameters according to the provided yaml file
		gridParams.origin.x = origin_x;
		gridParams.origin.y = origin_y;
		gridParams.origin.theta = origin_theta;
		gridParams.resolution = resolution;
		gridParams.width = new NativeLong(width);
		gridParams.height = new NativeLong(height);
		gridParams.dynamic_size = false;
		gridParams.debug = debug;
		
		init();
	}
}