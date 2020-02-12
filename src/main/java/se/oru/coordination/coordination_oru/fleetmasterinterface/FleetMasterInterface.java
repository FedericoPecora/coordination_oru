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
	 * ATTENTION: It is required the user to check that all the paths will lay in the given area.
	 * @param origin_x The x origin of the map.
	 * @param origin_y The y origin of the map.
	 * @param origin_theta The theta origin of the map.
	 * @param resolution The resolution of the map (in meters/cell).
	 * @param width Number of columns of the map (in cells).
	 * @param height Number of rows of the map (in cells).
	 * @param debug <code>true</code> enables writing to screen debugging info.
	 */
	@Deprecated
	public FleetMasterInterface(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean debug) {
		super(origin_x, origin_y, origin_theta, resolution, width,  height, debug);
		init();
	}
	
	/**
	 * Class constructor.
	 * ATTENTION: It is required the user to check that all the paths will lay in the given area.
	 * @param origin_x The x origin of the map.
	 * @param origin_y The y origin of the map.
	 * @param origin_theta The theta origin of the map.
	 * @param resolution The resolution of the map (in meters/cell).
	 * @param width Number of columns of the map (in cells).
	 * @param height Number of rows of the map (in cells).
	 */
	@Deprecated
	public FleetMasterInterface(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height) {
		this(origin_x, origin_y, origin_theta, resolution, width,  height, false);
	}
	
	/**
	 * Class constructor.
	 * @param mapYAMLFile file to the global map of the environment (path specified as "/.../...").
	 * Possible: absolute paths or stored in the maps directory of the coordination_oru package (e.g., "maps/map-empty");
	 * Note: it is required the map to contain all the possible paths.
	 */
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
		gridParams.debug = debug;
		
		init();
	}
}