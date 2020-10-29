package se.oru.coordination.coordination_oru.motionplanning;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.logging.Logger;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.awt.ShapeWriter;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.util.AffineTransformation;

public class OccupancyMap {

	protected static String TEMP_MAP_DIR = ".tempMaps";
	protected static int numCalls = 0;
	
	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
	
	public static boolean deleteDir(File dir) {
	    if (dir.isDirectory()) {
	        String[] children = dir.list();
	        for (int i=0; i<children.length; i++) {
	            boolean success = deleteDir(new File(dir, children[i]));
	            if (!success) {
	                return false;
	            }
	        }
	    }
	    return dir.delete();
	}
	
	static {
		deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();
	}
	
	private int mapWidth, mapHeight;
	private BitSet occupancyMapLinearBits = null;
	private double threshold = 0.3;
	private double mapResolution = 0.1;
	private Coordinate mapOrigin = new Coordinate(0.,0.);
	private BufferedImage bimg = null;
	private BufferedImage bimg_original = null;
	private ArrayList<Geometry> obstacles = new ArrayList<Geometry>();

	/**
	 * Create a new empty occupancy map (no obstacles, all in C_free).
	 * @param width The width of the map to create (in meters).
	 * @param height The height of the map to create (in meters).
	 * @param resolution The resolution of the map to create (in meters/pixel).
	 * @param mapOriginX The origin x of the map in the global frame in meters.
	 * @param mapOriginY The origin y of the map in the global frame in meters.
	 */
	public OccupancyMap(double width, double height, double resolution, double mapOriginX, double mapOriginY) {
		this.mapWidth = (int)(width/resolution);
		this.mapHeight= (int)(height/resolution);
		this.mapOrigin = new Coordinate(mapOriginX, mapOriginY);
		bimg = new BufferedImage(this.mapWidth, this.mapHeight, BufferedImage.TYPE_INT_RGB);
		Graphics2D g2 = bimg.createGraphics();
		g2.setPaint(Color.white);
		g2.fillRect(0, 0, this.mapWidth, this.mapHeight);
		g2.dispose();
		//--
		this.createOccupancyMap();
		this.bimg_original = deepCopy(this.bimg);
	}
	
	/**
	 * Create a new empty occupancy map (no obstacles, all in C_free).
	 * @param width The width of the map to create (in meters).
	 * @param height The height of the map to create (in meters).
	 * @param resolution The resolution of the map to create (in meters/pixel).
	 */
	public OccupancyMap(double width, double height, double resolution) {
		this(width, height, resolution, 0., 0.);
	}
	
	/**
	 * Create a new occupancy map that is identical to a given occupancy map.
	 * @param om The occupancy map to copy.
	 * @param copyObstacles <code>true</code> whether obstacles of the given map should be copied in the new map.
	 */
	public OccupancyMap(OccupancyMap om, boolean copyObstacles) {
		if (om == null) throw new Error("Null occupancy map passed as parameter.");
		this.mapWidth = om.mapWidth;
		this.mapHeight= om.mapHeight;
		this.mapOrigin = new Coordinate(om.mapOrigin.x, om.mapOrigin.y);
		this.threshold = om.threshold;
		this.mapResolution = om.mapResolution;
		if (copyObstacles) {
			this.obstacles = new ArrayList<Geometry>(om.obstacles);
			this.bimg = deepCopy(om.bimg);
		}
		else this.bimg = deepCopy(om.bimg_original);
		this.createOccupancyMap();
		this.bimg_original = deepCopy(om.bimg_original);
	}
	
	/**
	 * Create a new occupancy map from a given YAML file. The file is expected to look like this:
	 * <code>
	 * image: <mapfile.png>
	 * resolution: <resolution_in_meters/pixel>
	 * occupied_thresh: <least_pixel_value_that_is_considered_occupied>
	 * </code> 
	 * @param yamlFile The YAML file to construct the occupancy map from.
	 */
	public OccupancyMap(String yamlFile) {
		this.readMap(yamlFile);
		//--
		this.createOccupancyMap();
		this.bimg_original = deepCopy(this.bimg);
	}
	
	/**
	 * Get a {@link BufferedImage} of this occupancy map.
	 * @return A {@link BufferedImage} representing this occupancy map.
	 */
	public BufferedImage getMapImage() {
		return this.bimg;
	}
	
	private static BufferedImage deepCopy(BufferedImage bi) {
		 ColorModel cm = bi.getColorModel();
		 boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
		 WritableRaster raster = bi.copyData(null);
		 return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
	}

	/**
	 * Clear previously added obstacles from the occupancy map. This applies to all obstacles added
	 * via the {@link #addObstacles(Geometry...)} or {@link #addObstacles(Geometry, Pose...)} methods.
	 */
	public void clearObstacles() {
		this.bimg = deepCopy(this.bimg_original);
		this.obstacles.clear();
	}
	
	/**
	 * Get the geometries of the obstacles added to this occupancy map.
	 * @return The geometries of all obstacles that have been added to this occupancy map.
	 */
	public Geometry[] getObstacles() {
		if (this.obstacles.isEmpty()) return new Geometry[]{};
		return this.obstacles.toArray(new Geometry[this.obstacles.size()]);
	}
	
	/**
	 * Add obstacles to this occupancy map.
	 * @param obstacles One or more geometries of obstacles to add to this occupancy map.
	 */
	public void addObstacles(Geometry ... obstacles) {
		Graphics2D g2 = bimg.createGraphics();
		ShapeWriter writer = new ShapeWriter();
		g2.setPaint(Color.black);
		for (Geometry g : obstacles) {
			AffineTransformation at = new AffineTransformation();
			at.translate(-mapOrigin.x, -mapOrigin.y);
			at.scale(1.0/mapResolution, -1.0/mapResolution);
			at.translate(0, bimg.getHeight());
			Geometry scaledGeom = at.transform(g);
			Shape shape = writer.toShape(scaledGeom);
			//System.out.println("Shape: " + shape.getBounds2D());
			g2.fill(shape);
			this.obstacles.add(g);
		}
		g2.dispose();
		this.createOccupancyMap();
	}
	
	/**
	 * Save an image of the occupancy map with extra markings to indicate start and goal poses
	 * of a robot with a given footprint.
	 * @param startPose The start pose to mark.
	 * @param goalPose The end pose to mark.
	 * @param robotFoot The footprint to use in marking the start and goal poses.
	 * @param collidingPose One of the poses of path which is colliding with some obstacles.
	 */
	public void saveDebugObstacleImage(Pose startPose, Pose goalPose, Geometry robotFoot, Pose collidingPose) {
		BufferedImage copyForDebug = new BufferedImage(bimg.getWidth(), bimg.getHeight(), BufferedImage.TYPE_INT_RGB);
		Graphics2D g2 = copyForDebug.createGraphics();
		g2.drawImage(bimg, 0, 0, bimg.getWidth(), bimg.getHeight(), 0, 0, bimg.getWidth(), bimg.getHeight(), null);
		
		ShapeWriter writer = new ShapeWriter();
		float dash1[] = {2.0f};
	    BasicStroke dashed = new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10.0f, dash1, 0.0f);
	    g2.setStroke(dashed);
		
		g2.setPaint(Color.red);
		AffineTransformation atStart = new AffineTransformation();
		atStart.rotate(startPose.getTheta());
		atStart.translate(startPose.getX()-mapOrigin.x, startPose.getY()-mapOrigin.y);
		Geometry robotAtStart = atStart.transform(robotFoot);
		AffineTransformation atStartScale = new AffineTransformation();
		atStartScale.scale(1.0/mapResolution, -1.0/mapResolution);
		atStartScale.translate(0, copyForDebug.getHeight());
		Geometry scaledGeomStart = atStartScale.transform(robotAtStart);
		Shape shapeAtStart = writer.toShape(scaledGeomStart);
		g2.draw(shapeAtStart);
		//g2.fill(shapeAtStart);
		
		g2.setPaint(Color.green);
		AffineTransformation atGoal = new AffineTransformation();
		atGoal.rotate(goalPose.getTheta());
		atGoal.translate(goalPose.getX()-mapOrigin.x, goalPose.getY()-mapOrigin.y);
		Geometry robotAtGoal = atGoal.transform(robotFoot);
		AffineTransformation atGoalScale = new AffineTransformation();
		atGoalScale.scale(1.0/mapResolution, -1.0/mapResolution);
		atGoalScale.translate(0, copyForDebug.getHeight());
		Geometry scaledGeomGoal = atGoalScale.transform(robotAtGoal);
		Shape shapeAtGoal = writer.toShape(scaledGeomGoal);
		g2.draw(shapeAtGoal);
		//g2.fill(shapeAtGoal);
			
		if (collidingPose != null) {
			g2.setPaint(Color.blue);
			AffineTransformation inPose = new AffineTransformation();
			inPose.rotate(collidingPose.getTheta());
			inPose.translate(collidingPose.getX()-mapOrigin.x, collidingPose.getY()-mapOrigin.y);
			Geometry robotInPose = inPose.transform(robotFoot);
			AffineTransformation inPoseScale = new AffineTransformation();
			inPoseScale.scale(1.0/mapResolution, -1.0/mapResolution);
			inPoseScale.translate(0, copyForDebug.getHeight());
			Geometry scaledGeomPose = inPoseScale.transform(robotInPose);
			Shape shapeInPose = writer.toShape(scaledGeomPose);
			g2.draw(shapeInPose);
			//g2.fill(shapeInPose);
		}
		
		g2.dispose();
		
		//Save the map for debugging
		try {
			String filename = TEMP_MAP_DIR + File.separator + "tempMap_" + (numCalls++) + "_" + System.identityHashCode(this) + ".png";
			File outputfile = new File(filename);
			ImageIO.write(copyForDebug, "png", outputfile);
			metaCSPLogger.info("See image " + outputfile.getAbsolutePath() + " for more info on recent planning failure.");
		}
		catch (IOException e) { e.printStackTrace(); }

	}
	
	/**
	 * Add one or more obstacles with a given geometry placed in given poses. 
	 * @param geom The geometry of the obstacles to add.
	 * @param poses The poses in which to add obstacles.
	 * @return A list of geometries representing the added obstacles.
	 */
	public ArrayList<Geometry> addObstacles(Geometry geom, Pose ... poses) {
		ArrayList<Geometry> obstacles = new ArrayList<Geometry>();
		for (Pose pose : poses) {
			AffineTransformation atObs = new AffineTransformation();
			atObs.rotate(pose.getTheta());
			atObs.translate(pose.getX(), pose.getY());
			Geometry obs = atObs.transform(geom);
			obstacles.add(obs);			
		}
		this.addObstacles(obstacles.toArray(new Geometry[obstacles.size()]));
		return obstacles;
	}

	/**
	 * Get the resolution of this occupancy map.
	 * @return The resolution of this occupancy map (in meters/pixel).
	 */
	public double getResolution() {
		return this.mapResolution;
	}
	
	/**
	 * Get the origin of this occupancy map.
	 * @return The coordinates of the origin of this occupancy map in global frame.
	 */
	public Coordinate getMapOrigin() {
		return this.mapOrigin;
	}

	/**
	 * Get the threshold pixel value below which the pixel is considered to be occupied.
	 * @return The threshold pixel value below which the pixel is considered to be occupied.
	 */
	public double getThreshold() {
		return this.threshold;
	}
	
	/**
	 * Get a linear byte array representation of this occupancy map. Pixel (i,j) is in location (i*mapWidth+j).
	 * @return Linear byte array representation of this occupancy map, where (i,j) is in location (i*mapWidth+j).
	 */
	public byte[] asByteArray() {
		return this.occupancyMapLinearBits.toByteArray();
	}

	/**
	 * Get a {@link BufferedImage} representing this occupancy map.
	 * @return A {@link BufferedImage} representing this occupancy map.
	 */
	public BufferedImage asBufferedImage() {
		return this.bimg;
	}

	/**
	 * Get a two-color {@link BufferedImage} representing this occupancy map. A pixel is black iff it is considered occupied.
	 * @return A two-color {@link BufferedImage} representing this occupancy map, where all black pixels are occupied.
	 */
	public BufferedImage asThresholdedBufferedImage() {
		BufferedImage oimg = new BufferedImage(bimg.getWidth(), bimg.getHeight(), BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < this.mapHeight; y++) {
			for (int x = 0; x < this.mapWidth; x++) {
				if (this.isOccupied(x, y)) oimg.setRGB(x, y, new Color(0,0,0).getRGB());
				else oimg.setRGB(x, y, new Color(255,255,255).getRGB());
			}
		}
		return oimg;
	}

	/**
	 * Get the coordinates in pixel space corresponding to a given {@link Coordinate} in the workspace.
	 * @param coord A {@link Coordinate} within the workspace.
	 * @return The coordinates in pixel space corresponding to the given {@link Coordinate} in the workspace.
	 */
	public int[] toPixels(Coordinate coord) {
		return new int[] { (int)((coord.x-this.mapOrigin.x)/this.mapResolution), this.mapHeight-((int)((coord.y-this.mapOrigin.y)/this.mapResolution)) };
	}

	/**
	 * Get the {@link Coordinate}s in workspace corresponding to given coordinates in pixel space.
	 * @param x The x coordinate of the pixel in the occupancy map.
	 * @param y The y coordinate of the pixel in the occupancy map.
	 * @return The {@link Coordinate}s in workspace corresponding to given coordinates in pixel space.
	 */
	public Coordinate toWorldCoordiantes(int x, int y) {
		return new Coordinate(this.mapOrigin.x+(x+0.5)*this.mapResolution, this.mapOrigin.y+(this.mapHeight-y+0.5)*this.mapResolution);
	}

	/**
	 * Get the width of this occupancy map in pixels.
	 * @return The width of this occupancy map in pixels.
	 */
	public int getPixelWidth() {
		return this.mapWidth;
	}

	/**
	 * Get the height of this occupancy map in pixels.
	 * @return The height of this occupancy map in pixels.
	 */
	public int getPixelHeight() {
		return this.mapHeight;
	}

	/**
	 * Get the width of this occupancy map in the workspace coordinates.
	 * @return The width of this occupancy map in the workspace coordinates.
	 */
	public double getWorldWidth() {
		return this.mapWidth*mapResolution;
	}

	/**
	 * Get the height of this occupancy map in the workspace coordinates.
	 * @return The height of this occupancy map in the workspace coordinates.
	 */
	public double getWorldHeight() {
		return this.mapHeight*mapResolution;
	}

	/**
	 * Get the value of the occupancy map in a given pixel.
	 * @param pixelX The x coordinate of the pixel.
	 * @param pixelY The y coordinate of the pixel.
	 * @return The value of the occupancy map in the given pixel.
	 */
	public double getOccupancyValue(int pixelX, int pixelY) {
		if (this.bimg == null) throw new Error("No occupancy map!");
		return new Color(bimg.getRGB(pixelX,pixelY)).getRed()/255.0;
	}
	
	/**
	 * Return whether a given pixel of the occupancy map is occupied.
	 * @param pixelX The x coordinate of the pixel.
	 * @param pixelY The y coordinate of the pixel.
	 * @return <code>true</code> iff the given pixel of the occupancy map is occupied.
	 */
	public boolean isOccupied(int pixelX, int pixelY) {
		if (this.occupancyMapLinearBits == null) return false;
		return this.occupancyMapLinearBits.get(this.mapWidth*pixelY+pixelX);
	}

	/**
	 * Return whether a given {@link Coordinate} in the workspace is occupied.
	 * @param coord The coordinate to check in the workspace.
	 * @return <code>true</code> iff the given {@link Coordinate} in the workspace is occupied.
	 */
	public boolean isOccupied(Coordinate coord) {
		int[] pixel = toPixels(coord);
		return this.isOccupied(pixel[0], pixel[1]);
	}

	private void createOccupancyMap() {
		this.occupancyMapLinearBits = new BitSet();
		for(int y=0; y < bimg.getHeight(); y++){
			for(int x=0; x < bimg.getWidth(); x++){
				Color c = new Color(bimg.getRGB(x,y));
				this.occupancyMapLinearBits.set(y*mapWidth+x, c.getRed()/255.0 < this.threshold ? true : false);
			}
		}
		this.occupancyMapLinearBits.set(bimg.getHeight()*bimg.getWidth(), true);
	}

	private void readMap(String mapYAMLFile) {
		try {
			File file = new File(mapYAMLFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String st;
			while((st=br.readLine()) != null){
				if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
					String key = st.substring(0, st.indexOf(":")).trim();
					String value = st.substring(st.indexOf(":")+1).trim();
					if (key.equals("image")) this.loadImage(file.getParentFile()+File.separator+value);
					else if (key.equals("resolution")) this.mapResolution = Double.parseDouble(value);
					else if (key.equals("occupied_thresh")) this.threshold = Double.parseDouble(value);
					else if (key.equals("origin")) {
						String x = value.substring(1, value.indexOf(",")).trim();
						String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
						this.mapOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
					}
				}
			}
			br.close();
		}
		catch (IOException e) { e.printStackTrace(); }
	}

	private void loadImage(String imageFilename) {
		try {
			this.bimg = ImageIO.read(new File(imageFilename));

			for(int y=0; y < this.bimg.getHeight(); y++){
				for(int x=0; x < this.bimg.getWidth(); x++){
					Color color = new Color(this.bimg.getRGB(x,y));
					int graylevel = (color.getRed() + color.getGreen() + color.getBlue()) / 3;
					int r = graylevel;
					int g = graylevel;
					int b = graylevel;
					int rgb = 0xff000000 | (r << 16) | (g << 8) | b;
					this.bimg.setRGB(x, y, rgb);
				}
			}

			this.mapWidth = this.bimg.getWidth();
			this.mapHeight = this.bimg.getHeight();

		}
		catch (IOException e) { e.printStackTrace(); }
	}

}
