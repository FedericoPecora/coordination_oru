package se.oru.coordination.coordination_oru.motionplanning;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.awt.ShapeWriter;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.util.AffineTransformation;

public class OccupancyMap {

	private int mapWidth, mapHeight;
	private double[][] occupancyMap = null;
	private double[] occupancyMapLinear = null;
	private double threshold = 0.3;
	private double mapResolution = 0.1;
	private BufferedImage bimg = null;
	private BufferedImage oimg = null;
	private BufferedImage bimg_original = null;
	private ArrayList<Geometry> obstacles = new ArrayList<Geometry>();

	public OccupancyMap(double width, double height, double resolution) {
		this.mapWidth = (int)(width/resolution);
		this.mapHeight= (int)(height/resolution);
		bimg = new BufferedImage(this.mapWidth, this.mapHeight, BufferedImage.TYPE_INT_RGB);
		Graphics2D g2 = bimg.createGraphics();
		g2.setPaint(Color.white);
		g2.fillRect(0, 0, this.mapWidth, this.mapHeight);
		g2.dispose();
		//--
		this.createOccupancyMap();
		this.bimg_original = this.bimg;
	}
	
	public OccupancyMap(String yamlFile) {
		this.readMap(yamlFile);
		//--
		this.createOccupancyMap();
		this.bimg_original = this.bimg;
	}

	public void clearObstacles() {
		this.bimg = this.bimg_original;
		this.obstacles.clear();
	}
	
	public Geometry[] getObstacles() {
		if (this.obstacles.isEmpty()) return new Geometry[]{};
		return this.obstacles.toArray(new Geometry[this.obstacles.size()]);
	}
	
	public void addObstacles(Geometry ... obstacles) {
		Graphics2D g2 = bimg.createGraphics();
		ShapeWriter writer = new ShapeWriter();
		g2.setPaint(Color.black);
		for (Geometry g : obstacles) {
			AffineTransformation at = new AffineTransformation();
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
	
	public void addObstacles(Geometry geom, Pose ... poses) {
		ArrayList<Geometry> obstacles = new ArrayList<Geometry>();
		for (Pose pose : poses) {
			AffineTransformation atObs = new AffineTransformation();
			atObs.rotate(pose.getTheta());
			atObs.translate(pose.getX(), pose.getY());
			Geometry obs = atObs.transform(geom);
			obstacles.add(obs);			
		}
		this.addObstacles(obstacles.toArray(new Geometry[obstacles.size()]));
	}

	public double getResolution() {
		return this.mapResolution;
	}

	public double getThreshold() {
		return this.threshold;
	}

	public double[][] as2DArray() {
		return this.occupancyMap;
	}

	public double[] as1DArray() {
		return this.occupancyMapLinear;
	}

	public BufferedImage asBufferedImage() {
		return this.bimg;
	}

	public BufferedImage asThresholdedBufferedImage() {
		return this.oimg;
	}

	public int[] toPixels(Coordinate coord) {
		return new int[] { this.mapHeight-((int)(coord.y*this.mapResolution)), (int)(coord.x*(this.mapResolution)) };
	}

	public Coordinate toWorldCoordiantes(int x, int y) {
		return new Coordinate(x*this.mapResolution, (this.mapHeight-y)*this.mapResolution);
	}

	public int getPixelWidth() {
		return this.mapWidth;
	}

	public int getPixelHeight() {
		return this.mapHeight;
	}

	public double getWorldWidth() {
		return this.mapWidth*mapResolution;
	}

	public double getWorldHeight() {
		return this.mapHeight*mapResolution;
	}

	public boolean isOccupied(int pixelX, int pixelY) {
		if (this.occupancyMap == null) return false;
		return this.occupancyMap[pixelY][pixelX] < this.threshold;
	}

	public boolean isOccupied(Coordinate coord) {
		int[] pixel = toPixels(coord);
		return this.isOccupied(pixel[0], pixel[1]);
	}

	private void createOccupancyMap() {
		oimg = new BufferedImage(bimg.getWidth(), bimg.getHeight(), BufferedImage.TYPE_INT_RGB);
		this.occupancyMap = new double[this.mapHeight][this.mapWidth];
		this.occupancyMapLinear = new double[bimg.getHeight()*bimg.getWidth()];
		for(int y=0; y < bimg.getHeight(); y++){
			for(int x=0; x < bimg.getWidth(); x++){
				Color c = new Color(bimg.getRGB(x,y));
				this.occupancyMap[y][x] = c.getRed()/255.0;
				this.occupancyMapLinear[y*mapWidth+x] = this.occupancyMap[y][x];
				if (this.occupancyMap[y][x] <= threshold) oimg.setRGB(x, y, new Color(0,0,0).getRGB());
				else oimg.setRGB(x, y, new Color(255,255,255).getRGB());
			}
		}
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
						//FIXME: deal with map origin
						//this.setMapOrigin(new Coordinate(Double.parseDouble(x),Double.parseDouble(y)));
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
