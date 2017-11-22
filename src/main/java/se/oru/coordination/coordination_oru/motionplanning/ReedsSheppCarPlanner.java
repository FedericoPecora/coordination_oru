package se.oru.coordination.coordination_oru.motionplanning;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.awt.ShapeWriter;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlannerLib.PathPose;
import se.oru.coordination.coordination_oru.util.GeometrySmoother;
import se.oru.coordination.coordination_oru.util.GeometrySmoother.SmootherControl;
import se.oru.coordination.coordination_oru.util.Missions;

public class ReedsSheppCarPlanner {

	private static String TEMP_MAP_DIR = ".tempMaps";
	private int numObstacles = 0;
	private String mapFilename = null;
	private String mapFilenameBAK = null;
	private double mapResolution = 1.0;
	private double robotRadius = 1.0;
	private Pose start = null;
	private Pose[] goal = null;
	private PointerByReference path = null;
	private IntByReference pathLength = null;
	private double distanceBetweenPathPoints = 0.5;
	private double turningRadius = 1.0;
	private PoseSteering[] pathPS = null;
	private Coordinate[] collisionCircleCenters = null;
	
	public static ReedsSheppCarPlannerLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("simplereedssheppcarplanner", "SimpleReedsSheppCarPlanner");
		INSTANCE = Native.loadLibrary("simplereedssheppcarplanner", ReedsSheppCarPlannerLib.class);
	}
	
	public void setFootprint(Coordinate ... coords) {
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[coords.length+1];
		for (int i = 0; i < coords.length; i++) {
			newCoords[i] = coords[i];
		}
		newCoords[newCoords.length-1] = coords[0];
		Polygon footprint = gf.createPolygon(newCoords);
		GeometrySmoother gs = new GeometrySmoother(gf);
		SmootherControl sc = new SmootherControl() {
	        public double getMinLength() {
	            return robotRadius;
	        }
	        
	        public int getNumVertices(double length) {
	            return (int)(length/(2*robotRadius))+2;
	        }
	    };
	    gs.setControl(sc);
	    Polygon smoothFootprint = gs.smooth(footprint, 1.0);
		collisionCircleCenters = smoothFootprint.getCoordinates();
	}
	
	public Coordinate[] getCollisionCircleCenters() {
		return collisionCircleCenters;
	}
	
	public ReedsSheppCarPlanner() {
		deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();
	}

	public void setStart(Pose p) {
		Pose newStart = new Pose(p.getX(),p.getY(),Missions.wrapAngle180(p.getTheta()));
		this.start = newStart;
	}

	@Deprecated
	public void setGoal(Pose p) {
		this.setGoals(p);
	}
	
	public void setGoals(Pose ... p) {
		ArrayList<Pose> newGoals = new ArrayList<Pose>();
		for (Pose pose : p) newGoals.add(new Pose(pose.getX(),pose.getY(),Missions.wrapAngle180(pose.getTheta())));
		this.goal = newGoals.toArray(new Pose[newGoals.size()]);
	}
	
	public void setCirclePositions(Coordinate ... circlePositions) {
		this.collisionCircleCenters = circlePositions;
	}

	public void setMapFilename(String filename) {
		this.mapFilename = filename;
		this.mapFilenameBAK = filename;
	}

	public void setMapResolution(double res) {
		this.mapResolution = res;
	}

	@Deprecated
	public void setRobotRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setDistanceBetweenPathPoints(double maxDistance) {
		this.distanceBetweenPathPoints = maxDistance;
	}

	public void setTurningRadius(double rad) {
		this.turningRadius = rad;
	}

	public void addObstacles(Geometry ... geom) {
		if (this.mapFilename == null) throw new Error("Please set a map file first!");
		BufferedImage img = null;
		try {
			img = ImageIO.read(new File(mapFilename)); 
			System.out.println("IMGTYPE: " + img.getType());
			Graphics2D g2 = img.createGraphics();
			ShapeWriter writer = new ShapeWriter();
			g2.setPaint(Color.black);
			for (Geometry g : geom) {
				AffineTransformation at = new AffineTransformation();
				at.scale(1.0/mapResolution, -1.0/mapResolution);
				at.translate(0, img.getHeight());
				Geometry scaledGeom = at.transform(g);
				Shape shape = writer.toShape(scaledGeom);
				System.out.println("Shape: " + shape.getBounds2D());
				g2.fill(shape);
			}
			File outputfile = new File(TEMP_MAP_DIR + File.separator + "tempMap" + (numObstacles++) + ".png");
			ImageIO.write(img, "png", outputfile);
			this.mapFilename = outputfile.getAbsolutePath();
		}
		catch (IOException e) { e.printStackTrace(); }		
	}

	public void addObstacles(Geometry geom, Pose ... poses) {
		if (this.mapFilename == null) throw new Error("Please set a map file first!");
		BufferedImage img = null;
		try {
			img = ImageIO.read(new File(mapFilename)); 
			Graphics2D g2 = img.createGraphics();
			ShapeWriter writer = new ShapeWriter();
			g2.setPaint(Color.black);
			for (Pose pose : poses) {
				AffineTransformation at = new AffineTransformation();
				at.rotate(pose.getTheta());
				at.translate(pose.getX(), pose.getY());
				at.scale(1.0/mapResolution, -1.0/mapResolution);
				at.translate(0, img.getHeight());
				Geometry scaledGeom = at.transform(geom);
				Shape shape = writer.toShape(scaledGeom);
				System.out.println("Shape: " + shape.getBounds2D());
				g2.fill(shape);
			}
			File outputfile = new File(TEMP_MAP_DIR + File.separator + "tempMap" + (numObstacles++) + ".png");
			ImageIO.write(img, "png", outputfile);
			this.mapFilename = outputfile.getAbsolutePath();
		}
		catch (IOException e) { e.printStackTrace(); }
	}

	public void clearObstacles() {
		this.setMapFilename(this.mapFilenameBAK);
	}

	public PoseSteering[] getPath() {
		return this.pathPS;
	}
	
	public PoseSteering[] getPathInv() {
		ArrayList<PoseSteering> inv = new ArrayList<PoseSteering>();
		for (PoseSteering ps : this.pathPS) inv.add(ps);
		Collections.reverse(inv);
		return inv.toArray(new PoseSteering[inv.size()]);
	}

	public boolean plan() {
		ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();  
		for (int i = 0; i < this.goal.length; i++) {
			Pose start_ = null;
			Pose goal_ = this.goal[i];
			if (i == 0) start_ = this.start;
			else start_ = this.goal[i-1];
			path = new PointerByReference();
			pathLength = new IntByReference();
			if (collisionCircleCenters == null) {
				if (!INSTANCE.plan(mapFilename, mapResolution, robotRadius, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius)) return false;
			}
			else {
				double[] xCoords = new double[collisionCircleCenters.length];
				double[] yCoords = new double[collisionCircleCenters.length];
				int numCoords = collisionCircleCenters.length;
				for (int j = 0; j < collisionCircleCenters.length; j++) {
					xCoords[j] = collisionCircleCenters[j].x;
					yCoords[j] = collisionCircleCenters[j].y;
				}
				System.out.println("Path planning with " + collisionCircleCenters.length + " circle positions");
				if (this.mapFilename != null) {
					if (!INSTANCE.plan_multiple_circles(mapFilename, mapResolution, robotRadius, xCoords, yCoords, numCoords, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius)) return false;					
				}
				else {
					if (!INSTANCE.plan_multiple_circles_nomap(xCoords, yCoords, numCoords, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius)) return false;					
				}
			}
			final Pointer pathVals = path.getValue();
			final PathPose valsRef = new PathPose(pathVals);
			valsRef.read();
			int numVals = pathLength.getValue();
			PathPose[] pathPoses = (PathPose[])valsRef.toArray(numVals);
			if (i == 0) finalPath.add(new PoseSteering(pathPoses[0].x, pathPoses[0].y, pathPoses[0].theta, 0.0));
			for (int j = 1; j < pathPoses.length; j++) finalPath.add(new PoseSteering(pathPoses[j].x, pathPoses[j].y, pathPoses[j].theta, 0.0));
			INSTANCE.cleanupPath(pathVals);
		}
		this.pathPS = finalPath.toArray(new PoseSteering[finalPath.size()]);
		return true;
	}

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

}
