package se.oru.coordination.coordination_oru.motionplanning;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import javax.imageio.ImageIO;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;
import com.vividsolutions.jts.awt.ShapeWriter;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlannerLib.PathPose;

public class ReedsSheppCarPlanner {

	private static String TEMP_MAP_DIR = ".tempMaps";
	private static int numObstacles = 0;
	private String mapFilename = null;
	private String mapFilenameBAK = null;
	private double mapResolution = 1.0;
	private double robotRadius = 1.0;
	private Pose start = null;
	private Pose goal = null;
	private PointerByReference path = null;
	private IntByReference pathLength = null;
	private int numInterpolationPoints = 0;
	private double turningRadius = 1.0;
	private PoseSteering[] pathPS = null;

	public ReedsSheppCarPlanner() {
		deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();
	}

	public void setStart(Pose p) {
		this.start = p;
	}

	public void setGoal(Pose p) {
		this.goal = p;
	}

	public void setMapFilename(String filename) {
		this.mapFilename = filename;
		this.mapFilenameBAK = filename;
	}

	public void setMapResolution(double res) {
		this.mapResolution = res;
	}

	public void setRobotRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setNumInterpolationPoints(int num) {
		this.numInterpolationPoints = num;
	}

	public void setTurningRadius(double rad) {
		this.turningRadius = rad;
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
		path = new PointerByReference();
		pathLength = new IntByReference();
		if (ReedsSheppCarPlannerLib.INSTANCE.plan(mapFilename, mapResolution, robotRadius, start.getX(), start.getY(), start.getTheta(), goal.getX(), goal.getY(), goal.getTheta(), path, pathLength, numInterpolationPoints, turningRadius)) {
			final Pointer pathVals = path.getValue();
			final PathPose valsRef = new PathPose(pathVals);
			valsRef.read();
			int numVals = pathLength.getValue();
			PathPose[] pathPoses = (PathPose[])valsRef.toArray(numVals);
			pathPS = new PoseSteering[pathPoses.length];
			for (int i = 0; i < pathPoses.length; i++) {
				pathPS[i] = new PoseSteering(pathPoses[i].x, pathPoses[i].y, pathPoses[i].theta, 0.0);
			}
			ReedsSheppCarPlannerLib.INSTANCE.cleanupPath(pathVals);
			return true;
		}
		return false;
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
