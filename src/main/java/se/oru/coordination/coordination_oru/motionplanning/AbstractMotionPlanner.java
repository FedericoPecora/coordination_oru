package se.oru.coordination.coordination_oru.motionplanning;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.util.Missions;

public abstract class AbstractMotionPlanner {

	protected Logger metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
	
	protected Pose start = null;
	protected Pose[] goal = null;
	protected Coordinate[] footprintCoords = null;
	protected boolean verifyPlanning = true;
	protected Pose collidingPose = null;
	protected OccupancyMap om = null;
	protected boolean noMap = true;
	
	protected PoseSteering[] pathPS = null;

	public abstract AbstractMotionPlanner getCopy(boolean copyObstacles);
	
	public void setFootprint(Coordinate ... coords) {
		this.footprintCoords = coords;
	}
	
	public void setStart(Pose p) {
		Pose newStart = new Pose(p.getX(),p.getY(),Missions.wrapAngle180b(p.getTheta()));
		//Pose newStart = new Pose(p.getX(),p.getY(),Missions.wrapAngle180a(p.getTheta()));
		//Pose newStart = new Pose(p.getX(),p.getY(),Missions.wrapAngle180(p.getTheta()));
		//Pose newStart = new Pose(p.getX(),p.getY(),Missions.wrapAngle360(p.getTheta()));
		this.start = newStart;
	}
	
	@Deprecated
	public void setGoal(Pose p) {
		this.setGoals(p);
	}
	
	public void addGoals(Pose ... p) {
		ArrayList<Pose> newGoals = new ArrayList<Pose>();
		if (this.goal != null && this.goal.length > 0) {
			for (Pose g : this.goal) newGoals.add(g);
		}
		for (Pose ng : p) newGoals.add(ng);
		this.setGoals(newGoals.toArray(new Pose[newGoals.size()]));
	}
	
	public void setGoals(Pose ... p) {
		ArrayList<Pose> newGoals = new ArrayList<Pose>();
		if (p != null) {
			Pose prev = null;
			for (Pose pose : p) {
				if (prev == null || prev != null && !prev.equals(pose))
					newGoals.add(new Pose(pose.getX(),pose.getY(),Missions.wrapAngle180b(pose.getTheta())));
				else metaCSPLogger.warning("Removing duplicated useless goal " + pose.toString() + ".");
				prev = pose;
			}
		}
		//for (Pose pose : p) newGoals.add(new Pose(pose.getX(),pose.getY(),Missions.wrapAngle180a(pose.getTheta())));
		//for (Pose pose : p) newGoals.add(new Pose(pose.getX(),pose.getY(),Missions.wrapAngle180(pose.getTheta())));
		//for (Pose pose : p) newGoals.add(new Pose(pose.getX(),pose.getY(),Missions.wrapAngle360(pose.getTheta())));
		this.goal = newGoals.toArray(new Pose[newGoals.size()]);
	}
		
	public void setMap(String mapYAMLFile) {
		this.noMap = false;
		this.om = new OccupancyMap(mapYAMLFile);
	}
		
	public PoseSteering[] getPath() {
		return this.pathPS;
	}
	
	public PoseSteering[] getPathInv() {
		if (this.pathPS == null) return this.pathPS;
		ArrayList<PoseSteering> inv = new ArrayList<PoseSteering>();
		for (PoseSteering ps : this.pathPS) inv.add(ps);
		Collections.reverse(inv);
		return inv.toArray(new PoseSteering[inv.size()]);
	}
	
	public synchronized void addObstacles(Geometry geom, Pose ... poses) {
		if (this.om == null) this.om = new OccupancyMap(1000, 1000, 1);
		this.om.addObstacles(geom, poses);
	}
	
	public synchronized void addObstacles(Geometry ... geom) {
		if (this.om == null) this.om = new OccupancyMap(1000, 1000, 1);
		this.om.addObstacles(geom);
	}
	
	public synchronized void writeDebugImage() {
		om.saveDebugObstacleImage(this.start, this.goal[this.goal.length-1], getFootprintAsGeometry(), this.collidingPose);
	}
	
	public synchronized void clearObstacles() {
		if (this.noMap) this.om = null;
		else this.om.clearObstacles();
	}
	
	public OccupancyMap getOccupancyMap() {
		return this.om;
	}

	/**
	 * This method needs to write the protected field PoseSteering[] pathPS.
	 * @return <code>true</code> iff path planning was successful.
	 */
	public abstract boolean doPlanning();
	
	private Geometry getFootprintAsGeometry() {
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newFoot = new Coordinate[footprintCoords.length+1];
		for (int j = 0; j < footprintCoords.length; j++) {
			newFoot[j] = footprintCoords[j];
		}
		newFoot[footprintCoords.length] = footprintCoords[0];
		Geometry foot = gf.createPolygon(newFoot);
		return foot;
	}
	
	public synchronized boolean plan() {
		Geometry goalFoot = getFootprintAsGeometry();
		AffineTransformation at = new AffineTransformation();
		at.rotate(this.goal[this.goal.length-1].getTheta());
		at.translate(this.goal[this.goal.length-1].getX(), this.goal[this.goal.length-1].getY());
		goalFoot = at.transform(goalFoot);
		if (this.om != null) {
			for (Geometry obs : this.om.getObstacles()) {
				if (obs.intersects(goalFoot)) {
					metaCSPLogger.info("Goal intersects with an obstacle, no path can exist");
					return false;
				}
			}
		}
		
		boolean ret = doPlanning();
		
		if (!verifyPlanning) return ret;
		
		PoseSteering[] path = getPath();
		if (path == null) {
			metaCSPLogger.info("Path planner could not find a plan");
			return false;
		}

		for (int i = 0; i < path.length; i++) {
			Pose p = path[i].getPose();
			Geometry checkFoot = getFootprintAsGeometry();
			at = new AffineTransformation();
			at.rotate(p.getTheta());
			at.translate(p.getX(), p.getY());
			checkFoot = at.transform(checkFoot);
			if (this.om != null) {
				for (Geometry obs : this.om.getObstacles()) {
					if (obs.intersects(checkFoot)) {
						collidingPose = new Pose(p.getX(),p.getY(),p.getTheta());
						metaCSPLogger.info("Path verification failed");
						return false;
					}
				}
			}
		}
	
		return ret;
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
	
	public boolean isFree(Pose p, boolean copyObstacles) {
		AbstractMotionPlanner planner = this.getCopy(copyObstacles);
		planner.setStart(p);
		planner.setGoals(p);
		return planner.plan();
	}

}
