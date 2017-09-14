package se.oru.coordination.coordination_oru.util;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Scanner;
import java.util.TreeMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.UI.JTSDrawingPanel;

import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;

public class PathEditor {

	private String fileName = null;
	private boolean selectionInputListen = false;
	private String selectedPathPoint = "";
	//private int selectedPathPointInt = -1;
	private ArrayList<Integer> selectedPathPointInt = new ArrayList<Integer>();
	private ArrayList<PoseSteering> path = null;
	private ArrayList<ArrayList<PoseSteering>> oldPaths = new ArrayList<ArrayList<PoseSteering>>(); 
	private JTSDrawingPanel panel = null;
	private double deltaX = 0.1;
	private double deltaY = 0.1;
	private double deltaT = 0.1;
	private double deltaTR = 0.1;
	private String newFileSuffix = ".new";
	private double minDistance = 1.0;
	private double maxTurningRadius = 4.0;

	public PathEditor(String fileName, double deltaX, double deltaY, double deltaTheta, String newFileSuffix, double minDistance, double maxTurningRadius) {
		this.fileName = fileName;
		this.deltaX = deltaX;
		this.deltaY = deltaY;
		this.deltaT = deltaTheta;
		this.newFileSuffix = newFileSuffix;
		this.minDistance = minDistance;
		this.maxTurningRadius = maxTurningRadius;
		this.setupGUI();
		this.readPath(fileName);
	}
	
	public PathEditor(String fileName) {
		this(fileName,0.1,0.1,0.1,".new",0.4,4.0);
	}
	
	private void setupGUI() {
		panel = JTSDrawingPanel.makeEmpty("Path Editor");
		panel.addKeyListener(new KeyListener() {
			@Override
			public void keyTyped(KeyEvent e) { }
			@Override
			public void keyReleased(KeyEvent e) { }
			@Override
			public void keyPressed(KeyEvent e) {
				if (path != null && path.size() > 0) {
					if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
						selectedPathPoint = "";
						selectedPathPointInt.clear();
						selectionInputListen = false;
						for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
						panel.updatePanel();
					}
					else if (e.getKeyCode() == KeyEvent.VK_INSERT) {
						if (!selectedPathPointInt.isEmpty()) {
							ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
							oldPaths.add(oldPath);
							HashMap<Integer,PoseSteering> toAdd = new HashMap<Integer, PoseSteering>();
							for (int selectedPathPointOneInt : selectedPathPointInt) {
								PoseSteering newPoseSteering = new PoseSteering(path.get(selectedPathPointOneInt).getPose().getX()+deltaX, path.get(selectedPathPointOneInt).getPose().getY()+deltaY, path.get(selectedPathPointOneInt).getPose().getTheta(), path.get(selectedPathPointOneInt).getSteering());
								toAdd.put(selectedPathPointOneInt, newPoseSteering);
							}
							for (int selectedPathPointOneInt : selectedPathPointInt) {
								path.add(selectedPathPointInt.get(0), toAdd.get(selectedPathPointOneInt));
							}
							for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
							for (int selectedPathPointOneInt : selectedPathPointInt) {
								panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
							}
							panel.updatePanel();
						}				
					}
					else if (e.getKeyCode() == KeyEvent.VK_Z && e.isControlDown()) {
						if (!oldPaths.isEmpty()) {
							for (int i = 0; i < path.size(); i++) panel.removeGeometry(""+i);
							path = oldPaths.get(oldPaths.size()-1);
							oldPaths.remove(oldPaths.size()-1);
							for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
							if (!selectedPathPointInt.isEmpty()) {
								for (int selectedPathPointOneInt : selectedPathPointInt) {
									panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
								}
							}
							panel.updatePanel();
						}
					}
					else if (e.getKeyCode() == KeyEvent.VK_S && e.isControlDown()) {
						String newFileName = fileName+newFileSuffix;
						writePath(newFileName);
						System.out.println("Saved " + newFileName);
					}
					else if (e.getKeyCode() == KeyEvent.VK_S) {
						if (!selectionInputListen) {
							selectedPathPoint = "";
							selectedPathPointInt.clear();;
							for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
							panel.updatePanel();
							System.out.println("Input selection: " + selectedPathPoint);
						}
						else if (selectionInputListen) {
							for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
							try {
								if (selectedPathPoint.contains("-")) {
									int first = Integer.parseInt(selectedPathPoint.substring(0,selectedPathPoint.indexOf("-")));
									int last = Integer.parseInt(selectedPathPoint.substring(selectedPathPoint.indexOf("-")+1));
									for (int i = first; i <= last; i++) selectedPathPointInt.add(i);
								}
								else if (selectedPathPoint.contains(",")) {
									String[] points = selectedPathPoint.split(",");
									for (String point : points) selectedPathPointInt.add(Integer.parseInt(point));
								}
								else selectedPathPointInt.add(Integer.parseInt(selectedPathPoint));
								boolean fail = false;
								for (int selectedPathPointOneInt : selectedPathPointInt) {
									if (selectedPathPointOneInt >= 0 && selectedPathPointOneInt < path.size()) {
										panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
									}
									else {
										fail = true;
									}									
								}
								if (fail) {
									selectedPathPoint = "";
									selectedPathPointInt.clear();
									for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
								}
								panel.updatePanel();
								System.out.println("Current selection: " + selectedPathPointInt);
							}
							catch(NumberFormatException ex) { }
						}
						selectionInputListen = !selectionInputListen;
					}
					else if (selectionInputListen) {
						selectedPathPoint += e.getKeyChar();
						System.out.println("Input selection: (press 's' again to set): " + selectedPathPoint);
					}
					else if (!selectionInputListen && !selectedPathPointInt.isEmpty()) {
						if (e.getKeyCode() == KeyEvent.VK_DELETE) {
							for (int i = 0; i < path.size(); i++) panel.removeGeometry(""+i);
							ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
							oldPaths.add(oldPath);
							ArrayList<PoseSteering> toRemove = new ArrayList<PoseSteering>();
							for (int selectedPathPointOneInt : selectedPathPointInt) {
								toRemove.add(path.get(selectedPathPointOneInt));
							}
							path.removeAll(toRemove);
							selectedPathPoint = "";
							selectedPathPointInt.clear();
							for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
							panel.updatePanel();
						}
						else if (e.getKeyCode() == KeyEvent.VK_P && e.isControlDown()) {
							if (selectedPathPointInt.size() == 2) {
								PoseSteering[] newSubPath = computePath(path.get(selectedPathPointInt.get(0)), path.get(selectedPathPointInt.get(1)));
								if (newSubPath != null) {
									ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
									oldPaths.add(oldPath);
									TreeMap<Integer,PoseSteering> toAdd = new TreeMap<Integer, PoseSteering>();
									int selectedStart = selectedPathPointInt.get(0)+1;
									for (int i = 1; i < newSubPath.length-1; i++) {
										PoseSteering newPoseSteering = new PoseSteering(newSubPath[i].getPose().getX(), newSubPath[i].getPose().getY(), newSubPath[i].getPose().getTheta(), newSubPath[i].getSteering());
										toAdd.put(i+selectedStart-1, newPoseSteering);
									}
									selectedPathPointInt.clear();
									for (Entry<Integer,PoseSteering> en : toAdd.entrySet()) {
										path.add(en.getKey(), en.getValue());
										selectedPathPointInt.add(en.getKey());
									}
									System.out.println("NEW INTS: " + selectedPathPointInt);
									for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
									for (int selectedPathPointOneInt : selectedPathPointInt) {
										panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
									}
									panel.updatePanel();
								}
							}
						}
						else {
							ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
							oldPaths.add(oldPath);
							for (int selectedPathPointOneInt : selectedPathPointInt) {
								double x = path.get(selectedPathPointOneInt).getPose().getX();
								double y = path.get(selectedPathPointOneInt).getPose().getY();
								double th = path.get(selectedPathPointOneInt).getPose().getTheta();
								if (e.getKeyCode() == KeyEvent.VK_LEFT) x -= deltaX;
								else if (e.getKeyCode() == KeyEvent.VK_RIGHT) x += deltaX;
								else if (e.getKeyCode() == KeyEvent.VK_DOWN) y -= deltaY;
								else if (e.getKeyCode() == KeyEvent.VK_UP) y += deltaY;
								else if (e.getKeyCode() == KeyEvent.VK_PAGE_DOWN) {
									th -= deltaT;
									th = wrapAngle180(th);
								}
								else if (e.getKeyCode() == KeyEvent.VK_PAGE_UP) {
									th += deltaT;
									th = wrapAngle180(th);
								}
								PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
								path.set(selectedPathPointOneInt, newPoseSteering);
							}
							for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
							for (int selectedPathPointOneInt : selectedPathPointInt) {
								panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
							}
							panel.updatePanel();
						}
					}
					
					if (e.getKeyCode() == KeyEvent.VK_LESS && e.isShiftDown()) {
						maxTurningRadius += deltaTR;
						System.out.println("Turning radius (>): "+  maxTurningRadius);
					}
					else if (e.getKeyCode() == KeyEvent.VK_LESS) {
						maxTurningRadius -= deltaTR;
						System.out.println("Turning radius (<): "+  maxTurningRadius);
					}
				}
			}
		});
		
		panel.setFocusable(true);
		panel.setArrowHeadSizeInMeters(9.0);
		panel.setTextSizeInMeters(0.3);
	}
	
	private double wrapAngle360(double th) {
		return th-Math.PI*2.0*Math.floor(th/(Math.PI*2.0));
	}
	
	public static double wrapAngle180(double th) {
	    return Math.atan2(Math.sin(th), Math.cos(th));
	}
	
	private void writePath(String fileName) {
        try {
            File file = new File(fileName);
            PrintWriter writer = new PrintWriter(file);
            for (PoseSteering ps : path) {
            	writer.println(ps.getPose().getX() + " " + ps.getPose().getY() + " " + ps.getPose().getTheta() + " " + ps.getSteering());
            }
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	private void readPath(String fileName) {
		ArrayList<PoseSteering> ret = new ArrayList<PoseSteering>();
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0) {
					String[] oneline = line.split(" ");
					PoseSteering ps = null;
					if (oneline.length == 4) {
						ps = new PoseSteering(
								new Double(oneline[0]).doubleValue(),
								new Double(oneline[1]).doubleValue(),
								wrapAngle180(new Double(oneline[2]).doubleValue()),
								wrapAngle180(new Double(oneline[3]).doubleValue()));
					}
					else {
						ps = new PoseSteering(
								new Double(oneline[0]).doubleValue(),
								new Double(oneline[1]).doubleValue(),
								wrapAngle180(new Double(oneline[2]).doubleValue()),
								0.0);					
					}
					ret.add(ps);
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
		this.path = ret;
		for (int i = 0; i < path.size(); i++) {
			Pose pose = path.get(i).getPose();
			panel.addArrow(""+i, pose, Color.gray);
		}
		panel.updatePanel();
	}
	
	private PoseSteering[] computePath(PoseSteering from, PoseSteering to) {
		double middle = 20.0;
		double minX = Math.min(from.getPose().getX(), to.getPose().getX());
		double minY = Math.min(from.getPose().getY(), to.getPose().getY());
		PoseSteering newFrom = new PoseSteering(from.getPose().getX()-minX+middle,from.getPose().getY()-minY+middle,from.getPose().getTheta(), from.getSteering());
		PoseSteering newTo = new PoseSteering(to.getPose().getX()-minX+middle,to.getPose().getY()-minY+middle,to.getPose().getTheta(), to.getSteering());
		System.out.println("Coords are now " + newFrom.getPose() + " and " + newTo.getPose());
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		String yamlFile = "maps/map-empty.yaml";
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		rsp.setMapResolution(res*10);
		rsp.setRobotRadius(0.1);
		rsp.setTurningRadius(maxTurningRadius);
		rsp.setDistanceBetweenPathPoints(minDistance);
		rsp.setStart(newFrom.getPose());
		rsp.setGoals(newTo.getPose());
		if (!rsp.plan()) return null;
		PoseSteering[] ret = rsp.getPath();
		for (int i = 0; i < ret.length; i++) {
			PoseSteering newPose = new PoseSteering(ret[i].getPose().getX()+minX-middle, ret[i].getPose().getY()+minY-middle, ret[i].getPose().getTheta(), ret[i].getSteering());
			ret[i] = newPose;
		}
		return ret;
	}

	public static void main(String[] args) {
		String fileName = "paths/path2.path";
		//String fileName = "/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/paths/elsite_smooth_paths/elsite_paths_left.path0.path.new.new";
		new PathEditor(fileName);
	}

}
