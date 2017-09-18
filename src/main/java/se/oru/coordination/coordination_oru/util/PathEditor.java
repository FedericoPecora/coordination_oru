package se.oru.coordination.coordination_oru.util;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Scanner;
import java.util.TreeMap;

import javax.imageio.ImageIO;
import javax.swing.AbstractAction;
import javax.swing.JOptionPane;
import javax.swing.KeyStroke;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.UI.JTSDrawingPanel;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;

public class PathEditor {

	private static int EMPTY_MAP_DIM = 10000;
	private static double OBSTACLE_SIZE = 2.0;
	private static double MAX_TURNING_RADIUS = 4.0;
	private static double MIN_DISTANCE_BETWEEN_PATH_POINTS = 0.4;
	private String fileName = null;
	private boolean selectionPathPointInputListen = false;
	private String selectionString = "";
	private ArrayList<Integer> selectedPathPointsInt = new ArrayList<Integer>();
	private boolean selectionObsInputListen = false;
	private ArrayList<Integer> selectedObsInt = new ArrayList<Integer>();
	private ArrayList<Geometry> obstacles = new ArrayList<Geometry>();
	private ArrayList<PoseSteering> path = null;
	private ArrayList<ArrayList<PoseSteering>> oldPaths = new ArrayList<ArrayList<PoseSteering>>(); 
	private JTSDrawingPanel panel = null;
	private double deltaX = 0.1;
	private double deltaY = 0.1;
	private double deltaT = 0.1;
	private double deltaTR = 0.1;
	private String newFileSuffix = ".new";
	
	private static String TEMP_MAP_DIR = ".tempMapsPathEditor";
	
	public PathEditor(String fileName, double deltaX, double deltaY, double deltaTheta, String newFileSuffix) {
		this.fileName = fileName;
		this.deltaX = deltaX;
		this.deltaY = deltaY;
		this.deltaT = deltaTheta;
		this.newFileSuffix = newFileSuffix;
		this.setupGUI();
		this.readPath(fileName);
		this.deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();
	}
	
	public boolean deleteDir(File dir) {
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
	
	public PathEditor(String fileName) {
		this(fileName,0.1,0.1,0.1,".new");
	}
	
	private double[] getMinXYMaxXY() {
		double maxX = -Double.MAX_VALUE;
		double maxY = -Double.MAX_VALUE;
		double minX = Double.MAX_VALUE;
		double minY = Double.MAX_VALUE;
		for (Geometry obs : this.obstacles) {
			// (minx miny, maxx miny, maxx maxy, minx maxy, minx miny)
			double oneMinX = obs.getEnvelope().getCoordinates()[0].x;
			double oneMinY = obs.getEnvelope().getCoordinates()[0].y;
			double oneMaxX = obs.getEnvelope().getCoordinates()[1].x;
			double oneMaxY = obs.getEnvelope().getCoordinates()[1].y;
			if (oneMinX < minX) minX = oneMinX;
			if (oneMinY < minY) minY = oneMinY;
			if (oneMaxX > maxX) maxX = oneMaxX;
			if (oneMaxY > maxY) maxY = oneMaxY;
		}
		for (PoseSteering ps : this.path) {
			if (ps.getX() < minX) minX = ps.getX();
			if (ps.getY() < minY) minY = ps.getY();
			if (ps.getX() > maxX) maxX = ps.getX();
			if (ps.getY() > maxY) maxY = ps.getY();
		}
		return new double[] {minX,minY,maxX,maxY};
	}
	
	private String getHelp() {
		String ret = "";
		TreeMap<String,String> helpText = new TreeMap<String, String>();
		for (KeyStroke key : panel.getInputMap().allKeys()) {
			if (key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_0,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_1,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_2,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_3,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_4,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_5,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_6,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_7,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_8,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_9,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_MINUS,0)) ||
					key.equals(KeyStroke.getKeyStroke(KeyEvent.VK_COMMA,0)))
				continue;
			if (key.getModifiers() != 0) helpText.put(key.toString().replaceAll("pressed ", "").replaceAll(" ", "-"), panel.getInputMap().get(key).toString());
			else helpText.put(key.toString().replaceAll("pressed ", ""), panel.getInputMap().get(key).toString());
		}
		for (Entry<String,String> en : helpText.entrySet()) {
			ret += en.getKey() + ": " + en.getValue() + "\n";
		}
		return ret;
	}
	
	private Geometry makeObstacle(Pose p) {
		GeometryFactory gf = new GeometryFactory();
		Geometry geom = gf.createPolygon(new Coordinate[] { new Coordinate(0.0,0.0), new Coordinate(0.0,OBSTACLE_SIZE), new Coordinate(OBSTACLE_SIZE,OBSTACLE_SIZE), new Coordinate(OBSTACLE_SIZE,0.0), new Coordinate(0.0,0.0) });
		AffineTransformation at = new AffineTransformation();
		at.rotate(p.getTheta());
		at.translate(p.getX(), p.getY());
		Geometry transGeom = at.transform(geom);
		return transGeom;
	}
	
	private void highlightPathPoints() {
		for (int selectedPathPointOneInt : selectedPathPointsInt) {
			if (selectedPathPointOneInt >= 0 && selectedPathPointOneInt < path.size()) {
				panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
			}
		}
		panel.updatePanel();
	}
	
	private void highlightObstacles() {
		for (int selectedObsOneInt : selectedObsInt) {
			if (selectedObsOneInt >= 0 && selectedObsOneInt < obstacles.size()) {
				panel.addGeometry("obs_"+selectedObsOneInt, obstacles.get(selectedObsOneInt), false, false, true, "#cc3300");
			}
		}
		panel.updatePanel();
	}
	
	private void clearObstacleSelection() {
		selectionString = "";
		selectedObsInt.clear();
		for (int i = 0; i < obstacles.size(); i++) panel.addGeometry("obs_"+i, obstacles.get(i), false, false, true, "#666699");
		panel.updatePanel();
	}
	
	private void clearPathPointSelection() {
		selectionString = "";
		selectedPathPointsInt.clear();
		for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
		panel.updatePanel();
	}
	
	private void setupGUI() {
		panel = JTSDrawingPanel.makeEmpty("Path Editor");

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_0,0),"Digit0");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_1,0),"Digit1");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_2,0),"Digit2");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_3,0),"Digit3");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_4,0),"Digit4");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_5,0),"Digit5");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_6,0),"Digit6");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_7,0),"Digit7");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_8,0),"Digit8");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_9,0),"Digit9");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_MINUS,0),"Digit-");
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_COMMA,0),"Digit,");
		AbstractAction actInputSelection = new AbstractAction() {
			private static final long serialVersionUID = -1398168416006978350L;
			@Override
			public void actionPerformed(ActionEvent e) {
				selectionString += e.getActionCommand();
				System.out.println("Selection: " + selectionString);
			}
		};
		panel.getActionMap().put("Digit0",actInputSelection);
		panel.getActionMap().put("Digit1",actInputSelection);
		panel.getActionMap().put("Digit2",actInputSelection);
		panel.getActionMap().put("Digit3",actInputSelection);
		panel.getActionMap().put("Digit4",actInputSelection);
		panel.getActionMap().put("Digit5",actInputSelection);
		panel.getActionMap().put("Digit6",actInputSelection);
		panel.getActionMap().put("Digit7",actInputSelection);
		panel.getActionMap().put("Digit8",actInputSelection);
		panel.getActionMap().put("Digit9",actInputSelection);
		panel.getActionMap().put("Digit-",actInputSelection);
		panel.getActionMap().put("Digit,",actInputSelection);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_O,0),"Add obstacle(s) near selected pose(s)");
		AbstractAction actObs = new AbstractAction() {
			private static final long serialVersionUID = -1398168416006978750L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					for (int i = 0; i < obstacles.size(); i++) panel.addGeometry("obs_"+i, obstacles.get(i), false, false, true, "#666699");
					selectedObsInt.clear();
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						Geometry obs = makeObstacle(path.get(selectedPathPointOneInt).getPose());
						obstacles.add(obs);
						int id = obstacles.size()-1;
						panel.addGeometry("obs_"+id, obs, false, false, true, "#cc3300");
						selectedObsInt.add(id);
					}
					clearPathPointSelection();
					highlightObstacles();
				}
			}
		};
		panel.getActionMap().put("Add obstacle(s) near selected pose(s)",actObs);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE,0),"Cancel selection");
		AbstractAction actCancel = new AbstractAction() {
			private static final long serialVersionUID = -1398168416006978350L;
			@Override
			public void actionPerformed(ActionEvent e) {
				selectionPathPointInputListen = false;
				clearObstacleSelection();
				clearPathPointSelection();
			}
		};
		panel.getActionMap().put("Cancel selection",actCancel);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_P,0),"Add pose(s)");
		AbstractAction actInsert = new AbstractAction() {
			private static final long serialVersionUID = -8804517791543118334L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					HashMap<Integer,PoseSteering> toAdd = new HashMap<Integer, PoseSteering>();
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						PoseSteering newPoseSteering = new PoseSteering(path.get(selectedPathPointOneInt).getPose().getX()+deltaX, path.get(selectedPathPointOneInt).getPose().getY()+deltaY, path.get(selectedPathPointOneInt).getPose().getTheta(), path.get(selectedPathPointOneInt).getSteering());
						toAdd.put(selectedPathPointOneInt, newPoseSteering);
					}
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						path.add(selectedPathPointsInt.get(0), toAdd.get(selectedPathPointOneInt));
					}
					clearPathPointSelection();
					highlightPathPoints();
				}
			}
		};
		panel.getActionMap().put("Add pose(s)",actInsert);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_Z,KeyEvent.CTRL_DOWN_MASK),"Undo");
		AbstractAction actUndo = new AbstractAction() {
			private static final long serialVersionUID = 5597593272769688561L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!oldPaths.isEmpty()) {
					for (int i = 0; i < path.size(); i++) panel.removeGeometry(""+i);
					path = oldPaths.get(oldPaths.size()-1);
					oldPaths.remove(oldPaths.size()-1);
					clearPathPointSelection();
				}
			}
		};
		panel.getActionMap().put("Undo",actUndo);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,KeyEvent.CTRL_DOWN_MASK),"Save");
		AbstractAction actSave = new AbstractAction() {
			private static final long serialVersionUID = 8788274388808789051L;
			@Override
			public void actionPerformed(ActionEvent e) {
				String newFileName = fileName+newFileSuffix;
				writePath(newFileName);
				System.out.println("Saved " + newFileName);
			}
		};
		panel.getActionMap().put("Save",actSave);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_H,0),"Help");
		AbstractAction actHelp = new AbstractAction() {
			private static final long serialVersionUID = 8788274388808789053L;
			@Override
			public void actionPerformed(ActionEvent e) {
				JOptionPane.showMessageDialog(panel,getHelp());
//				System.out.println(getHelp());
			}
		};
		panel.getActionMap().put("Help",actHelp);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,0),"Select pose(s)");
		AbstractAction actSelect = new AbstractAction() {
			private static final long serialVersionUID = -4218195170958172222L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionPathPointInputListen) {
					clearPathPointSelection();
					System.out.println("Input selection (poses): " + selectionString);
				}
				else if (selectionPathPointInputListen) {
					for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
					try {
						if (selectionString.contains("-")) {
							int first = Integer.parseInt(selectionString.substring(0,selectionString.indexOf("-")));
							int last = Integer.parseInt(selectionString.substring(selectionString.indexOf("-")+1));
							for (int i = first; i <= last; i++) selectedPathPointsInt.add(i);
						}
						else if (selectionString.contains(",")) {
							String[] points = selectionString.split(",");
							for (String point : points) selectedPathPointsInt.add(Integer.parseInt(point));
						}
						else selectedPathPointsInt.add(Integer.parseInt(selectionString));
						clearObstacleSelection();
						highlightPathPoints();
						System.out.println("Current selection (poses): " + selectedPathPointsInt);
					}
					catch(NumberFormatException ex) { }
				}
				selectionPathPointInputListen = !selectionPathPointInputListen;
			}
		};
		panel.getActionMap().put("Select pose(s)",actSelect);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_G,0),"Select obstacle(s)");
		AbstractAction actObsSelect = new AbstractAction() {
			private static final long serialVersionUID = -4218195170858172222L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionObsInputListen) {
					clearObstacleSelection();
					System.out.println("Input selection (obstacles): " + selectionString);
				}
				else if (selectionObsInputListen) {
					for (int i = 0; i < obstacles.size(); i++) panel.addGeometry("obs_"+i, obstacles.get(i), false, false, true, "#666699");
					try {
						if (selectionString.contains("-")) {
							int first = Integer.parseInt(selectionString.substring(0,selectionString.indexOf("-")));
							int last = Integer.parseInt(selectionString.substring(selectionString.indexOf("-")+1));
							for (int i = first; i <= last; i++) selectedObsInt.add(i);
						}
						else if (selectionString.contains(",")) {
							String[] obss = selectionString.split(",");
							for (String obs : obss) selectedObsInt.add(Integer.parseInt(obs));
						}
						else selectedObsInt.add(Integer.parseInt(selectionString));
						highlightObstacles();
						System.out.println("Current selection (obstacles): " + selectedObsInt);
						clearPathPointSelection();
					}
					catch(NumberFormatException ex) { }
				}
				selectionObsInputListen = !selectionObsInputListen;
			}
		};
		panel.getActionMap().put("Select obstacle(s)",actObsSelect);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE,0),"Delete selected pose(s) and obstacle(s)");
		AbstractAction actDelete = new AbstractAction() {
			private static final long serialVersionUID = 4455373738365388356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					for (int i = 0; i < path.size(); i++) panel.removeGeometry(""+i);
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					ArrayList<PoseSteering> toRemove = new ArrayList<PoseSteering>();
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						toRemove.add(path.get(selectedPathPointOneInt));
					}
					path.removeAll(toRemove);
					clearPathPointSelection();
				}
				if (!selectedObsInt.isEmpty()) {
					ArrayList<Geometry> toRemove = new ArrayList<Geometry>();
					for (int i = 0; i < obstacles.size(); i++) panel.removeGeometry("obs_"+i);
					for (int selectedObsOneInt : selectedObsInt) {
						toRemove.add(obstacles.get(selectedObsOneInt));
					}
					obstacles.removeAll(toRemove);
					clearObstacleSelection();
				}
				panel.updatePanel();
			}
		};
		panel.getActionMap().put("Delete selected pose(s) and obstacle(s)",actDelete);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_P,KeyEvent.CTRL_DOWN_MASK),"Plan path between selected pair");
		AbstractAction actPlan = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762752293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (selectedPathPointsInt.size() >= 2) {
					PoseSteering startPose = path.get(selectedPathPointsInt.get(0));
					PoseSteering[] goalPoses = new PoseSteering[selectedPathPointsInt.size()-1];
					for (int i = 0; i < goalPoses.length; i++) {
						goalPoses[i] = path.get(selectedPathPointsInt.get(i));
					}
					PoseSteering[] newSubPath = computePath(startPose,goalPoses);
					if (newSubPath != null) {
						ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
						oldPaths.add(oldPath);
						TreeMap<Integer,PoseSteering> toAdd = new TreeMap<Integer, PoseSteering>();
						ArrayList<PoseSteering> toRemove = new ArrayList<PoseSteering>();
						for (int i = 1; i < selectedPathPointsInt.size()-1; i++) toRemove.add(path.get(selectedPathPointsInt.get(i)));
						if (!toRemove.isEmpty()) path.removeAll(toRemove);
						int selectedStart = selectedPathPointsInt.get(0)+1;
						for (int i = 1; i < newSubPath.length-1; i++) {
							PoseSteering newPoseSteering = new PoseSteering(newSubPath[i].getPose().getX(), newSubPath[i].getPose().getY(), newSubPath[i].getPose().getTheta(), newSubPath[i].getSteering());
							toAdd.put(i+selectedStart-1, newPoseSteering);
						}
						for (Entry<Integer,PoseSteering> en : toAdd.entrySet()) {
							path.add(en.getKey(), en.getValue());
						}
						clearPathPointSelection();
						for (Entry<Integer,PoseSteering> en : toAdd.entrySet()) {
							selectedPathPointsInt.add(en.getKey());
						}
						highlightPathPoints();
					}
				}
			}
		};
		panel.getActionMap().put("Plan path between selected pair",actPlan);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT,0),"Decrease X of selected pose(s)");
		AbstractAction actXMinus = new AbstractAction() {
			private static final long serialVersionUID = 1767256680398690970L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						double x = path.get(selectedPathPointOneInt).getPose().getX();
						double y = path.get(selectedPathPointOneInt).getPose().getY();
						double th = path.get(selectedPathPointOneInt).getPose().getTheta();
						x -= deltaX;
						PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
						path.set(selectedPathPointOneInt, newPoseSteering);
					}
				}
				if (!selectedObsInt.isEmpty()) {
					for (int selectedObsOneInt : selectedObsInt) {
						Geometry obs = obstacles.get(selectedObsOneInt);
						AffineTransformation at = new AffineTransformation();
						at.translate(-deltaX, 0);
						obstacles.set(selectedObsOneInt,at.transform(obs));
					}
				}
				highlightPathPoints();
				highlightObstacles();
			}
		};
		panel.getActionMap().put("Decrease X of selected pose(s)",actXMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT,0),"Increase X of selected pose(s)");
		AbstractAction actXPlus = new AbstractAction() {
			private static final long serialVersionUID = 6900418766755234220L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						double x = path.get(selectedPathPointOneInt).getPose().getX();
						double y = path.get(selectedPathPointOneInt).getPose().getY();
						double th = path.get(selectedPathPointOneInt).getPose().getTheta();
						x += deltaX;
						PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
						path.set(selectedPathPointOneInt, newPoseSteering);
					}
				}
				if (!selectedObsInt.isEmpty()) {
					for (int selectedObsOneInt : selectedObsInt) {
						Geometry obs = obstacles.get(selectedObsOneInt);
						AffineTransformation at = new AffineTransformation();
						at.translate(deltaX, 0);
						obstacles.set(selectedObsOneInt,at.transform(obs));
					}
				}
				highlightPathPoints();
				highlightObstacles();
			}
		};
		panel.getActionMap().put("Increase X of selected pose(s)",actXPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_UP,0),"Increase Y of selected pose(s)");
		AbstractAction actYPlus = new AbstractAction() {
			private static final long serialVersionUID = 2627197997139919535L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						double x = path.get(selectedPathPointOneInt).getPose().getX();
						double y = path.get(selectedPathPointOneInt).getPose().getY();
						double th = path.get(selectedPathPointOneInt).getPose().getTheta();
						y += deltaY;
						PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
						path.set(selectedPathPointOneInt, newPoseSteering);
					}
				}
				if (!selectedObsInt.isEmpty()) {
					for (int selectedObsOneInt : selectedObsInt) {
						Geometry obs = obstacles.get(selectedObsOneInt);
						AffineTransformation at = new AffineTransformation();
						at.translate(0, deltaY);
						obstacles.set(selectedObsOneInt,at.transform(obs));
					}
				}
				highlightPathPoints();
				highlightObstacles();			
			}
		};
		panel.getActionMap().put("Increase Y of selected pose(s)",actYPlus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DOWN,0),"Decrease Y of selected pose(s)");
		AbstractAction actYMinus = new AbstractAction() {
			private static final long serialVersionUID = 6487878455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						double x = path.get(selectedPathPointOneInt).getPose().getX();
						double y = path.get(selectedPathPointOneInt).getPose().getY();
						double th = path.get(selectedPathPointOneInt).getPose().getTheta();
						y -= deltaY;
						PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
						path.set(selectedPathPointOneInt, newPoseSteering);
					}
				}
				if (!selectedObsInt.isEmpty()) {
					for (int selectedObsOneInt : selectedObsInt) {
						Geometry obs = obstacles.get(selectedObsOneInt);
						AffineTransformation at = new AffineTransformation();
						at.translate(0, -deltaY);
						obstacles.set(selectedObsOneInt,at.transform(obs));
					}
				}
				highlightPathPoints();
				highlightObstacles();			
			}
		};
		panel.getActionMap().put("Decrease Y of selected pose(s)",actYMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_DOWN,0),"Decrease theta of selected pose(s)");
		AbstractAction actTMinus = new AbstractAction() {
			private static final long serialVersionUID = -7391970411254721019L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						double x = path.get(selectedPathPointOneInt).getPose().getX();
						double y = path.get(selectedPathPointOneInt).getPose().getY();
						double th = path.get(selectedPathPointOneInt).getPose().getTheta();
						th -= deltaT;
						th = Missions.wrapAngle360(th);
						PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
						path.set(selectedPathPointOneInt, newPoseSteering);
					}
				}
				if (!selectedObsInt.isEmpty()) {
					for (int selectedObsOneInt : selectedObsInt) {
						Geometry obs = obstacles.get(selectedObsOneInt);
						AffineTransformation at = new AffineTransformation();
						double toOriginX = obs.getCentroid().getX();
						double toOriginY = obs.getCentroid().getY();
						at.translate(-toOriginX,-toOriginY);
						at.rotate(-deltaT);
						at.translate(toOriginX,toOriginY);
						obstacles.set(selectedObsOneInt,at.transform(obs));
					}
				}
				highlightPathPoints();
				highlightObstacles();			
			}
		};
		panel.getActionMap().put("Decrease theta of selected pose(s)",actTMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_UP,0),"Increase theta of selected pose(s)");
		AbstractAction actTPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedPathPointsInt.isEmpty()) {
					ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
					oldPaths.add(oldPath);
					for (int selectedPathPointOneInt : selectedPathPointsInt) {
						double x = path.get(selectedPathPointOneInt).getPose().getX();
						double y = path.get(selectedPathPointOneInt).getPose().getY();
						double th = path.get(selectedPathPointOneInt).getPose().getTheta();
						th += deltaT;
						th = Missions.wrapAngle360(th);
						PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
						path.set(selectedPathPointOneInt, newPoseSteering);
					}
				}
				if (!selectedObsInt.isEmpty()) {
					for (int selectedObsOneInt : selectedObsInt) {
						Geometry obs = obstacles.get(selectedObsOneInt);
						AffineTransformation at = new AffineTransformation();
						double toOriginX = obs.getCentroid().getX();
						double toOriginY = obs.getCentroid().getY();
						at.translate(-toOriginX,-toOriginY);
						at.rotate(deltaT);
						at.translate(toOriginX,toOriginY);
						obstacles.set(selectedObsOneInt,at.transform(obs));
					}
				}
				highlightPathPoints();
				highlightObstacles();			
			}
		};
		panel.getActionMap().put("Increase theta of selected pose(s)",actTPlus);


		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_I,0),"Info");
		AbstractAction actInfo = new AbstractAction() {
			private static final long serialVersionUID = 8424380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				String obs = "Obstacles:";
				for (int i = 0; i < obstacles.size(); i++) obs += ("\n   obs_" + i + ": " + obstacles.get(i).getCentroid().getCoordinate());
				System.out.println(obs);
			}
		};
		panel.getActionMap().put("Info",actInfo);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_M,KeyEvent.SHIFT_DOWN_MASK),"Increase minimum distance between path points for path planning");
		AbstractAction actMDPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				MIN_DISTANCE_BETWEEN_PATH_POINTS += deltaX;
				System.out.println("Minimum distance between path points (>): " + MIN_DISTANCE_BETWEEN_PATH_POINTS);
			}
		};
		panel.getActionMap().put("Increase minimum distance between path points for path planning",actMDPlus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_M,0),"Decrease minimum distance between path points for path planning");
		AbstractAction actMDMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (MIN_DISTANCE_BETWEEN_PATH_POINTS-deltaX >= 0) MIN_DISTANCE_BETWEEN_PATH_POINTS -= deltaX;
				System.out.println("Minimum distance between path points (<): " + MIN_DISTANCE_BETWEEN_PATH_POINTS);
			}
		};
		panel.getActionMap().put("Decrease minimum distance between path points for path planning",actMDMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,KeyEvent.SHIFT_DOWN_MASK),"Increase maximum turning radius");
		AbstractAction actTRPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				MAX_TURNING_RADIUS += deltaTR;
				System.out.println("Turning radius (>): " + MAX_TURNING_RADIUS);
			}
		};
		panel.getActionMap().put("Increase maximum turning radius",actTRPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,0),"Decrease maximum turning radius");
		AbstractAction actTRMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (MAX_TURNING_RADIUS-deltaTR >= 0) MAX_TURNING_RADIUS -= deltaTR;
				System.out.println("Turning radius (<): " + MAX_TURNING_RADIUS);
			}
		};
		panel.getActionMap().put("Decrease maximum turning radius",actTRMinus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,KeyEvent.SHIFT_DOWN_MASK | KeyEvent.CTRL_DOWN_MASK),"Increase obstacle size");
		AbstractAction actOSPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				OBSTACLE_SIZE += deltaX;
				System.out.println("Obstacle size (>): " + OBSTACLE_SIZE);
			}
		};
		panel.getActionMap().put("Increase obstacle size",actOSPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,KeyEvent.CTRL_DOWN_MASK),"Decrease obstacle size");
		AbstractAction actOSMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (OBSTACLE_SIZE-deltaX >= 0) OBSTACLE_SIZE -= deltaX;
				System.out.println("Osbtacle size (<): " + OBSTACLE_SIZE);
			}
		};
		panel.getActionMap().put("Decrease obstacle size",actOSMinus);

		panel.setFocusable(true);
		panel.setArrowHeadSizeInMeters(9.0);
		panel.setTextSizeInMeters(0.3);
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
		this.path = ret;
		for (int i = 0; i < path.size(); i++) {
			Pose pose = path.get(i).getPose();
			panel.addArrow(""+i, pose, Color.gray);
		}
		panel.updatePanel();
	}
	
	private String makeEmptyMapMap() {
		BufferedImage img = new BufferedImage(EMPTY_MAP_DIM, EMPTY_MAP_DIM, BufferedImage.TYPE_INT_RGB);
		Graphics2D g2 = img.createGraphics();
		g2.setColor(Color.white);
		g2.fillRect(0, 0, EMPTY_MAP_DIM, EMPTY_MAP_DIM);
		File outputfile = new File(TEMP_MAP_DIR+File.separator+"tempMapEmpty.png");
		try { ImageIO.write(img, "png", outputfile); }
		catch (IOException e) { e.printStackTrace(); }
		return outputfile.getAbsolutePath();
	}
	
	private PoseSteering[] computePath(PoseSteering from, PoseSteering ... to) {
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();		
		rsp.setMapFilename(makeEmptyMapMap());
		rsp.setMapResolution(1.0);
		rsp.setRobotRadius(1.0);
		rsp.setTurningRadius(MAX_TURNING_RADIUS);
		rsp.setDistanceBetweenPathPoints(MIN_DISTANCE_BETWEEN_PATH_POINTS);
		rsp.setStart(from.getPose());
		Pose[] goalPoses = new Pose[to.length];
		for (int i = 0; i < goalPoses.length; i++) goalPoses[i] = to[i].getPose();
		rsp.setGoals(goalPoses);
		rsp.addObstacles(obstacles.toArray(new Geometry[obstacles.size()]));
		if (!rsp.plan()) return null;
		PoseSteering[] ret = rsp.getPath();
		return ret;
	}

	public static void main(String[] args) {
		String fileName = "paths/path2.path";
		//String fileName = "/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/paths/elsite_smooth_paths/elsite_paths_left.path0.path.new.new";
		new PathEditor(fileName);
	}

}
