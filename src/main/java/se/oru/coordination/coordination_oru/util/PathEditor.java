package se.oru.coordination.coordination_oru.util;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.beans.PropertyChangeListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Scanner;
import java.util.TreeMap;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.KeyStroke;

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
				selectedPathPoint += e.getActionCommand();
				System.out.println("Selection: " + selectedPathPoint);
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

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE,0),"Cancel selection");
		AbstractAction actCancel = new AbstractAction() {
			private static final long serialVersionUID = -1398168416006978350L;
			@Override
			public void actionPerformed(ActionEvent e) {
				selectedPathPoint = "";
				selectedPathPointInt.clear();
				selectionInputListen = false;
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				panel.updatePanel();
			}
		};
		panel.getActionMap().put("Cancel selection",actCancel);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_INSERT,0),"Insert pose(s)");
		AbstractAction actInsert = new AbstractAction() {
			private static final long serialVersionUID = -8804517791543118334L;
			@Override
			public void actionPerformed(ActionEvent e) {
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
		};
		panel.getActionMap().put("Insert pose(s)",actInsert);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_Z,KeyEvent.CTRL_DOWN_MASK),"Undo");
		AbstractAction actUndo = new AbstractAction() {
			private static final long serialVersionUID = 5597593272769688561L;
			@Override
			public void actionPerformed(ActionEvent e) {
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
				System.out.println(getHelp());
			}
		};
		panel.getActionMap().put("Help",actHelp);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,0),"Select pose(s)");
		AbstractAction actSelect = new AbstractAction() {
			private static final long serialVersionUID = -4218195170958172222L;
			@Override
			public void actionPerformed(ActionEvent e) {
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
		};
		panel.getActionMap().put("Select pose(s)",actSelect);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE,0),"Delete selected pose(s)");
		AbstractAction actDelete = new AbstractAction() {
			private static final long serialVersionUID = 4455373738365388356L;
			@Override
			public void actionPerformed(ActionEvent e) {
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
		};
		panel.getActionMap().put("Delete selected pose(s)",actDelete);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_P,KeyEvent.CTRL_DOWN_MASK),"Plan path between selected pair");
		AbstractAction actPlan = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762752293L;
			@Override
			public void actionPerformed(ActionEvent e) {
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
		};
		panel.getActionMap().put("Plan path between selected pair",actPlan);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT,0),"Decrease X of selected pose(s)");
		AbstractAction actXMinus = new AbstractAction() {
			private static final long serialVersionUID = 1767256680398690970L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
				oldPaths.add(oldPath);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					double x = path.get(selectedPathPointOneInt).getPose().getX();
					double y = path.get(selectedPathPointOneInt).getPose().getY();
					double th = path.get(selectedPathPointOneInt).getPose().getTheta();
					x -= deltaX;
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
					path.set(selectedPathPointOneInt, newPoseSteering);
				}
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
				}
				panel.updatePanel();
			}
		};
		panel.getActionMap().put("Decrease X of selected pose(s)",actXMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT,0),"Increase X of selected pose(s)");
		AbstractAction actXPlus = new AbstractAction() {
			private static final long serialVersionUID = 6900418766755234220L;
			@Override
			public void actionPerformed(ActionEvent e) {
				System.out.println(e.getActionCommand());
				ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
				oldPaths.add(oldPath);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					double x = path.get(selectedPathPointOneInt).getPose().getX();
					double y = path.get(selectedPathPointOneInt).getPose().getY();
					double th = path.get(selectedPathPointOneInt).getPose().getTheta();
					x += deltaX;
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
					path.set(selectedPathPointOneInt, newPoseSteering);
				}
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
				}
				panel.updatePanel();
			}
		};
		panel.getActionMap().put("Increase X of selected pose(s)",actXPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_UP,0),"Increase Y of selected pose(s)");
		AbstractAction actYPlus = new AbstractAction() {
			private static final long serialVersionUID = 2627197997139919535L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
				oldPaths.add(oldPath);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					double x = path.get(selectedPathPointOneInt).getPose().getX();
					double y = path.get(selectedPathPointOneInt).getPose().getY();
					double th = path.get(selectedPathPointOneInt).getPose().getTheta();
					y += deltaY;
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
					path.set(selectedPathPointOneInt, newPoseSteering);
				}
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
				}
				panel.updatePanel();
			
			}
		};
		panel.getActionMap().put("Increase Y of selected pose(s)",actYPlus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DOWN,0),"Decrease Y of selected pose(s)");
		AbstractAction actYMinus = new AbstractAction() {
			private static final long serialVersionUID = 6487878455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
				oldPaths.add(oldPath);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					double x = path.get(selectedPathPointOneInt).getPose().getX();
					double y = path.get(selectedPathPointOneInt).getPose().getY();
					double th = path.get(selectedPathPointOneInt).getPose().getTheta();
					y -= deltaY;
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
					path.set(selectedPathPointOneInt, newPoseSteering);
				}
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
				}
				panel.updatePanel();
			
			}
		};
		panel.getActionMap().put("Decrease Y of selected pose(s)",actYMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_DOWN,0),"Decrease theta of selected pose(s)");
		AbstractAction actTMinus = new AbstractAction() {
			private static final long serialVersionUID = -7391970411254721019L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
				oldPaths.add(oldPath);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					double x = path.get(selectedPathPointOneInt).getPose().getX();
					double y = path.get(selectedPathPointOneInt).getPose().getY();
					double th = path.get(selectedPathPointOneInt).getPose().getTheta();
					th -= deltaT;
					th = Missions.wrapAngle360(th);
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
					path.set(selectedPathPointOneInt, newPoseSteering);
				}
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
				}
				panel.updatePanel();
			
			}
		};
		panel.getActionMap().put("Decrease theta of selected pose(s)",actTMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_UP,0),"Increase theta of selected pose(s)");
		AbstractAction actTPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<PoseSteering> oldPath = new ArrayList<PoseSteering>(path);
				oldPaths.add(oldPath);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					double x = path.get(selectedPathPointOneInt).getPose().getX();
					double y = path.get(selectedPathPointOneInt).getPose().getY();
					double th = path.get(selectedPathPointOneInt).getPose().getTheta();
					th += deltaT;
					th = Missions.wrapAngle360(th);
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path.get(selectedPathPointOneInt).getSteering());
					path.set(selectedPathPointOneInt, newPoseSteering);
				}
				for (int i = 0; i < path.size(); i++) panel.addArrow(""+i, path.get(i).getPose(), Color.gray);
				for (int selectedPathPointOneInt : selectedPathPointInt) {
					panel.addArrow(""+selectedPathPointOneInt, path.get(selectedPathPointOneInt).getPose(), Color.red);
				}
				panel.updatePanel();
			
			}
		};
		panel.getActionMap().put("Increase theta of selected pose(s)",actTPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,KeyEvent.SHIFT_DOWN_MASK),"Increase maximum turning radius");
		AbstractAction actTRPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				maxTurningRadius += deltaTR;
				System.out.println("Turning radius (>): "+  maxTurningRadius);
			}
		};
		panel.getActionMap().put("Increase maximum turning radius",actTRPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,0),"Decrease maximum turning radius");
		AbstractAction actTRMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				maxTurningRadius -= deltaTR;
				System.out.println("Turning radius (<): "+  maxTurningRadius);
			}
		};
		panel.getActionMap().put("Decrease maximum turning radius",actTRMinus);
		
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
		//String fileName = "paths/path2.path";
		String fileName = "/home/fpa/gitroot.gitlab/volvo_ce/coordination_oru_vce/paths/elsite_smooth_paths/elsite_paths_left.path0.path.new.new";
		new PathEditor(fileName);
	}

}
