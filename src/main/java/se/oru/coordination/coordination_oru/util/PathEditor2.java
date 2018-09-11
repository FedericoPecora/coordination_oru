package se.oru.coordination.coordination_oru.util;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Scanner;
import java.util.Map.Entry;
import java.util.TreeMap;

import javax.imageio.ImageIO;
import javax.swing.AbstractAction;
import javax.swing.JOptionPane;
import javax.swing.KeyStroke;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.UI.JTSDrawingPanel;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.util.splines.BezierSplineFactory;

public class PathEditor2 implements MouseMotionListener {

	private static int newLocationCounter = 0;
	private String selectionsFile = null;
	private String outputDir = null;
	private static String NEW_SUFFIX = ".new";
	private static int EMPTY_MAP_DIM = 10000;
	private static double MAX_TURNING_RADIUS = 5.0;
	private static double SPLINE_DISTANCE = 20.0;
	private static double DISTANCE_BETWEEN_PATH_POINTS = 1.5;
	private String locationsAndPathsFilename = null;
	private String mapFilename = null;
	private String mapImgFilename = null;
	private double mapRes = 1.0;
	private boolean selectionInputListen = false;
	private ArrayList<String> selectionStringsNames = null;
	private ArrayList<String> selectionStrings = null;
	private ArrayList<ArrayList<Integer>> selectedLocationsInts = null;
	private HashMap<Integer,ArrayList<Integer>> selectionGroupsToSelections = null;
	private int selectedGroup = -1;
	
	private Point2D mousePosition = null;

	private String selectionString = "";
	private ArrayList<Integer> selectedLocationsInt = new ArrayList<Integer>();
	private ArrayList<String> locationIDs = new ArrayList<String>();
	private HashMap<String,ArrayList<PoseSteering>> allPaths = null;
	private HashMap<String,ArrayList<ArrayList<PoseSteering>>> oldPaths = new HashMap<String,ArrayList<ArrayList<PoseSteering>>>(); 
	private JTSDrawingPanel panel = null;
	private double deltaX = 0.1;
	private double deltaY = 0.1;
	private double deltaT = 0.01;
	private double deltaTR = 0.1;
	private double deltaSD = 0.5;
	private double deltaD = 0.1;
	
	private double radius = 3.0;
	private Coordinate[] footprint = null;
	
	private static String TEMP_MAP_DIR = ".tempMapsPathEditor";

	public PathEditor2() {
		this(null, null, null, 0.1, 0.1, 0.01);
	}
	
	public void setDeltaX(double dx) {
		this.deltaX = dx;
	}

	public void setDeltaY(double dy) {
		this.deltaY = dy;
	}

	public void setDeltaTheta(double dt) {
		this.deltaT = dt;
	}
	
	public void setSplineDistance(double val) {
		SPLINE_DISTANCE = val;
	}
	
	public void setDistanceBetweenPathPoints(double val) {
		DISTANCE_BETWEEN_PATH_POINTS = val;
	}
	
	public void setMaxTurningRadius(double val) {
		MAX_TURNING_RADIUS = val;
	}

	public PathEditor2(String posesAndPaths, String mapFilename) {
		this(posesAndPaths, mapFilename, null, 0.1, 0.1, 0.01);
	}

	public PathEditor2(String posesAndPaths, String mapFilename, String selectionsFile) {
		this(posesAndPaths, mapFilename, selectionsFile, 0.1, 0.1, 0.01);
	}
			
	public PathEditor2(String posesAndPaths, String mapFN, String selectionsF, double deltaX, double deltaY, double deltaTheta) {
		this.deltaX = deltaX;
		this.deltaY = deltaY;
		this.deltaT = deltaTheta;
		this.mapFilename = mapFN;
		if (this.mapFilename != null) {
			String path = this.mapFilename.substring(0, this.mapFilename.lastIndexOf(File.separator)+1);
			this.mapImgFilename = path+Missions.getProperty("image", this.mapFilename);
			this.mapRes = Double.parseDouble(Missions.getProperty("resolution", this.mapFilename));
			System.out.println("MAP YAML: " + this.mapFilename);
			System.out.println("MAP IMG : " + this.mapImgFilename);
			System.out.println("MAP RES : " + this.mapRes);
		}
		this.setupGUI();
		if (this.mapFilename != null) {
			panel.setMap(this.mapFilename);
		}
		this.deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();

		this.outputDir = "output";
		this.deleteDir(new File(this.outputDir));
		new File(outputDir).mkdir();

		this.allPaths = new HashMap<String, ArrayList<PoseSteering>>();

		if (posesAndPaths != null) {
			this.locationsAndPathsFilename = posesAndPaths;
			String pathURI = this.locationsAndPathsFilename.substring(0, this.locationsAndPathsFilename.lastIndexOf(File.separator)+1);
			Missions.loadLocationAndPathData(this.locationsAndPathsFilename);
			addAllKnownLocations();
			HashMap<String,Pose> locations = Missions.getLocations();
			for (Entry<String,Pose> entry : locations.entrySet()) {
				//System.out.println("Added location " + entry.getKey() + ": " + entry.getValue());
				for (Entry<String,Pose> entry1 : locations.entrySet()) {
					if (!entry1.equals(entry)) {
						try {
							String pathFile = Missions.getPathFile(entry.getKey(), entry1.getKey());
							PoseSteering[] path = Missions.loadPathFromFile(pathURI+pathFile);
							ArrayList<PoseSteering> pathAL = new ArrayList<PoseSteering>();
							for (PoseSteering ps : path) pathAL.add(ps);
							allPaths.put(entry.getKey()+"->"+entry1.getKey(), pathAL);							
						}
						catch(Error e) {
							//System.out.println("No path for " + entry.getKey()+"->"+entry1.getKey());
						}
						try {
							String pathFile = Missions.getPathFile(entry1.getKey(), entry.getKey());
							PoseSteering[] path = Missions.loadPathFromFile(pathURI+pathFile);
							ArrayList<PoseSteering> pathAL = new ArrayList<PoseSteering>();
							for (PoseSteering ps : path) pathAL.add(ps);
							allPaths.put(entry1.getKey()+"->"+entry.getKey(), pathAL);
						}
						catch(Error e) {
							//System.out.println("No path for " + entry1.getKey()+"->"+entry.getKey());
						}
					}
				}
			}
			for (Entry<String,ArrayList<PoseSteering>> onePath : allPaths.entrySet()) {
				String pathName = onePath.getKey();
				ArrayList<PoseSteering> path = onePath.getValue();
				for (int i = 0; i < path.size(); i++) {
					panel.addArrow(pathName+"."+i, path.get(i).getPose(), Color.blue);
				}
			}
			panel.updatePanel();
		}
		
		if (selectionsF != null) {
			this.selectionsFile = selectionsF;
			loadSelectionsFile();
		}
		
		panel.addMouseMotionListener(this);


	}
	
	public void setPathPlanningFootprint(Coordinate ... footprint) {
		this.footprint = footprint;
	}
	
	public void setPathPlanningRadius(double rad) {
		this.radius = rad;
	}
	
	public void loadSelectionsFile() {
		try {
			Scanner in = new Scanner(new FileReader(selectionsFile));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0 && !line.startsWith("#")) {
					if (selectionStringsNames == null) selectionStringsNames = new ArrayList<String>();
					selectionStringsNames.add(line);
					String[] oneline = line.split(" |\t");
					ArrayList<Integer> oneSelection = new ArrayList<Integer>();
					String str = "";
					int groupNumber = -1;
					for (int i = 0; i < oneline.length; i++) {
						try {
							groupNumber = Integer.parseInt(oneline[i]);
						}
						catch (NumberFormatException e) { }
						for (int j = 0; j < locationIDs.size(); j++) {
							if (locationIDs.get(j).equals(oneline[i])) {
								oneSelection.add(j);
								str+=(""+j);
								if (i != oneline.length-1) str+=",";
								break;
							}
						}
						
					}
					if (selectedLocationsInts == null) selectedLocationsInts = new ArrayList<ArrayList<Integer>>();
					if (selectionStrings == null) selectionStrings = new ArrayList<String>();
					selectedLocationsInts.add(oneSelection);
					selectionStrings.add(str);
					if (groupNumber != -1) {
						if (selectionGroupsToSelections == null) selectionGroupsToSelections = new HashMap<Integer, ArrayList<Integer>>();
						if (!selectionGroupsToSelections.containsKey(groupNumber)) selectionGroupsToSelections.put(groupNumber, new ArrayList<Integer>());
						selectionGroupsToSelections.get(groupNumber).add(selectedLocationsInts.size()-1);
					}
				}
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
	}
	
	public void updatePaths() {
		for (Entry<String,ArrayList<PoseSteering>> onePath : allPaths.entrySet()) {
			String pathName = onePath.getKey();
			ArrayList<PoseSteering> path = onePath.getValue();
			for (int i = 0; i < path.size(); i++) {
				panel.addArrow(pathName+"."+i, path.get(i).getPose(), Color.blue);
			}
			panel.addArrow(pathName+".all", path.get(0).getPose(), path.get(path.size()-1).getPose(), Color.lightGray);
		}
		panel.updatePanel();
	}

	public void removeKnownLocation(String locationName) {
		int toRemove = -1;
		for (int i = 0; i < locationIDs.size(); i++) {
			if (locationIDs.get(i).equals(locationName)) {
				panel.removeGeometry("LOC:"+locationIDs.get(i));
				toRemove = i;
				break;
			}
		}
		locationIDs.remove(toRemove);
	}
	
	public void removeAllKnownLocations() {
		for (int i = 0; i < locationIDs.size(); i++) {
			panel.removeGeometry("LOC:"+locationIDs.get(i));
		}
	}
	
	public void addAllKnownLocations() {
		locationIDs = new ArrayList<String>();
		for (Entry<String,Pose> entry : Missions.getLocations().entrySet()) {
			if (entry.getKey().startsWith("AUX_")) newLocationCounter++;
			panel.addArrow("LOC:"+entry.getKey(), entry.getValue(), Color.green);
			locationIDs.add(entry.getKey());
		}
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
	
	protected void dumpLocationAndPathData(HashMap<String,Pose> locs, HashMap<String,ArrayList<PoseSteering>> pths) {
		String st = "#Locations\n";
		for (Entry<String,Pose> entry : locs.entrySet()) {
			st += entry.getKey() + "\t" + entry.getValue().getX() + "\t" + entry.getValue().getY() + "\t" + entry.getValue().getTheta() + "\n";
		}
		st+="\n#Paths\n";
		for (Entry<String,ArrayList<PoseSteering>> entry : pths.entrySet()) {
			String pathFilename = entry.getKey().replaceAll("->", "-")+".path";
			st += entry.getKey().replaceAll("->", " -> ") + "\t" + pathFilename + "\n";
			Missions.writePath(outputDir+File.separator+pathFilename, entry.getValue());
		}
        try {
        	String newFilename = outputDir+File.separator+"locations_and_paths.txt";
            File file = new File(newFilename);
            PrintWriter writer = new PrintWriter(file);
            writer.write(st);
            writer.close();
            System.out.println("Saved locations and paths file: " + newFilename);
        }
        catch (Exception ex) { ex.printStackTrace(); }
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
	
	private void highlightSelectedLocations() {
		clearLocations();
		for (int selectedPathPointOneInt : selectedLocationsInt) {
			if (selectedPathPointOneInt >= 0 && selectedPathPointOneInt < locationIDs.size()) {
				panel.addArrow("LOC:"+locationIDs.get(selectedPathPointOneInt), Missions.getLocation(locationIDs.get(selectedPathPointOneInt)), Color.red);
			}
		}
		panel.updatePanel();
	}
		
	private void clearLocations() {
		for (int i = 0; i < locationIDs.size(); i++) {
			panel.addArrow("LOC:"+locationIDs.get(i), Missions.getLocation(locationIDs.get(i)), Color.green);
		}
		panel.updatePanel();
	}
	
//	private void backupPath() {
//		ArrayList<PoseSteering> backup = new ArrayList<PoseSteering>();
//		for (PoseSteering ps : this.currentPath) {
//			PoseSteering newPS = new PoseSteering(ps.getX(), ps.getY(), ps.getTheta(), ps.getSteering());
//			backup.add(newPS);
//		}
//		oldCurrentPaths.add(backup);
//	}
//	
//	private void restorePath() {
//		if (!oldCurrentPaths.isEmpty()) {
//			for (int i = 0; i < currentPath.size(); i++) panel.removeGeometry(""+i);
//			currentPath = oldCurrentPaths.get(oldCurrentPaths.size()-1);
//			oldCurrentPaths.remove(oldCurrentPaths.size()-1);
//			clearPathPointSelection();
//		}
//	}
	
	public void addAbstractAction(AbstractAction aa, int keyEvent, int modifiers, String description) {
		panel.getInputMap().put(KeyStroke.getKeyStroke(keyEvent,modifiers),description);
		panel.getActionMap().put(description,aa);
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

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE,0),"Cancel selection");
		AbstractAction actCancel = new AbstractAction() {
			private static final long serialVersionUID = -1398168416006978350L;
			@Override
			public void actionPerformed(ActionEvent e) {
				selectionInputListen = false;
				selectedLocationsInt = new ArrayList<Integer>();
				selectionString = "";
				clearLocations();
			}
		};
		panel.getActionMap().put("Cancel selection",actCancel);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,0),"Select a selection preset");
		AbstractAction actSelectSS = new AbstractAction() {
			private static final long serialVersionUID = -8804517791543118334L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionInputListen) {
					selectedLocationsInt = new ArrayList<Integer>();
					selectionString = "";
					selectedGroup = -1;
					clearLocations();
					System.out.println("Input one path index in [0.." + (selectionStrings.size()-1) + "]: " + selectionString);
				}
				else if (selectionInputListen) {
					clearLocations();
					try {
						int ssNumber = Integer.parseInt(selectionString);
						selectionString = selectionStrings.get(ssNumber);
						selectedLocationsInt = selectedLocationsInts.get(ssNumber);
						highlightSelectedLocations();
						System.out.println("Current selection (locations): " + selectedLocationsInt);
					}
					catch(NumberFormatException ex) { }
					catch(IndexOutOfBoundsException ex) { }
				}
				selectionInputListen = !selectionInputListen;
			}
		};
		panel.getActionMap().put("Select a selection preset",actSelectSS);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_P,0),"Select path");
		AbstractAction actSelectPath = new AbstractAction() {
			private static final long serialVersionUID = -8804517791543118334L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionInputListen) {
					selectedLocationsInt = new ArrayList<Integer>();
					selectionString = "";
					clearLocations();
					System.out.println("Input one path index in [0.." + (allPaths.size()-1) + "]: " + selectionString);
				}
				else if (selectionInputListen) {
					clearLocations();
					try {
						int pathNumber = Integer.parseInt(selectionString);
						ArrayList<Entry<String,ArrayList<PoseSteering>>> allEntries = new ArrayList<Entry<String,ArrayList<PoseSteering>>>(allPaths.entrySet());
						if (allEntries.size() > pathNumber) {
							Entry<String,ArrayList<PoseSteering>> toSelect = allEntries.get(pathNumber);
							int first = -1;
							int last = -1;
							String fromLoc = toSelect.getKey().substring(0,toSelect.getKey().indexOf("->"));
							String toLoc = toSelect.getKey().substring(toSelect.getKey().indexOf("->")+2);
							for (int i = 0; i < locationIDs.size(); i++) {
								if (locationIDs.get(i).equals(fromLoc)) first = i;
								else if (locationIDs.get(i).equals(toLoc)) last = i;
							}
							selectionString = first + "," + last;
							selectedLocationsInt = new ArrayList<Integer>();
							selectedLocationsInt.add(first);
							selectedLocationsInt.add(last);
							highlightSelectedLocations();
						}
					}
					catch(NumberFormatException ex) { }
				}
				selectionInputListen = !selectionInputListen;
			}
		};
		panel.getActionMap().put("Select path",actSelectPath);

//		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_Z,KeyEvent.CTRL_DOWN_MASK),"Undo");
//		AbstractAction actUndo = new AbstractAction() {
//			private static final long serialVersionUID = 5597593272769688561L;
//			@Override
//			public void actionPerformed(ActionEvent e) {
//				restorePath();
//			}
//		};
//		panel.getActionMap().put("Undo",actUndo);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,KeyEvent.CTRL_DOWN_MASK),"Save locations and paths");
		AbstractAction actSave = new AbstractAction() {
			private static final long serialVersionUID = 8788274388808789051L;
			@Override
			public void actionPerformed(ActionEvent e) {
				HashMap<String,Pose> locs = new HashMap<String,Pose>();
				for (String locationName : locationIDs) {
					locs.put(locationName, Missions.getLocation(locationName));
				}
				dumpLocationAndPathData(locs, allPaths);
				
//				String st = "#Locations\n";
//				for (String locationName : locationIDs) {
//					Pose locPose = Missions.getLocation(locationName);
//					st += locationName + "\t" + locPose.getX() + "\t" + locPose.getY() + "\t" + locPose.getTheta() + "\n";
//				}
//				st+="\n#Paths\n";
//				for (Entry<String,ArrayList<PoseSteering>> entry : allPaths.entrySet()) {
//					String pathFilename = entry.getKey().replaceAll("->", "-")+".path";
//					st += entry.getKey().replaceAll("->", " -> ") + "\t" + pathFilename + "\n";
//					Missions.writePath(outputDir+File.separator+pathFilename, entry.getValue());
//				}
//		        try {
//		        	String newFilename = outputDir+File.separator+"locations_and_paths.txt";
//		            File file = new File(newFilename);
//		            PrintWriter writer = new PrintWriter(file);
//		            writer.write(st);
//		            writer.close();
//		            System.out.println("Saved locations and paths file: " + newFilename);
//		        }
//		        catch (Exception ex) { ex.printStackTrace(); }
			}
		};
		panel.getActionMap().put("Save locations and paths",actSave);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_H,0),"Help");
		AbstractAction actHelp = new AbstractAction() {
			private static final long serialVersionUID = 8788274388808789053L;
			@Override
			public void actionPerformed(ActionEvent e) {
				JOptionPane.showMessageDialog(panel,getHelp());
			}
		};
		panel.getActionMap().put("Help",actHelp);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_L,0),"Select location(s)");
		AbstractAction actSelectLocations = new AbstractAction() {
			private static final long serialVersionUID = -4218195170958172222L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionInputListen) {
					selectedLocationsInt = new ArrayList<Integer>();
					selectionString = "";
					selectedGroup = -1;
					clearLocations();
					System.out.println("Input list or range of locations ('x' or 'x,y,..,z' or 'x-z'): " + selectionString);
				}
				else if (selectionInputListen) {
					clearLocations();
					try {
						if (selectionString.contains("-")) {
							int first = Math.min(locationIDs.size()-1,Integer.parseInt(selectionString.substring(0,selectionString.indexOf("-"))));
							int last = Math.min(locationIDs.size()-1,Integer.parseInt(selectionString.substring(selectionString.indexOf("-")+1)));
							for (int i = first; i <= last; i++) selectedLocationsInt.add(i);
						}
						else if (selectionString.contains(",")) {
							String[] points = selectionString.split(",");
							for (String point : points) selectedLocationsInt.add(Math.min(locationIDs.size()-1,Integer.parseInt(point)));
						}
						else selectedLocationsInt.add(Math.min(locationIDs.size()-1,Integer.parseInt(selectionString)));
						highlightSelectedLocations();
						System.out.println("Current selection (locations): " + selectedLocationsInt);
					}
					catch(NumberFormatException ex) { }
				}
				selectionInputListen = !selectionInputListen;
			}
		};
		panel.getActionMap().put("Select location(s)",actSelectLocations);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_G,0),"Select group");
		AbstractAction actSelectGroup = new AbstractAction() {
			private static final long serialVersionUID = -4218195170958172242L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionInputListen) {
					selectedLocationsInt = new ArrayList<Integer>();
					selectionString = "";
					selectedGroup = -1;
					clearLocations();
					System.out.println("Input group index ('x'): " + selectionString);
				}
				else if (selectionInputListen) {
					clearLocations();
					try {
						selectedGroup = Integer.parseInt(selectionString);
						System.out.println(selectionGroupsToSelections);
						if (selectionGroupsToSelections.containsKey(selectedGroup)) {
							for (Integer g : selectionGroupsToSelections.get(selectedGroup)) {
								selectedLocationsInt.addAll(selectedLocationsInts.get(g));
							}
							highlightSelectedLocations();
							System.out.println("Current selection (all poses in group " + selectedGroup + "): " + selectedLocationsInt);
						}
					}
					catch(NumberFormatException ex) { }
				}
				selectionInputListen = !selectionInputListen;
			}
		};
		panel.getActionMap().put("Select group",actSelectGroup);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE,0),"Delete path between selected locations");
		AbstractAction actDelete = new AbstractAction() {
			private static final long serialVersionUID = 4455373738365388356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (selectedLocationsInt.size() >= 2) {
					String pathName = locationIDs.get(selectedLocationsInt.get(0)) + "->" + locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1));
					String pathNameInv =  locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1)) + "->" + locationIDs.get(selectedLocationsInt.get(0));
					ArrayList<PoseSteering> pathToRemove = allPaths.get(pathName);
					ArrayList<PoseSteering> pathToRemoveInv = allPaths.get(pathNameInv);
					if (pathToRemove != null) {
						for (int i = 0; i < pathToRemove.size(); i++) panel.removeGeometry(pathName+"."+i);
						panel.removeGeometry(pathName+".all");
						allPaths.remove(pathName);
						System.out.println("Removed path " + pathName);
						updatePaths();
					}
					if (pathToRemoveInv != null) {
						for (int i = 0; i < pathToRemoveInv.size(); i++) panel.removeGeometry(pathNameInv+"."+i);
						panel.removeGeometry(pathNameInv+".all");
						allPaths.remove(pathNameInv);
						System.out.println("Removed path " + pathNameInv);
						updatePaths();
					}
				}
			}
		};
		panel.getActionMap().put("Delete path between selected locations",actDelete);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_C,0),"Concatenate paths traversing selection");
		AbstractAction actMergePaths = new AbstractAction() {
			private static final long serialVersionUID = 4455371118322388356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (selectedLocationsInt.size() > 2) {
					ArrayList<PoseSteering> newPath = new ArrayList<PoseSteering>();
					ArrayList<PoseSteering> newPathInv = new ArrayList<PoseSteering>();
					for (int i = 0; i < selectedLocationsInt.size()-1; i++) {
						String pathName = locationIDs.get(selectedLocationsInt.get(i)) + "->" + locationIDs.get(selectedLocationsInt.get(i+1));
						System.out.println("\tdoing " + pathName);
						ArrayList<PoseSteering> path = allPaths.get(pathName);
						for (int j = 0; j < path.size(); j++) {
							if (j > 0 || (j == 0 && i == 0)) newPath.add(path.get(j));
						}
					}
					String newPathName = locationIDs.get(selectedLocationsInt.get(0)) + "->" + locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1));
					String newPathNameInv = locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1)) + "->" + locationIDs.get(selectedLocationsInt.get(0)); 
					for (PoseSteering ps : newPath) newPathInv.add(ps);
					Collections.reverse(newPathInv);
					allPaths.put(newPathName, newPath);
					allPaths.put(newPathNameInv, newPathInv);
					updatePaths();
					System.out.println("Concatenated paths: " + newPathName);
				}
			}
		};
		panel.getActionMap().put("Concatenate paths traversing selection",actMergePaths);
		

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE,KeyEvent.SHIFT_DOWN_MASK),"Delete path between all locations");
		AbstractAction actDeleteAll = new AbstractAction() {
			private static final long serialVersionUID = 4455373738365788356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				for (String pathName : allPaths.keySet()) {
					ArrayList<PoseSteering> pathToRemove = allPaths.get(pathName);
					if (pathToRemove != null) {
						for (int i = 0; i < pathToRemove.size(); i++) panel.removeGeometry(pathName+"."+i);
						System.out.println("Removed path " + pathName);
					}
				}
				allPaths.clear();
				updatePaths();
			}
		};
		panel.getActionMap().put("Delete path between all locations",actDeleteAll);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_BACK_SPACE,0),"Delete selected location(s)");
		AbstractAction actDeleteLoc = new AbstractAction() {
			private static final long serialVersionUID = 4455173731365388356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				for (int selectedLocationOneInt : selectedLocationsInt) {
					String locationName = locationIDs.get(selectedLocationOneInt);
					Missions.removeLocation(locationName);
				}
				removeAllKnownLocations();
				addAllKnownLocations();
				selectedLocationsInt = new ArrayList<Integer>();
				selectionString = "";
				panel.updatePanel();
			}
		};
		panel.getActionMap().put("Delete selected location(s)",actDeleteLoc);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_W,0),"Add location nearest to mouse pointer to current selection");
		AbstractAction actAddNearest = new AbstractAction() {
			private static final long serialVersionUID = 3855143731365388356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				Coordinate mouseCoord = new Coordinate(mousePosition.getX(), mousePosition.getY());
				double min = Double.MAX_VALUE;
				int locID = -1;
				for (int i = 0; i < locationIDs.size(); i++) {
					Coordinate locPosition = Missions.getLocation(locationIDs.get(i)).getPosition();
					double dist = locPosition.distance(mouseCoord);
					if (dist < min) {
						min = dist;
						locID = i;
					}
				}
				if (locID != -1 && !selectedLocationsInt.contains(locID)) {
					selectedLocationsInt.add(locID);
					System.out.println("Added location nearest pointer to selection: " + selectedLocationsInt);
					clearLocations();
					highlightSelectedLocations();
				}
			}
		};
		panel.getActionMap().put("Add location nearest to mouse pointer to current selection",actAddNearest);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_Z,0),"Deselect last added element of the selection stack");
		AbstractAction actRemoveLastAdded = new AbstractAction() {
			private static final long serialVersionUID = 3855143731335358356L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					selectedLocationsInt.remove(selectedLocationsInt.size()-1);
					System.out.println("Deselected last element of selection: " + selectedLocationsInt);
					highlightSelectedLocations();
				}
			}
		};
		panel.getActionMap().put("Deselect last added element of the selection stack",actRemoveLastAdded);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_INSERT,0),"Add locations(s)");
		AbstractAction actInsert = new AbstractAction() {
			private static final long serialVersionUID = -8804517791543118334L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					selectionString = locationIDs.size() + "-" + (locationIDs.size()+selectedLocationsInt.size()-1);
					int selectedLocationOneInt = selectedLocationsInt.get(selectedLocationsInt.size()-1);
					Pose pOld = Missions.getLocation(locationIDs.get(selectedLocationOneInt));
					double theta = Math.atan2(pOld.getY() - mousePosition.getY(), pOld.getX() - mousePosition.getX());
					Pose pNew = new Pose(mousePosition.getX(), mousePosition.getY(), theta);
					String newPoseName = "AUX_"+newLocationCounter++;
					Missions.setLocation(newPoseName, pNew);
					locationIDs.add(newPoseName);
					selectedLocationsInt.add(locationIDs.size()-1);
					//panel.addArrow((locationIDs.size()-1)+":"+newPoseName, pNew, Color.green);
					clearLocations();
					System.out.println("Inserted new locations " + selectedLocationsInt);
					highlightSelectedLocations();
				}
				else {
					ArrayList<Integer> newSelection = new ArrayList<Integer>();
					//Pose pNew = new Pose(0.0, 0.0, 0.0);
					Pose pNew = new Pose(mousePosition.getX(), mousePosition.getY(), 0.0);
					String newPoseName = "AUX_"+newLocationCounter++;
					Missions.setLocation(newPoseName, pNew);
					locationIDs.add(newPoseName);
					newSelection.add(locationIDs.size()-1);
					//panel.addArrow((locationIDs.size()-1)+":"+newPoseName, pNew, Color.green);
					clearLocations();
					selectedLocationsInt = newSelection;
					System.out.println("Inserted new locations " + selectedLocationsInt);
					highlightSelectedLocations();
				}
			}
		};
		panel.getActionMap().put("Add locations(s)",actInsert);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,KeyEvent.ALT_DOWN_MASK),"Compute spline between selected locations");
		AbstractAction actSpline = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762752293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<PoseSteering> overallPath = new ArrayList<PoseSteering>();
				for (int i = 0; i < selectedLocationsInt.size()-1; i++) {
					Pose startPose = Missions.getLocation(locationIDs.get(selectedLocationsInt.get(i)));
					Pose goalPose = Missions.getLocation(locationIDs.get(selectedLocationsInt.get(i+1)));
					PoseSteering[] path = computeSpline(startPose, goalPose);
					if (i == 0) overallPath.add(path[0]);
					for (int j = 1; j < path.length; j++) overallPath.add(path[j]);
				}
				String pathName = locationIDs.get(selectedLocationsInt.get(0))+"->"+locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1));
				allPaths.put(pathName, overallPath);
				String pathNameInv = locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1))+"->"+locationIDs.get(selectedLocationsInt.get(0));
				ArrayList<PoseSteering> overallPathInv = new ArrayList<PoseSteering>();
				for (PoseSteering ps : overallPath) overallPathInv.add(ps);
				Collections.reverse(overallPathInv);
				allPaths.put(pathNameInv, overallPathInv);
				updatePaths();
			}
		};
		panel.getActionMap().put("Compute spline between selected locations",actSpline);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,KeyEvent.ALT_DOWN_MASK|KeyEvent.SHIFT_DOWN_MASK),"Compute spline for all preset selections");
		AbstractAction actSplineAll = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762752293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				ArrayList<ArrayList<Integer>> selectionsToPlan = selectedLocationsInts;
				if (selectedGroup != -1) {
					selectionsToPlan = new ArrayList<ArrayList<Integer>>();
					for (Integer g : selectionGroupsToSelections.get(selectedGroup)) {
						selectionsToPlan.add(selectedLocationsInts.get(g));
					}
				}
				for (ArrayList<Integer> onePreset : selectionsToPlan) {
					if (onePreset.size() >= 2) {
						ArrayList<PoseSteering> overallPath = new ArrayList<PoseSteering>();
						ArrayList<PoseSteering> overallPathInv = new ArrayList<PoseSteering>();
						String pathName = locationIDs.get(onePreset.get(0))+"->"+locationIDs.get(onePreset.get(onePreset.size()-1));
						String pathNameInv = locationIDs.get(onePreset.get(onePreset.size()-1))+"->"+locationIDs.get(onePreset.get(0));
						if (!allPaths.containsKey(pathName)) {
							for (int i = 0; i < onePreset.size()-1; i++) {
								Pose startPose = Missions.getLocation(locationIDs.get(onePreset.get(i)));
								Pose goalPose = Missions.getLocation(locationIDs.get(onePreset.get(i+1)));
								PoseSteering[] path = computeSpline(startPose, goalPose);
								if (i == 0) overallPath.add(path[0]);
								for (int j = 1; j < path.length; j++) overallPath.add(path[j]);
							}
							allPaths.put(pathName, overallPath);
							for (PoseSteering ps : overallPath) overallPathInv.add(ps);
							Collections.reverse(overallPathInv);
							allPaths.put(pathNameInv, overallPathInv);
						}
					}
				}
				updatePaths();
			}
		};
		panel.getActionMap().put("Compute spline for all preset selections",actSplineAll);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_R,0),"Reverse order of selection");
		AbstractAction reverseSelection = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762733293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (selectedLocationsInt.size() >= 2) {
					Collections.reverse(selectedLocationsInt);
					clearLocations();
					System.out.println("Reversed selection: " + selectedLocationsInt);
					highlightSelectedLocations();
					
				}
			}
		};
		panel.getActionMap().put("Reverse order of selection",reverseSelection);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_U,0),"Delete last element of selection");
		AbstractAction removeLastInSelection = new AbstractAction() {
			private static final long serialVersionUID = -3348585469762733293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (selectedLocationsInt.size() >= 1) {
					int selectedLocationOneInt = selectedLocationsInt.remove(selectedLocationsInt.size()-1);
					String locationName = locationIDs.get(selectedLocationOneInt);
					Missions.removeLocation(locationName);
					removeKnownLocation(locationName);
					clearLocations();
					System.out.println("Deleted last element of selection: " + selectedLocationsInt);
					highlightSelectedLocations();
				}
			}
		};
		panel.getActionMap().put("Delete last element of selection",removeLastInSelection);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_P,KeyEvent.ALT_DOWN_MASK),"Plan path between selected locations");
		AbstractAction actPlan = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762752293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (selectedLocationsInt.size() >= 2) {
					Pose startPose = Missions.getLocation(locationIDs.get(selectedLocationsInt.get(0)));
					Pose[] goalPoses = new Pose[selectedLocationsInt.size()-1];
					for (int i = 0; i < goalPoses.length; i++) {
						goalPoses[i] = Missions.getLocation(locationIDs.get(selectedLocationsInt.get(i+1)));
					}
					PoseSteering[] newPath = computePath(startPose,goalPoses);
					if (newPath != null) {
						String pathName = locationIDs.get(selectedLocationsInt.get(0))+"->"+locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1));
						String pathNameInv = locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1))+"->"+locationIDs.get(selectedLocationsInt.get(0));
						ArrayList<PoseSteering> newPathAL = new ArrayList<PoseSteering>();
						ArrayList<PoseSteering> newPathALInv = new ArrayList<PoseSteering>();
						for (PoseSteering ps : newPath) {
							newPathAL.add(ps);
							newPathALInv.add(ps);
						}
						Collections.reverse(newPathALInv);
						allPaths.put(pathName,newPathAL);
						allPaths.put(pathNameInv,newPathALInv);
						updatePaths();
					}
				}
			}
		};
		panel.getActionMap().put("Plan path between selected locations",actPlan);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_P,KeyEvent.ALT_DOWN_MASK|KeyEvent.SHIFT_DOWN_MASK),"Plan path for all selection presets");
		AbstractAction actPlanAll = new AbstractAction() {
			private static final long serialVersionUID = -3238585469762752293L;
			@Override
			public void actionPerformed(ActionEvent e) {
				for (ArrayList<Integer> onePreset : selectedLocationsInts) {
					if (onePreset.size() >= 2) {
						String pathName = locationIDs.get(onePreset.get(0))+"->"+locationIDs.get(onePreset.get(onePreset.size()-1));
						String pathNameInv = locationIDs.get(onePreset.get(onePreset.size()-1))+"->"+locationIDs.get(onePreset.get(0));
						if (!allPaths.containsKey(pathName)) {
							Pose startPose = Missions.getLocation(locationIDs.get(onePreset.get(0)));
							Pose[] goalPoses = new Pose[onePreset.size()-1];
							for (int i = 0; i < goalPoses.length; i++) {
								goalPoses[i] = Missions.getLocation(locationIDs.get(onePreset.get(i+1)));
							}
							PoseSteering[] newPath = computePath(startPose,goalPoses);
							if (newPath != null) {
								ArrayList<PoseSteering> newPathAL = new ArrayList<PoseSteering>();
								ArrayList<PoseSteering> newPathALInv = new ArrayList<PoseSteering>();
								for (PoseSteering ps : newPath) {
									newPathAL.add(ps);
									newPathALInv.add(ps);
								}
								allPaths.put(pathName,newPathAL);
								Collections.reverse(newPathALInv);
								allPaths.put(pathNameInv,newPathALInv);
							}
						}
					}
					updatePaths();
				}
			}
		};
		panel.getActionMap().put("Plan path for all selection presets",actPlanAll);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT,0),"Decrease X of selected location(s)");
		AbstractAction actXMinus = new AbstractAction() {
			private static final long serialVersionUID = 1767256680398690970L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedLocationOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedLocationOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						x -= deltaX;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Decrease X of selected location(s)",actXMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT,KeyEvent.SHIFT_DOWN_MASK),"Speed decrease X of selected location(s)");
		AbstractAction actXMinusFast = new AbstractAction() {
			private static final long serialVersionUID = 1767256680448690970L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						x -= (10*deltaX);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Speed decrease X of selected location(s)",actXMinusFast);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT,0),"Increase X of selected location(s)");
		AbstractAction actXPlus = new AbstractAction() {
			private static final long serialVersionUID = 6900418766755234220L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						x += deltaX;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Increase X of selected location(s)",actXPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT,KeyEvent.SHIFT_DOWN_MASK),"Speed increase X of selected location(s)");
		AbstractAction actXPlusFast = new AbstractAction() {
			private static final long serialVersionUID = 6980418766755234220L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						x += (10*deltaX);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Speed increase X of selected location(s)",actXPlusFast);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_UP,0),"Increase Y of selected location(s)");
		AbstractAction actYPlus = new AbstractAction() {
			private static final long serialVersionUID = 2627197997139919535L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						y += deltaY;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Increase Y of selected location(s)",actYPlus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_UP,KeyEvent.SHIFT_DOWN_MASK),"Speed increase Y of selected location(s)");
		AbstractAction actYPlusFast = new AbstractAction() {
			private static final long serialVersionUID = 2627197997139919525L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						y += (10*deltaY);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Speed increase Y of selected location(s)",actYPlusFast);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DOWN,0),"Decrease Y of selected location(s)");
		AbstractAction actYMinus = new AbstractAction() {
			private static final long serialVersionUID = 6487878455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						y -= deltaY;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Decrease Y of selected location(s)",actYMinus);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DOWN,KeyEvent.SHIFT_DOWN_MASK),"Speed decrease Y of selected location(s)");
		AbstractAction actYMinusFast = new AbstractAction() {
			private static final long serialVersionUID = 6487878415015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						y -= (10*deltaY);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Speed decrease Y of selected location(s)",actYMinusFast);
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_DOWN,0),"Decrease theta of selected location(s)");
		AbstractAction actTMinus = new AbstractAction() {
			private static final long serialVersionUID = -7391970411254721019L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						th -= deltaT;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Decrease theta of selected location(s)",actTMinus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_DOWN,KeyEvent.SHIFT_DOWN_MASK),"Speed decrease theta of selected location(s)");
		AbstractAction actTMinusS = new AbstractAction() {
			private static final long serialVersionUID = -7391970411254721019L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						th -= (10*deltaT);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Speed decrease theta of selected location(s)",actTMinusS);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_UP,0),"Increase theta of selected location(s)");
		AbstractAction actTPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						th += deltaT;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Increase theta of selected location(s)",actTPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PAGE_UP,KeyEvent.SHIFT_DOWN_MASK),"Speed increase theta of selected location(s)");
		AbstractAction actTPlusS = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocation(locationName).getX();
						double y = Missions.getLocation(locationName).getY();
						double th = Missions.getLocation(locationName).getTheta();
						th += (10*deltaT);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}
				}
			}
		};
		panel.getActionMap().put("Speed increase theta of selected location(s)",actTPlusS);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_I,0),"Info");
		AbstractAction actInfo = new AbstractAction() {
			private static final long serialVersionUID = 8424380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				System.out.println("Locations:");
				for (int i = 0; i < locationIDs.size(); i++) System.out.println("   " + i + ": " + locationIDs.get(i));
				if (selectionStrings != null) {
					System.out.println("Selection presets:");
					//for (int i = 0; i < selectionStrings.size(); i++) System.out.println("   " + i + ":" + selectionStrings.get(i));
					for (int i = 0; i < selectedLocationsInts.size(); i++) {
						System.out.println("   " + i + ": " + selectionStringsNames.get(i) + " " + selectedLocationsInts.get(i));
					}
				}
			}
		};
		panel.getActionMap().put("Info",actInfo);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_M,KeyEvent.SHIFT_DOWN_MASK),"Increase minimum distance between path points for path planning");
		AbstractAction actMDPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				DISTANCE_BETWEEN_PATH_POINTS += deltaD;
				System.out.println("Minimum distance between path points (>): " + DISTANCE_BETWEEN_PATH_POINTS);
			}
		};
		panel.getActionMap().put("Increase minimum distance between path points for path planning",actMDPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_M,0),"Decrease minimum distance between path points for path planning");
		AbstractAction actMDMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (DISTANCE_BETWEEN_PATH_POINTS-deltaD >= 0) DISTANCE_BETWEEN_PATH_POINTS -= deltaD;
				System.out.println("Minimum distance between path points (<): " + DISTANCE_BETWEEN_PATH_POINTS);
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
		
		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PLUS,0),"Increase spline distance");
		AbstractAction actSDPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				SPLINE_DISTANCE += deltaSD;
				System.out.println("Spline distance (>): " + SPLINE_DISTANCE);
			}
		};
		panel.getActionMap().put("Increase spline distance",actSDPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_MINUS,0),"Decrease spline distance");
		AbstractAction actSDMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				SPLINE_DISTANCE -= deltaSD;
				System.out.println("Spline distance (<): " + SPLINE_DISTANCE);
			}
		};
		panel.getActionMap().put("Decrease spline distance",actSDMinus);


		panel.setFocusable(true);
		panel.setArrowHeadSizeInMeters(2.4);
		panel.setTextSizeInMeters(2.3);
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

	private PoseSteering[] computeSpline(Pose from, Pose to) {
		Coordinate[] controlPoints = new Coordinate[4];
		controlPoints[0] = new Coordinate(from.getX(),from.getY(),0.0);
		controlPoints[1] = new Coordinate(from.getX()+SPLINE_DISTANCE*Math.cos(from.getTheta()), from.getY()+SPLINE_DISTANCE*Math.sin(from.getTheta()), 0.0);
		controlPoints[2] = new Coordinate(to.getX()-SPLINE_DISTANCE*Math.cos(to.getTheta()), to.getY()-SPLINE_DISTANCE*Math.sin(to.getTheta()), 0.0);
		controlPoints[3] = new Coordinate(to.getX(),to.getY(),0.0);
		Coordinate[] spline = BezierSplineFactory.createBezierSpline(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3], DISTANCE_BETWEEN_PATH_POINTS);
		return BezierSplineFactory.asPoseSteering(spline);
		//Spline3D spline1 = SplineFactory.createBezier(controlPoints, DISTANCE_BETWEEN_PATH_POINTS);
		//return spline1.asPoseSteerings();
	}

	private PoseSteering[] computePath(Pose from, Pose ... to) {
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		if (this.mapFilename == null) {
			rsp.setMapFilename(makeEmptyMapMap());
			rsp.setMapResolution(1.0);
		}
		else {
			rsp.setMapFilename(mapImgFilename);
			rsp.setMapResolution(mapRes);
		}
		rsp.setRadius(this.radius);
		rsp.setFootprint(this.footprint);
		rsp.setTurningRadius(MAX_TURNING_RADIUS);
		rsp.setDistanceBetweenPathPoints(DISTANCE_BETWEEN_PATH_POINTS);
		rsp.setStart(from);
		Pose[] goalPoses = new Pose[to.length];
		for (int i = 0; i < goalPoses.length; i++) goalPoses[i] = to[i];
		rsp.setGoals(goalPoses);
//		rsp.addObstacles(obstacles.toArray(new Geometry[obstacles.size()]));
		if (!rsp.plan()) return null;
		PoseSteering[] ret = rsp.getPath();
		return ret;
	}

	public static void main(String[] args) {

		//ICAPS 2
		String mapFilename = "/home/fpa/catkin_ws/src/coordination_oru_ros/maps/map-partial-2.yaml";
		String locs = "/home/fpa/catkin_ws/src/coordination_oru_ros/missions/icaps_locations_and_paths_1.txt";
		String sel = "/home/fpa/catkin_ws/src/coordination_oru_ros/missions/icaps_selections.txt";
		PathEditor2 pe2 = new PathEditor2(locs,mapFilename,null);
		pe2.setDeltaX(0.5);
		pe2.setDeltaY(0.5);
		pe2.setSplineDistance(3.0);
		pe2.setDistanceBetweenPathPoints(0.5);
		pe2.setPathPlanningRadius(0.1);
		pe2.setMaxTurningRadius(2.0);
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		pe2.setPathPlanningFootprint(footprint1,footprint2,footprint3,footprint4,footprint1);

//		//ICAPS 1
//		String mapFilename = "/home/fpa/catkin_ws/src/coordination_oru_ros/maps/map-partial-1.yaml";
//		String locs = "/home/fpa/catkin_ws/src/coordination_oru_ros/missions/icaps_locations_and_paths.txt";
//		String sel = "/home/fpa/catkin_ws/src/coordination_oru_ros/missions/icaps_selections.txt";
//		PathEditor2 pe2 = new PathEditor2(locs,mapFilename,sel);
//		pe2.setDeltaX(0.5);
//		pe2.setDeltaY(0.5);
//		pe2.setSplineDistance(3.0);
//		pe2.setDistanceBetweenPathPoints(0.5);
//		pe2.setPathPlanningRadius(0.1);
//		pe2.setMaxTurningRadius(2.0);
//		Coordinate footprint1 = new Coordinate(-1.0,0.5);
//		Coordinate footprint2 = new Coordinate(1.0,0.5);
//		Coordinate footprint3 = new Coordinate(1.0,-0.5);
//		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
//		pe2.setPathPlanningFootprint(footprint1,footprint2,footprint3,footprint4,footprint1);

//		String locAndPathFilename = "paths/locations.txt";
//		String selectionsFile = "paths/selections.txt";
//		PathEditor2 pe2 = new PathEditor2(locAndPathFilename,null,selectionsFile);
//		pe2.setDeltaX(5.0);
//		pe2.setDeltaY(5.0);
//		pe2.setSplineDistance(3.0);
//		pe2.setDistanceBetweenPathPoints(0.3);
		
		//Volvo GTO
//		String locAndPathFilename = "/home/fpa/catkin_ws/src/volvo_gto/coordination_gto/missions/GTO_locations_and_paths.txt";
//		String selectionsFile = null;
//		String mapFilename = "/home/fpa/catkin_ws/src/volvo_gto/gazebo_vgto/gazebo_worlds_vgto/maps/vgto_plant.yaml";
//		//new PathEditor2(locAndPathFilename,mapFilename);
//		new PathEditor2(locAndPathFilename,mapFilename,selectionsFile);
		
//		//Volvo CE
//		String locAndPathFilename = "paths/elsite_locations.bare.txt";
//		String selectionsFile = "paths/selections.txt";
//		String mapFilename = "maps/elsite_1m.yaml";
//		//new PathEditor2(locAndPathFilename,mapFilename);
//		new PathEditor2(locAndPathFilename,mapFilename,selectionsFile);
				
	}

	@Override
	public void mouseDragged(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseMoved(MouseEvent e) {
		AffineTransform geomToScreen = panel.getMapTransform();
		try {
			AffineTransform geomToScreenInv = geomToScreen.createInverse();
			this.mousePosition = geomToScreenInv.transform(new Point2D.Double(e.getX(),e.getY()), null);
			//System.out.println(mousePosition);
		} catch (NoninvertibleTransformException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
	}

}
