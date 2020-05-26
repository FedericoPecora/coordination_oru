package se.oru.coordination.coordination_oru.util;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Scanner;
import java.util.Set;
import java.util.TreeMap;

import javax.swing.AbstractAction;
import javax.swing.JOptionPane;
import javax.swing.KeyStroke;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.UI.JTSDrawingPanel;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.util.AffineTransformation;

import se.oru.coordination.coordination_oru.motionplanning.OccupancyMap;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner.PLANNING_ALGORITHM;
import se.oru.coordination.coordination_oru.util.splines.BezierSplineFactory;

public class PathEditor2 {

	private static int newLocationCounter = 0;
	private String selectionsFile = null;
	private String outputDir = null;

	private double PP_max_turning_radius = 5.0;
	private double SP_spline_distance = 20.0;
	private double PP_SP_distance_between_path_points = 1.5;
	private double PP_radius = 0.2;
	private Coordinate[] PP_footprint = null;
	private Geometry PP_footprintGeom = null;

	private String locationsAndPathsFilename = null;
	private String mapFilename = null;
	private boolean selectionInputListen = false;
	private ArrayList<String> selectionStringsNames = null;
	private ArrayList<String> selectionStrings = null;
	private ArrayList<ArrayList<Integer>> selectedLocationsInts = null;
	private HashMap<Integer,ArrayList<Integer>> selectionGroupsToSelections = null;
	private int selectedGroup = -1;
	private ReedsSheppCarPlanner mp = null;

	private OccupancyMap om = null;

	private String selectionString = "";
	private ArrayList<Integer> selectedLocationsInt = new ArrayList<Integer>();
	private ArrayList<String> locationIDs = new ArrayList<String>();
	private HashMap<String,ArrayList<PoseSteering>> allPaths = null;
	private HashMap<String,Boolean> isInversePath = null;
	private JTSDrawingPanel panel = null;
	private double deltaX = 0.1;
	private double deltaY = 0.1;
	private double deltaT = 0.01;
	private double deltaTR = 0.1;
	private double deltaSD = 0.5;
	private double deltaD = 0.1;

	private static String TEMP_MAP_DIR = ".tempMapsPathEditor";

	public void setPanAcceleration(double accel) {
		this.panel.setPanAcceleration(accel);
	}

	public void setZoomIntensity(double intens) {
		this.panel.setZoomIntensity(intens);
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
		SP_spline_distance = val;
	}

	public void setDistanceBetweenPathPoints(double val) {
		PP_SP_distance_between_path_points = val;
	}

	public void setMaxTurningRadius(double val) {
		PP_max_turning_radius = val;
	}

	@Deprecated
	public PathEditor2() {
		this(null, null, null, 0.1, 0.1, 0.01, (Coordinate[])null);
	}

	@Deprecated
	public PathEditor2(String posesAndPaths) {
		this(posesAndPaths, null, null, 0.1, 0.1, 0.01, (Coordinate[])null);
	}

	@Deprecated
	public PathEditor2(String posesAndPaths, String mapFilename) {
		this(posesAndPaths, mapFilename, null, 0.1, 0.1, 0.01, (Coordinate[])null);
	}

	@Deprecated
	public PathEditor2(String posesAndPaths, String mapFilename, String selectionsFile) {
		this(posesAndPaths, mapFilename, selectionsFile, 0.1, 0.1, 0.01, (Coordinate[])null);
	}

	@Deprecated
	public PathEditor2(String posesAndPaths, String mapFN, String selectionsF, double deltaX, double deltaY, double deltaTheta) {
		this(posesAndPaths, mapFN, selectionsF, deltaX, deltaY, deltaTheta, (Coordinate[])null);
	}

	public PathEditor2(Coordinate ... footprint) {
		this(null, null, null, 0.1, 0.1, 0.01, footprint);
	}

	public PathEditor2(String posesAndPaths, Coordinate ... footprint) {
		this(posesAndPaths, null, null, 0.1, 0.1, 0.01, footprint);
	}

	public PathEditor2(String posesAndPaths, String mapFilename, Coordinate ... footprint) {
		this(posesAndPaths, mapFilename, null, 0.1, 0.1, 0.01, footprint);
	}

	public PathEditor2(String posesAndPaths, String mapFilename, String selectionsFile, Coordinate ... footprint) {
		this(posesAndPaths, mapFilename, selectionsFile, 0.1, 0.1, 0.01, footprint);
	}

	public PathEditor2(String posesAndPaths, String mapFilename, String selectionsFilename, double deltaX, double deltaY, double deltaTheta, Coordinate ... footprint) {
		this.deltaX = deltaX;
		this.deltaY = deltaY;
		this.deltaT = deltaTheta;
		this.mapFilename = mapFilename;
		this.PP_footprint = footprint;
		this.setPathPlanningFootprint(footprint);
		if (this.mapFilename != null) {
			System.out.println("MAP YAML: " + this.mapFilename);
			this.om = new OccupancyMap(mapFilename);
		}
		this.setupGUI();
		if (this.mapFilename != null) {
			panel.setMap(this.mapFilename);
		}
		this.deleteDir(new File(TEMP_MAP_DIR));
		new File(TEMP_MAP_DIR).mkdir();

		this.outputDir = "output";

		this.allPaths = new HashMap<String, ArrayList<PoseSteering>>();
		this.isInversePath = new HashMap<String, Boolean>();

		if (posesAndPaths != null) {
			this.locationsAndPathsFilename = posesAndPaths;
			String pathURI = this.locationsAndPathsFilename.substring(0, this.locationsAndPathsFilename.lastIndexOf(File.separator)+1);
			Missions.loadRoadMap(this.locationsAndPathsFilename);
			addAllKnownLocations();
			HashMap<String,Pose> locations = Missions.getLocationsAndPoses();
			for (Entry<String,Pose> entry : locations.entrySet()) {
				//System.out.println("Added location " + entry.getKey() + ": " + entry.getValue());
				for (Entry<String,Pose> entry1 : locations.entrySet()) {
					if (!entry1.equals(entry)) {
						if (Missions.isKnownPath(entry.getKey(), entry1.getKey())) {
							PoseSteering[] path = Missions.getShortestPath(entry.getKey(),entry1.getKey());								
							ArrayList<PoseSteering> pathAL = new ArrayList<PoseSteering>();
							for (PoseSteering ps : path) pathAL.add(ps);
							allPaths.put(entry.getKey()+"->"+entry1.getKey(), pathAL);
							isInversePath.put(entry.getKey()+"->"+entry1.getKey(), false);
						}
						if (Missions.isKnownPath(entry1.getKey(), entry.getKey())) {
							PoseSteering[] path = Missions.getShortestPath(entry1.getKey(),entry.getKey());								
							ArrayList<PoseSteering> pathAL = new ArrayList<PoseSteering>();
							for (PoseSteering ps : path) pathAL.add(ps);
							allPaths.put(entry1.getKey()+"->"+entry.getKey(), pathAL);
							isInversePath.put(entry1.getKey()+"->"+entry.getKey(), false);
						}
					}
				}
			}
			updatePaths2();
		}

		if (selectionsFilename != null) {
			this.selectionsFile = selectionsFilename;
			loadSelectionsFile();
		}

	}

	public void setPathPlanningFootprint(Coordinate ... footprint) {
		//for (int i = 0; i < footprint.length; i++) System.out.println(i + ": " + footprint[i]);
		if (!footprint[0].equals(footprint[footprint.length-1])) {
			Coordinate[] fn = new Coordinate[footprint.length+1];
			for (int i = 0; i < footprint.length; i++) fn[i] = footprint[i];
			fn[fn.length-1] = fn[0];
			//for (int j = 0; j < fn.length; j++) System.out.println(j + "*: " + fn[j]);
			this.PP_footprint = fn;
			this.PP_footprintGeom = TrajectoryEnvelope.createFootprintPolygon(fn);
		}
		else {
			this.PP_footprint = footprint;
			this.PP_footprintGeom = TrajectoryEnvelope.createFootprintPolygon(footprint);
		}
	}

	public void setPathPlanningRadius(double rad) {
		this.PP_radius = rad;
	}

	public Geometry makeFootprint(double x, double y, double theta) {
		AffineTransformation at = new AffineTransformation();
		at.rotate(theta);
		at.translate(x,y);
		Geometry rect = at.transform(PP_footprintGeom);
		return rect;
	}

	public Geometry createEnvelope(ArrayList<PoseSteering> path) {
		Geometry onePoly = null;
		Geometry prevPoly = null;
		for (PoseSteering ps : path) {
			Geometry rect = makeFootprint(ps.getX(), ps.getY(), ps.getTheta());			
			if (onePoly == null) {
				onePoly = rect;
				prevPoly = rect;
			}
			else {
				Geometry auxPoly = prevPoly.union(rect);
				onePoly = onePoly.union(auxPoly.convexHull());
				prevPoly = rect;
			}
		}
		return onePoly;
		//return onePoly.getCoordinates();
	}

	public void updatePaths2() {

		//Remove all envelopes, add all edges
		for (Entry<String,ArrayList<PoseSteering>> onePath : allPaths.entrySet()) {
			String pathName = onePath.getKey();
			ArrayList<PoseSteering> path = onePath.getValue();
			panel.removeGeometry("_"+pathName+".env");
			//panel.addArrow(pathName+".all", path.get(0).getPose(), path.get(path.size()-1).getPose(), Color.lightGray);
		}

		//Add envelopes of path connected to selected points
		if (selectedLocationsInt.size() > 0) {
			Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
			if (selectedLocationsInt.size() == 2) {
				String from = locationIDs.get(selectedLocationsInt.get(0));
				String to = locationIDs.get(selectedLocationsInt.get(1));
				for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
					if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
						ArrayList<PoseSteering> path = entry.getValue();
						Geometry env = createEnvelope(path);
						//addGeometry(String id, Geometry geom, boolean empty, boolean thick, boolean transp, String color)
						panel.addGeometry("_"+entry.getKey()+".env",env, false, false, true, "#ff4f4f");
					}
				}
			}
			else {
				for (int oneLocation : selectedLocationsInt) {
					String locationName = locationIDs.get(oneLocation);
					for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
						if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
							ArrayList<PoseSteering> path = entry.getValue();
							Geometry env = createEnvelope(path);
							//addGeometry(String id, Geometry geom, boolean empty, boolean thick, boolean transp, String color)
							panel.addGeometry("_"+entry.getKey()+".env",env, false, false, true, "#ff4f4f");
						}
					}
				}
			}
		}
		panel.updatePanel();
	}

	public void deleteForwardAndInversePaths(ArrayList<Integer> selectedLocsInt) {
		if (selectedLocsInt.size() > 0) {
			ArrayList<String> toRemove = new ArrayList<String>();
			if (selectedLocationsInt.size() == 2) {
				String from = locationIDs.get(selectedLocationsInt.get(0));
				String to = locationIDs.get(selectedLocationsInt.get(1));
				for (Entry<String, ArrayList<PoseSteering>> entry : allPaths.entrySet()) {
					if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
						toRemove.add(entry.getKey());
					}
					if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(to) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(from)) {
						toRemove.add(entry.getKey());
					}
				}
			}
			else {
				for (int oneLocation : selectedLocsInt) {
					String locationName = locationIDs.get(oneLocation);
					for (Entry<String, ArrayList<PoseSteering>> entry : allPaths.entrySet()) {
						if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
							toRemove.add(entry.getKey());
						}
					}
				}
			}
			for (String rem : toRemove) {
				panel.removeGeometry("_"+rem+".env");
				panel.removeGeometry(rem+".all");
				allPaths.remove(rem);
				isInversePath.remove(rem);
				System.out.println("Removed path " + rem);
				updatePaths2();
			}			
		}
	}

	public void computeForwardAndInversePathsWithSpline(ArrayList<Integer> selectedLocsInt) {
		ArrayList<PoseSteering> overallPath = new ArrayList<PoseSteering>();
		for (int i = 0; i < selectedLocsInt.size()-1; i++) {
			Pose startPose = Missions.getLocationPose(locationIDs.get(selectedLocsInt.get(i)));
			Pose goalPose = Missions.getLocationPose(locationIDs.get(selectedLocsInt.get(i+1)));
			PoseSteering[] path = computeSpline(startPose, goalPose);
			if (i == 0) overallPath.add(path[0]);
			for (int j = 1; j < path.length; j++) overallPath.add(path[j]);
		}
		String pathName = locationIDs.get(selectedLocsInt.get(0))+"->"+locationIDs.get(selectedLocsInt.get(selectedLocsInt.size()-1));
		allPaths.put(pathName, overallPath);
		isInversePath.put(pathName, false);
		String pathNameInv = locationIDs.get(selectedLocsInt.get(selectedLocsInt.size()-1))+"->"+locationIDs.get(selectedLocsInt.get(0));
		ArrayList<PoseSteering> overallPathInv = new ArrayList<PoseSteering>();
		for (PoseSteering ps : overallPath) overallPathInv.add(ps);
		Collections.reverse(overallPathInv);
		allPaths.put(pathNameInv, overallPathInv);
		isInversePath.put(pathNameInv, true);
		updatePaths2();
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

	//	public void updatePaths() {
	//		for (Entry<String,ArrayList<PoseSteering>> onePath : allPaths.entrySet()) {
	//			String pathName = onePath.getKey();
	//			ArrayList<PoseSteering> path = onePath.getValue();
	//			for (int i = 0; i < path.size(); i++) {
	//				panel.addArrow("_"+pathName+"."+i, path.get(i).getPose(), Color.blue);
	//			}
	//			panel.addArrow(pathName+".all", path.get(0).getPose(), path.get(path.size()-1).getPose(), Color.lightGray);
	//		}
	//		panel.updatePanel();
	//	}

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
		for (Entry<String,Pose> entry : Missions.getLocationsAndPoses().entrySet()) {
			//if (entry.getKey().startsWith("AUX_")) newLocationCounter++;
			newLocationCounter++;
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
		for (Entry<String,Pose> entry : locs.entrySet()) {
			Missions.addLocationToRoadMap(entry.getKey(), entry.getValue());
		}
		for (Entry<String,ArrayList<PoseSteering>> entry : pths.entrySet()) {
			Missions.addPathToRoadMap(entry.getKey().substring(0,entry.getKey().indexOf("->")), entry.getKey().substring(entry.getKey().indexOf("->")+2), entry.getValue().toArray(new PoseSteering[entry.getValue().size()]));
		}
		this.deleteDir(new File(this.outputDir));
		//new File(outputDir).mkdir();
		Missions.saveRoadMap(outputDir);
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
				panel.addArrow("LOC:"+locationIDs.get(selectedPathPointOneInt), Missions.getLocationPose(locationIDs.get(selectedPathPointOneInt)), Color.red);
			}
		}
		panel.updatePanel();
	}

	private void clearLocations() {
		for (int i = 0; i < locationIDs.size(); i++) {
			panel.addArrow("LOC:"+locationIDs.get(i), Missions.getLocationPose(locationIDs.get(i)), Color.green);
		}
		panel.updatePanel();
	}

	public void addAbstractAction(AbstractAction aa, int keyEvent, int modifiers, String description) {
		panel.getInputMap().put(KeyStroke.getKeyStroke(keyEvent,modifiers),description);
		panel.getActionMap().put(description,aa);
	}

	private Point2D getMousePositionInMap() {
		Point2D mousePositionInMap = null;
		AffineTransform geomToScreen = panel.getMapTransform();
		try {
			AffineTransform geomToScreenInv = geomToScreen.createInverse();
			mousePositionInMap = geomToScreenInv.transform(new Point2D.Double(panel.getMousePosition().x,panel.getMousePosition().y), null);
			//System.out.println(mousePosition);
		} catch (NoninvertibleTransformException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		return mousePositionInMap;
	}
	
	private void clearESCAPE() {
		selectionInputListen = false;
		selectedLocationsInt = new ArrayList<Integer>();
		selectionString = "";
		clearLocations();
		updatePaths2();
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
				clearESCAPE();
				/*
				selectionInputListen = false;
				selectedLocationsInt = new ArrayList<Integer>();
				selectionString = "";
				clearLocations();
				updatePaths2();
				*/
			}
		};
		panel.getActionMap().put("Cancel selection",actCancel);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,0),"Select a selection preset");
		AbstractAction actSelectSS = new AbstractAction() {
			private static final long serialVersionUID = -8804517791543118334L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectionInputListen) {
					if (selectionStrings != null) {
						selectedLocationsInt = new ArrayList<Integer>();
						selectionString = "";
						selectedGroup = -1;
						clearLocations();
						System.out.println("Input one path index in [0.." + (selectionStrings.size()-1) + "]: " + selectionString);
					}
				}
				else if (selectionInputListen) {
					clearLocations();
					try {
						if (selectionStrings != null) {
							int ssNumber = Integer.parseInt(selectionString);
							selectionString = selectionStrings.get(ssNumber);
							selectedLocationsInt = selectedLocationsInts.get(ssNumber);
							highlightSelectedLocations();
							updatePaths2();
							System.out.println("Current selection (locations): " + selectedLocationsInt);
						}
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
							updatePaths2();
						}
					}
					catch(NumberFormatException ex) { }
				}
				selectionInputListen = !selectionInputListen;
			}
		};
		panel.getActionMap().put("Select path",actSelectPath);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_S,KeyEvent.CTRL_DOWN_MASK),"Save locations and paths");
		AbstractAction actSave = new AbstractAction() {
			private static final long serialVersionUID = 8788274388808789051L;
			@Override
			public void actionPerformed(ActionEvent e) {
				HashMap<String,Pose> locs = new HashMap<String,Pose>();
				for (String locationName : locationIDs) {
					locs.put(locationName, Missions.getLocationPose(locationName));
				}
				dumpLocationAndPathData(locs, allPaths);
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

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_A,0),"Rename with given label template");
		AbstractAction actRename = new AbstractAction() {
			private static final long serialVersionUID = 8788274388899789053L;
			@Override
			public void actionPerformed(ActionEvent e) {
				String nameTemplate = JOptionPane.showInputDialog("Enter a prefix to use");
				int templateCounter = 0;
				for (int selectedLocationOneInt : selectedLocationsInt) {
					String oldName = locationIDs.get(selectedLocationOneInt);
					Pose oldPose = Missions.getLocationPose(oldName);
					String newName = nameTemplate;
					if (selectedLocationsInt.size() > 1) newName += (templateCounter++);
					locationIDs.remove(selectedLocationOneInt);
					locationIDs.add(selectedLocationOneInt, newName);
					Missions.removeLocation(oldName);
					Missions.setLocation(newName, oldPose);
					System.out.println("Renamed " + oldName + " to " + newName);
					panel.removeGeometry("LOC:"+oldName);
					ArrayList<String> pathNamesToChange = new ArrayList<String>();
					for (String pathName : allPaths.keySet()) {
						if (pathName.contains(oldName+"->") || pathName.contains("->"+oldName)) {
							pathNamesToChange.add(pathName);
						}
					}
					for (String pathNameToChange : pathNamesToChange) {
						ArrayList<PoseSteering> path = allPaths.get(pathNameToChange);
						String newPathName = pathNameToChange.replace(oldName, newName);
						allPaths.remove(pathNameToChange);
						panel.removeGeometry("_"+pathNameToChange+".env");
						allPaths.put(newPathName, path);
						System.out.println("Renamed path " + pathNameToChange + " to " + newPathName);
					}	
					clearLocations();
					highlightSelectedLocations();
				}
			}
		};
		panel.getActionMap().put("Rename with given label template",actRename);

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
						updatePaths2();
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
				deleteForwardAndInversePaths(selectedLocationsInt);
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
					isInversePath.put(newPathName, false);
					allPaths.put(newPathNameInv, newPathInv);
					isInversePath.put(newPathNameInv, true);
					updatePaths2();
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
						for (int i = 0; i < pathToRemove.size(); i++) panel.removeGeometry("_"+pathName+"."+i);
						System.out.println("Removed path " + pathName);
					}
				}
				allPaths.clear();
				updatePaths2();
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
				//Coordinate mouseCoord = new Coordinate(mousePositionInMap.getX(), mousePositionInMap.getY());

				Coordinate mouseCoord = new Coordinate(getMousePositionInMap().getX(), getMousePositionInMap().getY());
				double min = Double.MAX_VALUE;
				int locID = -1;
				for (int i = 0; i < locationIDs.size(); i++) {
					Coordinate locPosition = Missions.getLocationPose(locationIDs.get(i)).getPosition();
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
					updatePaths2();
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
					//selectionString = locationIDs.size() + "-" + (locationIDs.size()+selectedLocationsInt.size()-1);
					int selectedLocationOneInt = selectedLocationsInt.get(selectedLocationsInt.size()-1);
					Pose pOld = Missions.getLocationPose(locationIDs.get(selectedLocationOneInt));
					double theta = Math.atan2(pOld.getY() - getMousePositionInMap().getY(), pOld.getX() - getMousePositionInMap().getX());
					Pose pNew = new Pose(getMousePositionInMap().getX(), getMousePositionInMap().getY(), theta);
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
					Pose pNew = new Pose(getMousePositionInMap().getX(), getMousePositionInMap().getY(), 0.0);
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
				computeForwardAndInversePathsWithSpline(selectedLocationsInt);
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
								Pose startPose = Missions.getLocationPose(locationIDs.get(onePreset.get(i)));
								Pose goalPose = Missions.getLocationPose(locationIDs.get(onePreset.get(i+1)));
								PoseSteering[] path = computeSpline(startPose, goalPose);
								if (i == 0) overallPath.add(path[0]);
								for (int j = 1; j < path.length; j++) overallPath.add(path[j]);
							}
							allPaths.put(pathName, overallPath);
							isInversePath.put(pathName, false);
							for (PoseSteering ps : overallPath) overallPathInv.add(ps);
							Collections.reverse(overallPathInv);
							allPaths.put(pathNameInv, overallPathInv);
							isInversePath.put(pathNameInv, true);
						}
					}
				}
				updatePaths2();
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
					Pose startPose = Missions.getLocationPose(locationIDs.get(selectedLocationsInt.get(0)));
					Pose[] goalPoses = new Pose[selectedLocationsInt.size()-1];
					for (int i = 0; i < goalPoses.length; i++) {
						goalPoses[i] = Missions.getLocationPose(locationIDs.get(selectedLocationsInt.get(i+1)));
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
						isInversePath.put(pathName, false);
						allPaths.put(pathNameInv,newPathALInv);
						isInversePath.put(pathNameInv, true);
						updatePaths2();
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
							Pose startPose = Missions.getLocationPose(locationIDs.get(onePreset.get(0)));
							Pose[] goalPoses = new Pose[onePreset.size()-1];
							for (int i = 0; i < goalPoses.length; i++) {
								goalPoses[i] = Missions.getLocationPose(locationIDs.get(onePreset.get(i+1)));
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
								isInversePath.put(pathName, false);
								Collections.reverse(newPathALInv);
								allPaths.put(pathNameInv,newPathALInv);
								isInversePath.put(pathNameInv, true);
							}
						}
					}
					updatePaths2();
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						x -= deltaX;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						x -= (10*deltaX);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						x += deltaX;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						x += (10*deltaX);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						y += deltaY;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////

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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						y += (10*deltaY);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
				}
			}
		};
		panel.getActionMap().put("Speed increase Y of selected location(s)",actYPlusFast);


		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_X,0),"Scale all poses (locations and paths) by a factor");
		AbstractAction actScale = new AbstractAction() {
			private static final long serialVersionUID = 6487332455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				String scaleFactor = JOptionPane.showInputDialog("Enter a scaling factor");
				try {
					double scale = Double.parseDouble(scaleFactor);
					Set<Entry<String,Pose>> allLocations = Missions.getLocationsAndPoses().entrySet(); 
					for (Entry<String,Pose> oneLocation : allLocations) {
						double newX = oneLocation.getValue().getX()*scale;
						double newY = oneLocation.getValue().getY()*scale;
						Pose newPose = new Pose(newX,newY,oneLocation.getValue().getTheta());
						Missions.setLocation(oneLocation.getKey(), newPose);
					}

					Set<Entry<String,ArrayList<PoseSteering>>> paths = allPaths.entrySet();
					for (Entry<String,ArrayList<PoseSteering>> onePath : paths) {
						ArrayList<PoseSteering> newPath = new ArrayList<PoseSteering>();
						for (PoseSteering ps : onePath.getValue()) {
							double newX = ps.getX()*scale;
							double newY = ps.getY()*scale;
							PoseSteering newPoseSteering = new PoseSteering(newX,newY,ps.getTheta(),ps.getSteering());
							newPath.add(newPoseSteering);
							allPaths.put(onePath.getKey(), newPath);
							isInversePath.put(onePath.getKey(), false);
						}
					}
				}
				catch(NumberFormatException e1) { System.out.println("Not a number"); }
				clearLocations();
				highlightSelectedLocations();
				updatePaths2();
			}
		};
		panel.getActionMap().put("Scale all poses (locations and paths) by a factor",actScale);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_D,0),"Measure distance between selected points");
		AbstractAction actMeasure = new AbstractAction() {
			private static final long serialVersionUID = 6487332455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					Pose p1 = Missions.getLocationPose(locationIDs.get(selectedLocationsInt.get(0)));
					Pose p2 = Missions.getLocationPose(locationIDs.get(selectedLocationsInt.get(selectedLocationsInt.size()-1)));
					System.out.println(p1.distanceTo(p2));
				}
			}
		};
		panel.getActionMap().put("Measure distance between selected points",actMeasure);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_B,0),"Select all poses (and paths)");
		AbstractAction actSelectAll = new AbstractAction() {
			private static final long serialVersionUID = 6487432455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				selectedLocationsInt.clear();
				for (int i = 0; i < locationIDs.size(); i++) selectedLocationsInt.add(i);
				clearLocations();
				highlightSelectedLocations();
				updatePaths2();
			}
		};
		panel.getActionMap().put("Select all poses (and paths)",actSelectAll);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_DOWN,0),"Decrease Y of selected location(s)");
		AbstractAction actYMinus = new AbstractAction() {
			private static final long serialVersionUID = 6487878455015786029L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (!selectedLocationsInt.isEmpty()) {
					for (int selectedPathPointOneInt : selectedLocationsInt) {
						String locationName = locationIDs.get(selectedPathPointOneInt);
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						y -= deltaY;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						y -= (10*deltaY);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						th -= deltaT;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						th -= (10*deltaT);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						th += deltaT;
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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
						double x = Missions.getLocationPose(locationName).getX();
						double y = Missions.getLocationPose(locationName).getY();
						double th = Missions.getLocationPose(locationName).getTheta();
						th += (10*deltaT);
						Pose newPose = new Pose(x,y,th);
						Missions.setLocation(locationName, newPose);
						panel.addArrow("LOC:"+locationName, newPose, Color.green);
						highlightSelectedLocations();
					}

					/////
					Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
					//
					if (selectedLocationsInt.size() == 2) {
						String from = locationIDs.get(selectedLocationsInt.get(0));
						String to = locationIDs.get(selectedLocationsInt.get(1));
						String pathToRecompute = null;
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
								if (!isInversePath.get(entry.getKey())) {
									pathToRecompute = entry.getKey();
								}
							}
						}
						deleteForwardAndInversePaths(selectedLocationsInt);
						System.out.println("Affected path: " + pathToRecompute);
						computeForwardAndInversePathsWithSpline(selectedLocationsInt);
					}
					//
					else {
						for (int oneLocation : selectedLocationsInt) {
							String locationName = locationIDs.get(oneLocation);
							ArrayList<String> pathsToRecompute = new ArrayList<String>();
							for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
								if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
									if (!isInversePath.get(entry.getKey())) {
										pathsToRecompute.add(entry.getKey());
									}
								}
							}
							ArrayList<Integer> newSelection = new ArrayList<Integer>();
							newSelection.add(oneLocation);
							deleteForwardAndInversePaths(newSelection);
							for (String pathName : pathsToRecompute) {
								System.out.println("Affected path: " + pathName);
								String from = pathName.substring(0, pathName.indexOf("->"));
								String to = pathName.substring(pathName.indexOf("->")+2);
								int fromIndex = locationIDs.indexOf(from);
								int toIndex = locationIDs.indexOf(to);
								newSelection.clear();
								newSelection.add(fromIndex);
								newSelection.add(toIndex);
								computeForwardAndInversePathsWithSpline(newSelection);
							}
						}
					}
					/////
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

				PP_SP_distance_between_path_points += deltaD;
				System.out.println("Minimum distance between path points (>): " + PP_SP_distance_between_path_points);

				/////
				Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
				//
				if (selectedLocationsInt.size() == 2) {
					String from = locationIDs.get(selectedLocationsInt.get(0));
					String to = locationIDs.get(selectedLocationsInt.get(1));
					String pathToRecompute = null;
					for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
						if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
							if (!isInversePath.get(entry.getKey())) {
								pathToRecompute = entry.getKey();
							}
						}
					}
					deleteForwardAndInversePaths(selectedLocationsInt);
					System.out.println("Affected path: " + pathToRecompute);
					computeForwardAndInversePathsWithSpline(selectedLocationsInt);
				}
				//
				else {
					for (int oneLocation : selectedLocationsInt) {
						String locationName = locationIDs.get(oneLocation);
						ArrayList<String> pathsToRecompute = new ArrayList<String>();
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
								if (!isInversePath.get(entry.getKey())) {
									pathsToRecompute.add(entry.getKey());
								}
							}
						}
						ArrayList<Integer> newSelection = new ArrayList<Integer>();
						newSelection.add(oneLocation);
						deleteForwardAndInversePaths(newSelection);
						for (String pathName : pathsToRecompute) {
							System.out.println("Affected path: " + pathName);
							String from = pathName.substring(0, pathName.indexOf("->"));
							String to = pathName.substring(pathName.indexOf("->")+2);
							int fromIndex = locationIDs.indexOf(from);
							int toIndex = locationIDs.indexOf(to);
							newSelection.clear();
							newSelection.add(fromIndex);
							newSelection.add(toIndex);
							computeForwardAndInversePathsWithSpline(newSelection);
						}
					}
				}
				/////

			}
		};
		panel.getActionMap().put("Increase minimum distance between path points for path planning",actMDPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_M,0),"Decrease minimum distance between path points for path planning");
		AbstractAction actMDMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {

				if (PP_SP_distance_between_path_points-deltaD >= 0) PP_SP_distance_between_path_points -= deltaD;
				System.out.println("Minimum distance between path points (<): " + PP_SP_distance_between_path_points);

				/////
				Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
				//
				if (selectedLocationsInt.size() == 2) {
					String from = locationIDs.get(selectedLocationsInt.get(0));
					String to = locationIDs.get(selectedLocationsInt.get(1));
					String pathToRecompute = null;
					for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
						if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
							if (!isInversePath.get(entry.getKey())) {
								pathToRecompute = entry.getKey();
							}
						}
					}
					deleteForwardAndInversePaths(selectedLocationsInt);
					System.out.println("Affected path: " + pathToRecompute);
					computeForwardAndInversePathsWithSpline(selectedLocationsInt);
				}
				//
				else {
					for (int oneLocation : selectedLocationsInt) {
						String locationName = locationIDs.get(oneLocation);
						ArrayList<String> pathsToRecompute = new ArrayList<String>();
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
								if (!isInversePath.get(entry.getKey())) {
									pathsToRecompute.add(entry.getKey());
								}
							}
						}
						ArrayList<Integer> newSelection = new ArrayList<Integer>();
						newSelection.add(oneLocation);
						deleteForwardAndInversePaths(newSelection);
						for (String pathName : pathsToRecompute) {
							System.out.println("Affected path: " + pathName);
							String from = pathName.substring(0, pathName.indexOf("->"));
							String to = pathName.substring(pathName.indexOf("->")+2);
							int fromIndex = locationIDs.indexOf(from);
							int toIndex = locationIDs.indexOf(to);
							newSelection.clear();
							newSelection.add(fromIndex);
							newSelection.add(toIndex);
							computeForwardAndInversePathsWithSpline(newSelection);
						}
					}
				}
				/////

			}
		};
		panel.getActionMap().put("Decrease minimum distance between path points for path planning",actMDMinus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,KeyEvent.SHIFT_DOWN_MASK),"Increase maximum turning radius");
		AbstractAction actTRPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				PP_max_turning_radius += deltaTR;
				System.out.println("Turning radius (>): " + PP_max_turning_radius);
			}
		};
		panel.getActionMap().put("Increase maximum turning radius",actTRPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_LESS,0),"Decrease maximum turning radius");
		AbstractAction actTRMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {
				if (PP_max_turning_radius-deltaTR >= 0) PP_max_turning_radius -= deltaTR;
				System.out.println("Turning radius (<): " + PP_max_turning_radius);
			}
		};
		panel.getActionMap().put("Decrease maximum turning radius",actTRMinus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_PLUS,0),"Increase spline distance");
		AbstractAction actSDPlus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {

				SP_spline_distance += deltaSD;
				System.out.println("Spline distance (>): " + SP_spline_distance);

				/////
				Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
				//
				if (selectedLocationsInt.size() == 2) {
					String from = locationIDs.get(selectedLocationsInt.get(0));
					String to = locationIDs.get(selectedLocationsInt.get(1));
					String pathToRecompute = null;
					for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
						if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
							if (!isInversePath.get(entry.getKey())) {
								pathToRecompute = entry.getKey();
							}
						}
					}
					deleteForwardAndInversePaths(selectedLocationsInt);
					System.out.println("Affected path: " + pathToRecompute);
					computeForwardAndInversePathsWithSpline(selectedLocationsInt);
				}
				//
				else {
					for (int oneLocation : selectedLocationsInt) {
						String locationName = locationIDs.get(oneLocation);
						ArrayList<String> pathsToRecompute = new ArrayList<String>();
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
								if (!isInversePath.get(entry.getKey())) {
									pathsToRecompute.add(entry.getKey());
								}
							}
						}
						ArrayList<Integer> newSelection = new ArrayList<Integer>();
						newSelection.add(oneLocation);
						deleteForwardAndInversePaths(newSelection);
						for (String pathName : pathsToRecompute) {
							System.out.println("Affected path: " + pathName);
							String from = pathName.substring(0, pathName.indexOf("->"));
							String to = pathName.substring(pathName.indexOf("->")+2);
							int fromIndex = locationIDs.indexOf(from);
							int toIndex = locationIDs.indexOf(to);
							newSelection.clear();
							newSelection.add(fromIndex);
							newSelection.add(toIndex);
							computeForwardAndInversePathsWithSpline(newSelection);
						}
					}
				}
				/////

			}
		};
		panel.getActionMap().put("Increase spline distance",actSDPlus);

		panel.getInputMap().put(KeyStroke.getKeyStroke(KeyEvent.VK_MINUS,0),"Decrease spline distance");
		AbstractAction actSDMinus = new AbstractAction() {
			private static final long serialVersionUID = 8414380724212398117L;
			@Override
			public void actionPerformed(ActionEvent e) {

				SP_spline_distance -= deltaSD;
				System.out.println("Spline distance (<): " + SP_spline_distance);

				/////
				Set<Entry<String,ArrayList<PoseSteering>>> allEntries = allPaths.entrySet();
				//
				if (selectedLocationsInt.size() == 2) {
					String from = locationIDs.get(selectedLocationsInt.get(0));
					String to = locationIDs.get(selectedLocationsInt.get(1));
					String pathToRecompute = null;
					for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
						if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(from) && entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(to)) {
							if (!isInversePath.get(entry.getKey())) {
								pathToRecompute = entry.getKey();
							}
						}
					}
					deleteForwardAndInversePaths(selectedLocationsInt);
					System.out.println("Affected path: " + pathToRecompute);
					computeForwardAndInversePathsWithSpline(selectedLocationsInt);
				}
				//
				else {
					for (int oneLocation : selectedLocationsInt) {
						String locationName = locationIDs.get(oneLocation);
						ArrayList<String> pathsToRecompute = new ArrayList<String>();
						for (Entry<String, ArrayList<PoseSteering>> entry : allEntries) {
							if (entry.getKey().substring(0, entry.getKey().indexOf("->")).equals(locationName) || entry.getKey().substring(entry.getKey().indexOf("->")+2).equals(locationName)) {
								if (!isInversePath.get(entry.getKey())) {
									pathsToRecompute.add(entry.getKey());
								}
							}
						}
						ArrayList<Integer> newSelection = new ArrayList<Integer>();
						newSelection.add(oneLocation);
						deleteForwardAndInversePaths(newSelection);
						for (String pathName : pathsToRecompute) {
							System.out.println("Affected path: " + pathName);
							String from = pathName.substring(0, pathName.indexOf("->"));
							String to = pathName.substring(pathName.indexOf("->")+2);
							int fromIndex = locationIDs.indexOf(from);
							int toIndex = locationIDs.indexOf(to);
							newSelection.clear();
							newSelection.add(fromIndex);
							newSelection.add(toIndex);
							computeForwardAndInversePathsWithSpline(newSelection);
						}
					}
				}
				/////			

			}
		};
		panel.getActionMap().put("Decrease spline distance",actSDMinus);

		panel.setFocusable(true);
		panel.setArrowHeadSizeInMeters(1.4);
		panel.setTextSizeInMeters(20.3);
	}

	private PoseSteering[] computeSpline(Pose from, Pose to) {
		Coordinate[] controlPoints = new Coordinate[4];
		controlPoints[0] = new Coordinate(from.getX(),from.getY(),0.0);
		controlPoints[1] = new Coordinate(from.getX()+SP_spline_distance*Math.cos(from.getTheta()), from.getY()+SP_spline_distance*Math.sin(from.getTheta()), 0.0);
		controlPoints[2] = new Coordinate(to.getX()-SP_spline_distance*Math.cos(to.getTheta()), to.getY()-SP_spline_distance*Math.sin(to.getTheta()), 0.0);
		controlPoints[3] = new Coordinate(to.getX(),to.getY(),0.0);
		Coordinate[] spline = BezierSplineFactory.createBezierSpline(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3], PP_SP_distance_between_path_points);
		return BezierSplineFactory.asPoseSteering(spline);
		//Spline3D spline1 = SplineFactory.createBezier(controlPoints, DISTANCE_BETWEEN_PATH_POINTS);
		//return spline1.asPoseSteerings();
	}

	private PoseSteering[] computePath(Pose from, Pose ... to) {
		if (this.mp == null) this.mp = new ReedsSheppCarPlanner(PLANNING_ALGORITHM.PRMstar);
		mp.setPlanningTimeInSecs(5);
		mp.setFootprint(PP_footprint);
		mp.setTurningRadius(PP_max_turning_radius);
		mp.setDistanceBetweenPathPoints(PP_SP_distance_between_path_points);
		mp.setRadius(PP_radius);
		mp.setMap(mapFilename);
		mp.setStart(from);
		mp.setGoals(to);
		if (!mp.plan()) return null;
		PoseSteering[] ret = mp.getPath();
		return ret;
	}

	//	@Override
	//	public void mouseDragged(MouseEvent e) { }
	//
	//	@Override
	//	public void mouseMoved(MouseEvent e) {
	//		AffineTransform geomToScreen = panel.getMapTransform();
	//		try {
	//			AffineTransform geomToScreenInv = geomToScreen.createInverse();
	//			this.mousePositionInMap = geomToScreenInv.transform(new Point2D.Double(e.getX(),e.getY()), null);
	//			//System.out.println(mousePosition);
	//		} catch (NoninvertibleTransformException e1) {
	//			// TODO Auto-generated catch block
	//			e1.printStackTrace();
	//		}
	//	}

	public static void main(String[] args) {
		//Robot Footprint
		Coordinate[] footprint = new Coordinate[] {
				new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5),
		};

		PathEditor2 pe = new PathEditor2(footprint);
		pe.setZoomIntensity(0.1);
		pe.setPanAcceleration(10);
		pe.setSplineDistance(7);
	}

}
