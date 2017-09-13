package se.oru.coordination.coordination_oru.util;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Scanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.UI.JTSDrawingPanel;

public class PathEditor {

	private String fileName = null;
	private boolean selectionInputListen = false;
	private String selectedPathPoint = "";
	private int selectedPathPointInt = -1;
	private PoseSteering[] path = null;
	private JTSDrawingPanel panel = null;
	private double deltaX = 0.1;
	private double deltaY = 0.1;
	private double deltaT = 0.1;
	private String newFileSuffix = ".new";

	public PathEditor(String fileName, double deltaX, double deltaY, double deltaTheta, String newFileSuffix) {
		this.fileName = fileName;
		this.deltaX = deltaX;
		this.deltaY = deltaY;
		this.deltaT = deltaTheta;
		this.newFileSuffix = newFileSuffix;
		this.readPath(fileName);
		this.setupGUI();
	}
	
	public PathEditor(String fileName) {
		this(fileName,0.1,0.1,0.1,".new");
	}
	
	private void setupGUI() {
		panel = JTSDrawingPanel.makeEmpty("Path Editor");
		panel.addKeyListener(new KeyListener() {
			@Override
			public void keyTyped(KeyEvent e) {
				if (e.getKeyChar()=='r' || e.getKeyChar()=='R') {
					String newFileName = fileName+".new";
					writePath(newFileName);
					System.out.println("Saved " + newFileName);
				}
				if (e.getKeyChar()=='s' || e.getKeyChar()=='S') {
					if (!selectionInputListen) {
						selectedPathPoint = "";
						selectedPathPointInt = -1;
						for (int i = 0; i < path.length; i++) panel.addArrow(""+i, path[i].getPose(), Color.gray);
						panel.updatePanel();
						System.out.println("Input selection: (press 's' again to set): " + selectedPathPoint);
					}
					else if (selectionInputListen) {
						for (int i = 0; i < path.length; i++) panel.addArrow(""+i, path[i].getPose(), Color.gray);
						try {
							selectedPathPointInt = Integer.parseInt(selectedPathPoint);
							if (selectedPathPointInt >= 0 && selectedPathPointInt < path.length) {
								panel.addArrow(""+selectedPathPoint, path[selectedPathPointInt].getPose(), Color.red);
								panel.updatePanel();
							}
							else {
								selectedPathPoint = "";
								selectedPathPointInt = -1;								
							}
						}
						catch(NumberFormatException ex) { }
					}
					selectionInputListen = !selectionInputListen;
				}
				else if (selectionInputListen) {
					selectedPathPoint += e.getKeyChar();
					System.out.println("Input selection: (press 's' again to set): " + selectedPathPoint);
				}
				else if (!selectionInputListen && selectedPathPointInt != -1) {
					double x = path[selectedPathPointInt].getPose().getX();
					double y = path[selectedPathPointInt].getPose().getY();
					double th = path[selectedPathPointInt].getPose().getTheta();
					if (e.getKeyChar()=='x') x -= deltaX;
					else if (e.getKeyChar()=='X') x += deltaX;
					else if (e.getKeyChar()=='y') y -= deltaY;
					else if (e.getKeyChar()=='Y') y += deltaY;
					else if (e.getKeyChar()=='t') th -= deltaT;
					else if (e.getKeyChar()=='T') th += deltaT;
					PoseSteering newPoseSteering = new PoseSteering(x, y, th, path[selectedPathPointInt].getSteering());
					path[selectedPathPointInt] = newPoseSteering;
					panel.addArrow(""+selectedPathPoint, path[selectedPathPointInt].getPose(), Color.red);
					panel.updatePanel();
				}
			}
			@Override
			public void keyReleased(KeyEvent e) { }
			@Override
			public void keyPressed(KeyEvent e) { }
		});
		
		panel.setFocusable(true);
		path = readPath(fileName);
		panel.setArrowHeadSizeInMeters(9.0);
		panel.setTextSizeInMeters(0.3);
		for (int i = 0; i < path.length; i++) {
			Pose pose = path[i].getPose();
			panel.addArrow(""+i, pose, Color.gray);
		}
		panel.updatePanel();
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
	
	private PoseSteering[] readPath(String fileName) {
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
		return ret.toArray(new PoseSteering[ret.size()]);
	}

	public static void main(String[] args) {
		final String fileName = "paths/path2.path";
		new PathEditor(fileName);
	}

}
