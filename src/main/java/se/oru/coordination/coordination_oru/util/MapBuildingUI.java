package se.oru.coordination.coordination_oru.util;
import java.util.ArrayList;
import java.util.HashMap;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.util.PathEditor2;

public class MapBuildingUI {

	public static void main(String[] args) {

		//Robot Footprint
		Coordinate[] robotFootprint = new Coordinate[] {
				new Coordinate(-1,0.5),
				new Coordinate(-1,-0.5),
				new Coordinate(1,-0.5),
				new Coordinate(1,0.5),
				new Coordinate(-1,0.5),
		};
		
		//String locationsFile = "ACT-scenario/ACT-locations_and_paths.txt";
		//String selectionsFile = "ACT-scenario/ACT-selections.txt";
		//String yamlFile = "ACT-scenario/ACT-map.yaml";
		
		String locationsFile = "output-ACT-planner/locations_and_paths.txt";
		String selectionsFile = "output-ACT-planner/ACT-selections.txt";
		String yamlFile = "output-ACT-planner/ACT-map.yaml";

		
		//PathEditor2 pe = new PathEditor2(locationsFile,yamlFile);
		PathEditor2 pe = new PathEditor2(locationsFile, yamlFile, selectionsFile);
		
		//This is the shape of the robot
		pe.setPathPlanningFootprint(robotFootprint);

		//For spline-based method
		//pe.setSplineDistance(7);
		
		pe.setPathPlanningRadius(0.1);

		//UI
		pe.setZoomIntensity(0.1);
		pe.setPanAcceleration(1);

	}
	
	
}
