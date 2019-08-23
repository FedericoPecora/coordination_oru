package se.oru.coordination.coordination_oru.util;
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
		
		String locationsFile = "per-ACTOR/locations_and_paths.txt";
		String selectionsFile = "per-ACTOR/selections.txt";
		String yamlFile = "maps/map-partial-2.yaml";
		
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
