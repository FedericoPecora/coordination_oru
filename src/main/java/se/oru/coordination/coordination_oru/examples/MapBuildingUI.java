package se.oru.coordination.coordination_oru.examples;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.util.PathEditor2;

public class MapBuildingUI {

	public static void main(String[] args) {

		//ST-18 Footprint
		Coordinate[] footprint = new Coordinate[] {
				new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5),
		};

		String locAndPathFilename = "locations_and_paths.txt";
		//String selectionsFile = "paths/selections.txt";
		String mapFilename = "maps/map-empty.yaml";
		PathEditor2 pe = new PathEditor2(locAndPathFilename,mapFilename,footprint);
		pe.setZoomIntensity(0.1);
		pe.setPanAcceleration(10);
		pe.setSplineDistance(7);

	}
}

