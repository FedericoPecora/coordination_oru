package se.oru.coordination.coordination_oru.motionplanning.tests.ompl;


import org.metacsp.utility.UI.JTSDrawingPanel;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

public class TestReedsSheppCarPlannerSmoothGeometry {
	
	public static void main(String[] args) {
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		Coordinate footprint1 = new Coordinate(-2.0,0.5);
		Coordinate footprint2 = new Coordinate(2.0,0.5);
		Coordinate footprint3 = new Coordinate(2.0,-0.5);
		Coordinate footprint4 = new Coordinate(-2.0,-0.5);
		rsp.setRadius(0.5);
		rsp.setFootprint(footprint1,footprint2,footprint3,footprint4);
		JTSDrawingPanel panel = JTSDrawingPanel.makeEmpty("debug");
		GeometryFactory gf = new GeometryFactory();
		Polygon footprint = gf.createPolygon(new Coordinate[] {footprint1,footprint2,footprint3,footprint4,footprint1});
		Polygon smoothedFootprint = gf.createPolygon(rsp.getCollisionCircleCenters());
		System.out.println("Smoothing went from " + footprint.getCoordinates().length + " to " + smoothedFootprint.getCoordinates().length + " points");
		panel.addGeometry("orig", footprint, true, true, true, "#00FF00");
		panel.addGeometry("smooth", smoothedFootprint, false, false, true, "#FF0000");
		for (int i = 0; i < smoothedFootprint.getCoordinates().length; i++) {
			double delta = 0.1;
			Coordinate p1 = new Coordinate(smoothedFootprint.getCoordinates()[i].x,smoothedFootprint.getCoordinates()[i].y);
			Coordinate p2 = new Coordinate(smoothedFootprint.getCoordinates()[i].x+delta,smoothedFootprint.getCoordinates()[i].y+delta);
			Coordinate p3 = new Coordinate(smoothedFootprint.getCoordinates()[i].x-delta,smoothedFootprint.getCoordinates()[i].y+delta);
			panel.addGeometry("_cp"+i, gf.createPolygon(new Coordinate[] {p1,p2,p3,p1}), false, false, false, "#000000");
		}
		

	}

}
