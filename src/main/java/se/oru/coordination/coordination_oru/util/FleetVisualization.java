package se.oru.coordination.coordination_oru.util;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.RobotReport;

public interface FleetVisualization {
	
	@Deprecated
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String ... extraStatusInfo);
	
	public void displayRobotState(Polygon fp, RobotReport rr, String ... extraStatusInfo);
	
	public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor);
	
	public void addEnvelope(TrajectoryEnvelope te);
	
	public void removeEnvelope(TrajectoryEnvelope te);
	
	public void updateVisualization();
	
	public void setMap(String yamlFile);
	
	public int periodicEnvelopeRefreshInMillis();

}
