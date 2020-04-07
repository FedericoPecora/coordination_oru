package se.oru.coordination.coordination_oru.motionplanning.ompl;

import java.util.Arrays;
import java.util.List;

import com.sun.jna.Library;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;

public interface ReedsSheppCarPlannerLib extends Library {
	
	public boolean plan(String mapFilename, double mapResolution, double robotRadius, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs);

	public boolean plan_multiple_circles(String mapFilename, double mapResolution, double robotRadius, double[] xCoords, double[] yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs);
	
	public boolean plan_multiple_circles_nomap(double[] xCoords, double[] yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PointerByReference path, IntByReference pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs);

	public void cleanupPath(Pointer p);
	
	public static class PathPose extends Structure {
		public static class ByReference extends PathPose implements Structure.ByReference {}

		public double x;
		public double y;
		public double theta;
		
		public PathPose() {}
		public PathPose(Pointer p) {
			super(p);
		}
		@Override
		protected List<String> getFieldOrder() {
			return Arrays.asList(new String[] {"x", "y", "theta"});
		}
	}
}
