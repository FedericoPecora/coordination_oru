package se.oru.coordination.coordination_oru.util;

import java.awt.color.ColorSpace;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.awt.image.ColorConvertOp;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.ros.RosCore;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.Duration;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import com.vividsolutions.jts.geom.Coordinate;

import geometry_msgs.Transform;
import nav_msgs.OccupancyGrid;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.util.FleetVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import com.vividsolutions.jts.geom.Polygon;
//import org.ros.visualization_msgs.MarkerArray;
import visualization_msgs.MarkerArray;

public class RVizVisualization implements FleetVisualization, NodeMain {

	//private final String mapFrameID = "/map_laser2d";
	private String mapFrameID = "/map";
	private ConnectedNode node = null;
	private HashMap<String,Publisher<visualization_msgs.MarkerArray>> boxMarkerPublishers = null;
	private HashMap<String,ArrayList<visualization_msgs.Marker>> boxMarkerMarkers = null;

	private HashMap<Integer,Publisher<visualization_msgs.MarkerArray>> robotStatusPublishers = null;
	private HashMap<Integer,Publisher<visualization_msgs.MarkerArray>> dependencyPublishers = null;
	private HashMap<Integer,ArrayList<visualization_msgs.Marker>> robotStatusMarkers = null;
	private HashMap<Integer,ArrayList<visualization_msgs.Marker>> dependencyMarkers = null;
	private HashMap<Integer,visualization_msgs.Marker> envelopeMarkers = null;
	private boolean ready = false;
	private String mapFileName = null;
	private boolean darkColors = true;
	
	private static String rvizEntry = ""+
			"    - Class: rviz/MarkerArray\n" + 
			"      Enabled: true\n" + 
			"      Marker Topic: /ROBOTi/deps\n" + 
			"      Name: MarkerArray\n" + 
			"      Namespaces:\n" + 
			"        {}\n" + 
			"      Queue Size: 100\n" + 
			"      Value: true\n" + 
			"    - Class: rviz/MarkerArray\n" + 
			"      Enabled: true\n" + 
			"      Marker Topic: /ROBOTi/status\n" + 
			"      Name: MarkerArray\n" + 
			"      Namespaces:\n" + 
			"        {}\n" + 
			"      Queue Size: 100\n" + 
			"      Value: true\n";

	public static void writeRVizConfigFile(int ... robotIDs) {
		try {
			File file = new File(System.getProperty("user.home")+File.separator+"config.rviz");
			
			ClassLoader loader = Thread.currentThread().getContextClassLoader();
			
			//Read pre
			InputStream is = loader.getResourceAsStream("coordinator_default_config_pre.rviz");
			BufferedReader br = new BufferedReader(new InputStreamReader(is));
			String output = "";
			String oneLine = null;
			while ((oneLine = br.readLine()) != null) {
				output += (oneLine+"\n");  
			}
			br.close();
			is.close();
			
			//Make robot specific entries
			for (int robotID : robotIDs) {
            	output += rvizEntry.replaceAll("ROBOTi", "robot"+robotID);
            }
			
			//Read post
			is = loader.getResourceAsStream("coordinator_default_config_post.rviz");
			br = new BufferedReader(new InputStreamReader(is));
			oneLine = null;
			while ((oneLine = br.readLine()) != null) {
				output += (oneLine+"\n");  
			}
			br.close();
			is.close();
			
			//Dump it out
            PrintWriter writer = new PrintWriter(file);
            writer.write(output);
            writer.close();
		}
		catch (IOException e) { e.printStackTrace(); }

	}
	
	public RVizVisualization() {
		this("/map");
	}
	
	public RVizVisualization(String mapFrameID) {
		this(true,mapFrameID);
	}

	public RVizVisualization(ConnectedNode node) {
		this(node,"/map");
	}
	
	public RVizVisualization(ConnectedNode node, String mapFrameID) {
		this(false, mapFrameID);
		this.node = node;
		this.ready = true;
	}

	public RVizVisualization(boolean startROSCore) {
		this(startROSCore,"/map");
	}
	
	public RVizVisualization(boolean startROSCore, String mapFrameID) {
		this.mapFrameID = mapFrameID;
		this.robotStatusPublishers = new HashMap<Integer,Publisher<visualization_msgs.MarkerArray>>();
		this.dependencyPublishers = new HashMap<Integer,Publisher<visualization_msgs.MarkerArray>>();
		this.robotStatusMarkers = new HashMap<Integer,ArrayList<visualization_msgs.Marker>>();
		this.dependencyMarkers = new HashMap<Integer,ArrayList<visualization_msgs.Marker>>();

		this.boxMarkerPublishers = new HashMap<String,Publisher<visualization_msgs.MarkerArray>>();
		this.boxMarkerMarkers = new HashMap<String,ArrayList<visualization_msgs.Marker>>();
		
		if (startROSCore) {
			NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic("localhost");
			NodeMainExecutor executor = DefaultNodeMainExecutor.newDefault();
			executor.execute(this, nodeConfiguration);
	
			RosCore mRosCore = RosCore.newPublic("localhost", 11311);
			mRosCore.start();
			try {
				mRosCore.awaitStart(5, TimeUnit.SECONDS);
			}
			catch (InterruptedException e) { e.printStackTrace(); }
			System.out.println("ROS-core started");
		}
	}
	
	public void setDarkColors(boolean dark) {
		this.darkColors = dark;
	}

	private BufferedImage toGrayScale(BufferedImage imgIn) {
		BufferedImage img = new BufferedImage(imgIn.getWidth(), imgIn.getHeight(), BufferedImage.TYPE_BYTE_GRAY);
        ColorConvertOp op = new ColorConvertOp(ColorSpace.getInstance(ColorSpace.CS_GRAY), null);
        op.filter(imgIn, img);
        return img;
	}
	
	private BufferedImage toBlackAndWhite(BufferedImage image, int threshold) {
	    BufferedImage result = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_BYTE_GRAY);
	    result.getGraphics().drawImage(image, 0, 0, null);
	    WritableRaster raster = result.getRaster();
	    int[] pixels = new int[image.getWidth()];
	    for (int y = 0; y < image.getHeight(); y++) {
	        raster.getPixels(0, y, image.getWidth(), 1, pixels);
	        for (int i = 0; i < pixels.length; i++) {
	            if (pixels[i] < threshold) pixels[i] = 0;
	            else pixels[i] = 255;
	        }
	        raster.setPixels(0, y, image.getWidth(), 1, pixels);
	    }
	    return result;
	}
	
	private BufferedImage flipVertically(BufferedImage imgIn) {
        AffineTransform tx = AffineTransform.getScaleInstance(1, -1);
        tx.translate(0, -imgIn.getHeight(null));
        AffineTransformOp op1 = new AffineTransformOp(tx, AffineTransformOp.TYPE_NEAREST_NEIGHBOR);
        imgIn = op1.filter(imgIn, null);
        return imgIn;
	}

	private BufferedImage flipHorizontally(BufferedImage imgIn) {
		AffineTransform tx = AffineTransform.getScaleInstance(-1, 1);
		tx.translate(-imgIn.getWidth(null), 0);
		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_NEAREST_NEIGHBOR);
		imgIn = op.filter(imgIn, null);        
        return imgIn;
	}

	@Override
	public void setMap(String yamlFile) {
		String prefix = yamlFile.substring(0, yamlFile.indexOf(File.separator));
		this.setMapFileName(yamlFile, prefix, false, true);
	}
	
	public void setMapFileName(String mapYAMLFile, String prefix, boolean flipHorizontally, boolean flipVertically) {
		this.mapFileName = Missions.getProperty("image", mapYAMLFile);
		if (prefix != null) this.mapFileName = prefix + File.separator + this.mapFileName;
		while (!ready) {
			try { Thread.sleep(100); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		if (mapFileName != null) {
			try {
				final OccupancyGrid occMap = node.getTopicMessageFactory().newFromType(OccupancyGrid._TYPE);
				BufferedImage img = ImageIO.read(new File(mapFileName));
				//img = toGrayScale(img);
				img = toBlackAndWhite(img, 128);
				if (flipHorizontally) img = flipHorizontally(img);
				if (flipVertically) img = flipVertically(img);
				System.out.println("Loaded map: " + img.getHeight() + "x" + img.getWidth());
				WritableRaster raster = img.getRaster();
				DataBufferByte data = (DataBufferByte)raster.getDataBuffer();
				ChannelBuffer buffer = ChannelBuffers.copiedBuffer(ByteOrder.nativeOrder(), data.getData());
				occMap.setData(buffer);
				occMap.getHeader().setFrameId(mapFrameID);
				occMap.getInfo().setHeight((int)(img.getHeight()));
				occMap.getInfo().setWidth((int)(img.getWidth()));
				geometry_msgs.Pose pose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
				pose.getPosition().setX(0);
				pose.getPosition().setY(0);
				occMap.getInfo().setOrigin(pose);
				double res = Double.parseDouble(Missions.getProperty("resolution", mapYAMLFile));
				occMap.getInfo().setResolution((float)res);
				final Publisher<OccupancyGrid> publisher = node.newPublisher("/map", OccupancyGrid._TYPE);
				node.executeCancellableLoop(new CancellableLoop() {
					@Override
					protected void loop() throws InterruptedException {
						publisher.publish(occMap);
						Thread.sleep(1000);
					}
				});

			}
			catch (IOException e) { e.printStackTrace(); }	
		}

	}


	@Override
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String ... extraStatusInfo) {
		if (ready) {
			double x = rr.getPose().getX();
			double y = rr.getPose().getY();
			double theta = rr.getPose().getTheta();

			visualization_msgs.Marker marker = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			marker.getHeader().setFrameId(mapFrameID);
			marker.getScale().setX(0.2f);
			marker.getColor().setR(100f);
			marker.getColor().setG(0.0f);
			marker.getColor().setB(0.0f);
			marker.getColor().setA(0.8f);
			marker.setAction(visualization_msgs.Marker.ADD);                                
			marker.setNs("current_pose");
			marker.setType(visualization_msgs.Marker.LINE_STRIP);
			marker.setId(rr.getRobotID());
			marker.setLifetime(new Duration(10.0));

			ArrayList<geometry_msgs.Point> points = new ArrayList<geometry_msgs.Point>();
			Coordinate[] coords = TrajectoryEnvelope.getFootprint(te.getFootprint(), x, y, theta).getCoordinates();
			for (Coordinate coord : coords) {
				geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
				point.setX(coord.x);
				point.setY(coord.y);
				point.setZ(0.0);
				points.add(point);
			}
			points.add(points.get(0));
			marker.setPoints(points);
			if (!this.robotStatusPublishers.containsKey(rr.getRobotID())) {
				Publisher<visualization_msgs.MarkerArray> markerArrayPublisher = node.newPublisher("robot"+rr.getRobotID()+"/status", visualization_msgs.MarkerArray._TYPE);
				this.robotStatusPublishers.put(rr.getRobotID(), markerArrayPublisher);
				synchronized(robotStatusMarkers) {
					this.robotStatusMarkers.put(rr.getRobotID(), new ArrayList<visualization_msgs.Marker>());
				}
			}
			synchronized(robotStatusMarkers) {
				this.robotStatusMarkers.get(rr.getRobotID()).add(marker);
			}

			//////////////
			visualization_msgs.Marker markerName = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			markerName.getHeader().setFrameId(mapFrameID);
			markerName.getScale().setX(1.0f);
			markerName.getScale().setY(1.0f);
			markerName.getScale().setZ(1.0f);
			float R = 0.0f;
			float G = 0.0f;
			float B = 0.0f;
			float A = 1.0f;
			if (darkColors) {
				R = 1.0f;
				G = 1.0f;
				B = 1.0f;
				A = 0.8f;
			}
			markerName.getColor().setR(R);
            markerName.getColor().setG(G);
            markerName.getColor().setB(B);
            markerName.getColor().setA(A);
			markerName.setAction(visualization_msgs.Marker.ADD);                                
			markerName.setNs("robot_state");
			markerName.setType(visualization_msgs.Marker.TEXT_VIEW_FACING);
			//markerName.setId(te.getRobotID());
			markerName.setLifetime(new Duration(10.0));
			String markerText  = "R" + te.getRobotID() + ": " + rr.getPathIndex();
			if (extraStatusInfo != null) for (String extra : extraStatusInfo) markerText += "\n" + extra;
			markerName.setText(markerText);
			geometry_msgs.Pose pose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
			geometry_msgs.Point pos = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			pos.setX(x);
			pos.setY(y);
			pos.setZ(0);
			pose.setPosition(pos);
			markerName.setPose(pose);
			synchronized(robotStatusMarkers) {
				this.robotStatusMarkers.get(rr.getRobotID()).add(markerName);
			}

		}
	}
	
	@Override
	public void displayRobotState(Polygon fp, RobotReport rr, String ... extraStatusInfo) {
		if (ready) {
			double x = rr.getPose().getX();
			double y = rr.getPose().getY();
			double theta = rr.getPose().getTheta();

			visualization_msgs.Marker marker = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			marker.getHeader().setFrameId(mapFrameID);
			marker.getScale().setX(0.2f);
			marker.getColor().setR(100.0f);
			marker.getColor().setG(0.0f);
			marker.getColor().setB(0.0f);
			marker.getColor().setA(0.8f);
			marker.setAction(visualization_msgs.Marker.ADD);                                
			marker.setNs("current_pose");
			marker.setType(visualization_msgs.Marker.LINE_STRIP);
			marker.setId(rr.getRobotID());
			marker.setLifetime(new Duration(10.0));

			ArrayList<geometry_msgs.Point> points = new ArrayList<geometry_msgs.Point>();
			Coordinate[] coords = TrajectoryEnvelope.getFootprint(fp, x, y, theta).getCoordinates();
			for (Coordinate coord : coords) {
				geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
				point.setX(coord.x);
				point.setY(coord.y);
				point.setZ(0.0);
				points.add(point);
			}
			points.add(points.get(0));
			marker.setPoints(points);
			if (!this.robotStatusPublishers.containsKey(rr.getRobotID())) {
				Publisher<visualization_msgs.MarkerArray> markerArrayPublisher = node.newPublisher("robot"+rr.getRobotID()+"/status", visualization_msgs.MarkerArray._TYPE);
				this.robotStatusPublishers.put(rr.getRobotID(), markerArrayPublisher);
				synchronized(robotStatusMarkers) {
					this.robotStatusMarkers.put(rr.getRobotID(), new ArrayList<visualization_msgs.Marker>());
				}
			}
			synchronized(robotStatusMarkers) {
				this.robotStatusMarkers.get(rr.getRobotID()).add(marker);
			}

			//////////////
			visualization_msgs.Marker markerName = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			markerName.getHeader().setFrameId(mapFrameID);
			markerName.getScale().setX(1.0f);
			markerName.getScale().setY(1.0f);
			markerName.getScale().setZ(1.0f);
			float R = 0.0f;
			float G = 0.0f;
			float B = 0.0f;
			float A = 1.0f;
			if (darkColors) {
				R = 1.0f;
				G = 1.0f;
				B = 1.0f;
				A = 0.8f;
			}
			markerName.getColor().setR(R);
            markerName.getColor().setG(G);
            markerName.getColor().setB(B);
            markerName.getColor().setA(A);
			markerName.setAction(visualization_msgs.Marker.ADD);                                
			markerName.setNs("robot_state");
			markerName.setType(visualization_msgs.Marker.TEXT_VIEW_FACING);
			//markerName.setId(te.getRobotID());
			markerName.setLifetime(new Duration(10.0));
			String markerText  = "R" + rr.getRobotID() + ": " + rr.getPathIndex();
			if (extraStatusInfo != null) for (String extra : extraStatusInfo) markerText += "\n" + extra;
			markerName.setText(markerText);
			geometry_msgs.Pose pose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
			geometry_msgs.Point pos = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			pos.setX(x);
			pos.setY(y);
			pos.setZ(0);
			pose.setPosition(pos);
			markerName.setPose(pose);
			synchronized(robotStatusMarkers) {
				this.robotStatusMarkers.get(rr.getRobotID()).add(markerName);
			}

		}
	}
	
	public void displayBox(String markerLabel, Coordinate[] shape, int markerID, double x, double y, double durationInSeconds) {
		if (ready) {
			visualization_msgs.Marker marker = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			marker.getHeader().setFrameId(mapFrameID);
			marker.getScale().setX(0.2f);
			marker.getColor().setR(0.0f);
			marker.getColor().setG(100.0f);
			marker.getColor().setB(0.0f);
			marker.getColor().setA(0.8f);
			marker.setAction(visualization_msgs.Marker.ADD);                                
			marker.setNs("box_marker");
			marker.setType(visualization_msgs.Marker.LINE_STRIP);
			marker.setId(markerID);
			marker.setLifetime(new Duration(durationInSeconds));

			ArrayList<geometry_msgs.Point> points = new ArrayList<geometry_msgs.Point>();
			for (Coordinate coord : shape) {
				geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
				point.setX(coord.x);
				point.setY(coord.y);
				point.setZ(0.0);
				points.add(point);
			}
			points.add(points.get(0));
			marker.setPoints(points);
			if (!this.boxMarkerPublishers.containsKey(markerLabel)) {
				Publisher<visualization_msgs.MarkerArray> markerArrayPublisher = node.newPublisher(markerLabel, visualization_msgs.MarkerArray._TYPE);
				this.boxMarkerPublishers.put(markerLabel, markerArrayPublisher);
				synchronized(boxMarkerMarkers) {
					this.boxMarkerMarkers.put(markerLabel, new ArrayList<visualization_msgs.Marker>());
				}
			}
			synchronized(boxMarkerMarkers) {
				this.boxMarkerMarkers.get(markerLabel).add(marker);
			}

			//////////////
			visualization_msgs.Marker markerName = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			markerName.getHeader().setFrameId(mapFrameID);
			markerName.getScale().setX(1.0f);
			markerName.getScale().setY(1.0f);
			markerName.getScale().setZ(1.0f);
			markerName.getColor().setR(1.0f);
			markerName.getColor().setG(1.0f);
			markerName.getColor().setB(1.0f);
			markerName.getColor().setA(0.8f);
			markerName.setAction(visualization_msgs.Marker.ADD);                                
			markerName.setNs("label");
			markerName.setType(visualization_msgs.Marker.TEXT_VIEW_FACING);
			//markerName.setId(te.getRobotID());
			markerName.setLifetime(new Duration(durationInSeconds));
			markerName.setText(markerLabel);
			geometry_msgs.Pose pose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
			geometry_msgs.Point pos = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			pos.setX(x);
			pos.setY(y);
			pos.setZ(0);
			pose.setPosition(pos);
			markerName.setPose(pose);
			synchronized(boxMarkerMarkers) {
				this.boxMarkerMarkers.get(markerLabel).add(markerName);
			}
		}
	}


	@Override
	public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor) {
		if(ready) {
			Pose from = rrWaiting.getPose();
			Pose to = rrDriving.getPose();
			visualization_msgs.Marker mArrow = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			mArrow.setAction(visualization_msgs.Marker.ADD);
			mArrow.setNs(dependencyDescriptor);
			mArrow.setType(visualization_msgs.Marker.ARROW);
			ArrayList<geometry_msgs.Point> points = new ArrayList<geometry_msgs.Point>();		
			geometry_msgs.Point pointFrom = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			geometry_msgs.Point pointTo = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			pointFrom.setX(from.getX());
			pointFrom.setY(from.getY());
			pointFrom.setZ(0.0);
			points.add(pointFrom);
			pointTo.setX(to.getX());
			pointTo.setY(to.getY());
			pointTo.setZ(0.0);
			points.add(pointTo);
			mArrow.setPoints(points);
			mArrow.setId(dependencyDescriptor.hashCode());
			mArrow.getHeader().setFrameId(mapFrameID);
			mArrow.getScale().setX(0.4);
			mArrow.getScale().setY(1.0);
			mArrow.getScale().setZ(1.2);
			float R = 0.0f;
			float G = 0.0f;
			float B = 0.0f;
			float A = 0.3f;
			if (darkColors) {
				R = 15.0f;
				G = 100.0f;
				B = 200.0f;
				A = 0.2f;
			}
			mArrow.getColor().setR(R);
			mArrow.getColor().setG(G);
			mArrow.getColor().setB(B);
			mArrow.getColor().setA(A);
			mArrow.setLifetime(new Duration(1.0));
			if (!this.dependencyPublishers.containsKey(rrWaiting.getRobotID())) {
				Publisher<visualization_msgs.MarkerArray> markerArrayPublisher = node.newPublisher("robot"+rrWaiting.getRobotID()+"/deps", visualization_msgs.MarkerArray._TYPE);
				this.dependencyPublishers.put(rrWaiting.getRobotID(), markerArrayPublisher);
				synchronized(dependencyMarkers) {
					this.dependencyMarkers.put(rrWaiting.getRobotID(), new ArrayList<visualization_msgs.Marker>());
				}
			}
			synchronized(dependencyMarkers) {
				this.dependencyMarkers.get(rrWaiting.getRobotID()).add(mArrow);
			}
		}
	}

	@Override
	public void updateVisualization() {
		for (Entry<Integer, Publisher<MarkerArray>> entry : robotStatusPublishers.entrySet()) {
			synchronized(robotStatusMarkers) {
				if (!robotStatusMarkers.get(entry.getKey()).isEmpty()) { 
					visualization_msgs.MarkerArray ma = node.getTopicMessageFactory().newFromType(visualization_msgs.MarkerArray._TYPE);
					ArrayList<visualization_msgs.Marker> copy = new ArrayList<visualization_msgs.Marker>();
					for (visualization_msgs.Marker m : robotStatusMarkers.get(entry.getKey())) copy.add(m);
					if (envelopeMarkers != null) {
						synchronized(envelopeMarkers) {
							if (envelopeMarkers != null && envelopeMarkers.containsKey(entry.getKey())) copy.add(envelopeMarkers.get(entry.getKey()));
						}
					}
					ma.setMarkers(copy);
					entry.getValue().publish(ma);
					robotStatusMarkers.get(entry.getKey()).clear();
				}
			}
		}
		
		for (Entry<Integer, Publisher<MarkerArray>> entry : dependencyPublishers.entrySet()) {
			synchronized(dependencyMarkers) {
				if (!dependencyMarkers.get(entry.getKey()).isEmpty()) { 				
					visualization_msgs.MarkerArray ma = node.getTopicMessageFactory().newFromType(visualization_msgs.MarkerArray._TYPE);
					ArrayList<visualization_msgs.Marker> copy = new ArrayList<visualization_msgs.Marker>();
					for (visualization_msgs.Marker m : dependencyMarkers.get(entry.getKey())) copy.add(m);
					ma.setMarkers(copy);
					entry.getValue().publish(ma);
					dependencyMarkers.get(entry.getKey()).clear();
				}
			}
		}
		
		for (Entry<String, Publisher<MarkerArray>> entry : boxMarkerPublishers.entrySet()) {
			synchronized(boxMarkerMarkers) {
				if (!boxMarkerMarkers.get(entry.getKey()).isEmpty()) { 
					visualization_msgs.MarkerArray ma = node.getTopicMessageFactory().newFromType(visualization_msgs.MarkerArray._TYPE);
					ArrayList<visualization_msgs.Marker> copy = new ArrayList<visualization_msgs.Marker>();
					for (visualization_msgs.Marker m : boxMarkerMarkers.get(entry.getKey())) copy.add(m);
					ma.setMarkers(copy);
					entry.getValue().publish(ma);
					boxMarkerMarkers.get(entry.getKey()).clear();
				}
			}
		}

	}


	@Override
	public void onError(Node arg0, Throwable arg1) {
		// TODO Auto-generated method stub

	}


	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub

	}


	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub

	}


	@Override
	public void onStart(ConnectedNode arg0) {
		this.node = arg0;		
		while (true) {
			try {
				node.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		this.ready = true;
	}


	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("coordinator_viz");
	}

	@Override
	public void addEnvelope(TrajectoryEnvelope te) {
		GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
		Coordinate[] verts = dom.getGeometry().getCoordinates();

		visualization_msgs.Marker marker = node.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
		marker.getHeader().setFrameId(mapFrameID);
		marker.getScale().setX(0.1f);
		marker.getColor().setR(50f);
		marker.getColor().setG(50.0f);
		marker.getColor().setB(0.0f);
		marker.getColor().setA(0.8f);
		float R = 0.0f;
		float G = 0.0f;
		float B = 0.0f;
		float A = 0.7f;
		if (darkColors) {
			R = 50.0f;
			G = 50.0f;
			A = 0.8f;
		}
		marker.getColor().setR(R);
		marker.getColor().setG(G);
		marker.getColor().setB(B);
		marker.getColor().setA(A);
		marker.setAction(visualization_msgs.Marker.ADD);
		marker.setNs("current_envelope");
		marker.setType(visualization_msgs.Marker.LINE_STRIP);
		marker.setId(te.getRobotID());
		marker.setLifetime(new Duration(1.0));

		ArrayList<geometry_msgs.Point> points = new ArrayList<geometry_msgs.Point>();
		for (Coordinate coord : verts) {
			geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			point.setX(coord.x);
			point.setY(coord.y);
			point.setZ(0.0);
			points.add(point);
		}
		points.add(points.get(0));
		marker.setPoints(points);

		if (this.envelopeMarkers == null) this.envelopeMarkers = new HashMap<Integer,visualization_msgs.Marker>();
		synchronized(envelopeMarkers) {
			this.envelopeMarkers.put(te.getRobotID(),marker);
		}

	}

	@Override
	public void removeEnvelope(TrajectoryEnvelope te) {
		if (this.envelopeMarkers == null) return;
		synchronized(envelopeMarkers) {
			this.envelopeMarkers.remove(te.getRobotID());
		}		
	}
	
	public static void main(String[] args) {
		System.out.println("test");
	}

	@Override
	public int periodicEnvelopeRefreshInMillis() {
		// TODO Auto-generated method stub
		return 0;
	}

}
