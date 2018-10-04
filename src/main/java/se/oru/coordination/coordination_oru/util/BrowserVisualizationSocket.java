package se.oru.coordination.coordination_oru.util;


import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.HashSet;

import javax.imageio.ImageIO;

import org.eclipse.jetty.websocket.api.RemoteEndpoint;
import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.WebSocketAdapter;

import com.vividsolutions.jts.geom.Coordinate;

public class BrowserVisualizationSocket extends WebSocketAdapter {
	
	public static HashSet<RemoteEndpoint> ENDPOINTS = null;
	public static BufferedImage map = null;
	public static double resolution = 1;
	public static Coordinate origin = null;
	public static double initialScale = 1;
	public static Coordinate initialTranslation = null;
		
    @Override
    public void onWebSocketConnect(Session sess) {
        super.onWebSocketConnect(sess);
        System.out.println("Socket Connected: " + sess);
        if (ENDPOINTS == null) ENDPOINTS = new HashSet<RemoteEndpoint>();
        synchronized (ENDPOINTS) {
	        ENDPOINTS.add(super.getRemote());
	        //Send map and map parameters if present
	        if (BrowserVisualizationSocket.map != null) {
				try {
					System.out.println("Sending map metadata to newly connected client...");
					String setMetadataString = "{ \"operation\" : \"setMapMetadata\","
							+ "\"data\" : "
							+ "{ \"resolution\" : " + resolution + ", \"x\" : " + origin.x + ", \"y\" : " + origin.y + "}}";
					super.getRemote().sendString(setMetadataString);
					
					System.out.println("Sending map to newly connected client...");
					ByteArrayOutputStream baos = new ByteArrayOutputStream();
					ImageIO.write(map, "png", baos);
					baos.flush();
					byte[] imageInBytes = baos.toByteArray();
					baos.close();
					ByteBuffer bb = ByteBuffer.wrap(imageInBytes);
					super.getRemote().sendBytes(bb);
				}
				catch(IOException e) { e.printStackTrace(); }
	        }
	        if (BrowserVisualizationSocket.initialTranslation != null) {
				try {
					System.out.println("Sending initial transform to newly connected client...");
					String setInitialTransformString = "{ \"operation\" : \"setInitialTransform\","
							+ "\"data\" : "
							+ "{ \"scale\" : " + initialScale + ", \"x\" : " + initialTranslation.x + ", \"y\" : " + initialTranslation.y + "}}";
					super.getRemote().sendString(setInitialTransformString);					
				}
				catch(IOException e) { e.printStackTrace(); }
	        }
        }
	}
    
    @Override
    public void onWebSocketText(String message) {
        super.onWebSocketText(message);
        System.out.println("Received TEXT message: " + message);
    }
    
    @Override
    public void onWebSocketClose(int statusCode, String reason) {
    	System.out.println("Removing connection to client");
        ENDPOINTS.remove(super.getRemote());
        super.onWebSocketClose(statusCode,reason);
        System.out.println("Socket Closed: [" + statusCode + "] " + reason);
    }
    
    @Override
    public void onWebSocketError(Throwable cause) {
        super.onWebSocketError(cause);
        cause.printStackTrace(System.err);
    }
}