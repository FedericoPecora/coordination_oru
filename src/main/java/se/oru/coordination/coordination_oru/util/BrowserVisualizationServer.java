package se.oru.coordination.coordination_oru.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.eclipse.jetty.server.handler.AbstractHandler;

public class BrowserVisualizationServer extends AbstractHandler {

	private String serverHostNameOrIP = null;
	
	public BrowserVisualizationServer(String serverHostNameOrIP) {
		this.serverHostNameOrIP = serverHostNameOrIP;
	}
	
	@Override
	public void handle(String target, org.eclipse.jetty.server.Request baseRequest, HttpServletRequest request, HttpServletResponse response) throws IOException, ServletException {
		response.setContentType("text/html;charset=utf-8");
		response.setStatus(HttpServletResponse.SC_OK);
		baseRequest.setHandled(true);
		
		File file = new File("BrowserVisualizationResources" + File.separator + "styles.css"); 
		BufferedReader br = new BufferedReader(new FileReader(file)); 
		String style = "";
		String oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			style += (oneLine+"\n");  
		}
		br.close();

		file = new File("BrowserVisualizationResources" + File.separator + "Visualization.js"); 
		br = new BufferedReader(new FileReader(file)); 
		String script = "";
		oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			script += (oneLine+"\n");  
		}
		br.close();
		
		file = new File("BrowserVisualizationResources" + File.separator + "index.html"); 
		br = new BufferedReader(new FileReader(file)); 
		String page = "";
		oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			page += (oneLine+"\n");  
		}
		page = page.replace("PLACEHOLDER_STYLE", style);
		page = page.replace("PLACEHOLDER_SCRIPT", script);
		page = page.replace("PLACEHOLDER_IP", this.serverHostNameOrIP);

		//System.out.println("###\n" + page + "\n###");
		response.getWriter().println(page);
		br.close();

	}
}