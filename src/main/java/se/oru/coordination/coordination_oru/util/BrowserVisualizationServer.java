package se.oru.coordination.coordination_oru.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

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
		
		ClassLoader loader = Thread.currentThread().getContextClassLoader();
		
		InputStream is = loader.getResourceAsStream("styles.css");
		BufferedReader br = new BufferedReader(new InputStreamReader(is));
		String style = "";
		String oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			style += (oneLine+"\n");  
		}
		br.close();
		is.close();
		
		is = loader.getResourceAsStream("Visualization.js");
		br = new BufferedReader(new InputStreamReader(is));
		String script = "";
		oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			script += (oneLine+"\n");  
		}
		br.close();
		is.close();
		
		is = loader.getResourceAsStream("matrix.min.js");
		br = new BufferedReader(new InputStreamReader(is));
		String matrix = "";
		oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			matrix += (oneLine+"\n");  
		}
		br.close();
		is.close();

		is = loader.getResourceAsStream("index.html");
		br = new BufferedReader(new InputStreamReader(is));
		String page = "";
		oneLine = null;
		while ((oneLine = br.readLine()) != null) {
			page += (oneLine+"\n");  
		}
		br.close();
		is.close();
		
		page = page.replace("PLACEHOLDER_STYLE", style);
		page = page.replace("PLACEHOLDER_SCRIPT", script);
		page = page.replace("PLACEHOLDER_MATRIX", matrix);
		page = page.replace("PLACEHOLDER_IP", this.serverHostNameOrIP);

		response.getWriter().println(page);

	}
}