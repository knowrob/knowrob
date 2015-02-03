package org.knowrob.vis;

import org.apache.commons.logging.Log;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.knowrob.utils.ros.RosUtilities;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

public class WebServer extends AbstractNodeMain {

	private Server server;

	private ConnectedNode node;
	
	private Log log;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		log = connectedNode.getLog();
		// Need to start the webserver after node in order to able to use
		// ROS parameters for server configuration.
		startWebServer(1111);
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_vis_server");
	}

	public void startWebServer(int port) {

		server = new Server(port);

		ResourceHandler resource_handler = new ResourceHandler();
		
		String main_package = node.getParameterTree().getString("knowrob_html_package","knowrob_vis");
		String welcome_file = node.getParameterTree().getString("knowrob_welcome_file","robohow.html");

		resource_handler.setDirectoriesListed(true);
		resource_handler.setWelcomeFiles(new String[]{ "index.html", welcome_file });
		resource_handler.setResourceBase(RosUtilities.rospackFind(main_package) + "/html");

		DefaultHandler def = new DefaultHandler();
		def.setServeIcon(false);

		HandlerList handlers = new HandlerList();
		handlers.setHandlers(new Handler[] { resource_handler,  def});
		server.setHandler(handlers);

		try {
			server.start();
			//	        server.join();
		}
		catch (Exception e) {
			log.warn("Unable to start knowrob_vis server.", e);
		}
	}

}
