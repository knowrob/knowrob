/*
 * Copyright (c) 2014-15 Moritz Tenorth, Daniel Beßler
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

package org.knowrob.vis;

import java.io.IOException;
import java.io.File;

import org.apache.commons.logging.Log;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.eclipse.jetty.server.handler.AbstractHandler;
import org.eclipse.jetty.server.Request;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import org.knowrob.utils.ros.RosUtilities;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

/**
 * A web server for the visualization module of the KnowRob knowledge base.
 * The server is accessible on port 1111.
 * 
 * @author tenorth@cs.uni-bremen.de
 * @author danielb@cs.uni-bremen.de
 */
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
		
		ROSHandler ros_pkg_handler = new ROSHandler();
		VisWorkspaceHandler vis_ws_handler = new VisWorkspaceHandler();

		HandlerList handlers = new HandlerList();
		handlers.setHandlers(new Handler[] { vis_ws_handler, ros_pkg_handler, resource_handler,  def });
		server.setHandler(handlers);

		try {
			server.start();
			//	        server.join();
		}
		catch (Exception e) {
			log.warn("Unable to start knowrob_vis server.", e);
		}
	}
	
	public class VisWorkspaceHandler extends AbstractHandler {
		public void handle(
		                   String target,
		                   Request baseRequest,
		                   HttpServletRequest request,
		                   HttpServletResponse response)
		                   throws IOException, ServletException
		{
			String[] path = target.split("/");
			if(path.length < 3) return;
			String pkgName = path[1];
			if(pkgName.equals("lib")) return;
			String main_package = node.getParameterTree().getString("knowrob_html_package","knowrob_vis");
			File visPkg = new File(RosUtilities.rospackFind(main_package));
			File wsDir = visPkg.getParentFile().getParentFile();
			File pkgDir = new File(wsDir, pkgName);
			if(!pkgDir.exists()) return;
			// pass to standard resource handler
			ResourceHandler resource_handler = new ResourceHandler();
			resource_handler.setResourceBase(wsDir.getAbsolutePath());
			resource_handler.handle(target,baseRequest,request,response);
		}
	}
	
	public class ROSHandler extends AbstractHandler {
		public void handle(String target,
		                   Request baseRequest,
		                   HttpServletRequest request,
		                   HttpServletResponse response)
		                   throws IOException, ServletException
		{
			String[] path = target.split("/");
			if(path.length < 3) return;
			String pkgName = path[1];
			if(pkgName.equals("lib")) return;
			// find the basepath for the resource handler
			String pkgPath = RosUtilities.rospackFind(pkgName);
			if(pkgPath==null) return;
			File file = new File(pkgPath);
			String parentDirectory = file.getParentFile().getAbsolutePath();
			// pass to standard resource handler
			ResourceHandler resource_handler = new ResourceHandler();
			resource_handler.setResourceBase(parentDirectory);
			resource_handler.handle(target,baseRequest,request,response);
		}
	}
}
