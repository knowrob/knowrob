package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.Color;
import java.util.ArrayList;

import processing.core.*;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.ServiceServer;
import ros.pkg.mod_vis.srv.CommVisSetLeftImg;
import ros.pkg.mod_vis.srv.CommVisSetRightImg;
import ros.pkg.mod_vis.srv.CommVisSetReqText;
import ros.pkg.mod_vis.srv.CommVisSetResText;
import edu.tum.cs.ias.knowrob.utils.ResourceRetriever;

public class CommunicationVisApplet extends PApplet {

	private static final long serialVersionUID = -3913679727924919169L;
	private PrologVisualizationCanvas prologVisCanvas = null;

	////////////////////////////////////////////////////////////////////////////////
	// DISPLAY PROPERTIES (ROTATION, ZOOM, ...)

	PFont verdana;
	PFont verdanaBold;
	PFont dejavu;

	PImage leftImg, rightImg;


	String imagePath = "";

	////////////////////////////////////////////////////////////////////////////////
	// BUFFERS
	ArrayList<Integer> colors = new ArrayList<Integer>(12);       // the colors used in the vis. HumanTrajVis

	int transmitting = 0;
	int receiving    = 0;
	int FRAMES_TO_DISPLAY_MESSAGES = 70;

	int LINE_LEFT_START = 250;
	int LINE_RIGHT_END  = 550;
	int LINE_WIDTH=LINE_RIGHT_END-LINE_LEFT_START;

	int UPPER_LINE_YPOS = 280;
	int LOWER_LINE_YPOS = 330;

	int transmitPosition=LINE_LEFT_START;
	int receivePosition=LINE_RIGHT_END;

	SetLeftImgCallback leftImgCallback;
	SetRightImgCallback rightImgCallback;
	SetRequestTextCallback requestTextCallback;
	SetResponseTextCallback responseTextCallback;


	String request  = "";

	String response = "";


	static Ros ros;
	public static NodeHandle n;


	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init("knowrob_comm_vis_applet");
		}
		n = ros.createNodeHandle();

	}


	/**
	 * Applet setup routine
	 */
	public void setup() {

		size(800, 600, P2D);
		this.frame.setTitle("CommunicationVisualization");
		this.frame.setBackground(new Color(20, 20, 20));
		background(20, 20, 20);

		initRos();

		imagePath = ResourceRetriever.findPackage("mod_vis");

		dejavu = createFont("DejaVu Sans",13);
		textFont(dejavu);
		hint(ENABLE_ACCURATE_TEXTURES);
		ellipseMode(RADIUS);
		frameRate(25);


		//leftImg  = new PImage();
		leftImg = loadImage(imagePath+"/images/pr2.jpg");
		rightImg = loadImage(imagePath+"/images/roboearth.png");
		//rightImg = loadImage("/work/ros/tumros-internal/stacks/knowrob/mod_vis/images/pr2.jpg");

		// init ros environment
		leftImgCallback = new SetLeftImgCallback();
		rightImgCallback =  new SetRightImgCallback();
		requestTextCallback = new SetRequestTextCallback();
		responseTextCallback = new SetResponseTextCallback();

		Thread startRosServices = new Thread( new RosServiceThread() );
		startRosServices.start();


		draw();
		if(prologVisCanvas != null) prologVisCanvas.validate();

	}


	/**
	 * drawing function, is called at 25Hz
	 */
	public void draw() {

		this.frame.setBackground(new Color(20, 20, 20));
		background(20, 20, 20);

		cursor(CROSS);

		textFont(dejavu);
		textMode(SCREEN);
		fill(0xFFFFFFFF);stroke(0xFFFFFFFF);

		// draw images at the ends of the communication
		synchronized(leftImg) {
			if(leftImg != null)
				image(leftImg,  ((int)(150-0.5*leftImg.width)), ((int)(300-0.5*leftImg.height)));
		}

		synchronized(rightImg) {
			if(rightImg != null)
				image(rightImg, ((int)(650-0.5*rightImg.width)), ((int)(300-0.5*rightImg.height)));
		}

		// draw dotted line
		for(int x=LINE_LEFT_START;x<LINE_RIGHT_END;x+=20) {
			line(x, UPPER_LINE_YPOS, x+10, UPPER_LINE_YPOS);
			line(x, LOWER_LINE_YPOS, x+10, LOWER_LINE_YPOS);
		}

		// display query text, draw moving circle
		synchronized(request) {
			textAlign(LEFT, BOTTOM);
			text(request,  250, 30,  300, 230);
		}	
		if(transmitting>0) {
			ellipse(transmitPosition, UPPER_LINE_YPOS, 3, 3);
			transmitPosition= LINE_LEFT_START + ((transmitPosition - LINE_LEFT_START + 10)%LINE_WIDTH);
			transmitting--;
		} else {
			this.request="";
		}

		// display response text, draw moving circle
		synchronized(response) {
			textAlign(LEFT, TOP);
			text(response, 250, 370, 300, 250);
		}
		if(receiving>0) {

			ellipse(receivePosition,  LOWER_LINE_YPOS, 3, 3);
			receivePosition = LINE_RIGHT_END  - ((LINE_RIGHT_END   - receivePosition + 10)%LINE_WIDTH);
			receiving--;
		} else {
			this.response="";
		}
	}

	/**
	 * set the request text and activate transmission display
	 * @param request the query text to be displayed
	 */
	public void setRequest(String req) {

		synchronized(request) {
			this.request = req;
			this.response = "";
		}
		this.setTransmitting(true);

		draw();
	}


	/**
	 * set the response text and activate reception display
	 * @param response the response text to be displayed
	 */
	public void setResponse(String response) {
		synchronized(response) {
			this.response = response;
		}
		this.setReceiving(true);

		draw();
	}


	/**
	 * set the image on the left hand side of the communication
	 * @param filename
	 */
	public void setLeftImage(String filename) {
		synchronized(leftImg) {
			if(filename.length()>0)
				this.leftImg  = loadImage(imagePath+"/images/"+filename);
			else 
				this.leftImg = new PImage();
		}
	}


	/**
	 * set the image on the right hand side of the communication
	 * @param filename
	 */
	public void setRightImage(String filename) {
		synchronized(rightImg) {
			if(filename.length()>0)
				this.rightImg = loadImage(imagePath+"/images/"+filename);
			else 
				this.rightImg = new PImage();
		}
	}

	/**
	 * activate transmission display for the next FRAMES_TO_DISPLAY_MESSAGES frames
	 * @param on switch transmission display on or off
	 */
	public void setTransmitting(boolean on) {
		if(on)
			this.transmitting += FRAMES_TO_DISPLAY_MESSAGES;
		else
			this.transmitting = 0;
	}

	/**
	 * activate reception display for the next FRAMES_TO_DISPLAY_MESSAGES frames
	 * @param on switch reception display on or off
	 */
	public void setReceiving(boolean on) {
		if(on)
			this.receiving += FRAMES_TO_DISPLAY_MESSAGES;
		else
			this.receiving = 0;
	}

	/**
	 * debug: simulate visualization events
	 */
	public void keyPressed(){

		switch(keyCode) {
		case 82: // r
			setReceiving(true);
			break;

		case 84: // t
			setTransmitting(true);
			break;

		case ESC:
			leftImg  = new PImage();
			rightImg = loadImage(imagePath+"/images/rosie.png");
			this.request = "";
			this.response = "";
			key=0;
			break;
		}

		delay(50);
		redraw();


	}

	public void delay(int millis)
	{

		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
		}
	}


	/**
	 *
	 * Simple thread that creates the ROS services which update the internal state variables
	 *
	 * @author tenorth@cs.tum.edu
	 *
	 */
	public class RosServiceThread implements Runnable {

		@Override public void run() {

			try {

				initRos();

				n.advertiseService("comm_vis_set_left_img",  new CommVisSetLeftImg(),  new SetLeftImgCallback());
				n.advertiseService("comm_vis_set_right_img", new CommVisSetRightImg(), new SetRightImgCallback());
				n.advertiseService("comm_vis_set_req_text",  new CommVisSetReqText(),  new SetRequestTextCallback());
				n.advertiseService("comm_vis_set_res_text",  new CommVisSetResText(),  new SetResponseTextCallback());
				ros.spin();

			} catch(RosException e) {
				e.printStackTrace();
			}
		}
	}


	/**
	 *
	 * The callback class for updating the left image in the visualization
	 *
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetLeftImgCallback implements ServiceServer.Callback<CommVisSetLeftImg.Request, CommVisSetLeftImg.Response> {

		@Override
		public CommVisSetLeftImg.Response call(CommVisSetLeftImg.Request req) {

			CommVisSetLeftImg.Response res = new CommVisSetLeftImg.Response();
			res.success = 0;

			if (req.filename != null) {
				setLeftImage(req.filename);
			}

			return res;
		}
	}

	/**
	 *
	 * The callback class for updating the right image in the visualization
	 *
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetRightImgCallback implements ServiceServer.Callback<CommVisSetRightImg.Request, CommVisSetRightImg.Response> {

		@Override
		public CommVisSetRightImg.Response call(CommVisSetRightImg.Request req) {

			CommVisSetRightImg.Response res = new CommVisSetRightImg.Response();
			res.success = 0;

			if (req.filename != null) {
				setRightImage(req.filename);
			}

			return res;
		}
	}

	/**
	 *
	 * The callback class for updating the request text in the visualization
	 *
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetRequestTextCallback implements ServiceServer.Callback<CommVisSetReqText.Request, CommVisSetReqText.Response> {

		@Override
		public CommVisSetReqText.Response call(CommVisSetReqText.Request req) {

			CommVisSetReqText.Response res = new CommVisSetReqText.Response();
			res.success = 0;

			if (req.request != null) {
				setRequest(req.request);
			}

			return res;
		}
	}

	/**
	 *
	 * The callback class for updating the response text in the visualization
	 *
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetResponseTextCallback implements ServiceServer.Callback<CommVisSetResText.Request, CommVisSetResText.Response> {

		@Override
		public CommVisSetResText.Response call(CommVisSetResText.Request req) {

			CommVisSetResText.Response res = new CommVisSetResText.Response();
			res.success = 0;

			if (req.response != null) {
				setResponse(req.response);
			}

			return res;
		}
	}



	/**
	 * Helper functions for visualization: send visualization strings to an instance of the
	 * CommunicationVisApplet to display the ongoing communication.
	 *
	 * @param req The  String to be displayed in the 'request' field of the visualization
	 * @param res The  String to be displayed in the 'response' field of the visualization
	 * @param leftImg  Image to be displayed on the left side of the visualization
	 * @param rightImg Image to be displayed on the right side of the visualization
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 */
	public static void visualizeCommunication(String req, String res, String leftImg, String rightImg) {

		try {
			initRos();

			if(leftImg!=null) {
				try {

					CommVisSetLeftImg.Request vis_req = new CommVisSetLeftImg.Request();
					vis_req.filename = leftImg;
					ServiceClient<CommVisSetLeftImg.Request, CommVisSetLeftImg.Response, CommVisSetLeftImg> cl =
							n.serviceClient("/comm_vis_set_left_img", new CommVisSetLeftImg());
					cl.call(vis_req);
					cl.shutdown();                                
				} catch(RosException e) {
//					e.printStackTrace();
					// fail silently if service has not been advertised
				}
			}

			if(rightImg!=null) {
				try {

					CommVisSetRightImg.Request vis_req = new CommVisSetRightImg.Request();
					vis_req.filename = rightImg;
					ServiceClient<CommVisSetRightImg.Request, CommVisSetRightImg.Response, CommVisSetRightImg> cl =
							n.serviceClient("/comm_vis_set_right_img", new CommVisSetRightImg());
					cl.call(vis_req);
					cl.shutdown();                                
				} catch(RosException e) {
//					e.printStackTrace();
					// fail silently if service has not been advertised
				}
			}

			if(req!=null) {
				try {

					CommVisSetReqText.Request vis_req = new CommVisSetReqText.Request();
					vis_req.request = req;
					ServiceClient<CommVisSetReqText.Request, CommVisSetReqText.Response, CommVisSetReqText> cl =
							n.serviceClient("/comm_vis_set_req_text", new CommVisSetReqText());
					cl.call(vis_req);
					cl.shutdown();                                
				} catch(RosException e) {
					// fail silently if service has not been advertised
				}
			}

			if(res!=null) {

				try {
					CommVisSetResText.Request vis_req = new CommVisSetResText.Request();
					vis_req.response = res;
					ServiceClient<CommVisSetResText.Request, CommVisSetResText.Response, CommVisSetResText> cl =
							n.serviceClient("/comm_vis_set_res_text", new CommVisSetResText());
					cl.call(vis_req);
					cl.shutdown();

				} catch(RosException e) {
					// fail silently if service has not been advertised
				}

			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}



	public void setPrologVisCanvas(PrologVisualizationCanvas c){
		prologVisCanvas = c;
	}

	public static void main(String args[]) {

		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.vis.applets.CommunicationVisApplet" });
	}
}

