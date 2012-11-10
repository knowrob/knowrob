package edu.tum.cs.ias.knowrob.vis;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.Insets;
import java.awt.Label;
import java.awt.SystemColor;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Vector;
import javax.vecmath.Matrix4f;

import javax.vecmath.Vector3f;

import processing.core.PApplet;
import processing.core.PFont;
import processing.core.PGraphics3D;


public class Canvas extends PApplet implements MouseListener, MouseMotionListener {
	static final long serialVersionUID = 0;

	/**
	 * dimensions of the display
	 */
	protected int width = 800, height = 600; 
	
	/** 
	 * controls the view mode
	 */
	protected boolean useCamera;

	/**
	 * mouse tracking
	 */
	protected float leftMouseX = -1.0f, leftMouseY = -1.0f, rightMouseX = -1.0f, rightMouseY = -1.0f, centerMouseY = -1.0f;
	
	/** 
	 * parameters for non-camera-based viewing 
	 */	
	protected float xRotDisplay, yRotDisplay, xShiftDisplay, yShiftDisplay, zShiftDisplay, zoomDisplay;
	
	/**
	 * parameters for camera-based viewing
	 */ 		
	protected Vector3f eye, eyeTarget, eyeUp;
	
	/**
	 * size of the scene (diameter), used for scaling of the view
	 */
	protected float sceneSize = 4000;
	
	public boolean debugCamera;

	/**
	 * the collection of drawable items to be painted onto this canvas
	 */
	Vector<Drawable> items = new Vector<Drawable>();
	Vector<Drawable> items2D = new Vector<Drawable>();

	/**
	 * sets the width of the window (must set before the canvas is initialized using setup)
	 * @param width
	 */
	public void setWidth(int width) {
		this.width = width;
	}
	
	/**
	 * sets the height of the window (must set before the canvas is initialized using setup)
	 * @param width
	 */
	public void setHeight(int height) {
		this.height = height;
	}
	
	public int getHeight() {
		return height;
	}
	
	public int getWidth() {
		return width;
	}
	
	public void setSceneSize(float size) {
		this.sceneSize = size;
		this.zoomDisplay = 400 / sceneSize;
	}
	
	public Canvas() {
		useCamera = false;
		debugCamera = false;
		eye = new Vector3f(50.0f,50f,20f);
		eyeUp = new Vector3f(0,0,500);
		eyeTarget = new Vector3f(0,0,0);
		setStandardViewParams();
	}

	/**
	 * set standard parameters for non-camera-based view
	 */
	public void setStandardViewParams() {
		xRotDisplay = -106.25027f;
		yRotDisplay = 0.020062504f;
		xShiftDisplay = 103f;
		zShiftDisplay = 162f;
		setSceneSize(this.sceneSize);
	}
	
	public void setEyeTarget(Vector3f v) {
		this.eyeTarget.set(v);
	}
	
	public void setup() {
		size(width, height, PGraphics3D.P3D);
		lights();

		PFont font = createFont("Verdana", 12);
		textFont(font);

		ellipseMode(RADIUS);
		frameRate(20);

		noLoop();
		draw();
	}

	public void draw() {

		background(60, 60, 60);
		cursor(CROSS);
		
		if(!useCamera) {
			camera();
			
			pushMatrix();
			translate(width / 4.0f, height / 1.5f, -400.0f);
	
			lights();
			
			rotateZ(PI / 2);
			rotateY(-PI / 2);
			translate(0.0f, zShiftDisplay, xShiftDisplay);
	
			rotateZ(radians(xRotDisplay));
			rotateX(radians(yRotDisplay));
			
			scale(zoomDisplay);
			
			//System.out.println("zoom: " + zoomDisplay + " zShift: " + zShiftDisplay + " xShift: " + xShiftDisplay + " xRot: " + xRotDisplay + " yRot: " + yRotDisplay);
		}
		else {
			lights();
			//beginCamera();
			setCamera();			
		}

		preDrawItems();
		
		drawItems();

		// draw camera vectors
		if(debugCamera) {
			float coordlength = sceneSize / 8;
			Vector3f up = new Vector3f(eyeUp);
			up.scale(coordlength);
			up.add(eyeTarget);
			this.drawLine(eyeTarget, up, 0xff0000ff);
			Vector3f dir = new Vector3f(eyeTarget);
			dir.sub(eye);
			Vector3f third = new Vector3f();
			third.cross(dir, up);
			third.normalize();
			third.scale(coordlength);
			third.add(eyeTarget);
			this.drawLine(eyeTarget, third, 0xff00ff00);
			this.drawLine(eye, eyeTarget, 0xffff0000);
		}
			
		if(!useCamera)
			popMatrix();
		else {
			//endCamera();
		}
		
		for(Drawable d : items2D)
			d.draw(this);
	}
	
	protected void preDrawItems() {
		
	}
	
	protected void setCamera() {
		//beginCamera();
		camera(eye.x, eye.y, eye.z, eyeTarget.x, eyeTarget.y, eyeTarget.z, eyeUp.x, eyeUp.y, eyeUp.z);
		//camera(eye.x, eye.y, eye.z, eyeTarget.x, eyeTarget.y, eyeTarget.z, eye.x+eyeUp.x, eye.y+eyeUp.y, eye.z+eyeUp.z);
		//endCamera();
		
		//System.out.println("eye: " + eye + " -> " + eyeTarget + "  up: " + eyeUp);
	}
	
	public synchronized void drawItems() {
		for(Drawable d : items)
			d.draw(this);		
	}
	
	public void drawLine(Vector3f p1, Vector3f p2, int color) {
		//pushMatrix();
		drawLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, color);
		//popMatrix();
	}
	
	public void drawLine(float x1, float y1, float z1, float x2, float y2, float z2, int color) {
		stroke(color);
		line(x1,y1,z1,x2,y2,z2);
	}
	
	public synchronized void add(Drawable d) {
		this.items.add(d);
	}
	
	public synchronized void add2D(Drawable d) {
		this.items2D.add(d);
	}

	public void rotateAxis(double theta, Vector3f rotAxis){
		Matrix4f mat = edu.tum.cs.ias.knowrob.util.datastructures.Vector3f.getRotationMatrix(theta,rotAxis);
		applyMatrix(mat.m00, mat.m01, mat.m02, mat.m03,
					mat.m10, mat.m11, mat.m12, mat.m13,
					mat.m20, mat.m21, mat.m22, mat.m23,
					mat.m30, mat.m31, mat.m32, mat.m33);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// EVENT HANDLERS
	// 
	
	@Override
	public void keyPressed() {
		switch(keyCode) {
		case java.awt.event.KeyEvent.VK_C:
			useCamera = !useCamera;
			//setStandardViewParams();
			redraw();
			System.out.println("Turned camera " + (useCamera ? "on" : "off"));
			break;
		}
//		switch(key) {
//		case '+':
//			zoom(-2);
//			redraw();
//			break;
//		case '-':
//			zoom(2);
//			redraw();
//			break;
//		}		
	}
//
//	public void mousePressed(MouseEvent e) {
//
//		// general: save mouse positions for calculating rotation and translation
//		if (e.getButton() == MouseEvent.BUTTON1) {
//			leftMouseX = e.getX();
//			leftMouseY = e.getY();
//		}
//		else if (e.getButton() == MouseEvent.BUTTON3) {
//			rightMouseX = e.getX();
//			rightMouseY = e.getY();
//		}
//		else if (e.getButton() == MouseEvent.BUTTON2) {
//			centerMouseY = e.getY();
//		}
//
//	}
//
//	public void mouseReleased(MouseEvent e) {
//
//		if (e.getButton() == MouseEvent.BUTTON1) { // reset the buffers
//			leftMouseX = -1.0f;
//			leftMouseY = -1.0f;
//		}
//		else if (e.getButton() == MouseEvent.BUTTON3) { // reset the buffers
//			rightMouseX = -1.0f;
//			rightMouseY = -1.0f;
//		}
//		else if (e.getButton() == MouseEvent.BUTTON2) {
//			centerMouseY = -1.0f;
//		}
//
//	}
//
//	public void mouseDragged(MouseEvent e) {
//
//		if (leftMouseX != -1.0f) { // change rotation
//			if(!useCamera) {
//				yRotDisplay += (e.getY() - leftMouseY) * 0.03;
//				xRotDisplay += (e.getX() - leftMouseX) * 0.05;
//			}
//			else {
//				float dx = (e.getX() - leftMouseX) * 0.02f;
//				float dy = (e.getY() - leftMouseY) * 0.02f;
//				
//				// translate eye, so that the origin is its target
//				Vector3f negTarget = new Vector3f(eyeTarget);
//				negTarget.negate();
//				eye.add(negTarget);
//				
//				// rotation around vertical axis
//				eye.rotate(dx, eyeUp);
//	
//				// rotation around horizontal axis
//				Vector3f dir = new Vector3f(eyeTarget);
//				dir.subtract(eye);
//				Vector3f horDir = new Vector3f();
//				horDir.cross(eyeUp, dir);			
//				eye.rotate(dy, horDir);
//				//eyeUp.rotate(dy, horDir);
//				//eyeUp.normalize();
//				
//				// reverse translation
//				eye.add(eyeTarget);
//			}
//			
//			leftMouseX = e.getX();
//			leftMouseY = e.getY();
//		}
//		else if (rightMouseX != -1.0f) { // change translation
//			if(!useCamera) {
//				xShiftDisplay += -(e.getY() - rightMouseY) * 0.5;
//				zShiftDisplay += -(e.getX() - rightMouseX) * 0.5;
//			}
//			else {
//				float dx = (e.getX() - rightMouseX) * sceneSize / 1000;
//				float dy = (e.getY() - rightMouseY) * sceneSize / 1000;
//				
//				// horizontal translation
//				Vector3f dir = new Vector3f(eyeTarget);
//				dir.subtract(eye);
//				Vector3f horDir = new Vector3f();
//				horDir.cross(eyeUp, dir);
//				horDir.normalize();
//				horDir.scale(dx);
//				
//				// vertical translation
//				Vector3f vertDir = new Vector3f(eyeUp);
//				vertDir.normalize();
//				vertDir.scale(dy);
//				vertDir.negate();
//				//System.out.println("hor move: " + horDir);
//				//System.out.println("vert mode: " + vertDir);
//				
//				eye.add(horDir);
//				eye.add(vertDir);
//				eyeTarget.add(horDir);
//				eyeTarget.add(vertDir);
//			}
//			
//			rightMouseX = e.getX();
//			rightMouseY = e.getY();
//		}
//		else if (centerMouseY != -1.0f) { // zoom
//			zoom(e.getY() - (int)centerMouseY);
//			centerMouseY = e.getY();
//		}
//
//		redraw();
//	}
//	
//	public void zoom(int delta) {
//		if(!useCamera) {				
//			zoomDisplay += -(delta) * 10 / sceneSize;
//			if (zoomDisplay < 0.01) {
//				zoomDisplay = 0.01f;
//			}
//		}
//		else {
//			float dy = -(delta) * sceneSize / 1000;			
//			
//			Vector3f dir = new Vector3f(eyeTarget);
//			dir.subtract(eye);
//			dir.normalize();
//			dir.scale(dy);			
//			eye.add(dir);
//		}	
//	}


	/**
	 * runs this applet as a main application (copied from PApplet.main)
	 * @param args
	 */
	public void runMain() {
		// Disable abyssmally slow Sun renderer on OS X 10.5.
		if(platform == MACOSX) {
			// Only run this on OS X otherwise it can cause a permissions error.
			// http://dev.processing.org/bugs/show_bug.cgi?id=976
			System.setProperty("apple.awt.graphics.UseQuartz", "true");
		}

		try {
			boolean external = false;

			String name = null;
			boolean present = false;
			Color backgroundColor = Color.black; //BLACK;
			Color stopColor = Color.gray; //GRAY;
			GraphicsDevice displayDevice = null;
			boolean hideStop = false;

			// try to get the user folder. if running under java web start,
			// this may cause a security exception if the code is not signed.
			// http://processing.org/discourse/yabb_beta/YaBB.cgi?board=Integrate;action=display;num=1159386274
			String folder = null;
			try {
				folder = System.getProperty("user.dir");
			}
			catch (Exception e) {
			}

			// Set this property before getting into any GUI init code
			//System.setProperty("com.apple.mrj.application.apple.menu.about.name", name);
			// This )*)(*@#$ Apple crap don't work no matter where you put it
			// (static method of the class, at the top of main, wherever)

			if(displayDevice == null) {
				GraphicsEnvironment environment = GraphicsEnvironment
						.getLocalGraphicsEnvironment();
				displayDevice = environment.getDefaultScreenDevice();
			}

			Frame frame = new Frame(displayDevice.getDefaultConfiguration());

			//Dimension screen = Toolkit.getDefaultToolkit().getScreenSize();
			// remove the grow box by default
			// users who want it back can call frame.setResizable(true)
			frame.setResizable(false);

			// Set the trimmings around the image
			//!!!Image image = Toolkit.getDefaultToolkit().createImage(ICON_IMAGE);
			//frame.setIconImage(image);
			frame.setTitle(name);

			//	    Class c = Class.forName(name);
			//Class<?> c = Thread.currentThread().getContextClassLoader().loadClass(name);
			//PApplet applet = (PApplet) c.newInstance();
			PApplet applet = this;

			// these are needed before init/start
			applet.frame = frame;
			applet.sketchPath = folder;
			//applet.args = PApplet.subset(args, 1);
			//!!!applet.external = external;

			// For 0149, moving this code (up to the pack() method) before init().
			// For OpenGL (and perhaps other renderers in the future), a peer is
			// needed before a GLDrawable can be created. So pack() needs to be
			// called on the Frame before applet.init(), which itself calls size(),
			// and launches the Thread that will kick off setup().
			// http://dev.processing.org/bugs/show_bug.cgi?id=891
			// http://dev.processing.org/bugs/show_bug.cgi?id=908
			if(present) {
				frame.setUndecorated(true);
				frame.setBackground(backgroundColor);
				displayDevice.setFullScreenWindow(frame);
			}
			frame.setLayout(null);
			frame.add(applet);
			frame.pack();

			applet.init();

			// Wait until the applet has figured out its width.
			// In a static mode app, this will be after setup() has completed,
			// and the empty draw() has set "finished" to true.
			// TODO make sure this won't hang if the applet has an exception.
			while(applet.defaultSize && !applet.finished) {
				//System.out.println("default size");
				try {
					Thread.sleep(5);

				}
				catch (InterruptedException e) {
					//System.out.println("interrupt");
				}
			}
			//println("not default size " + applet.width + " " + applet.height);
			//println("  (g width/height is " + applet.g.width + "x" + applet.g.height + ")");

			if(present) {
				//	        frame.setUndecorated(true);
				//	        frame.setBackground(backgroundColor);
				//	        displayDevice.setFullScreenWindow(frame);

				//	        frame.add(applet);
				Dimension fullscreen = frame.getSize();
				applet.setBounds((fullscreen.width - applet.width) / 2,
						(fullscreen.height - applet.height) / 2, applet.width,
						applet.height);

				if(!hideStop) {
					Label label = new Label("stop");
					label.setForeground(stopColor);
					label.addMouseListener(new MouseAdapter() {
						public void mousePressed(MouseEvent e) {
							System.exit(0);
						}
					});
					frame.add(label);

					Dimension labelSize = label.getPreferredSize();
					// sometimes shows up truncated on mac
					//System.out.println("label width is " + labelSize.width);
					labelSize = new Dimension(100, labelSize.height);
					label.setSize(labelSize);
					label.setLocation(20, fullscreen.height - labelSize.height
							- 20);
				}

				// not always running externally when in present mode
				if(external) {
					applet.setupExternalMessages();
				}

			}
			else { // if not presenting
				// can't do pack earlier cuz present mode don't like it
				// (can't go full screen with a frame after calling pack)
				//	        frame.pack();  // get insets. get more.
				Insets insets = frame.getInsets();

				int windowW = Math.max(applet.width, MIN_WINDOW_WIDTH)
						+ insets.left + insets.right;
				int windowH = Math.max(applet.height, MIN_WINDOW_HEIGHT)
						+ insets.top + insets.bottom;

				frame.setSize(windowW, windowH);

				if(external) {
//					int locationX = editorLocation[0] - 20;
//					int locationY = editorLocation[1];
//
//					if(locationX - windowW > 10) {
//						// if it fits to the left of the window
//						frame.setLocation(locationX - windowW, locationY);
//
//					}
//					else { // doesn't fit
//						// if it fits inside the editor window,
//						// offset slightly from upper lefthand corner
//						// so that it's plunked inside the text area
//						locationX = editorLocation[0] + 66;
//						locationY = editorLocation[1] + 66;
//
//						if((locationX + windowW > applet.screenWidth - 33)
//								|| (locationY + windowH > applet.screenHeight - 33)) {
//							// otherwise center on screen
//							locationX = (applet.screenWidth - windowW) / 2;
//							locationY = (applet.screenHeight - windowH) / 2;
//						}
//						frame.setLocation(locationX, locationY);
//					}
				}
				else { // just center on screen
					frame.setLocation((applet.screenWidth - applet.width) / 2,
							(applet.screenHeight - applet.height) / 2);
				}

				//	        frame.setLayout(null);
				//	        frame.add(applet);

				if(backgroundColor == Color.black) { //BLACK) {
					// this means no bg color unless specified
					backgroundColor = SystemColor.control;
				}
				frame.setBackground(backgroundColor);

				int usableWindowH = windowH - insets.top - insets.bottom;
				applet.setBounds((windowW - applet.width) / 2, insets.top
						+ (usableWindowH - applet.height) / 2, applet.width,
						applet.height);

				if(external) {
					applet.setupExternalMessages();

				}
				else { // !external
					frame.addWindowListener(new WindowAdapter() {
						public void windowClosing(WindowEvent e) {
							System.exit(0);
						}
					});
				}

				// handle frame resizing events
				applet.setupFrameResizeListener();

				// all set for rockin
				if(applet.displayable()) {
					frame.setVisible(true);
				}
			}

			//System.out.println("showing frame");
			//System.out.println("applet requesting focus");
			applet.requestFocus(); // ask for keydowns
			//System.out.println("exiting main()");

		}
		catch (Exception e) {
			e.printStackTrace();
			System.exit(1);
		}
	}
}
