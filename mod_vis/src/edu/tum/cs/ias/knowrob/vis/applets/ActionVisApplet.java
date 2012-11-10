package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import processing.core.PApplet;
import processing.core.PFont;
import processing.core.PGraphics;
import processing.core.PGraphics3D;
import controlP5.ControlEvent;
import controlP5.RadioButton;

public class ActionVisApplet extends PApplet implements MouseListener, MouseMotionListener {

  static final long serialVersionUID=0;

  PGraphics buffer;
  private PrologVisualizationCanvas prologVisCanvas = null;

  ////////////////////////////////////////////////////////////////////////////////
  // DISPLAY PROPERTIES (ROTATION, ZOOM, ...)
  
  // init values for the display and mouse interaction
  // init values for the display and mouse interaction
  float leftMouseX=-1.0f, leftMouseY=-1.0f, rightMouseX=-1.0f, rightMouseY=-1.0f, centerMouseY=-1.0f;
  float xRotDisplay=-2.549f, yRotDisplay=86.79f, xShiftDisplay=410f, yShiftDisplay=15f, zoomDisplay=1f;

  PFont verdana;
  PFont verdanaBold;
  PFont dejavu;

  ArrayList<String> actionTypes;
  ArrayList<String> objectTypes;
    
  ////////////////////////////////////////////////////////////////////////////////
  // BUFFERS
  ArrayList<Integer> colors = new ArrayList<Integer>(12);	// the colors used in the vis. HumanTrajVis

  
  private ArrayList<ArrayList<ActionInformation>> leftHandActions = new ArrayList<ArrayList<ActionInformation>>();
  private ArrayList<ArrayList<ActionInformation>> rightHandActions = new ArrayList<ArrayList<ActionInformation>>();
  
  private HashMap<Integer, String> color2id = new HashMap<Integer, String>();
  
  ////////////////////////////////////////////////////////////////////////////////
  // INDICES
  
  public int actionIndex = 0;			// index for the array lists actionData and trajectoryData
  private int idColor=0xFFFF0000;

  ////////////////////////////////////////////////////////////////////////////////
  // FLAGS
  
  int maxLevel = 1000;
  boolean initialize = false;

	/**
	 * Initialize the Prolog engine.
	 */
//	static {
//		try {
//			Vector<String> args= new Vector<String>(Arrays.asList(Prolog.get_default_init_args()));
//			args.add( "-G128M" );
//			args.add( "-q" );
//			args.add( "-nosignals" );
//			Prolog.set_default_init_args( args.toArray( new String[0] ) );
//  			// load the appropriate startup file for this context
//			new Query("ensure_loaded('/home/tenorth/work/owl/gram_ias.pl')").oneSolution();
//			new Query( "initCmdProlog" ).allSolutions();
//	
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
	
	
  
  public void setup() {

    actionTypes= new ArrayList<String>();
    objectTypes= new ArrayList<String>();
                     
		size(550, 600, PGraphics3D.P3D);
		buffer = createGraphics(width, height, PGraphics3D.P3D);
    lights();
    
    verdana = createFont("Verdana", 11);
    verdanaBold = createFont("Verdana-Bold", 18);
    dejavu = createFont("DejaVu Sans",13);
    
    textFont(verdana);
    textFont(verdanaBold);
    textFont(dejavu);    
    hint(ENABLE_ACCURATE_TEXTURES);
    
    
    ellipseMode(RADIUS);
    frameRate(20);
    
    setColors();

    noLoop();
    draw();
    
    if(prologVisCanvas != null) prologVisCanvas.validate();
    
  }

// This applet has the information of the abstract level for actions
  public void draw() {
    
	background(20, 20, 20);
	
    cursor(CROSS);
    ortho(-width/2, width/2, -height/2, height/2, -10, 10); 

    pushMatrix();

	    translate(xShiftDisplay, yShiftDisplay, 0.0f);
	        
	    rotateX( radians(xRotDisplay) );  
	    rotateY( radians( yRotDisplay) );
    
    	translate(2000f, height, -400.0f);
    
    	//System.out.println("xShift: " + xShiftDisplay +", yShift: " + yShiftDisplay +", xRot: " + xRotDisplay +"yRot: " + yRotDisplay);

    	
    	lights();
    	pushMatrix();
	      
		  textFont(verdana);
		  textMode(SCREEN);
		  textAlign(CENTER);
		  fill(0xFFFFFFFF);stroke(0xFFFFFFFF);
		  
		  text("left hand",  screenX(0, 25, -50), screenY(0, 25, -50));
		  text("right hand", screenX(0, 25, 50),  screenY(0, 25, 50));
    	
		  text("abstraction level", screenX(0, -200, 0),  screenY(0, -200, 0));
		  
    	  strokeWeight(2);
	      line(0, 0, 0,   1000, 0, 0);
	      line(0, 0, 0,   0, -180, 0);
		      line(0, -180, 0,   0, -170, -5);
		      line(0, -180, 0,   0, -170, 5);
	      
	      line(0, 0, -100,   0, 0, 100);

        popMatrix(); 

    	pushMatrix();
	    	drawActionSequence(this.g, false);
        popMatrix(); 
	      
    popMatrix();

	
 }
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  // 
  // PRIMITIVE DRAWING FUNCTIONS (POINTS, BOXES, TRAJECTORIES,...)
  // 

  public void drawActionSequence(PGraphics g, boolean isBuffer) {

	g.noStroke();
	int i=0;
	
	if(leftHandActions.size()<1) return;
	if(rightHandActions.size()<1) return;

	int level=0;
	while(level <= maxLevel) {
		
		ArrayList<ArrayList<ActionInformation>> actions;
		for(String side: new String[]{"left", "right"}) {
			
			if(side.equals("left")) actions=leftHandActions;
			else 					actions=rightHandActions;
		
			// draw action sequence
			if(level>=actions.size()) continue; 
			for (i=0; i< actions.get(level).size(); i++) {
				
				if(i>=actions.get(level).size()) continue;
				
				ActionInformation action  = actions.get(level).get(i);


				float actionLengthFactor = 1.0f;
				float start = actionLengthFactor*action.getStarttime();
				float end   = actionLengthFactor*action.getEndtime();
				if(end==-1) end=start;
				
				float yshift = -40 * level;
				float zshift = (side.equals("left"))?(-40):(40);

				g.pushMatrix();
					g.fill(this.getColor(actions.get(level).get(i), isBuffer));
					
					g.translate(100*start   // start position
							+50*(end-start) // shift to the center of the box
							                // vv subtract first action start time
							-(100*actionLengthFactor*actions.get(0).get(actionIndex).getTime()),yshift,zshift);
					g.box(((end==start)?(1):(100*(end-start))), 20, 40);			
				g.popMatrix();

				
			}

		}
		
		level++;
	}
  }


  private int getColor(ActionInformation item, boolean isBuffer) {

	  // return unique color if drawn into buffer
	  if(isBuffer)
		  return(item.getIdColor());
	  
	  // check if frame
	  if(item.endtime==-1) {
		  return (0xFF000000 + (((int)(80*item.starttime++)%200)<< 16));
	  }
	  
	  // select color based on controlWindow.colorType	
	  if(prologVisCanvas != null && (prologVisCanvas.controlP5.getController("color_radio")!=null) && ((RadioButton) prologVisCanvas.controlP5.getGroup("color_radio")).getValue()==0) {
		  
		  // action type
		  String[] act = item.getTypes();
		  if(!actionTypes.contains(act[0])) {actionTypes.add(act[0]);}
		  int idx = actionTypes.indexOf(act[0]);
		  if(idx<0 || idx>this.colors.size()) {System.out.println("Missing color for " + act[0]); return 0;}
		  return this.colors.get(idx);
	  
	  } else if(prologVisCanvas != null && (prologVisCanvas.controlP5.getController("color_radio")!=null) && ((RadioButton) prologVisCanvas.controlP5.getGroup("color_radio")).getValue()==1) {
		  
		  // object type		  
		  String obj = item.getObjecttype();
		  if(!objectTypes.contains(obj)) {objectTypes.add(obj);}
		  int idx = objectTypes.indexOf(obj);
		  
		  if(idx<0 || idx>this.colors.size()) {System.out.println("Missing color for " + obj);  return 0;}
		  
		  return this.colors.get(idx);
		  
	  } else if(prologVisCanvas != null && (prologVisCanvas.controlP5.getController("color_radio")!=null) && ((RadioButton) prologVisCanvas.controlP5.getGroup("color_radio")).getValue()==2) {
		  
		  // missing in activity -> return red until activities are implemented
		  if(item.isInActivity()) {
			  return 0xFF00CC00;
		  }
		  else return 0xFFCC0000;
	  }
	  return 0;
  }
  
  

  
  
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  // 
  // LOADING DATA
  // 

  public void setActionInformation(String[][][] pl_list, String hand, int level) {
  	
	  try{
	  
  	// read list of actions from Prolog
  	ArrayList<ActionInformation> actionlist = new ArrayList<ActionInformation>();
  	for(String[][] infos : pl_list) {
  		
  		String label = infos[0][0];
  		
  		float starttime=0.0f;
  		try{
  			starttime=Float.valueOf(infos[1][0]);
  		} catch(NumberFormatException e) {
  			starttime=Float.valueOf((infos[1][0].split("#"))[1]);
  		}
  		
  		float endtime=0.0f;
  		try{
  			endtime=Float.valueOf(infos[2][0]);
  		} catch(NumberFormatException e) {
  			endtime=Float.valueOf((infos[2][0].split("#"))[1]);
  		}

  		String[] types = infos[3];
  		String objecttype = infos[4][0];
  		
  		color2id.put(idColor, label);
  		actionlist.add(new ActionInformation(label, starttime, endtime, types, objecttype, idColor++));
  		
  	}
  	
  	int l = Integer.valueOf(level);
  	
  	while(l>=leftHandActions.size()) {
  		leftHandActions.add(new ArrayList<ActionInformation>());
  	}

  	while(l>=rightHandActions.size()) {
  		rightHandActions.add(new ArrayList<ActionInformation>());
  	}
  	
  	if(hand.equals("left"))
  		this.leftHandActions.set(l, actionlist);
  	else
  		this.rightHandActions.set(l, actionlist);
  	
	  } catch(Exception e) {
		  e.printStackTrace();
	  }
  	
  	this.redraw();
  }


  public void setActionsInActivity(Object[] actions) {
	  System.out.println(actions.toString());
	  for(Object act : actions) {
		  System.out.println(act.toString());
		  for(ArrayList<ActionInformation> alist : this.leftHandActions) {
			  for(ActionInformation ainfo : alist) {
				  if(("'"+ainfo.getLabel()+"'").equals(act.toString())) {
					  ainfo.setInActivity(true);
				  }
			  }
		  }
	  }	  
	  for(Object act : actions) {
		  for(ArrayList<ActionInformation> alist : this.rightHandActions) {
			  for(ActionInformation ainfo : alist) {
				  if(("'"+ainfo.getLabel()+"'").equals(act.toString())) {
					  ainfo.setInActivity(true);
				  }
			  }
		  }
	  }
  }
  	
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    // 
    // EVENT HANDLER
    // 

  	
  public void mousePressed(MouseEvent e) {
	  
	  // general: save mouse positions for calculating rotation and translation
	  if(e.getButton()==MouseEvent.BUTTON1) {
			leftMouseX = e.getX();
			leftMouseY = e.getY();
	  } else if(e.getButton()==MouseEvent.BUTTON3) {
			rightMouseX = e.getX();
			rightMouseY = e.getY();
	  } else if (e.getButton()==MouseEvent.BUTTON2) {
			centerMouseY = e.getY();
	  }

	  			 
	}
  
  
	public void mouseReleased(MouseEvent e) {
		
		if(e.getButton()==MouseEvent.BUTTON1) {	// reset the buffers
			leftMouseX = -1.0f;
			leftMouseY = -1.0f;
		} else if(e.getButton()==MouseEvent.BUTTON3) {	// reset the buffers
			rightMouseX = -1.0f;
			rightMouseY = -1.0f;
		} else if (e.getButton()==MouseEvent.BUTTON2) {
			centerMouseY = -1.0f;
		}
	
	}

    public void mouseDragged(MouseEvent e) {


    	if(leftMouseX != -1.0f) {	// change rotation
    		  xRotDisplay+= (e.getY()-leftMouseY) * 0.05;
    		  yRotDisplay-= (e.getX()-leftMouseX) * 0.05;
  			leftMouseX = e.getX();
			leftMouseY = e.getY();
    		
		}else if(rightMouseX != -1.0f) {	// change translation
  		  yShiftDisplay+= (e.getY()-rightMouseY) * 0.5;
  		  xShiftDisplay+= (e.getX()-rightMouseX) * 0.5;
			rightMouseX = e.getX();
			rightMouseY = e.getY();
  		
		} else if (centerMouseY != -1.0f) {	// zoom
			  zoomDisplay+= (e.getY()-centerMouseY) * 0.02;
			  if(zoomDisplay<0.01){zoomDisplay=0.01f;}
			  centerMouseY = e.getY();
		}
    	
    	redraw();
    }
    
    public void mouseClicked() {

    	  // draw the scene in the buffer
    	  buffer.beginDraw();
    	  buffer.background(0); // since background is not an object, its id is 0
    	  
    	    buffer.ortho(-width/2, width/2, -height/2, height/2, -10, 10); 

    	    buffer.pushMatrix();

    		    buffer.translate(xShiftDisplay, yShiftDisplay, 0.0f);
    		        
    		    buffer.rotateX( radians(xRotDisplay) );  
    		    buffer.rotateY( radians( yRotDisplay) );
    	    
    		    buffer.translate(2000f, height, -400.0f);
    	    	drawActionSequence(buffer, true);
    	    	
    	    buffer.popMatrix();
    	  buffer.endDraw();
    	 
    	  // get the pixel color under the mouse
    	  int pick = buffer.get(mouseX, mouseY);
    	  
    	  String id = this.color2id.get(pick);
    	  if(id!=null && prologVisCanvas != null) {
		  	  prologVisCanvas.displayAction(id);
		  	  prologVisCanvas.mapObjectClicked(id);
    	  }
    }
    
//    public void keyPressed(){
//    	
//    	// hook: when running as slave, just propagate key events for proper synchronization
//  	    //if(prologVisCanvas != null) {
//  	    //	prologVisCanvas.keyPressed(keyCode);
//  	    //	return;
//  	    //}
//    	
//		switch(keyCode) {
//			case RIGHT:
//				actionIndex++;
//				break;
//			case LEFT:
//				actionIndex--;
//				break;
//			case java.awt.event.KeyEvent.VK_PAGE_UP:
//				actionIndex+=10;
//				break;
//			case java.awt.event.KeyEvent.VK_PAGE_DOWN:
//				actionIndex-=10;
//				break;
//		}
//		
//
//		if(actionIndex>leftHandActions.get(0).size()-1) 
//			actionIndex=leftHandActions.get(0).size()-1;
//		if(actionIndex<0) actionIndex=0;		
//		redraw();
//	}
    
    
    public 	void controlEvent(ControlEvent theEvent) {
        if(prologVisCanvas != null && theEvent.isGroup() && theEvent.getGroup().getId()==100) {
        	// act_radio
        	this.maxLevel =(int)prologVisCanvas.controlP5.getGroup("act_radio").getValue();

        } else if(theEvent.isGroup() && theEvent.getGroup().getId()==101) {
           	// act_color
        }
        redraw();
    }
    

  public int makeColor(int r,int g,int b){
  	  int col = 0xFF000000 + 0x0000FF00 * r + 0x000000FF * g + b;
  	  return col;
    }
  
  public void setPrologVisCanvas(PrologVisualizationCanvas c){
	  prologVisCanvas = c;
  }
    
    //fills the array colors with two colors for each action, one for the trajectory and one for the human pose
    public void setColors(){

    	for(int[] c : new int[][]{	
					    	    	
					    	    	{255,255,153},  {255,153,153},    	{153,0,102},
					    	    	{51,0,51},    	{102,102,102},    	{102,51,153},
					    	    	{255,204,0},	{255,51,0},	    	{153,204,0},
					    	    	{204,153,0},    {255,255,255},    	{255,153,102},
					    	    	{102,0,0},    	{204,255,102},    	{102,102,51},
					    	    	{204,204,204},  {255,255,51},    	{153,102,102},
					    	    	{153,0,51},    	{153,153,153},    	{51,153,0},
					    	    	{102,153,204},  {51,51,204},    	{0,153,102},
					    	    	{102,255,204},  {0,0,204},
					    	    	{102,153,204},  {51,51,204},    	{0,153,102},
					    	    	{255,255,153},  {255,153,153},    	{153,0,102},
					    	    	{153,0,51},    	{153,153,153},    	{51,153,0},
					    	    	{51,0,51},    	{102,102,102},    	{102,51,153},
					    	    	{204,204,204},  {255,255,51},    	{153,102,102},
					    	    	{102,0,0},    	{204,255,102},    	{102,102,51},
					    	    	{204,153,0},    {255,255,255},    	{255,153,102},
					    	    	{255,204,0},	{255,51,0},	    	{153,204,0},
					    	    	{102,255,204},  {0,0,204}}) {
    		colors.add(makeColor(c[0], c[1], c[2]));
    	}
    	

    }       

    
	private class ActionInformation implements Serializable {
		
		private static final long serialVersionUID = -1005928838950117046L;
		private String label;
		private float starttime;
		private float endtime=-1;
		private String[] types;
		private String objecttype;
		private int idColor;
		private boolean inActivity;


		public ActionInformation(String label, float starttime, float endtime, String[] types, String objecttype, int idColor) {
			super();
			this.label = label;
			this.starttime = starttime;
			this.endtime = endtime;
			this.types = types;
			this.objecttype = objecttype;
			this.idColor = idColor;
			this.inActivity = false;
		}


		@SuppressWarnings("unused")
		public ActionInformation(String label, float time, String[] types, String objecttype, int idColor) {
			super();
			this.label = label;
			this.starttime = time;
			this.types = types;
			this.objecttype = objecttype;
			this.idColor = idColor;
			this.inActivity = false;
		}


		public String getLabel() {
			return label;
		}


		@SuppressWarnings("unused")
		public void setLabel(String label) {
			this.label = label;
		}


		public String getObjecttype() {
			return objecttype;
		}


		@SuppressWarnings("unused")
		public void setObjecttype(String objecttype) {
			this.objecttype = objecttype;
		}


		public float getTime() {
			return starttime;
		}


		public int getIdColor() {
			return idColor;
		}


		@SuppressWarnings("unused")
		public void setIdColor(int idColor) {
			this.idColor = idColor;
		}


		@SuppressWarnings("unused")
		public void setTime(float time) {
			this.starttime = time;
		}


		public String[] getTypes() {
			return types;
		}


		@SuppressWarnings("unused")
		public void setTypes(String[] types) {
			this.types = types;
		}


		public float getEndtime() {
			return endtime;
		}


		@SuppressWarnings("unused")
		public void setEndtime(float endtime) {
			this.endtime = endtime;
		}


		public float getStarttime() {
			return starttime;
		}


		@SuppressWarnings("unused")
		public void setStarttime(float starttime) {
			this.starttime = starttime;
		}
		
		public boolean isInActivity() {
			return inActivity;
		}


		public void setInActivity(boolean inActivity) {
			this.inActivity = inActivity;
		}

	}

	
//	 public void save() {
//		 System.out.println("saving...");
//		  try{
//			   FileOutputStream fos = new FileOutputStream("/home/tenorth/lefthand.ser");
//			   ObjectOutputStream oos = new ObjectOutputStream(fos);
//			   oos.writeObject(this.leftHandActions);
//			   oos.flush();
//			   oos.close();
//			   
//			   fos = new FileOutputStream("/home/tenorth/righthand.ser");
//			   oos = new ObjectOutputStream(fos);
//			   oos.writeObject(this.rightHandActions);
//			   oos.flush();
//			   oos.close();
//			   
//		  }catch(Exception e){
//			  e.printStackTrace();
//		  }
//	}
	
	public static void main(String args[]) {
		PApplet.main(new String[] { "de.tum.in.fipm.kipm.gui.visualisation.applets.ActionVisApplet" });
	}
}

