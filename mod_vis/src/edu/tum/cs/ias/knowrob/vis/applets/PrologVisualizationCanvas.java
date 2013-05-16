package edu.tum.cs.ias.knowrob.vis.applets;

import java.awt.GridLayout;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.*;

import javax.swing.JFrame;

import controlP5.ControlP5;
import controlP5.ControlWindow;
import controlP5.Textarea;
import controlP5.Textfield;
import controlP5.ControlEvent;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;

import processing.core.*;
import jpl.Query;
public class PrologVisualizationCanvas extends PApplet implements MouseListener, MouseMotionListener, MapObjClickListener {

	private static final long serialVersionUID = 4575739930038583994L;
	public enum Part {KITCHEN_VIS, ACTION_VIS, VIDEO, ALL};

	/**
	 * true: loads test episode from DB and pushes it automatically
	 */
	private boolean debugMode = false;


	// components
	protected ActionVisApplet AVObject;
	protected SemanticMapVisApplet KVObject;
	public ControlP5 controlP5;
	protected ControlWindow controlWindow;
	//public ImageViewerApplet iviewer;
	public JFrame img_window;


	public PrologVisualizationCanvas() {

        img_window = null;

		// not required when calling from Prolog
		//initProlog();
		this.init();
		this.setSize(560, 620);

		AVObject = new ActionVisApplet();
		AVObject.init();
		AVObject.setSize(550, 600);
		AVObject.setPrologVisCanvas(this);


		KVObject = new SemanticMapVisApplet();
		KVObject.init();
		KVObject.setSize(720, 600);
		KVObject.setMapObjClickListener(this);


		this.add(KVObject);
		//this.add(AVObject);

		// deactivated control window (used for outdated action visualization only)
	    // initControlP5();


		this.draw();
		this.setVisible(true);
		this.setSize(1270, 620);

		if(debugMode) {
			(new Thread(new testDataLoader(this))).start();
		}

	}

	public void draw() {
		background(20, 20, 20);
	}


	/**
	 * displays an action via it's fixed identifier
	 * @param e.g. "http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2"
	 */
    public void displayFixed(String identifier) {
    	KVObject.displayActionFixedIdent(addSingleQuotes(identifier));
    	KVObject.redraw();
    }

	/**
	 * displays an action via it's fixed identifier
	 * @param e.g. "http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2"
	 */
    public void displayAction(String identifier) {
    	KVObject.displayAction(addSingleQuotes(identifier));
    	KVObject.redraw();
    }

    /**
     * display a 2D trajectory via its identifier
     * @param e.g. ""
     */
    public void displayEyeTrajectory(String identifier) {
    	KVObject.displayEyeTrajectory(addSingleQuotes(identifier));
    	KVObject.redraw();
    }

    public void displayHumanTrajectory(String identifier, String handUsed){
    	KVObject.displayHumanTrajectory(addSingleQuotes(identifier), addSingleQuotes(handUsed));
    	KVObject.redraw();
    }

	/**
	 * setActionInformation
	 * @param pl_list
	 * @param hand
	 * @param level
	 */
    public void setActionInformation(String[][][] pl_list, String hand, int level) {
    	if(AVObject != null)
    		AVObject.setActionInformation(pl_list, hand, level);
    	KVObject.redraw();
    }

    /**
     * completely clears the scene
     */
    public void clear(){
    	KVObject.clear();
    	KVObject.redraw();
    }

    /**
     * same as
     * clear();
     * addObject("http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls");
     */
    public void drawBackground(){
    	KVObject.drawBackground();
    	KVObject.redraw();
    }


    /**
     * adds Object to scene.
     * @param identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     */
    public void addObject(String identifier){
    	KVObject.addObject(addSingleQuotes(identifier));
    	KVObject.redraw();
    }

    /**
     * adds Objects to scene.
     * adds all items reachable from it via knowrob:properPhysicalPartTypes
     * adds all items from which this item can be reached via knowrob:describedInMap
     * @param identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     */
    public void addObjectWithChildren(String identifier) {
    	KVObject.addObjectWithChildren(addSingleQuotes(identifier));
    	KVObject.redraw();
    }

    /**
     * adds Trajectory to scene.
     * @param identifier List of identifiers, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     */
    public void addTrajectory(String[] identifiers){

  		for(String identifier : identifiers) {

  			addObject(identifier);

  			// draw in yellow
  			KVObject.highlightItem(addSingleQuotes(identifier), true, SemanticMapVisApplet.convertColor(255, 221, 0, 255));

  		}
    	KVObject.redraw();
    }

    /**
     * removes object from scene
     * @param identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     */
    public void removeObject(String identifier){
    	KVObject.removeObject(addSingleQuotes(identifier));
    	KVObject.redraw();
    }


    /**
     * removes Objects from scene.
     * removes all items reachable from it via knowrob:properPhysicalPartTypes
     * removes all items from which this item can be reached via knowrob:describedInMap
     * @param identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     */
    public void removeObjectWithChildren(String identifier) {
    	KVObject.removeObjectWithChildren(addSingleQuotes(identifier));
    	KVObject.redraw();
    }

    /**
     * highlights object
     * @param identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     * @param highlight true = highlight; false=unhighlight
     */
    public void highlight(String identifier, boolean highlight) {
    	KVObject.highlightItem(addSingleQuotes(identifier),highlight);
    	KVObject.redraw();
    }
    public void highlight(String identifier, boolean highlight, int color) {
    	KVObject.highlightItem(addSingleQuotes(identifier),highlight, color);
      KVObject.redraw();
    }
    public void highlight(String identifier, boolean highlight, int r, int g, int b) {

      int c = (((((255 << 8) + r) << 8) + g) << 8) + b;
      KVObject.highlightItem(addSingleQuotes(identifier),highlight, c);
      KVObject.redraw();

    }
    public void highlight(String identifier, boolean highlight, int r, int g, int b, String prob) {

    	// show prob by alpha channel
    	int alpha = (int) (255 * Float.valueOf(prob));
        int c = (((((alpha << 8) + r) << 8) + g) << 8) + b;

    	// show prob by hue
    	//int c = HSVtoRGB (1-Float.valueOf(prob), 1.0f, 1.0f)+ 0xFF000000;

    	KVObject.highlightItem(addSingleQuotes(identifier),highlight, c);
      KVObject.redraw();
    }
    /**
     * helper function: converts HSV to RGB color space
     * @param floats for hue, saturation, value
     */
	int HSVtoRGB (float h, float s, float v) {
		// h,s,v in [0,1]
		float rr = 0, gg = 0, bb = 0;
		float hh = (6 * h) % 6;
		int   c1 = (int) hh;
		float c2 = hh - c1;
		float x = (1 - s) * v;
		float y = (1 - (s * c2)) * v;
		float z = (1 - (s * (1 - c2))) * v;
		switch (c1) {
			case 0: rr=v; gg=z; bb=x; break;
			case 1: rr=y; gg=v; bb=x; break;
			case 2: rr=x; gg=v; bb=z; break;
			case 3: rr=x; gg=y; bb=v; break;
			case 4: rr=z; gg=x; bb=v; break;
			case 5: rr=v; gg=x; bb=y; break;
		}
		int N = 256;
		int r = Math.min(Math.round(rr*N),N-1);
		int g = Math.min(Math.round(gg*N),N-1);
		int b = Math.min(Math.round(bb*N),N-1);
		// create int-packed RGB-color:
		int rgb = ((r&0xff)<<16) | ((g&0xff)<<8) | b&0xff;
		return rgb;
	}

    /**
     * highlights object, and everything that is reachable from it via knowrob:properPhysicalPartTypes
     * @param identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
     * @param highlight true = highlight; false=unhighlight
     */
    public void highlightWithChildren(String identifier, boolean highlight) {
    	KVObject.highlightReachable(addSingleQuotes(identifier),highlight);
    	KVObject.redraw();
    }


    /**
     * clears all highlightings
     */
    public void clearHighlight() {
    	KVObject.clearHighlights();
    	KVObject.redraw();
    }


	/**
	 * displays an action via it's fixed identifier
	 * @param e.g. "http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2"
	 */
    public void mapObjectClicked(String identifier) {
    	displayInformationForEntity(addSingleQuotes(identifier));
    	KVObject.redraw();
    }


    public void setViewParameters(float xShift, float yShift,float xRot, float yRot, float zoom) {
    	KVObject.setViewParameters(xShift, yShift, xRot, yRot, zoom);
    	KVObject.redraw();
    }

	public void showImagesInNewWindow(String[] imgs) {

		img_window = new JFrame();
        img_window.setLayout(new GridLayout(1,imgs.length));

		int frameWidth = 0;
		int frameHeight = 30;

        int totalWidth = frameWidth;
        int totalHeight = frameHeight;

        for(String img : imgs) {
            img = img.replaceAll( "\"", "" ); //remove ""

            ImageViewerApplet iviewer = new ImageViewerApplet(img);
            iviewer.init();
		    //iviewer.setImage(img);
		    img_window.getContentPane().add(iviewer);
            iviewer.setSize(iviewer.width,iviewer.height);
            totalWidth += iviewer.width;
            totalHeight = max(iviewer.height + frameHeight, totalHeight);
        }

		img_window.setSize(totalWidth, totalHeight);
        img_window.pack();
		img_window.setVisible(true);
		img_window.setResizable(true);
	}


	  //////////////////////////////////////////////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////////////
	  //
	  // USER INTERFACE
	  //

		@SuppressWarnings("unused")
		private ArrayList<String[]> readInformationForEntity(String entity) {

			// read list of attributes from Prolog
		    ArrayList<String[]> bindings = new ArrayList<String[]>();
		    HashMap<String, Vector<String>> qres = PrologInterface.executeQuery("findall([P|O], (rdf_has("+entity+", P, O)), Cs)");

		    if(qres!=null) {
		    	for(String k : qres.keySet()) {

		    		Vector<String> res = qres.get(k);

		    		for(String o : res) {

		    			String[] list = o.split("'\\.'", 2);
		    			if(list.length<2) continue;
		    			String rest = list[1];

		    			bindings.addAll(dottedPairsToArrayList(rest));
		    		}
		    	}
		    }
			return bindings;
		}

		public void actionsInActivity() {
			HashMap<String, Vector<String>> qres = PrologInterface.executeQuery("rdf_has(Plan, rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string', 'set a table'))), " +
					                                            "comp_ehow:matching_actions(Plan, Act)");
			if(qres!=null) {
				Vector<String> act = qres.get("Act");

				if(act != null) {

					//System.out.println(tmp.toString());
					this.AVObject.setActionsInActivity(act.toArray());
				}
			}
		}

		private void displayInformationForEntity(String entity) {

			HashMap<String, Vector<String>> qres = PrologInterface.executeQuery("rdf_has("+entity+", P, O)");

			if(qres!=null) {
				Vector<String> P = qres.get("P");
				Vector<String> O = qres.get("O");
				String info = "";
				if(P != null && O != null)
					for(int i=0;i<P.size() && i<O.size();i++) {
						info += printKey(P.get(i)) + ": " + printValue(O.get(i)) + "\n";
					}

				if(controlP5!=null && controlP5.getController("CurrentAction")!=null)
					((Textfield) controlP5.getController("CurrentAction")).setValue(printKey(entity));
				if(controlP5!=null && controlP5.getGroup("CurrentAttributes")!=null)
					((Textarea) controlP5.getGroup("CurrentAttributes")).setText(info);

				System.out.println(info);
			}

		}

		private String printKey(String k) {
		   String[] ks = k.split("#");
			  if(ks.length>1) {
				  String res = ks[1].replaceAll("'", "");
				  return res;
			  }
			  else return k;
		}

	  	private String printValue(String v) {
	  		if (v.contains("#")) {
	  			String[]vs=v.split("#");
	  			return vs[1].replaceAll("'", "").replaceAll("\\(", "").replaceAll("\\)", "");

	  		} else if(!v.startsWith("literal")) {
	  			return v.replaceAll("'", "").replaceAll("\\(", "").replaceAll("\\)", "");

	  		} else {
	  			// extract the literal string
	  			String v1 = v.replaceFirst("literal\\(type\\('http://www\\.w3\\.org/2001/XMLSchema", "");
	  			return v1.replaceAll("'", "").replaceAll("\\(", "").replaceAll("\\)", "");
			  }
		}

//
// initialize control window for showing action/object properties (currently unused)
//
//	  private void initControlP5() {
//
//	  	controlP5 = new ControlP5(KVObject);
//	      controlWindow = controlP5.addControlWindow("controlP5window",660,230,400,350);
//	      controlWindow.setBackground(color(40));
//	      controlWindow.setUpdateMode(ControlWindow.NORMAL);
//	      controlWindow.frameRate(5);
//
//	      Textlabel act_label = controlP5.addTextlabel("act_label","Abstraction Level",80,40);
////	      act_label.setColorValueLabel(0xffffffff);
//	      act_label.setFont(3);
//	      act_label.moveTo(controlWindow);
//
//	      RadioButton act_radio = controlP5.addRadioButton("act_radio", 80, 60);
//	      act_radio.addItem("activity",3);
//	      act_radio.addItem("2",2);
//	      act_radio.addItem("1",1);
//	      act_radio.addItem("0",0);
//	      act_radio.moveTo(controlWindow);
//	      act_radio.setId(100);
//
//	      Textlabel color_label = controlP5.addTextlabel("color_label","Colors",220,40);
////	      color_label.setColorValueLabel(0xffffffff);
//	      color_label.setFont(3);
//	      color_label.moveTo(controlWindow);
//
//	      RadioButton color_radio = controlP5.addRadioButton("color_radio",220,60);
//	      color_radio.addItem("by action type",0);
//	      color_radio.addItem("by object",1);
//	      color_radio.addItem("missing in activity",2);
//	      color_radio.moveTo(controlWindow);
//	      color_radio.setId(101);
//
//	      controlP5.addTextfield("CurrentAction",80,120,200,20).moveTo(controlWindow);
//	      Textarea text = controlP5.addTextarea("CurrentAttributes","", 80,160,200,130);
//	      text.setColorBackground(0xFF000000);
//	      text.moveTo(controlWindow);
//
//	      controlWindow.hide();
//
//	  }

		public void controlEvent(ControlEvent e) {

				if(e.getController().getId()==101 && e.getValue()==2.0)
					actionsInActivity();

		}


	  private ArrayList<String[]> dottedPairsToArrayList(String rest) {
	  	ArrayList<String[]> bindings = new ArrayList<String[]>();
	  	while(rest.length()>0) {
	  		String[] l = rest.split("'\\.'", 2);

	  		if((l[0].equals("")) || (l[0].equals("("))) {
	  			if(l[0].startsWith("(")) {
	  				rest=l[1]; continue;

	  			} else  break;
	  		} else {
	  			bindings.add(l[0].substring(1, l[0].length()-2).split(", "));
	  			if(l.length>1) {
	  				rest=l[1];	continue;
	  			} else break;
	  		}

	  	}
	  	return bindings;
	  }


	  public static String removeSingleQuotes(String str) {
		  if(str.startsWith("'"))
			  str = str.substring(1);

		  if(str.endsWith("'"))
			  str = str.substring(0, str.length()-1);
		  return str;
	  }

	  public static String addSingleQuotes(String str) {
		  return "'"+removeSingleQuotes(str)+"'";
	  }


		public SemanticMapVisApplet getKitchenVisApplet() {
			return this.KVObject;
		}

	public static void main(String args[]) {
		PApplet.main(new String[] { "de.tum.in.fipm.kipm.gui.visualisation.base.PrologVisualizationCanvas" });
	}
}

class testDataLoader implements Runnable {

	PrologVisualizationCanvas c;
	public testDataLoader(PrologVisualizationCanvas c) {
		this.c = c;
	}

	@SuppressWarnings("unchecked")
	public void run() {

		try{
		Thread.sleep(20000);
		}catch(Exception e){};
		new Query("initCmdProlog").oneSolution();
		new Query("ensure_loaded('C:/Projects/Eclipse2/kipm/demo.pl')").oneSolution();

		new Query("ddLeft").oneSolution();
		try{
			FileInputStream fos = new FileInputStream("C:/Projects/Eclipse2/kipm/tmpDataLeft.ser");
			ObjectInputStream oos = new ObjectInputStream(fos);

			ArrayList<Object[]> o = (ArrayList<Object[]>)oos.readObject();

			for(Object[] obj : o){
				c.setActionInformation((String[][][])obj[0], (String)obj[1], (Integer)obj[2]);
			}

			oos.close();
		}catch(Exception e){}


		new Query("ddRight").oneSolution();
		try{
			FileInputStream fos = new FileInputStream("C:/Projects/Eclipse2/kipm/tmpDataRight.ser");
			ObjectInputStream oos = new ObjectInputStream(fos);

			ArrayList<Object[]> o = (ArrayList<Object[]>)oos.readObject();

			for(Object[] obj : o){
				c.setActionInformation((String[][][])obj[0], (String)obj[1], (Integer)obj[2]);
			}

			oos.close();
		}catch(Exception e){}


	}

}
