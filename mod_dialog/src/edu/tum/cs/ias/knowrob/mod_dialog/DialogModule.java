package edu.tum.cs.ias.knowrob.mod_dialog;

import instruction.gui.EHowInstructionPanel;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.List;
import java.util.Vector;

import javax.swing.JFrame;

import processing.core.PApplet;
import processing.core.PFont;
import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.Subscriber;
import ros.pkg.mary_tts.srv.SetMaryVoice;
import controlP5.CheckBox;
import controlP5.ControlEvent;
import controlP5.ControlFont;
import controlP5.ControlP5;
import controlP5.Textarea;
import controlP5.Textfield;
import edu.tum.cs.ias.knowrob.json_prolog.Prolog;
import edu.tum.cs.ias.knowrob.json_prolog.PrologBindings;
import edu.tum.cs.ias.knowrob.json_prolog.PrologQueryProxy;
import edu.tum.cs.ias.knowrob.json_prolog.PrologValue;
import edu.tum.cs.ias.knowrob.mod_dialog.queries.DialogQuery;



public class DialogModule extends PApplet  implements MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 4575739930038583994L;
	
	public static String TOP_LEVEL_STATE = "init";
	protected String state = TOP_LEVEL_STATE;	
	protected HashSet<String> registeredStates = new HashSet<String>();
	
	private static HashMap<String, String> prologToEnglish;
	private static HashMap<String, String> englishToProlog;
	
	protected String currentObject="";

	public JFrame img_window;
	public ImageViewerApplet iviewer;
	public JFrame ehow_window;
	public EHowInstructionPanel ehow_panel;
	
	protected boolean useSpeechInput = true;
	protected static boolean localProlog = false;


	static {
		// initialize the internal mappings en <-> pl
		setMappings(); 
	}
	
	ArrayList<DialogQuery> queries;
	
	// components
	public static ControlP5 controlP5;
	ControlFont font;
	CheckBox checkbox;	
	
	/**
	 * Thread-safe ROS initialization
	 */
	static Ros ros;
	public static NodeHandle n;

	static Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;

	
  Publisher<ros.pkg.std_msgs.msg.String> mary_pub;

	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("knowrob_dialog_applet");
		}
		n = ros.createNodeHandle();
		
		callback = new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>();
	}
	

	@SuppressWarnings("unchecked")
	public void setup() {

		size(550, 400, P2D);
	//	this.frame.setTitle("DialogModule");
		background(20, 20, 20);
	 //   this.frame.setBackground(new Color(20, 20, 20));
	    
		frameRate(10);
		initControlP5();
		draw();

		// load the classes for the different queries
		try {
			
			queries = new ArrayList<DialogQuery>();
			for(Class cl : getClasses("edu.tum.cs.ias.knowrob.mod_dialog.queries")) {
				
				if(Modifier.isAbstract(cl.getModifiers()))
					continue;
			//	System.out.println(cl.getSimpleName());
				
				try {
					Constructor<DialogQuery> constructor = cl.getConstructor(DialogModule.class);
					DialogQuery q;
					q = constructor.newInstance(this);
					queries.add(q);
				} catch (SecurityException e) {
					e.printStackTrace();
				} catch (NoSuchMethodException e) {
					e.printStackTrace();
				} catch (IllegalArgumentException e) {
					e.printStackTrace();
				} catch (InstantiationException e) {
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					e.printStackTrace();
				} catch (InvocationTargetException e) {
					e.printStackTrace();
				}
			}			

		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		
		// start ROS environment
		initRos();

	    Thread speechIn = new Thread( new SpeechInListenerThread() ); 
	    speechIn.start();
	    
	    Thread answerQueries = new Thread( new AnswerQueriesThread(this) ); 
	    answerQueries.start();
	    
      try {
        mary_pub = n.advertise("/mary/tts", new ros.pkg.std_msgs.msg.String(), 100);

      } catch (RosException e) {
          e.printStackTrace();
      } 
      

      // setup and pre-initialize the ehow window  	
	  	ehow_window = new JFrame();
	  	ehow_window.setVisible( false );
	  	ehow_window.setSize( 1100, 800 );
	  	ehow_window.setTitle( "Plan Importer GUI" );
	  	ehow_window.setLocation( 400, 300 );
	  	ehow_window.setBackground(new Color(20, 20, 20));
	  	
	  	ehow_window.addWindowListener( new WindowAdapter() {
	  		public void windowClosing( WindowEvent we ) {
	    	//System.exit( 0 );
	  			ehow_window.setVisible(false);
	    	} } );

	  	ehow_panel = new EHowInstructionPanel();
	  	ehow_window.add( ehow_panel );
	  	ehow_panel.init();
	  	ehow_panel.revalidate();

		if(localProlog) {
			initProlog();
		}

		 // load the initial Prolog environment
//			executeJSONPrologQuery("comp_cop:odufinder_listener(_)"); 
//			executeQuery("register_ros_package('ias_semantic_map')");
//			executeQuery("register_ros_package('ias_prolog_addons')");
//			executeQuery("use_module(library('comp_similarity'))");
	  	
	}

	public void draw() {
		
	//	this.frame.setBackground(new Color(20, 20, 20));
		background(20, 20, 20);
		controlP5.draw();
	}

	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// STATE MANAGEMENT
	// 
	
	public String getCurrentState() {
		return state;
	}
	
	public boolean stateExists(String state) {
		return state.equals(TOP_LEVEL_STATE) || registeredStates.contains(state);	
	}
	
	public void registerState(String state) {
		if(stateExists(state))
			throw new IllegalArgumentException("State already registered");
		registeredStates.add(state);
	}

	public void setState(String state) {
		if(!stateExists(state))
			throw new IllegalArgumentException("Tried to set invalid state");
		this.state = state;
	}
	
	public void setTopLevelState() {
		setState(TOP_LEVEL_STATE);
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// QUERY ANSWERING
	// 
	

	String answer_query(String q) {
		
		String res = null;
		try{
			
			// iterate over query classes until one of them matches the query string
			for(DialogQuery query : queries) {
				res = query.process(q);
				if(res != null) {
					System.out.println(query.getClass().getSimpleName() + " returned '" + res + "'");
					return res; 
				}
			}
			
			// default:
			return "I don't know.\n";

		} catch(Exception e) {
			e.printStackTrace();
			return "I don't know.\n";
		}
	}


	/**
	 * Execute the query
	 * @param query the query.
	 * @return the HashMap representing the bindings of the variables.
	 */
	public static HashMap<String, Vector<PrologValue>> executeJSONPrologQuery(String query) {

		if(localProlog) {
			return executeLocalPrologQuery(query);
		}
		
		System.out.println(query);
		
		HashMap<String, Vector<PrologValue>> result = new HashMap< String, Vector<PrologValue> >();

		Prolog pl = new Prolog("/json_prolog");
		PrologQueryProxy bdgs = pl.query("expand_goal(("+query+"),_9), call(_9)");

		// return null if there are no solutions (empty hash table == true)
		if(!bdgs.iterator().hasNext()) {
			return null;
		}
				
		for(PrologBindings bdg : bdgs) {

			 for(String key : bdg.getBdgs_().keySet()) {
				 
				 if(!result.containsKey(key)) {
					 result.put(key, new Vector<PrologValue>());
				 }
				 result.get(key).add(bdg.getBdgs_().get(key));
			 }
	    }
		 bdgs.finish();
		
		return result;
	}
	
	public static HashMap<String, Vector<PrologValue>> executeLispPrologQuery(String query) {

		System.out.println(query);
		
		HashMap<String, Vector<PrologValue>> result = new HashMap< String, Vector<PrologValue> >();

		Prolog pl = new Prolog("/execution_trace_server");
		PrologQueryProxy bdgs = pl.query(query);

		// return null if there are no solutions (empty hash table == true)
		if(!bdgs.iterator().hasNext()) {
			return null;
		}
				
		for(PrologBindings bdg : bdgs) {

			 for(String key : bdg.getBdgs_().keySet()) {
				 
				 if(!result.containsKey(key)) {
					 result.put(key, new Vector<PrologValue>());
				 }
				 result.get(key).add(bdg.getBdgs_().get(key));
			 }
	    }
		 bdgs.finish();
		
		return result;
	}
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// ROS STUFF
	// 
	
	
    public static class SpeechInListenerThread implements Runnable {
    	
    	@Override public void run() {
    		
    		try {

    			n.advertise("/knowrob/speech_in", new ros.pkg.std_msgs.msg.String(), 100);
    			Subscriber<ros.pkg.std_msgs.msg.String> sub = n.subscribe("/knowrob/speech_in", new ros.pkg.std_msgs.msg.String(), callback, 10);
	    		
	    		n.spin();
	    		sub.shutdown();
	    		
    		} catch(RosException e) {
    			e.printStackTrace();
    		}
    	}
    }
	
	
    public static class AnswerQueriesThread implements Runnable {
    	
    	DialogModule dialog_module;
    	
    	public AnswerQueriesThread(DialogModule parent) {
			this.dialog_module=parent;
		}
    	
    	@Override public void run() {

    		try {
    			
    			ros.pkg.std_msgs.msg.String res;
    			while (n.isValid()) {
    					
					res = callback.pop();
					
					if(((CheckBox) controlP5.getGroup("speech_in")).getState(0))
						dialog_module.handleTextQuery(res.data);
    					
    			}

    		} catch (InterruptedException e) {
    			e.printStackTrace();
    		}
    	}
    }



	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// QUERY UTILITY METHODS
	// 
	
    public void setVoice(String voice) {

        try {
	        ServiceClient<SetMaryVoice.Request, SetMaryVoice.Response, SetMaryVoice> cl =
	            n.serviceClient("/mary_tts/set_voice", new SetMaryVoice());
	        SetMaryVoice.Request req = new SetMaryVoice.Request();
	        req.voice_name=voice;
			cl.call(req);
		} catch (RosException e) {
		}
    }
    
	
	public String getCurrentObject() {
		return currentObject;
	}


	public void setCurrentObject(String currentObject) {
		this.currentObject = currentObject;
	}

	public static String removeSingleQuotes(String str) {
		if(str.startsWith("'"))
			str = str.substring(1);
	  
		if(str.endsWith("'"))
			str = str.substring(0, str.length()-1);
		return str;
	}

	protected static String startToUpperCase(String q) {
		return q.substring(0,1).toUpperCase() + q.substring(1, q.length());
	}
	
	public static String toProlog(String enString) {

		if(englishToProlog.containsKey(enString)) {
			return "knowrob:'"+englishToProlog.get(enString)+"'";
			
		} else if(  enString.startsWith("cupboard") ||
					enString.startsWith("oven") ||
					enString.startsWith("refrigerator") ||
					enString.startsWith("diswasher") ||
					enString.startsWith("table") ||
					enString.startsWith("kitchentable") ) {
			return "ias_map:'"+enString+"'";
			
		} else {
			return "knowrob:'"+startToUpperCase(enString)+"'";
		}
	}
	
	public static String toEnglish(String plString) {
		
		if(prologToEnglish.containsKey(plString)) {
			return prologToEnglish.get(plString);
			
		} else if(plString.contains("#")) { // strip URL
			return removeSingleQuotes(plString.split("#")[1]);
			
		} else if(plString.contains(":")) { // strip namespace
			return removeSingleQuotes(plString.split(":")[1]);
		} else {
			return plString;
		}
	}
	
	private static void setMappings() {
		
		prologToEnglish = new HashMap<String, String>();
		englishToProlog = new HashMap<String, String>();

		addMapping("type",                    "has the type");
		addMapping("depthOfObject",           "has the depth");
		addMapping("widthOfObject",           "has the width");
		addMapping("heightOfObject",          "has the height");
		addMapping("properPhysicalPartTypes", "has a part");
		addMapping("describedInMap",          "is part of the map");
		addMapping("subClassOf",              "is a sub-class of");
		
	}
	
	private static void addMapping(String pl, String en) {
		prologToEnglish.put(pl, en);
		englishToProlog.put(en, pl);
	}



	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// USER INTERFACE
	// 
	
	private void initControlP5() {

		controlP5 = new ControlP5(this);

		PFont pfont = createFont("Helvetica",10,true);
		font = new ControlFont(pfont);
		
		Textarea log = controlP5.addTextarea("Log","", 80,20,400,260);
		log.setColorBackground(0xFF000000);
		log.setColorForeground(0xFFCCCCCC);
		log.valueLabel().setFont(ControlP5.grixel);
		log.setLineHeight(16);
		log.showScrollbar();

		Textfield input = controlP5.addTextfield("Query", 80,300,400,25);
		input.setColorBackground(0xFF000000);
		input.valueLabel().setFont(ControlP5.grixel);

		//input.setText("What do you know about Cupboard1?");
		
		input.setAutoClear(true);
		input.setFocus(true);
		input.keepFocus(true);

  
		checkbox = controlP5.addCheckBox("speech_in",68,360);
		checkbox.setColorForeground(color(120));
		checkbox.setColorActive(color(200));
		checkbox.setColorLabel(color(128));
		
		// add items to a checkbox
		checkbox.addItem("Activate Speech Input",23);
		checkbox.activate("Activate Speech Input");


	}

	void controlEvent(ControlEvent theEvent) {
		
		if( (theEvent.isController()) && 
			(theEvent.controller() != null) && 
			(theEvent.controller() instanceof Textfield)) {
			handleTextQuery(theEvent.controller().stringValue());
			return;
		}
		
	}
	
	
	  public void keyPressed(){

			switch(keyCode) {
			case ESC:
				this.state = TOP_LEVEL_STATE;
				((Textarea)controlP5.getGroup("Log")).setText("");
				key=0;
				break;
			}
	  }


	void handleTextQuery(String textquery) {



			String log = ((Textarea)controlP5.getGroup("Log")).stringValue();
			log += "Op:    "+textquery+"\n";

			String response = answer_query(textquery);
			//if(state.equals(TOP_LEVEL_STATE)) {
				log += "Robot: "+response+"\n";				
			//}
			
			((Textarea)controlP5.getGroup("Log")).setText(log);
			((Textarea)controlP5.getGroup("Log")).scroll(1);

      // also send to tts system

          ros.pkg.std_msgs.msg.String m = new ros.pkg.std_msgs.msg.String();
          m.data = response;
          mary_pub.publish(m);
          
          System.out.println("out: "+ response);

	}
	
	public void showImageInNewWindow(String img) {
		
		int frameWidth = 0;
		int frameHeight = 30;
		
		iviewer = new ImageViewerApplet();
		img_window = new JFrame();
		img_window.getContentPane().add(iviewer, BorderLayout.CENTER);
		

		iviewer.init();
		iviewer.setImage(img);
		img_window.setSize(iviewer.width + frameWidth, iviewer.height + frameHeight);		
		
		img_window.setVisible(true);
		img_window.setResizable(false);

	}

	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// CLASS UTILITY METHODS
	// 
	
	
    /**
     * Scans all classes accessible from the context class loader which belong to the given package and subpackages.
     *
     * @param packageName The base package
     * @return The classes
     * @throws ClassNotFoundException
     * @throws IOException
     */
    private static Class[] getClasses(String packageName) throws ClassNotFoundException, IOException {
    	ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
		assert classLoader != null;
		String path = packageName.replace('.', '/');
		Enumeration<URL> resources = classLoader.getResources(path);
		List<File> dirs = new ArrayList<File>();
		while (resources.hasMoreElements()) {
		    URL resource = resources.nextElement();
		    dirs.add(new File(resource.getFile()));
		}
		ArrayList<Class> classes = new ArrayList<Class>();
		for (File directory : dirs) {
		    classes.addAll(findClasses(directory, packageName));
		}
		return classes.toArray(new Class[classes.size()]);
	}

    /**
     * Recursive method used to find all classes in a given directory and subdirs.
     *
     * @param directory   The base directory
     * @param packageName The package name for classes found inside the base directory
     * @return The classes
     * @throws ClassNotFoundException
     */
    private static List<Class> findClasses(File directory, String packageName) throws ClassNotFoundException {
        List<Class> classes = new ArrayList<Class>();
        if (!directory.exists()) {
            return classes;
        }
        File[] files = directory.listFiles();
        for (File file : files) {
            if (file.isDirectory()) {
                assert !file.getName().contains(".");
                classes.addAll(findClasses(file, packageName + "." + file.getName()));
            } else if (file.getName().endsWith(".class")) {
                classes.add(Class.forName(packageName + '.' + file.getName().substring(0, file.getName().length() - 6)));
            }
        }
        return classes;
    }

    /**
     * 
     * @param pkgname
     * @return
     */
	public static String findRosPackage(String pkgname) {
		
		try {	
			String line;		
			Process p = Runtime.getRuntime().exec( "rospack find " + pkgname +"");
			BufferedReader pathreader = new BufferedReader(new InputStreamReader(p.getInputStream(), "UTF-8"));
            if( (line = pathreader.readLine()) != null) {
            	return line;
            }
		} catch (IOException e) {
			e.printStackTrace(System.err);
		}
		return null;
	}
	
	
	

	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// LOCAL PROLOG METHODS
	// 
	
	private static void initProlog() {
		try {
			Vector<String> args= new Vector<String>(Arrays.asList(jpl.fli.Prolog.get_default_init_args()));
			args.add( "-G256M" );
			//args.add( "-q" );
			args.add( "-nosignals" );
			
			String rosprolog = findRosPackage("rosprolog");
			System.err.println(rosprolog+"/prolog/init.pl");
			jpl.fli.Prolog.set_default_init_args( args.toArray( new String[0] ) );

			// load the appropriate startup file for this context
			new jpl.Query("ensure_loaded('"+rosprolog+"/prolog/init.pl"+"')").oneSolution();
			new jpl.Query("register_ros_package('demo_cotesys_fall2010')").oneSolution();
			new jpl.Query("comp_cop:odufinder_listener(_)").oneSolution();

		} catch(Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Execute the given query. Assert the given premises and retract them after querying.
	 * @param query the query.
	 * @return the HashMap representing the bindings of the variables.
	 */
	public static HashMap<String, Vector<PrologValue>> executeLocalPrologQuery(String query) {

		jpl.Query q = new jpl.Query( "expand_goal(("+query+"),_9), call(_9)" );

		// return null if there are no solutions (empty hash table == true)
		if(!q.hasSolution()) {
			return null;
		}
		
		HashMap<String, Vector<PrologValue>> result = new HashMap<String, Vector<PrologValue>>();
				
		Hashtable[] solutions = q.allSolutions();
		
		for(Hashtable solution : q.allSolutions()) {

			 for(Object key : solution.keySet()) {
				 
				 jpl.Term t = (jpl.Term) solution.get(key);

				 if(!result.containsKey(key)) {
					 result.put(key.toString(), new Vector<PrologValue>());
				 }
				 result.get(key).add(new PrologValue(t.toString()));
			 }
	    }
		return result;
	}
	
	public boolean isLocalProlog() {
		return localProlog;
	}


	public void setLocalProlog(boolean localProlog) {
		this.localProlog = localProlog;
	}
    
	public static void main(String args[]) {
		
		DialogModule dialog = new DialogModule();
		JFrame window = new JFrame();
		window.add(dialog);
		dialog.init();		
		
		window.setSize(550, 440);
		window.setVisible(true);
		
		if(args.length>0 && args[0].equals("local")) {
			System.err.println("LOCAL PROLOG");
			dialog.setLocalProlog(true);
		}
	}
}


