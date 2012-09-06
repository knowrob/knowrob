package edu.tum.cs.ias.knowrob.vis.gui.applets;

import javax.swing.JFrame;

import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.vis.actions.Action;
import edu.tum.cs.ias.knowrob.vis.actions.ActionTransition;
import edu.tum.cs.ias.knowrob.vis.gui.applets.PlanVisAppletFsm;

/**
 * This is a test class for testing PlanVis functionality.
 * Used during development.
 * 
 * @author Stefan Profanter
 *
 */
public class PlanVisTest {
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		JFrame f = new JFrame();
		f.setSize(800,600);
		
		PlanVisAppletFsm applet = new PlanVisAppletFsm();
		applet.init();
		applet.frame = f;
		
		f.add(applet);
		f.setTitle("PlanVis");
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setLocationRelativeTo(null);
		f.setVisible(true);
		
		f.addMouseListener(applet);
		f.addMouseMotionListener(applet);
		f.addMouseWheelListener(applet);
		f.addKeyListener(applet);
		
		
		Action greet = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#GreetingAPerson", "Greeting a person");
		applet.setMainAction(greet);
		applet.addAction(greet);
		
		Action approach = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#ApproachPerson", "Approach the person");
		approach.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#ApproachingAPerson"));
		applet.addAction(approach);
		applet.addTransition(new ActionTransition(greet, approach, "OK"));

			Action detect = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#DetectPerson", "Detect persons");
			detect.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PersonDetection"));
			detect.addHasValue("minNumber", "1");
			approach.addSubAction(detect);
			
			Action localize = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#LocalizePerson", "Localize person");
			localize.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PersonLocalization"));
			localize.addHasValue("outputs", "person-pose1");
			approach.addSubAction(localize);
			
			Action navigate = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#NavigateToPerson", "Navigate to person");
			navigate.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#Navigation"));
			navigate.addHasValue("toLocation", "person-pose1");
			approach.addSubAction(navigate);


		Action identify = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#IdentifyPerson", "Identify the person");
		identify.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PersonIdentification"));
		applet.addAction(identify);
		applet.addTransition(new ActionTransition(approach, identify, "OK"));
		
		Action check_known = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#CheckIfKnown", "Check if Person known");
		check_known.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#CheckIfKnown"));
		applet.addAction(check_known);
		applet.addTransition(new ActionTransition(identify, check_known, "OK"));
		
		Action talk_to = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#SayHello", "Say hello");
		talk_to.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
		talk_to.addHasValue("spokenText", "Hello, NAME!");
		applet.addAction(talk_to);
		applet.addTransition(new ActionTransition(check_known, talk_to, "OK"));
		
		Action talk_to2 = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#SayHello2", "Say hello2");
		talk_to2.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
		talk_to2.addHasValue("spokenText", "Hello, NAME!");
		applet.addAction(talk_to2);
		applet.addTransition(new ActionTransition(talk_to, talk_to2, "OK"));

		Action ask_name = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#AskForName", "Ask for the name");
		ask_name.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
		ask_name.addHasValue("spokenText", "Hello, what is your name?");
		applet.addAction(ask_name);
		applet.addTransition(new ActionTransition(check_known, ask_name, "TIMEOUT"));
		
		Action ask_name2 = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#AskForName2", "Ask for the name2");
		ask_name2.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
		ask_name2.addHasValue("spokenText", "Hello, what is your name?");
		applet.addAction(ask_name2);
		applet.addTransition(new ActionTransition(ask_name, ask_name2, "OK"));
		applet.addTransition(new ActionTransition(ask_name2, check_known, "ERROR"));
		
		
//		Action brownies = Action.getAction("Cook brownies");
//		brownies.setProperty("Type", "chocolate brownie");
//		brownies.setProperty("Depends on", "someone ordered brownies");
//		brownies.setProperty("Approximate duration", "30 minutes");
//		
//		Action getEggs = Action.getAction("get eggs");
//		getEggs.setProperty("Number of eggs", "5");
//		
//
//		Action getBowl = Action.getAction("get bowl");
//		getBowl.setProperty("Bowl location", "On the table");
//		getBowl.setProperty("Bowl size", "1 liter");
//		
//		Action crackEggs = Action.getAction("crack eggs");
//		crackEggs.setProperty("Number of eggs", "5");
//		crackEggs.setProperty("Vessel", "Bowl");
//		
//		crackEggs.addParentAction(getEggs);
//		crackEggs.addParentAction(getBowl);
//		
//		crackEggs.addSequence(getEggs);
//		crackEggs.addSequence(getBowl);
//		Action openDrawer = Action.getAction("openDrawer");
//		getBowl.addSequence(openDrawer);
//		Action takeBowl1 = Action.getAction("takeBowl");
//		getBowl.addSequence(takeBowl1);
//		Action closeDrawer = Action.getAction("closeDrawer");
//		getBowl.addSequence(closeDrawer);
//		
//		Action getFlour = Action.getAction("get flour");
//		getFlour.setProperty("Amount", "500 g");
//		
//		Action prepareDough = Action.getAction("prepare dough");
//		brownies.addSequence(prepareDough);
//		
//		prepareDough.addSequence(crackEggs);
//		prepareDough.addSequence(getFlour);
//		
//		Action fillDoughIntoPan = Action.getAction("Fill dough into pan");
//		fillDoughIntoPan.setProperty("TestKEy", "TestValue");
//		brownies.addSequence(fillDoughIntoPan);
//		Action takeBowl = Action.getAction("take bowl");
//		getFlour.setProperty("Color", "green");
//		Action emptyBowl = Action.getAction("empty bowl");
//		fillDoughIntoPan.addSequence(takeBowl);
//		fillDoughIntoPan.addSequence(emptyBowl);
//		
//		Action getBakingPan = Action.getAction("get baking pan");
//		getBakingPan.setProperty("Color", "blue");
//		
//		fillDoughIntoPan.addParentAction(getBakingPan);
//		
//		Action putIntoOven = Action.getAction("put into oven");
//		brownies.addSequence(putIntoOven);
//		
//		Action openOven = Action.getAction("open oven");
//		putIntoOven.addSequence(openOven);
//		Action closeOven = Action.getAction("close oven");
//		putIntoOven.addSequence(closeOven);
//		
//		Action preheatOven = Action.getAction("preheat oven");
//		brownies.addParentAction(preheatOven);
//		
//		Action preheatOven2 = Action.getAction("preheat oven2");
//		brownies.addParentAction(preheatOven2);
//		
//
//		Action eatBrownies = Action.getAction("eat brownies");
//		brownies.addChildAction(eatBrownies);
//		Action drinkSomething = Action.getAction("drink something");
//		brownies.addChildAction(drinkSomething);
		

		
	}
}
