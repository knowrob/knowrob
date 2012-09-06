package edu.tum.cs.ias.knowrob.vis.gui.applets;

import javax.swing.JFrame;

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
		
		
		Action greet = new Action("Greeting a person");
		applet.setMainAction(greet);
		applet.addAction(greet);
		
		Action approach = new Action("Approach the person");
		approach.setProperty("type", "ApproachingAPerson");
		applet.addAction(approach);
		applet.addTransition(new ActionTransition(greet, approach, "OK"));

			Action detect = new Action("Detect persons");
			detect.setProperty("type", "PersonDetection");
			detect.setProperty("minNumber", "1");
			approach.addSequence(detect);
			
			Action localize = new Action("Localize person");
			localize.setProperty("type", "PersonLocalization");
			localize.setProperty("outputs", "person-pose1");
			approach.addSequence(localize);
			
			Action navigate = new Action("Navigate to person");
			navigate.setProperty("type", "Navigation");
			navigate.setProperty("toLocation", "person-pose1");
			approach.addSequence(navigate);


		Action identify = new Action("Identify the person");
		identify.setProperty("type", "PersonIdentification");
		applet.addAction(identify);
		applet.addTransition(new ActionTransition(approach, identify, "OK"));
		
		Action check_known = new Action("Check if Person known");
		check_known.setProperty("type", "CheckIfKnown");
		applet.addAction(check_known);
		applet.addTransition(new ActionTransition(identify, check_known, "OK"));
		
		Action talk_to = new Action("Say hello");
		talk_to.setProperty("type", "SpeechSynthesis");
		talk_to.setProperty("spokenText", "Hello, NAME!");
		applet.addAction(talk_to);
		applet.addTransition(new ActionTransition(check_known, talk_to, "OK"));
		
		Action talk_to2 = new Action("Say hello2");
		talk_to2.setProperty("type", "SpeechSynthesis");
		talk_to2.setProperty("spokenText", "Hello, NAME!");
		applet.addAction(talk_to2);
		applet.addTransition(new ActionTransition(talk_to, talk_to2, "OK"));

		Action ask_name = new Action("Ask for the name");
		ask_name.setProperty("type", "SpeechSynthesis");
		ask_name.setProperty("spokenText", "Hello, what is your name?");
		applet.addAction(ask_name);
		applet.addTransition(new ActionTransition(check_known, ask_name, "TIMEOUT"));
		
		Action ask_name2 = new Action("Ask for the name2");
		ask_name2.setProperty("type", "SpeechSynthesis");
		ask_name2.setProperty("spokenText", "Hello, what is your name?");
		applet.addAction(ask_name2);
		applet.addTransition(new ActionTransition(ask_name, ask_name2, "OK"));
		applet.addTransition(new ActionTransition(ask_name2, check_known, "ERROR"));
		
		
//		Action brownies = new Action("Cook brownies");
//		brownies.setProperty("Type", "chocolate brownie");
//		brownies.setProperty("Depends on", "someone ordered brownies");
//		brownies.setProperty("Approximate duration", "30 minutes");
//		
//		Action getEggs = new Action("get eggs");
//		getEggs.setProperty("Number of eggs", "5");
//		
//
//		Action getBowl = new Action("get bowl");
//		getBowl.setProperty("Bowl location", "On the table");
//		getBowl.setProperty("Bowl size", "1 liter");
//		
//		Action crackEggs = new Action("crack eggs");
//		crackEggs.setProperty("Number of eggs", "5");
//		crackEggs.setProperty("Vessel", "Bowl");
//		
//		crackEggs.addParentAction(getEggs);
//		crackEggs.addParentAction(getBowl);
//		
//		crackEggs.addSequence(getEggs);
//		crackEggs.addSequence(getBowl);
//		Action openDrawer = new Action("openDrawer");
//		getBowl.addSequence(openDrawer);
//		Action takeBowl1 = new Action("takeBowl");
//		getBowl.addSequence(takeBowl1);
//		Action closeDrawer = new Action("closeDrawer");
//		getBowl.addSequence(closeDrawer);
//		
//		Action getFlour = new Action("get flour");
//		getFlour.setProperty("Amount", "500 g");
//		
//		Action prepareDough = new Action("prepare dough");
//		brownies.addSequence(prepareDough);
//		
//		prepareDough.addSequence(crackEggs);
//		prepareDough.addSequence(getFlour);
//		
//		Action fillDoughIntoPan = new Action("Fill dough into pan");
//		fillDoughIntoPan.setProperty("TestKEy", "TestValue");
//		brownies.addSequence(fillDoughIntoPan);
//		Action takeBowl = new Action("take bowl");
//		getFlour.setProperty("Color", "green");
//		Action emptyBowl = new Action("empty bowl");
//		fillDoughIntoPan.addSequence(takeBowl);
//		fillDoughIntoPan.addSequence(emptyBowl);
//		
//		Action getBakingPan = new Action("get baking pan");
//		getBakingPan.setProperty("Color", "blue");
//		
//		fillDoughIntoPan.addParentAction(getBakingPan);
//		
//		Action putIntoOven = new Action("put into oven");
//		brownies.addSequence(putIntoOven);
//		
//		Action openOven = new Action("open oven");
//		putIntoOven.addSequence(openOven);
//		Action closeOven = new Action("close oven");
//		putIntoOven.addSequence(closeOven);
//		
//		Action preheatOven = new Action("preheat oven");
//		brownies.addParentAction(preheatOven);
//		
//		Action preheatOven2 = new Action("preheat oven2");
//		brownies.addParentAction(preheatOven2);
//		
//
//		Action eatBrownies = new Action("eat brownies");
//		brownies.addChildAction(eatBrownies);
//		Action drinkSomething = new Action("drink something");
//		brownies.addChildAction(drinkSomething);
		

		
	}
}
