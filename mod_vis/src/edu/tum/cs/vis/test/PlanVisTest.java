package edu.tum.cs.vis.test;

import javax.swing.JFrame;

import edu.tum.cs.vis.action.Action;
import edu.tum.cs.vis.gui.applet.PlanVisApplet;

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
		// TODO Auto-generated method stub
		
		JFrame f = new JFrame();
		f.setSize(800,600);
		
		PlanVisApplet applet = new PlanVisApplet();
		applet.init();
		applet.frame = f;
		
		f.add(applet);
		f.setTitle("PlanVis");
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setLocationRelativeTo(null);
		f.setVisible(true);
		
		Action brownies = new Action("Cook brownies");
		brownies.setProperty("Type", "chocklate brownie");
		brownies.setProperty("Depends on", "someone ordered brownies");
		brownies.setProperty("Approximate duration", "30 minutes");
		
		Action getEggs = new Action("get eggs");
		getEggs.setProperty("Number of eggs", "5");
		

		Action getBowl = new Action("get bowl");
		getBowl.setProperty("Bowl location", "On the table");
		getBowl.setProperty("Bowl size", "1 liter");
		
		Action crackEggs = new Action("crack eggs");
		crackEggs.setProperty("Number of eggs", "5");
		crackEggs.setProperty("Vessel", "Bowl");
		
		crackEggs.addParentAction(getEggs);
		crackEggs.addParentAction(getBowl);
		
		Action getFlour = new Action("get flour");
		getFlour.setProperty("Amount", "500 g");
		
		Action prepareDough = new Action("prepare dough");
		brownies.addSequence(prepareDough);
		
		prepareDough.addSequence(crackEggs);
		prepareDough.addSequence(getFlour);
		
		Action fillDoughIntoPan = new Action("Fill dough into pan");
		fillDoughIntoPan.setProperty("TestKEy", "TestValue");
		brownies.addSequence(fillDoughIntoPan);
		
		Action getBakingPan = new Action("get baking pan");
		getBakingPan.setProperty("Color", "blue");
		
		fillDoughIntoPan.addParentAction(getBakingPan);
		
		Action putIntoOven = new Action("put into oven");
		brownies.addSequence(putIntoOven);
		
		Action openOven = new Action("open oven");
		putIntoOven.addSequence(openOven);
		Action closeOven = new Action("close oven");
		putIntoOven.addSequence(closeOven);
		
		Action preheatOven = new Action("preheat oven");
		brownies.addParentAction(preheatOven);
		
		Action preheatOven2 = new Action("preheat oven2");
		brownies.addParentAction(preheatOven2);
		

		Action eatBrownies = new Action("eat brownies");
		brownies.addChildAction(eatBrownies);
		Action drinkSomething = new Action("drink something");
		brownies.addChildAction(drinkSomething);
		
		applet.setMainAction(brownies);
		
	}
}
