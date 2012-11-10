package edu.tum.cs.ias.knowrob.vis.applets;

import javax.swing.JFrame;

import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.vis.applets.PlanVisAppletFsm;

/**
 * This is a test class for testing PlanVis functionality.
 * Used during development.
 * 
 * @author Moritz Tenorth, Stefan Profanter
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
		PrologInterface.initJPLProlog("re_unr_interface");
//		PFont dejavuFont = applet.createFont("DejaVu Sans",13);
//	    applet.textFont(dejavuFont); // FIXME: problem with textFont, font seems OK, but the method throws a NullPointerException
		applet.loadPrologPlan("http://www.roboearth.org/kb/test-recipe-greeting-sm.owl#UnrGreetingTest");
		
		applet.init();
		applet.frame = f;

	    applet.drawActionsTreeLayout();
		applet.redraw();
		
		f.add(applet);
		f.setTitle("PlanVis");
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setLocationRelativeTo(null);
		f.setVisible(true);
		
		f.addMouseListener(applet);
		f.addMouseMotionListener(applet);
		f.addKeyListener(applet);
		

		
//		
//		Action recommend = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#RecommendingAProduct", "Recommend a product");
//		applet.setTask(recommend);
//
//		
//		
//		Action detect = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#DetectPerson", "Detect persons");
//		detect.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PersonDetection"));
//		detect.addHasValue("minNumber", "1");
//		applet.addAction(detect);
//		
//		recommend.addSubAction(detect);
//		recommend.getTransitions().setStartAction(detect);
//
//		
//
//		Action identify = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#IdentifyPerson", "Identify the person");
//		identify.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#PersonIdentification"));
//		identify.addHasValue("outputsCreated", "?NAME");
//		recommend.addSubAction(identify);
//		detect.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition2", detect, identify, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_OK"));
//		
//		
//		
//		Action talk_to = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#GreetPerson", "Greet and ask if help is needed");
//		talk_to.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
//		talk_to.addHasValue("spokenText", "Hello, ?NAME! Can I help you?");
//		talk_to.addHasValue("outputsCreated", "?RESPONSE");
//		recommend.addSubAction(talk_to);
//		identify.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition3", identify, talk_to, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_OK"));
//
//
//		
//		Action check_if_question = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#ProductPlacementQuestion", "Question for product location?");
//		check_if_question.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#ProductPlacementQuestion"));
//		check_if_question.addHasValue("evaluateOn", "?RESPONSE");
//		check_if_question.addHasValue("outputsCreated", "?OBJECT");
//		recommend.addSubAction(check_if_question);
//		talk_to.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition4", talk_to, check_if_question, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_OK"));
//
//		
//		Action compute_obj_pos = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#CompObjPos", "Compute object position");
//		compute_obj_pos.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#CompObjPos"));
//		compute_obj_pos.addHasValue("objectActedOn", "?OBJECT");
//		recommend.addSubAction(compute_obj_pos);
//		check_if_question.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition5", check_if_question, compute_obj_pos, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_CONDITION_TRUE"));
//
//		
//		
//		Action ask_operator = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#AskHumanOperator", "Ask human operator");
//		ask_operator.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#HumanOperatorQuestion"));
//		ask_operator.addHasValue("objectActedOn", "?OBJECT");
//		ask_operator.addHasValue("outputsCreated", "?STORAGE_LOC");
//		recommend.addSubAction(ask_operator);
//		compute_obj_pos.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition6", compute_obj_pos, ask_operator, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_TIMEOUT"));
//		ask_operator.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition7", ask_operator, compute_obj_pos, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_TIMEOUT"));
//
//
//		
//		Action describe_loc = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#DescribeLocation", "Describe location");
//		describe_loc.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
//		describe_loc.addHasValue("spokenText", "?OBJECT can be found at ?STORAGE_LOC");
//		recommend.addSubAction(describe_loc);
//		compute_obj_pos.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition8", compute_obj_pos, describe_loc, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_OK"));
//		
//		
//		
//		
//		Action goodbye = Action.getAction("http://ias.cs.tum.edu/kb/knowrob.owl#Goodbye", "Say goodbye");
//		goodbye.addSuperClass(OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#SpeechSynthesis"));
//		goodbye.addHasValue("spokenText", "Okay, then have a nice day!");
//		recommend.addSubAction(goodbye);
//		check_if_question.addTransition(ActionTransition.getActionTransition("http://ias.cs.tum.edu/kb/knowrob.owl#Transition9", check_if_question, goodbye, "http://www.roboearth.org/kb/unr_actions.owl#COMPLETED_CONDITION_FALSE"));



		
		
	}
}
