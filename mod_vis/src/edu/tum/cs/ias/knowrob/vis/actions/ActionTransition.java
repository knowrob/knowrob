package edu.tum.cs.ias.knowrob.vis.actions;

import java.awt.Color;

import processing.core.PApplet;

import edu.tum.cs.ias.knowrob.vis.gui.applets.PlanVisAppletFsm;

public class ActionTransition {

	Action from;
	Action to;
	String transitionType;

	
	public ActionTransition(Action from, Action to, String type) {
		this.from = from;
		this.to = to;
		this.transitionType = type;
	}


	public Action getFrom() {
		return from;
	}


	public void setFrom(Action from) {
		this.from = from;
	}


	public Action getTo() {
		return to;
	}


	public void setTo(Action to) {
		this.to = to;
	}


	public String getType() {
		return transitionType;
	}


	public void setType(String type) {
		this.transitionType = type;
	}


	public void drawConnection(PApplet app) {
		
		// default: grey

		if(this.transitionType.endsWith("OK")) {
			app.stroke(app.color(60, 100, 60));
			app.fill(app.color(80, 120, 80));
			
		} else if(this.transitionType.endsWith("ERROR")) {
			app.stroke(app.color(100, 60, 60));
			app.fill(app.color(120, 80, 80));
			
		} else if(this.transitionType.endsWith("TIMEOUT")) {
			app.stroke(app.color(100, 100, 60));
			app.fill(app.color(130, 120, 80));
			
		} else {
			app.stroke(app.color(80));
			app.fill(app.color(100));
		}
		
		
		PlanVisAppletFsm.arrowFromTo(app, this.from.getDrawInfo().getOutboundConnectorPos(), to.getDrawInfo().getInboundConnectorPos(), 5, -1);
		
	}
	
}
