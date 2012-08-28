package edu.tum.cs.ias.knowrob.vis.actions;

import javax.vecmath.Vector2f;

import processing.core.PApplet;

import edu.tum.cs.ias.knowrob.vis.gui.applets.PlanVisAppletFsm;

public class ActionTransition {

	Action from;
	Action to;
	String transitionType;

	Vector2f fromPosition;
	Vector2f toPosition;
	
	/** 
	 * flag if this transition is 'active', meaning that user 
	 * has clicked on it and it's currently being edited
	 */
	boolean active = false;
	
	
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

			if(active) {
				app.stroke(app.color(60, 140, 60));
				app.fill(app.color(80, 180, 80));
			} else {
				app.stroke(app.color(60, 100, 60));
				app.fill(app.color(80, 120, 80));
			}

		} else if(this.transitionType.endsWith("ERROR")) {
			if(active) {
				app.stroke(app.color(140, 20, 20));
				app.fill(app.color(180, 80, 80));
			} else {
				app.stroke(app.color(100, 60, 60));
				app.fill(app.color(120, 80, 80));
			}

		} else if(this.transitionType.endsWith("TIMEOUT")) {
			if(active) {
				app.stroke(app.color(140, 140, 20));
				app.fill(app.color(190, 180, 80));
			} else {
				app.stroke(app.color(100, 100, 60));
				app.fill(app.color(130, 120, 80));
			}

		} else {
			if(active) {
				app.stroke(app.color(120));
				app.fill(app.color(150));
			} else {
				app.stroke(app.color(80));
				app.fill(app.color(100));
			}
		}
		
		this.fromPosition = this.from.getDrawInfo().getOutboundConnectorPos();
		this.toPosition   = new Vector2f(to.getDrawInfo().getInboundConnectorPos().x,
										 to.getDrawInfo().getInboundConnectorPos().y-5);
		
		PlanVisAppletFsm.arrowFromTo(app, fromPosition, toPosition, 5, -1);
		
	}
	
	public boolean checkPosInArrow(Vector2f pos, PApplet app) {

		Vector2f[] vertices = PlanVisAppletFsm.getArrowVertices(app, fromPosition, toPosition, 5f, -1f).toArray(new Vector2f[]{});
		
		int i, j=vertices.length-1;
		int sides = vertices.length;
		boolean oddNodes = false;
		
		
		for (i=0; i<sides; i++) {
		
			if ((vertices[i].y < pos.y && vertices[j].y >= pos.y || 
				 vertices[j].y < pos.y && vertices[i].y >= pos.y) && (vertices[i].x <= pos.x || 
				 vertices[j].x <= pos.x)) {
				
				oddNodes^=(vertices[i].x + (pos.y-vertices[i].y)/(vertices[j].y - vertices[i].y)*(vertices[j].x-vertices[i].x)<pos.x);
			}
			j=i;
		}
		return oddNodes;

	}

	public boolean isActive() {
		return active;
	}

	public void setActive(boolean b) {
		active = b;
	}

	public void toggleActive() {
		active = !active;
	}
}
