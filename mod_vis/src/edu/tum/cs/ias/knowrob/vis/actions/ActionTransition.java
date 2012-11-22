package edu.tum.cs.ias.knowrob.vis.actions;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Vector;

import javax.vecmath.Vector2f;
import processing.core.PApplet;

import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.OWLIndividual;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.vis.applets.PlanVisAppletFsm;

public class ActionTransition extends OWLIndividual {

	Action from;
	Action to;
	String caused_by;

	Vector2f fromPosition;
	Vector2f toPosition;
	
//	public static enum TransitionType {OK, ERROR, ABORT, OUT_OF_RESOURCES, TIMEOUT};
	
	
	/** 
	 * flag if this transition is 'active', meaning that user 
	 * has clicked on it and it's currently being edited
	 */
	boolean active = false;
	

	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 */
	protected ActionTransition(String iri, String label) {
		
		super(iri, label);
		
	}


	/**
	 * Copy constructor: create ActionTransition from more generic {@link OWLIndividual}
	 * 
	 * @param ind {@link OWLIndividual} to be copied into this {@link ActionTransition}
	 */
	protected ActionTransition(OWLIndividual ind) {
		
		this(ind.getIRI(), ind.getLabel());
		this.types.addAll(ind.getTypes());
		this.data_props.putAll(ind.getDataProperties());
		this.obj_props.putAll(ind.getObjProperties());
	}
	
	
	/**
	 * ActionTransition factory. Return existing instance, if available, and create new
	 * ActionTransition instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link ActionTransition} with the specified IRI
	 */
	public static ActionTransition getActionTransition(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof ActionTransition) {
			return (ActionTransition) identifiers.get(iri);			
		}
		
		// create ActionTransition from higher-level objects if the existing object for this IRI has a more abstract type
		ActionTransition res = new ActionTransition(OWLIndividual.getOWLIndividual(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * ActionTransition factory. Return existing instance, if available, and create new
	 * ActionTransition instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link ActionTransition} with the specified IRI
	 */
	public static ActionTransition getActionTransition(String iri) {
		return getActionTransition(iri, null); 
	}
	
	/**
	 * ActionTransition factory. Return existing instance, if available, and create new
	 * ActionTransition instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link ActionTransition} with the specified IRI
	 */
	public static ActionTransition getActionTransition(String iri, Action from, Action to, String type) {
		ActionTransition res = getActionTransition(iri, null);
		res.setFrom(from);
		res.setTo(to);
		res.setCause(type);
		return res;
	}
	
	
	/**
	 * Read all properties of this action transition from Prolog.
	 */
	/**
	 * Recursively read all properties of this action and all its subactions from Prolog.
	 */
	public void readFromProlog() {	
		
		
		try {
			
			HashMap<String, Vector<String>> qProp = 
				PrologInterface.executeQuery("owl_has('" + iri + "', Prop, Val)");
			
			
			if(qProp != null) {
				
				Vector<String> prop = qProp.get("Prop");
				Vector<String> val  = qProp.get("Val");

				
				// Make sure each property is added only once 
				// (properties may be present two or more times in the result set)
				
				HashSet<String> alreadyAdded = new HashSet<String>();
				if(prop != null && val != null)
					
					for(int i=0;i<prop.size() && i<val.size();i++) {
						
						if (alreadyAdded.contains(prop.get(i)+val.get(i)))
							continue;
						
						alreadyAdded.add(prop.get(i)+val.get(i));
						String p = OWLThing.removeSingleQuotes(prop.get(i));
						String v = OWLThing.removeSingleQuotes(val.get(i));
						
						if(p.endsWith("fromState")) {

							if(!obj_props.containsKey(p))
								obj_props.put(p, new Vector<String>());
							this.obj_props.get(p).add(v);
							
							from = Action.getAction(v);
							
						}else if(p.endsWith("toState")) {

							if(!obj_props.containsKey(p))
								obj_props.put(p, new Vector<String>());
							this.obj_props.get(p).add(v);
							
							to = Action.getAction(v);
							
							
						} else if(p.endsWith("causedBy")) {
							this.caused_by=v;
							
						} else if(p.endsWith("type")) {
							this.types.add(OWLClass.getOWLClass(v));
						}
						
					}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}


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
		
		if(!types.isEmpty())
			return types.firstElement().getShortName();
		
		else return null;
	}


	public void setType(String type) {
		
		if(type.startsWith("http")) {
			this.types.add(OWLClass.getOWLClass(type));
		} else {
			if(OWLThing.getOWLThingByShortname(type) != null)
				this.types.add(OWLClass.getOWLClass(OWLThing.getOWLThingByShortname(type).getIRI()));
		}
	}


	public void drawConnection(PApplet app) {
		
		// default: grey
		if(this.caused_by != null) {
			
			if(this.caused_by.endsWith("OK")) {

				if(active) {
					app.stroke(app.color(60, 140, 60));
					app.fill(app.color(80, 180, 80));
				} else {
					app.stroke(app.color(60, 100, 60));
					app.fill(app.color(80, 120, 80));
				}

			} else if(this.caused_by.endsWith("CONDITION_TRUE")) {
				if(active) {
					app.stroke(app.color(20, 140, 140));  // light blue?
					app.fill(app.color(80, 180, 190));
				} else {
					app.stroke(app.color(60, 100, 100));
					app.fill(app.color(80, 120, 130));
				}

			} else if(this.caused_by.endsWith("CONDITION_FALSE")) {
				if(active) {
					app.stroke(app.color(10, 120, 120));  // dark blue
					app.fill(app.color(40, 140, 150));
				} else {
					app.stroke(app.color(30, 60, 60));
					app.fill(app.color(40, 90, 100));
				}

			} else if(this.caused_by.endsWith("ERROR")) {
				if(active) {
					app.stroke(app.color(130, 30, 30)); // dark red
					app.fill(app.color(140, 70, 70));
				} else {
					app.stroke(app.color(90, 30, 30));
					app.fill(app.color(100, 50, 50));
				}

			} else if(this.caused_by.endsWith("OUT_OF_RESOURCES")) {
				if(active) {
					app.stroke(app.color(140, 20, 20)); // light red
					app.fill(app.color(180, 80, 80));
				} else {
					app.stroke(app.color(100, 60, 60));
					app.fill(app.color(120, 80, 80));
				}

			} else if(this.caused_by.endsWith("ABORT")) {
				if(active) {
					app.stroke(app.color(140, 20, 140));  // purple
					app.fill(app.color(190, 80, 180));
				} else {
					app.stroke(app.color(100, 60, 100));
					app.fill(app.color(130, 80, 120));
				}

			} else if(this.caused_by.endsWith("TIMEOUT")) {
				if(active) {
					app.stroke(app.color(160, 140, 20));  // orange
					app.fill(app.color(220, 180, 80));
				} else {
					app.stroke(app.color(120, 100, 60));
					app.fill(app.color(160, 120, 80));
				}
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
	
	public String getCause() {
		return caused_by;
	}
	public void setCause(String cause) {
		caused_by = cause;
	}
	
	
}
