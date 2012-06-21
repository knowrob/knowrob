package edu.tum.cs.vis.model.view;

import java.util.ArrayList;

import javax.vecmath.Vector3f;

import peasy.PeasyCam;
import processing.core.PApplet;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Viewing applet for showing the results of the reasoning process.
 * 
 * @author Stefan Profanter
 * 
 */
public final class MeshReasoningView extends PApplet {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= 984696039698156574L;

	/**
	 * Cam for manipulating the view
	 */
	private PeasyCam			cam;

	/**
	 * List of all CASes which were manipulated with AnalysisEngines.
	 */
	private ArrayList<MeshCas>	casList				= new ArrayList<MeshCas>();

	@Override
	public void draw() {

		background(16);
		// draw axis
		noFill();
		strokeWeight(1);
		stroke(125, 0, 0);
		strokeWeight(1);
		line(0, 0, 0, width, 0, 0);
		stroke(0, 125, 0);
		line(0, 0, 0, 0, 0, width);
		stroke(0, 0, 125);
		line(0, 0, 0, 0, height, 0);

		noStroke();

		fill(127);

		Vector3f camPos = new Vector3f(cam.getPosition());

		lights();
		pointLight(80f, 80f, 100f, camPos.x, camPos.y, camPos.z);

		scale(20);

		for (MeshCas c : casList) {
			c.draw(this);
		}
	}

	/**
	 * Get list of all CAS objects (see UIMA Framework)
	 * 
	 * @return ArrayList of MeshCas
	 */
	public ArrayList<MeshCas> getCasList() {
		return casList;
	}

	/**
	 * Set list of all CAS objects (see UIMA Framework)
	 * 
	 * @param casList
	 *            list to set
	 */
	public void setCasList(ArrayList<MeshCas> casList) {
		this.casList = casList;
	}

	@Override
	public void setup() {
		size(1000, 1000, P3D);

		frameRate(500);
		cam = new PeasyCam(this, 0, 0, 0, 10);
		cam.setMinimumDistance(0.01);
		cam.setMaximumDistance(500);

		cam.setRightDragHandler(cam.getPanDragHandler());
		cam.setLeftDragHandler(cam.getRotateDragHandler());

		cam.setDistance(40);

		cam.rotateX((float) Math.PI / 2f);

		draw();
	}
}
