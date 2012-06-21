package edu.tum.cs.vis.model.view;

import java.awt.Color;
import java.awt.event.MouseEvent;
import java.util.ArrayList;

import javax.swing.event.MouseInputListener;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import peasy.PeasyCam;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Polygon;

/**
 * Viewing applet for showing the results of the reasoning process.
 * 
 * @author Stefan Profanter
 * 
 */
public final class MeshReasoningView extends PAppletSelection implements MouseInputListener {

	/**
	 * 
	 */
	private static final long			serialVersionUID	= 984696039698156574L;

	/**
	 * Cam for manipulating the view
	 */
	PeasyCam							cam;

	private final Color					bgcolor				= new Color(10, 10, 10);

	private Point3f						rayStart			= new Point3f();
	private Point3f						rayEnd				= new Point3f(1, 1, 1);
	private final Point3f				intersect			= new Point3f(1, 1, 1);

	/**
	 * List of all CASes which were manipulated with AnalysisEngines.
	 */
	private ArrayList<MeshCas>			casList				= new ArrayList<MeshCas>();

	private final ArrayList<Polygon>	selectedPolygons	= new ArrayList<Polygon>();

	@Override
	public void draw() {

		scale(20);
		background(bgcolor.getRed(), bgcolor.getGreen(), bgcolor.getBlue());
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

		/*strokeWeight(5);
		stroke(255, 255, 0);

		synchronized (rayStart) {
			line(rayStart.x, rayStart.y, rayStart.z, rayEnd.x, rayEnd.y, rayEnd.z);
		}
		pushMatrix();

		translate(intersect.x, intersect.y, intersect.z);
		noStroke();
		scale(0.1f);
		fill(0, 0, 255);
		sphere(1);
		popMatrix();

		fill(127);*/

		Vector3f camPos = new Vector3f(cam.getPosition());

		lights();
		pointLight(80f, 80f, 100f, camPos.x, camPos.y, camPos.z);

		// Must be called AFTER all scale, transform, rotate, ... calls
		captureViewMatrix();

		for (MeshCas c : casList) {
			// c.draw(g);
			c.getGroup().draw(g, null);
		}

		synchronized (selectedPolygons) {

			for (Polygon p : selectedPolygons) {
				p.draw(g, new Color(255, 0, 0));
			}
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

	@Override
	public void mouseClicked(MouseEvent e) {
		if (e.getButton() == 1) {
			calculatePickPoints(e.getX(), e.getY());
			if (!isMouseRayValid()) {
				System.out.println("Mouse ray not valid!!!");
				return;
			}
			rayStart = getMouseRayStart();
			rayEnd = getMouseRayEnd();

			synchronized (selectedPolygons) {
				selectedPolygons.clear();
				for (MeshCas c : casList) {
					c.getGroup().getIntersectedPolygons(rayEnd, rayStart, selectedPolygons,
							intersect);
				}
			}
			System.out.println("Tr: " + selectedPolygons.size());
		}

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

		cam.setDistance(10);

		cam.rotateX((float) Math.PI / 2f);

		captureViewMatrix();

		perspective();

		draw();
	}
}
