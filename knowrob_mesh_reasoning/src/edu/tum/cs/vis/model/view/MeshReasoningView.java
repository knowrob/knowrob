/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashSet;

import javax.swing.event.MouseInputListener;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import peasy.PeasyCam;
import processing.core.PConstants;
import processing.core.PGraphics;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.IntersectedTriangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Viewing applet for showing the results of the reasoning process.
 * 
 * Supported keys: + Increase scale, - Decrease scale
 * 
 * @author Stefan Profanter
 * 
 */
public final class MeshReasoningView extends PAppletSelection implements MouseInputListener,
		KeyListener {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 984696039698156574L;

	/**
	 * Draw a cone or cylinder with given properties on graphics context
	 * 
	 * @param g
	 *            graphics context
	 * @param sides
	 *            number of sides to use (indicates the level of detail to draw cone)
	 * @param r1
	 *            bottom radius
	 * @param r2
	 *            top radius
	 * @param h
	 *            height
	 * @param top
	 *            draw top cap
	 * @param bottom
	 *            draw bottom cap
	 */
	public static void drawCylinder(PGraphics g, int sides, float r1, float r2, float h,
			boolean top, boolean bottom) {
		float angle = (float) (2 * Math.PI / sides);
		float halfHeight = h / 2;
		if (top) {
			// top
			g.beginShape();
			for (int i = 0; i < sides; i++) {
				float x = (float) (Math.cos(i * angle) * r1);
				float y = (float) (Math.sin(i * angle) * r1);
				g.vertex(x, y, -halfHeight);
			}
			g.endShape(PConstants.CLOSE);
		}
		if (bottom) {
			// bottom
			g.beginShape();
			for (int i = 0; i < sides; i++) {
				float x = (float) (Math.cos(i * angle) * r2);
				float y = (float) (Math.sin(i * angle) * r2);
				g.vertex(x, y, halfHeight);
			}
			g.endShape(PConstants.CLOSE);
		}
		// draw body
		g.beginShape(PConstants.TRIANGLE_STRIP);
		for (int i = 0; i < sides + 1; i++) {
			float x1 = (float) (Math.cos(i * angle) * r1);
			float y1 = (float) (Math.sin(i * angle) * r1);
			float x2 = (float) (Math.cos(i * angle) * r2);
			float y2 = (float) (Math.sin(i * angle) * r2);
			g.vertex(x1, y1, -halfHeight);
			g.vertex(x2, y2, halfHeight);
		}
		g.endShape(PConstants.CLOSE);
	}

	/**
	 * Cam for manipulating the view
	 */
	PeasyCam										cam;
	/**
	 * Background color dark
	 */
	private final Color								bgcolorDark			= new Color(10, 10, 10);

	/**
	 * Background color white
	 */
	private final Color								bgcolorWhite		= new Color(255, 255, 255);

	/**
	 * Should background be white
	 */
	private boolean									backgroundWhite		= false;
	/**
	 * draw normals for each vertex?
	 */
	private boolean									drawVertexNormals	= false;
	/**
	 * draw curvature properties for each vertex?
	 */
	private boolean									drawVertexCurvature	= false;

	/**
	 * draw voronoi area for each vertex?
	 */
	private boolean									drawVoronoiArea		= false;
	/**
	 * Select only nearest triangle or all intersecting with mouse ray?
	 */
	private boolean									selectNearestOnly	= true;
	/**
	 * draw bounding box for each group?
	 */
	private boolean									drawBoundingBox		= false;

	/**
	 * Start point of mouse click ray
	 */
	private Point3f									rayStart			= new Point3f();

	/**
	 * End point of mouse click ray
	 */
	private Point3f									rayEnd				= new Point3f(1, 1, 1);

	/**
	 * List of all CASes which were manipulated with AnalysisEngines.
	 */
	private ArrayList<MeshCas>						casList				= new ArrayList<MeshCas>();

	/**
	 * current scale factor
	 */
	private float									modelScale			= 1f;

	/**
	 * User can manually scale model by pressing '+' or '-'
	 */
	private float									userScale			= 30f;

	/**
	 * Path where to save image if user clicks on "save image". Will be evaluated in draw method
	 */
	private String									imageSavePath		= null;

	/**
	 * List of selected triangles (triangles which intersect with mouse ray)
	 */
	private final ArrayList<IntersectedTriangle>	selectedTriangles	= new ArrayList<IntersectedTriangle>();

	/**
	 * List of selected annotations (annotations which contain one of selectedTriangles)
	 * */
	@SuppressWarnings("rawtypes")
	private final HashSet<MeshAnnotation>			selectedAnnotations	= new HashSet<MeshAnnotation>();

	/**
	 * The controller for this view
	 */
	private MeshReasoningViewControl				control				= null;

	/**
	 * Add annotation to selected annotations
	 * 
	 * @param a
	 *            annotation to add
	 */
	public void addSelectedAnnotation(@SuppressWarnings("rawtypes") MeshAnnotation a) {
		synchronized (selectedAnnotations) {
			selectedAnnotations.add(a);
		}
		control.showSelectedAnnotation(selectedAnnotations);
	};

	/**
	 * Remove / unselect all selected annotations
	 */
	public void clearSelectedAnnotations() {
		synchronized (selectedAnnotations) {
			selectedAnnotations.clear();
		}
		control.showSelectedAnnotation(selectedAnnotations);
	}

	@Override
	public void draw() {

		scale(modelScale * userScale);
		if (backgroundWhite)
			background(bgcolorWhite.getRed(), bgcolorWhite.getGreen(), bgcolorWhite.getBlue());
		else
			background(bgcolorDark.getRed(), bgcolorDark.getGreen(), bgcolorDark.getBlue());
		// draw axis
		noFill();
		strokeWeight(1);
		stroke(125, 0, 0);
		strokeWeight(1);
		line(0, 0, 0, 1, 0, 0);
		stroke(0, 125, 0);
		line(0, 0, 0, 0, 1, 0);
		stroke(0, 0, 125);
		line(0, 0, 0, 0, 0, 1);

		Vector3f camPos = new Vector3f(cam.getPosition());

		lights();
		pointLight(80f, 80f, 100f, camPos.x, camPos.y, camPos.z);

		// Must be called AFTER all scale, transform, rotate, ... calls
		captureViewMatrix();

		getSelectionGraphics().setDrawWithTransparency(selectedAnnotations.size() > 0);

		for (MeshCas c : casList) {
			c.draw(g);
			if (drawBoundingBox) {
				g.noFill();
				g.stroke(255, 125, 0);
				c.getModel().getGroup().drawBoundingBox(g, true);
			}

		}

		getSelectionGraphics().setDrawWithTransparency(false);

		synchronized (selectedAnnotations) {

			for (@SuppressWarnings("rawtypes")
			MeshAnnotation ma : selectedAnnotations) {
				ma.getMesh().drawTriangles(
						g,
						selectedAnnotations.size() > 1 ? new Color(255, 125, 0, 200) : new Color(
								255, 50, 0, 200));
			}

			if (selectedAnnotations.size() == 1) {
				@SuppressWarnings("rawtypes")
				MeshAnnotation a = selectedAnnotations.iterator().next();

				if (a instanceof PrimitiveAnnotation) {
					@SuppressWarnings("rawtypes")
					PrimitiveAnnotation an = (PrimitiveAnnotation) a;

					an.drawPrimitiveAnnotation(g);

				}
			}

		}

		if (drawVertexNormals || drawVertexCurvature || drawVoronoiArea) {
			g.strokeWeight(2f);
			for (MeshCas c : casList) {
				for (Vertex v : c.getModel().getVertices()) {
					if (drawVertexNormals || drawVoronoiArea) {
						g.stroke(41, 120, 37);
						Vector3f n = (Vector3f) v.getNormalVector().clone();
						n.scale(0.05f);
						g.line(v.x, v.y, v.z, v.x + n.x, v.y + n.y, v.z + n.z);
						g.fill(35, 148, 143);
						g.noStroke();
						g.sphereDetail(20);
						if (drawVoronoiArea) {
							g.pushMatrix();
							g.translate(v.x + n.x, v.y + n.y, v.z + n.z);
							g.sphere(v.getPointarea());
							g.popMatrix();
						}
					}
					if (drawVertexCurvature) {
						Curvature curv = c.getCurvature(v);

						g.stroke(0, 72, 153);

						Vector3f max = (Vector3f) curv.getPrincipleDirectionMax().clone();
						max.scale(curv.getCurvatureMax() / 20f);
						g.line(v.x, v.y, v.z, v.x + max.x, v.y + max.y, v.z + max.z);

						g.stroke(243, 146, 0);
						Vector3f min = (Vector3f) curv.getPrincipleDirectionMin().clone();
						min.scale(curv.getCurvatureMax() / 20f);
						g.line(v.x, v.y, v.z, v.x + min.x, v.y + min.y, v.z + min.z);
					}
				}
			}
		}

		// Check if user wants to save current view
		if (imageSavePath != null) {
			save(imageSavePath);
			FileWriter outFile;
			try {
				outFile = new FileWriter(imageSavePath + ".txt");
				PrintWriter out = new PrintWriter(outFile);
				String current = Math.round(getRotation()[0] * 180f / Math.PI) + ","
						+ Math.round(getRotation()[1] * 180f / Math.PI) + ","
						+ Math.round(getRotation()[2] * 180f / Math.PI);
				out.println("View angles: " + current);

				out.close();
			} catch (IOException e) {
				e.printStackTrace();
			}

			Logger.getRootLogger().info("Image saved as: " + imageSavePath);
			imageSavePath = null;
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
	 * @return the control
	 */
	public MeshReasoningViewControl getControl() {
		return control;
	}

	/**
	 * get current cam rotation
	 * 
	 * @return rotation for each axis
	 */
	public float[] getRotation() {
		return cam.getRotations();
	}

	/**
	 * get current model scale
	 * 
	 * @return model scale
	 */
	public float getScale() {
		return modelScale;
	}

	/**
	 * Should background be drawn white?
	 * 
	 * @return the backgroundWhite
	 */
	public boolean isBackgroundWhite() {
		return backgroundWhite;
	}

	/**
	 * Should bounding box be drawn for each group?
	 * 
	 * @return the drawBoundingBox
	 */
	public boolean isDrawBoundingBox() {
		return drawBoundingBox;
	}

	/**
	 * Should curvature properties for each vertex be drawn?
	 * 
	 * @return the drawVertexCurvature
	 */
	public boolean isDrawVertexCurvature() {
		return drawVertexCurvature;
	}

	/**
	 * Should vertex normals be drawn
	 * 
	 * @return the drawVertexNormals
	 */
	public boolean isDrawVertexNormals() {
		return drawVertexNormals;
	}

	/**
	 * Should voronoi area be drawn?
	 * 
	 * @return the drawVoronoiArea
	 */
	public boolean isDrawVoronoiArea() {
		return drawVoronoiArea;
	}

	/**
	 * Should only the nearest triangle or all triangles intersecting mouse ray be selected?
	 * 
	 * @return the selectNearestOnly
	 */
	public boolean isSelectNearestOnly() {
		return selectNearestOnly;
	}

	@Override
	public void keyTyped(KeyEvent e) {
		char c = e.getKeyChar();
		if (c == '+') {
			userScale *= 1.5;
		} else if (c == '-') {
			userScale = userScale / 1.5f;
		} else if (c == ',') {
			cam.setDistance(cam.getDistance() * 0.8, 250);
		} else if (c == '.') {
			cam.setDistance(cam.getDistance() * 1.2, 250);
		}/* else if (c == 'm') {
			test++;
			} else if (c == 'n') {
			test--;
			}
			System.out.println("Test: " + test);*/
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

			boolean found = false;
			// Check if clicked on one of previous selected triangles
			for (IntersectedTriangle p : selectedTriangles) {
				Point3f newIntersect = new Point3f();
				if (p.t.intersectsRay(rayEnd, rayStart, newIntersect)) {
					selectedTriangles.clear();
					p.intersection = newIntersect;
					selectedTriangles.add(p);
					found = true;
					break;
				}
			}

			if (!found) {
				// It is new selection
				selectedTriangles.clear();
				for (MeshCas c : casList) {
					if (c.getModel() == null)
						continue;
					c.getModel().getGroup()
							.getIntersectedTriangles(rayEnd, rayStart, selectedTriangles);
				}
			}

			// Check if one of selected triangles is in already selected annotation
			ArrayList<IntersectedTriangle> newSelected = new ArrayList<IntersectedTriangle>();
			for (IntersectedTriangle p : selectedTriangles) {
				synchronized (selectedAnnotations) {
					for (@SuppressWarnings("rawtypes")
					MeshAnnotation ma : selectedAnnotations)
						if (ma.meshContainsTriangle(p.t)) {
							newSelected.add(p);
						}
				}
			}
			if (newSelected.size() > 0) {
				// Currently selected was in one or more of the selected annotations, so select
				// out
				// of current annotations
				selectedTriangles.clear();
				selectedTriangles.addAll(newSelected);
			}

			if (selectNearestOnly && selectedTriangles.size() > 1) {
				IntersectedTriangle nearest = null;
				float nearestDist = Float.MAX_VALUE;
				for (IntersectedTriangle p : selectedTriangles) {
					Vector3f camPos = new Vector3f(cam.getPosition());
					camPos.sub(p.intersection);
					float dist = camPos.lengthSquared();
					if (dist < nearestDist) {
						nearestDist = dist;
						nearest = p;
					}
				}
				selectedTriangles.clear();
				selectedTriangles.add(nearest);
			}

			selectedTrianglesChanged();
		}

	}

	/**
	 * Saves the current viewable context into a PNG image file with the given filename. If filename
	 * doesn't end with '.png', it will be added.
	 * 
	 * @param filename
	 *            Filename for the image to save
	 */
	public void saveImage(String filename) {
		String path = filename;
		if (!path.endsWith(".png"))
			path += ".png";

		if (!(new File(path).isAbsolute()))
			path = new File(sketchPath + "/images", path).getAbsolutePath();

		String basePath = path;
		int num = 0;

		while (new File(path).exists()) {
			num++;
			path = basePath.substring(0, basePath.lastIndexOf('.')) + "-" + num + ".png";

		}

		imageSavePath = path;

	}

	/**
	 * Called to update list of selectedAnnotations after list of selectedTriangless has changed
	 */
	private void selectedTrianglesChanged() {
		synchronized (selectedAnnotations) {
			selectedAnnotations.clear();
		}
		for (MeshCas c : casList) {
			for (Annotation a : c.getAnnotations()) {
				if (!(a instanceof DrawableAnnotation))
					continue;
				@SuppressWarnings("rawtypes")
				MeshAnnotation ma = (MeshAnnotation) a;
				if (!ma.isDrawAnnotation())
					continue; // Skip not visible annotations
				for (IntersectedTriangle p : selectedTriangles)
					if (ma.meshContainsTriangle(p.t)) {
						synchronized (selectedAnnotations) {
							selectedAnnotations.add(ma);
						}
						break;
					}
			}
		}
		control.showSelectedAnnotation(selectedAnnotations);

	}

	/**
	 * Set to true if background should be white
	 * 
	 * @param backgroundWhite
	 *            the backgroundWhite to set
	 */
	public void setBackgroundWhite(boolean backgroundWhite) {
		this.backgroundWhite = backgroundWhite;
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

	/**
	 * Sets control panel for this view
	 * 
	 * @param control
	 *            the control to set
	 */
	public void setControl(MeshReasoningViewControl control) {
		this.control = control;
	}

	/**
	 * Should bounding box be drawn for each group?
	 * 
	 * @param drawBoundingBox
	 *            the drawBoundingBox to set
	 */
	public void setDrawBoundingBox(boolean drawBoundingBox) {
		this.drawBoundingBox = drawBoundingBox;
	}

	/**
	 * Should model colored by curvature?
	 * 
	 * @param drawCurvatureColor
	 *            true if color by curvature
	 */
	public void setDrawCurvatureColor(boolean drawCurvatureColor) {

		for (MeshCas c : casList) {
			for (Vertex v : c.getModel().getVertices()) {
				if (drawCurvatureColor)
					v.overrideColor = c.getCurvature(v).getColor();
				else
					v.overrideColor = null;
			}
		}
	}

	/**
	 * 
	 * @param drawVertexCurvature
	 *            the drawVertexCurvature to set
	 */
	public void setDrawVertexCurvature(boolean drawVertexCurvature) {
		this.drawVertexCurvature = drawVertexCurvature;
	}

	/**
	 * @param drawVertexNormals
	 *            the drawVertexNormals to set
	 */
	public void setDrawVertexNormals(boolean drawVertexNormals) {
		this.drawVertexNormals = drawVertexNormals;
	}

	/**
	 * @param drawVoronoiArea
	 *            the drawVoronoiArea to set
	 */
	public void setDrawVoronoiArea(boolean drawVoronoiArea) {
		this.drawVoronoiArea = drawVoronoiArea;
	}

	/**
	 * Set manual rotation of camera
	 * 
	 * @param pitch
	 *            rotation around x
	 * @param yaw
	 *            rotation around y
	 * @param roll
	 *            rotation around z
	 */
	public void setManualRotation(float pitch, float yaw, float roll) {
		cam.setRotations(pitch, yaw, roll);
	}

	/**
	 * Set model scale to scale model manually
	 * 
	 * @param modelScale
	 *            new scale factor
	 */
	public void setScale(float modelScale) {
		this.modelScale = modelScale;
	}

	/**
	 * @param selectNearestOnly
	 *            the selectNearestOnly to set
	 */
	public void setSelectNearestOnly(boolean selectNearestOnly) {
		this.selectNearestOnly = selectNearestOnly;
	}

	@Override
	public void setup() {
		size(1000, 1000, "edu.tum.cs.vis.model.view.PAppletSelectionGraphics");

		frameRate(10);
		cam = new PeasyCam(this, 0, 0, 0, 10);
		cam.setMinimumDistance(0.01);
		cam.setMaximumDistance(500);

		cam.setRightDragHandler(cam.getPanDragHandler());

		cam.setDistance(50);

		cam.rotateX((float) Math.PI / 2f);
		cam.rotateZ((float) Math.PI);

		captureViewMatrix();

		perspective();

		draw();
	}
}
