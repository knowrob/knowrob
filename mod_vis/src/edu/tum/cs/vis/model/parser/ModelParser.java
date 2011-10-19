package edu.tum.cs.vis.model.parser;

import java.util.LinkedList;

import javax.vecmath.Point3f;

import processing.core.PApplet;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Base class for all ModelParsers. Used to parse models from file an draw them onto the Processing applet.
 * 
 * @author Stefan Profanter
 *
 */
public abstract class ModelParser {

	protected LinkedList<Triangle> triangles = new LinkedList<Triangle>();
	protected LinkedList<Line> lines = new LinkedList<Line>();

	private Float minX = null;
	private Float maxX = null;
	private Float minY = null;
	private Float maxY = null;
	private Float minZ = null;
	private Float maxZ = null;
	private Point3f currentPosition = null;

	/**
	 * Draw the triangles list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	protected void drawTriangles(PApplet applet) {

		// Shapes are not effected by translate
		for (Triangle tri : triangles) {

			tri.draw(applet);
		}
	}

	/**
	 * Draw the lines list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	protected void drawLines(PApplet applet) {

		// Shapes are not effected by translate
		for (Line line : lines) {

			line.draw(applet);
		}
	}

	/**
	 * Draws the bounding box around the model with the current style
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void drawBoundingBox(PApplet applet) {
		// Save current translation
		applet.pushMatrix();
		applet.translate(currentPosition.x, currentPosition.y,
				currentPosition.z);
		applet.box(getModelWidth(), getModelHeight(), getModelDepth());

		// Restore last translation
		applet.popMatrix();
	}

	/**
	 * Moves the model so that the center of the bounding box is the point and
	 * further calls to setModelPosition result in the correct position (0,0,0)
	 */
	protected void centerModel() {
		// Initialize min and max variables
		getModelWidth();
		getModelHeight();
		getModelDepth();
		currentPosition = new Point3f(minX + (getModelWidth() / 2), minY
				+ (getModelHeight() / 2), minZ + (getModelDepth() / 2));
		setModelPosition(new Point3f(0, 0, 0));
	}

	/**
	 * 
	 * @param pos
	 */
	protected void setModelPosition(Point3f pos) {
		if (currentPosition == null)
			centerModel();
		for (Triangle tri : triangles) {

			tri.translate(pos.x - currentPosition.x, pos.y - currentPosition.y,
					pos.z - currentPosition.z);
		}
		currentPosition = pos;
	}

	/**
	 * Draw method to draw the model on the applet.
	 * @param applet The applet to draw on.
	 */
	public abstract void draw(PApplet applet);
	
	/**
	 * Returns the height of the model by searching the biggest distance on the
	 * y-axis between the vectors.
	 * 
	 * @return float as height of the model
	 */
	public float getModelHeight() {
		if (minY != null && maxY != null)
			return maxY - minY;
		minY = Float.MAX_VALUE;
		maxY = Float.MIN_VALUE;

		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				minY = Math.min(tri.position[v].y, minY);
				maxY = Math.max(tri.position[v].y, maxY);
			}
		}
		return maxY - minY;
	}

	/**
	 * Returns the width of the model by searching the biggest distance on the
	 * x-axis between the vectors.
	 * 
	 * @return float as width of the model
	 */
	public float getModelWidth() {
		if (minX != null && maxX != null)
			return maxX - minX;
		minX = Float.MAX_VALUE;
		maxX = Float.MIN_VALUE;

		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				minX = Math.min(tri.position[v].x, minX);
				maxX = Math.max(tri.position[v].x, maxX);
			}
		}
		return maxX - minX;
	}

	/**
	 * Returns the depth of the model by searching the biggest distance on the
	 * z-axis between the vectors.
	 * 
	 * @return float as depth of the model
	 */
	public float getModelDepth() {
		if (minZ != null && maxZ != null)
			return maxZ - minZ;
		minZ = Float.MAX_VALUE;
		maxZ = Float.MIN_VALUE;

		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				minZ = Math.min(tri.position[v].z, minZ);
				maxZ = Math.max(tri.position[v].z, maxZ);
			}
		}
		return maxZ - minZ;
	}

}
