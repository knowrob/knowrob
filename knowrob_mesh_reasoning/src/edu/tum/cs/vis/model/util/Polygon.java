package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import processing.core.PApplet;
import processing.core.PConstants;
import edu.tum.cs.vis.model.uima.analyzer.FlatSurfaceAnalyzer;

/**
 * DrawObject which represents a polygon (object with more than 2 points, eg. Triangle).
 * 
 * @author Stefan Profanter
 * 
 */
public class Polygon extends DrawObject {

	private static Logger			logger			= Logger.getLogger(FlatSurfaceAnalyzer.class);

	/**
	 * Texture-Points
	 */
	protected Point2f				texPosition[];

	/**
	 * Polygons may have normal vector
	 */
	protected Vector3d				normalVector	= null;

	/**
	 * Centroid of polygon
	 */
	protected Point3d				centroid;

	/**
	 * List of all direct neighbor polygons
	 */
	protected ArrayList<Polygon>	neighbors;

	/**
	 * Add a neighbor to the neighbors list. If list contains already neighbor this method doesn't
	 * add it again. If not, the neighbor will be added and this instance will be added as a
	 * neighbor in the given neighbor instance.
	 * 
	 * @param neighbor
	 *            neighbor to add.
	 */
	public void addNeighbor(Polygon neighbor) {
		if (neighbors == null)
			neighbors = new ArrayList<Polygon>();
		if (neighbors.contains(neighbor))
			return;

		boolean add = false;
		for (Point3f p1 : position) {
			for (Point3f p2 : neighbor.position) {
				if (p1.equals(p2))
					add = true;
			}
		}
		if (add) {
			synchronized (this) {
				neighbors.add(neighbor);
			}
			synchronized (neighbor) {
				neighbor.addNeighbor(this);
			}
		}
	}

	/**
	 * Draw the polygons onto the applet.
	 * 
	 * @param applet
	 *            Applet to draw on
	 * @param overrideColor
	 *            overrides the color to draw
	 */
	public void draw(PApplet applet, Color overrideColor) {
		applyColor(applet, overrideColor);
		if (appearance.getImageReference() == null || overrideColor != null) {
			// no texture only color
			applet.beginShape(PConstants.TRIANGLES);

			for (int i = 0; i < position.length; i++)
				applet.vertex(position[i].x, position[i].y, position[i].z);

			applet.endShape();

		} else {
			// has texture
			applet.beginShape(PConstants.TRIANGLES);
			applet.texture(appearance.getImageReference());

			for (int i = 0; i < position.length; i++)
				applet.vertex(position[i].x, position[i].y, position[i].z, texPosition[i].x,
						texPosition[i].y);

			applet.endShape();

		}
		/*applet.stroke(255,0,0);
			
		applet.strokeWeight(5);
		applet.line(centroid.x, centroid.y, centroid.z, normalVector.x+centroid.x,
				normalVector.y+centroid.y, normalVector.z+centroid.z);*/
	}

	public float getArea() {
		if (position.length == 3) {

			Point3f v = (Point3f) position[1].clone();
			v.sub(position[0]);
			Point3f w = (Point3f) position[2].clone();
			w.sub(position[0]);

			Vector3f v1 = new Vector3f(v);
			Vector3f w1 = new Vector3f(w);

			Vector3f cross = new Vector3f();
			cross.cross(v1, w1);
			return cross.length() / 2f;
		}

		logger.error("getArea not implemented for " + position.length + "-Polygon");
		return 0;
	}

	/**
	 * Get list of all direct neighbor polygons.
	 * 
	 * @return list of polygons
	 * @see Polygon#isNeighbor
	 */
	public ArrayList<Polygon> getNeighbors() {
		return neighbors;
	}

	/**
	 * @return the normalVector
	 */
	public Vector3d getNormalVector() {
		return normalVector;
	}

	/**
	 * @return the texPosition
	 */
	public Point2f[] getTexPosition() {
		return texPosition;
	}

	/**
	 * Checks if <code>tr</code> is in the list of neighbors. If not the points of <code>this</code>
	 * and <code>tr</code> are compared. If one of them is equal to each other <code>true</code>
	 * will be returned.
	 * 
	 * @param tr
	 *            polygon to check if it is a neighbor
	 * @return true if <code>tr</code> is a neighbor
	 */
	public boolean isNeighbor(Polygon tr) {
		if (neighbors != null && neighbors.contains(tr))
			return true;
		for (Point3f p1 : position) {
			for (Point3f p2 : tr.position) {
				if (p1.equals(p2))
					return true;
			}
		}
		return false;
	}

	/**
	 * Set the list of direct neighbor polygons.
	 * 
	 * @param neighbors
	 *            polygons list.
	 */
	public void setNeighbors(ArrayList<Polygon> neighbors) {
		this.neighbors = neighbors;
	}

	/**
	 * @param normalVector
	 *            the normalVector to set
	 */
	public void setNormalVector(Vector3d normalVector) {
		this.normalVector = normalVector;
	}

	/**
	 * @param texPosition
	 *            the texPosition to set
	 */
	public void setTexPosition(Point2f[] texPosition) {
		this.texPosition = texPosition;
	}

	@Override
	public void updateNormalVector() {
		/* Uses Newell's method to calculate normal vector */
		normalVector = new Vector3d(0, 0, 0);
		centroid = new Point3d(0, 0, 0);

		for (int i = 0; i < position.length; i++) {
			Vector3f current = new Vector3f(position[i].x, position[i].y, position[i].z);
			Vector3f next = new Vector3f(position[(i + 1) % position.length].x, position[(i + 1)
					% position.length].y, position[(i + 1) % position.length].z);

			normalVector.x += (current.y - next.y) * (current.z + next.z);
			normalVector.y += (current.z - next.z) * (current.x + next.x);
			normalVector.z += (current.x - next.x) * (current.y + next.y);

			centroid.x += position[i].x;
			centroid.y += position[i].y;
			centroid.z += position[i].z;
		}
		normalVector.normalize();
		centroid.x /= position.length;
		centroid.y /= position.length;
		centroid.z /= position.length;

	}

}
