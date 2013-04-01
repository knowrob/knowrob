/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;

/**
 * Represents a Mesh-Group with a Name. A group can multiple child groups so it will present a mesh
 * hierarchy.
 * 
 * @author Stefan Profanter
 * 
 */
public class Group implements Serializable {

	/**
	 * parent model of this group
	 */
	private final Model			model;

	/**
	 * Auto generated
	 */
	private static final long	serialVersionUID	= -3347910663268579041L;
	/**
	 * Minimum x-position of group
	 */
	protected Float				minX				= null;
	/**
	 * Maximum x-position of group
	 */
	protected Float				maxX				= null;
	/**
	 * Minimum x-position of group
	 */
	protected Float				minY				= null;
	/**
	 * Maximum x-position of group
	 */
	protected Float				maxY				= null;
	/**
	 * Minimum x-position of group
	 */
	protected Float				minZ				= null;
	/**
	 * Maximum x-position of group
	 */
	protected Float				maxZ				= null;

	/**
	 * Holds the Mesh for this group
	 */
	private Mesh				mesh				= new Mesh();

	/**
	 * Name of this group
	 */
	private String				name;

	/**
	 * Child groups
	 */
	private ArrayList<Group>	children			= new ArrayList<Group>();

	/**
	 * Constructor for group.
	 * 
	 * @param parent
	 *            Parent model of this group.
	 */
	public Group(Model parent) {
		model = parent;
	}

	/**
	 * Add a child to this group
	 * 
	 * @param g
	 *            group to add as a child of this
	 */
	public void addChild(Group g) {
		children.add(g);
	}

	/**
	 * Draw method to draw the model on the applet.
	 * 
	 * @param g
	 *            The applet to draw on.
	 * @param drawSettings
	 *            override the draw color, texture (and other settings). Draw whole object in the
	 *            given color if != null
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		mesh.drawLines(g, drawSettings);
		mesh.drawTriangles(g, drawSettings);
		for (Group gr : children) {
			gr.draw(g, drawSettings);
		}
	}

	/**
	 * Remove triangle from group or child group.
	 * 
	 * @param t
	 *            triangle to remove.
	 * @return true if triangle is found and removed.
	 */
	public boolean removeTriangle(Triangle t) {
		synchronized (mesh.getTriangles()) {
			if (mesh.getTriangles().remove(t))
				return true;
		}
		for (Group gr : children) {
			boolean ret = gr.removeTriangle(t);
			if (ret)
				return ret;
		}
		return false;
	}

	/**
	 * Remove all triangles from group or child group.
	 * 
	 * @param t
	 *            triangles to remove.
	 */
	public void removeTriangle(Collection<Triangle> t) {
		synchronized (mesh.getTriangles()) {
			mesh.getTriangles().removeAll(t);
		}
		for (Group gr : children) {
			gr.removeTriangle(t);
		}
	}

	/**
	 * Draws the bounding box around the model with the current style
	 * 
	 * @param gr
	 *            graphics context
	 * 
	 * @param recursive
	 *            Draw also Bounding-Box of children
	 */
	public void drawBoundingBox(PGraphics gr, boolean recursive) {
		mesh.drawBoundingBox(gr);
		if (recursive) {
			for (Group g : children) {
				g.drawBoundingBox(gr, recursive);
			}
		}
	}

	/**
	 * Add triangle to the mesh of this group and also to parent model.
	 * 
	 * @param t
	 *            triangle to add.
	 */
	public void addTriangle(Triangle t) {
		synchronized (mesh.getTriangles()) {
			mesh.getTriangles().add(t);
		}
		synchronized (model.getTriangles()) {
			model.getTriangles().add(t);
		}
	}

	/**
	 * Get all direct child groups
	 * 
	 * @return list with all child groups
	 */
	public ArrayList<Group> getChildren() {
		return children;
	}

	/**
	 * Searches all triangles which intersect the given ray (rayStart, rayEnd) and adds them to
	 * intersectedTriangles. Not only the segment between rayStart and rayEnd is checked but the
	 * whole ray from -infinity to +infinity.
	 * 
	 * @param rayStart
	 *            start point of the ray.
	 * @param rayEnd
	 *            end point of the ray.
	 * @param intersectedTriangles
	 *            list where to add intersecting triangles
	 */
	public void getIntersectedTriangles(final Point3f rayStart, final Point3f rayEnd,
			final ArrayList<IntersectedTriangle> intersectedTriangles) {
		mesh.getIntersectedTriangles(rayStart, rayEnd, intersectedTriangles);

		for (Group g : children) {
			g.getIntersectedTriangles(rayStart, rayEnd, intersectedTriangles);
		}

	}

	/**
	 * Add all triangles enclosed by this group and all child groups to the <code>triangles</code>
	 * collection.
	 * 
	 * @param triangles
	 *            collection where all the triangles are added.
	 */
	public void getAllTriangles(final Collection<Triangle> triangles) {
		mesh.getAllTriangles(triangles);

		for (Group g : children) {
			g.getAllTriangles(triangles);
		}
	}

	/**
	 * Set maximum x coordinate of this group. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum x
	 */
	public Float getMaxX() {
		if (maxX == null)
			getTotalWidth();
		return maxX;
	}

	/**
	 * Set maximum y coordinate of this group. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum y
	 */
	public Float getMaxY() {
		if (maxY == null)
			getTotalHeight();
		return maxY;
	}

	/**
	 * Set maximum z coordinate of this group. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum z
	 */
	public Float getMaxZ() {
		if (maxZ == null)
			getTotalDepth();
		return maxZ;
	}

	/**
	 * Get the mesh containing triangles and lines
	 * 
	 * @return the mesh
	 */
	public Mesh getMesh() {
		return mesh;
	}

	/**
	 * Set minimum x coordinate of this group. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum x
	 */
	public Float getMinX() {
		if (minX == null)
			getTotalWidth();
		return minX;
	}

	/**
	 * Set minimum y coordinate of this group. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum y
	 */
	public Float getMinY() {
		if (minY == null)
			getTotalHeight();
		return minY;
	}

	/**
	 * Set minimum z coordinate of this group. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum z
	 */
	public Float getMinZ() {
		if (minZ == null)
			getTotalDepth();
		return minZ;
	}

	/**
	 * Get parent model.
	 * 
	 * @return the model
	 */
	public Model getModel() {
		return model;
	}

	/**
	 * Get name of this group (normally parsed from model file)
	 * 
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * Returns the total depth of the group by searching the biggest distance on the z-axis between
	 * the vectors.
	 * 
	 * @return float as depth of the group
	 */
	public float getTotalDepth() {
		if (minZ == null || maxZ == null) {
			minZ = mesh.getMinZ();
			maxZ = mesh.getMaxZ();

			for (Group g : children) {
				minZ = Math.min(minZ, g.getMinZ());
				maxZ = Math.max(maxZ, g.getMaxZ());

			}
		}
		return maxZ - minZ;
	}

	/**
	 * Returns the total height of the group by searching the biggest distance on the y-axis between
	 * the vectors.
	 * 
	 * @return float as height of the group
	 */
	public float getTotalHeight() {
		if (minY == null || maxY == null) {
			minY = mesh.getMinY();
			maxY = mesh.getMaxY();

			for (Group g : children) {
				minY = Math.min(minY, g.getMinY());
				maxY = Math.max(maxY, g.getMaxY());

			}
		}
		return maxY - minY;
	}

	/**
	 * Returns the total width of the group by searching the biggest distance on the x-axis between
	 * the vectors.
	 * 
	 * @return float as width of the group
	 */
	public float getTotalWidth() {
		if (minX == null || maxX == null) {
			minX = mesh.getMinX();
			maxX = mesh.getMaxX();

			for (Group g : children) {
				minX = Math.min(minX, g.getMinX());
				maxX = Math.max(maxX, g.getMaxX());

			}
		}
		return maxX - minX;
	}

	/**
	 * Call this function only in the root group! It will center the whole Group with its children
	 * to the Point (0,0,0) which means the volumetric center will then be at point 0.
	 * 
	 * @param textureBasePath
	 *            Base path for relative file names
	 * 
	 */
	public void initialize(String textureBasePath) {
		initialize(textureBasePath, true);
	}

	/**
	 * Call this function only in the root group! It will center the whole Group with its children
	 * to the Point (0,0,0) which means the volumetric center will then be at point 0.
	 * 
	 * @param textureBasePath
	 *            base path for a relative file path of texture images.
	 * @param isRoot
	 *            this should only be true for the root group.
	 */
	private void initialize(String textureBasePath, boolean isRoot) {
		// Initialize min and max values of mesh
		mesh.setTextureBasePath(textureBasePath);
		mesh.getWidth();
		mesh.getHeight();
		mesh.getDepth();
		for (Group g : children) {
			g.initialize(textureBasePath, false);
		}
		getTotalWidth();
		getTotalHeight();
		getTotalDepth();
		if (isRoot) {
			Vector3f translation = new Vector3f((minX + (getTotalWidth() / 2)) * (-1),
					(minY + (getTotalHeight() / 2)) * (-1), (minZ + (getTotalDepth() / 2)) * (-1));
			for (Vertex v : model.getVertices())
				v.add(translation);
		}
	}

	/**
	 * Set minX,maxX, minY,maxY, minZ, maxZ to null to force recalculation when they are accessed
	 * next time
	 */
	public void resetMinMaxValues() {
		mesh.resetMinMaxValues();

		minX = maxX = minY = maxY = minZ = maxZ = null;

		for (Group g : children) {
			g.resetMinMaxValues();
		}
	}

	/**
	 * Set child groups of this group. Used to create a hierarchy
	 * 
	 * @param children
	 *            list of children
	 */
	public void setChildren(ArrayList<Group> children) {
		this.children = children;
	}

	/**
	 * Set the mesh containing triangles and lines
	 * 
	 * @param mesh
	 *            the mesh
	 */
	public void setMesh(Mesh mesh) {
		this.mesh = mesh;
	}

	/**
	 * Set name of this group (normally parsed from model file)
	 * 
	 * @param name
	 *            to set
	 */
	public void setName(String name) {
		this.name = name;
	}

}
