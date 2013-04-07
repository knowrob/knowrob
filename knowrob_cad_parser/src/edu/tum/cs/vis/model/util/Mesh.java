/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.imageio.ImageIO;
import javax.vecmath.Point3f;

import processing.core.PGraphics;
import processing.core.PImage;
import edu.tum.cs.ias.knowrob.utils.FileUtil;

/**
 * A Mesh contains list of Triangles and Lines which represent an Object.
 * 
 * @see Triangle
 * @see Line
 * 
 * @author Stefan Profanter
 * 
 */
public class Mesh implements Serializable {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -2761386921853967131L;

	/**
	 * Minimum x-position of model
	 */
	protected Float				minX				= null;
	/**
	 * Maximum x-position of model
	 */
	protected Float				maxX				= null;
	/**
	 * Minimum y-position of model
	 */
	protected Float				minY				= null;
	/**
	 * Maximum y-position of model
	 */
	protected Float				maxY				= null;
	/**
	 * Minimum z-position of model
	 */
	protected Float				minZ				= null;
	/**
	 * Maximum z-position of model
	 */
	protected Float				maxZ				= null;
	/**
	 * Is false until <code>setTextureImage</code> is called and the textures of all triangles are
	 * initialized.
	 */
	private boolean				texturesInitialized	= false;

	/**
	 * Base path for relative paths in texture filenames.
	 */
	private String				textureBasePath;
	/**
	 * List of all triangles parsed from file
	 */
	private ArrayList<Triangle>	triangles			= new ArrayList<Triangle>();

	/**
	 * List of all lines parsed from file
	 */
	private ArrayList<Line>		lines				= new ArrayList<Line>();

	/**
	 * Draws the bounding box around the model with the current style
	 * 
	 * @param g
	 *            Graphics context
	 */
	public void drawBoundingBox(PGraphics g) {
		if (triangles.size() == 0 && lines.size() == 0)
			return;
		// Save current translation
		g.pushMatrix();
		g.translate(getMaxX() - getWidth() / 2f, getMaxY() - getHeight() / 2f, getMaxZ()
				- getDepth() / 2f);
		g.box(getWidth(), getHeight(), getDepth());

		// Restore last translation
		g.popMatrix();
	}

	/**
	 * Draw the lines list to the applet
	 * 
	 * @param g
	 *            Applet to draw on
	 * @param drawSettings
	 *            override the draw color, texture (and other settings). Draw whole object in the
	 *            given color if != null
	 */
	public void drawLines(PGraphics g, DrawSettings drawSettings) {
		for (Line line : lines) {

			line.draw(g, drawSettings);
		}
	}

	/**
	 * Draw the triangles list to the applet
	 * 
	 * @param g
	 *            Graphics to draw on
	 * @param drawSettings
	 *            override the draw color, texture (and other settings). Draw whole object in the
	 *            given color if != null
	 */
	public void drawTriangles(PGraphics g, DrawSettings drawSettings) {
		if (!texturesInitialized)
			setTextureImage();
		synchronized (triangles) {
			for (Triangle tri : triangles) {
				tri.draw(g, drawSettings);
			}
		}
	}

	/**
	 * Returns the depth of the model by searching the biggest distance on the z-axis between the
	 * vectors.
	 * 
	 * @return float as depth of the model
	 */
	public float getDepth() {
		if (minZ != null && maxZ != null)
			return Math.abs(maxZ - minZ);
		minZ = Float.MAX_VALUE;
		maxZ = Float.MIN_VALUE;
		if (triangles.size() > 0) {

			for (Triangle tri : triangles) {
				for (int v = 0; v < 3; v++) {
					minZ = Math.min(tri.position[v].z, minZ);
					maxZ = Math.max(tri.position[v].z, maxZ);
				}
			}
			for (Line line : lines) {
				for (int v = 0; v < 2; v++) {
					minZ = Math.min(line.position[v].z, minZ);
					maxZ = Math.max(line.position[v].z, maxZ);
				}
			}
		}
		return maxZ - minZ;
	}

	/**
	 * Returns the height of the model by searching the biggest distance on the y-axis between the
	 * vectors.
	 * 
	 * @return float as height of the model
	 */
	public float getHeight() {
		if (minY != null && maxY != null)
			return Math.abs(maxY - minY);
		minY = Float.MAX_VALUE;
		maxY = Float.MIN_VALUE;
		if (triangles.size() > 0) {

			for (Triangle tri : triangles) {
				for (int v = 0; v < 3; v++) {
					minY = Math.min(tri.position[v].y, minY);
					maxY = Math.max(tri.position[v].y, maxY);
				}
			}
			for (Line line : lines) {
				for (int v = 0; v < 2; v++) {
					minY = Math.min(line.position[v].y, minY);
					maxY = Math.max(line.position[v].y, maxY);
				}
			}
		}
		return maxY - minY;
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

		for (Triangle tri : triangles) {

			Point3f intersect = new Point3f();
			if (tri.intersectsRay(rayStart, rayEnd, intersect))
				intersectedTriangles.add(new IntersectedTriangle(tri, intersect));
		}

	}

	/**
	 * Get all lines of this mesh
	 * 
	 * @return the lines
	 */
	public ArrayList<Line> getLines() {
		return lines;
	}

	/**
	 * Set maximum x coordinate of this mesh. No coordinate of any of its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum x
	 */
	public Float getMaxX() {
		if (maxX == null)
			getWidth();
		return maxX;
	}

	/**
	 * Set maximum y coordinate of this mesh. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum y
	 */
	public Float getMaxY() {
		if (maxY == null)
			getHeight();
		return maxY;
	}

	/**
	 * Set maximum z coordinate of this mesh. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum z
	 */
	public Float getMaxZ() {
		if (maxZ == null)
			getDepth();
		return maxZ;
	}

	/**
	 * Set minimum x coordinate of this mesh. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum x
	 */
	public Float getMinX() {
		if (minX == null)
			getWidth();
		return minX;
	}

	/**
	 * Set minimum y coordinate of this mesh. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum y
	 */
	public Float getMinY() {
		if (minY == null)
			getHeight();
		return minY;
	}

	/**
	 * Set minimum z coordinate of this mesh. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum z
	 */
	public Float getMinZ() {
		if (minZ == null)
			getDepth();
		return minZ;
	}

	/**
	 * Return base path for relative paths of texture images.
	 * 
	 * @return the base path
	 */
	public String getTextureBasePath() {
		return textureBasePath;
	}

	/**
	 * Get all triangles of this mesh
	 * 
	 * @return the triangles
	 */
	public List<Triangle> getTriangles() {
		return triangles;
	}

	/**
	 * Get all vertices of this mesh
	 * 
	 * @return the vertices
	 */
	public Set<Vertex> getVertices() {
		Set<Vertex> v = new HashSet<Vertex>(triangles.size() * 2);
		for (Triangle t : triangles)
			v.addAll(Arrays.asList(t.getPosition()));
		return v;
	}

	/**
	 * Returns the width of the model by searching the biggest distance on the x-axis between the
	 * vectors.
	 * 
	 * @return float as width of the model
	 */
	public float getWidth() {
		if (minX != null && maxX != null)
			return Math.abs(maxX - minX);

		minX = Float.MAX_VALUE;
		maxX = Float.MIN_VALUE;
		if (triangles.size() > 0) {

			for (Triangle tri : triangles) {
				for (int v = 0; v < 3; v++) {
					minX = Math.min(tri.position[v].x, minX);
					maxX = Math.max(tri.position[v].x, maxX);
				}
			}
			for (Line line : lines) {
				for (int v = 0; v < 2; v++) {
					minX = Math.min(line.position[v].x, minX);
					maxX = Math.max(line.position[v].x, maxX);
				}
			}
		}
		return maxX - minX;
	}

	/**
	 * Set minX,maxX, minY,maxY, minZ, maxZ to null to force recalculation when they are accessed
	 * next time
	 */
	public void resetMinMaxValues() {
		minX = maxX = minY = maxY = minZ = maxZ = null;
	}

	/**
	 * Set lines array of this mesh.
	 * 
	 * @param lines
	 *            the lines to set
	 */
	public void setLines(ArrayList<Line> lines) {
		this.lines = lines;
	}

	/**
	 * Set base path for relative paths of texture images.
	 * 
	 * @param textureBasePath
	 *            the base path
	 */
	public void setTextureBasePath(String textureBasePath) {
		this.textureBasePath = textureBasePath;
	}

	/**
	 * Only used for initialization! per definition is: textureProcessing x = textureSketchup x *
	 * picture.width textureProcessing y = picture.height - textureSketchup y * picture.height it
	 * also creates an PImage to each Triangle (if it contains any Texture)
	 */
	private void setTextureImage() {
		if (textureBasePath == null)
			return;
		// load all Texture-Images only once (memory efficiency)
		HashMap<String, PImage> pictures = new HashMap<String, PImage>();
		HashSet<String> alreadyLoaded = new HashSet<String>();// if error reading file,
																// pictures<path> is null. So avoid
																// trying again
		synchronized (triangles) {
			for (Triangle tri : triangles) {
				if (tri.appearance.getImageFileName() == null)
					continue;
				String texfile = FileUtil.getAbsoluteFilePath(textureBasePath,
						tri.appearance.getImageFileName());
				if (pictures.get(texfile) == null && !alreadyLoaded.contains(texfile)) {

					alreadyLoaded.add(texfile);
					BufferedImage bimg = null;
					try {
						bimg = ImageIO.read(new File(texfile));
						// Convert BufferedImage to Image otherwise PImage constructor will fail!!

						Image i = bimg.getScaledInstance(bimg.getWidth(), bimg.getHeight(), 0);

						PImage pImg = new PImage(i);

						pictures.put(texfile, pImg);
					} catch (IOException e) {
						System.err.println("Couldn't read file: " + texfile);
					}

				}
			}

		}

		// Now remove offset of texture coordinates because there is a bug with P3D when texture
		// should repeated
		synchronized (triangles) {
			for (Triangle tri : triangles) {

				if (tri.appearance.getImageFileName() == null)
					continue;

				String texfile = FileUtil.getAbsoluteFilePath(textureBasePath,
						tri.appearance.getImageFileName());
				// PImage tex = applet.loadImage(texfile);
				PImage tex = pictures.get(texfile);
				if (tex == null)
					continue;

				double xMin = Double.MAX_VALUE;
				double yMin = Double.MAX_VALUE;

				for (int i = 0; i < 3; i++) {

					double x = tri.texPosition[i].x * tex.width;
					double y = tex.height - tri.texPosition[i].y * tex.height;
					tri.texPosition[i].x = (float) x;
					tri.texPosition[i].y = (float) y;
					xMin = Math.min(x, xMin);
					yMin = Math.min(y, yMin);
				}

				// Remove offset of texture coordinate if all coordinates are greater than texture
				xMin = Math.floor(xMin / tex.width);
				yMin = Math.floor(yMin / tex.height);

				for (int i = 0; i < 3; i++) {
					tri.texPosition[i].x -= xMin * tex.width;
					tri.texPosition[i].y -= yMin * tex.height;
				}

				tri.appearance.setImageReference(tex);
			}
		}
		texturesInitialized = true;
	}

	/**
	 * Set triangles array of this mesh
	 * 
	 * @param triangles
	 *            the triangles to set
	 */
	public void setTriangles(ArrayList<Triangle> triangles) {
		this.triangles = triangles;
	}

	/**
	 * Add all triangles contained in this mesh to the provided collection.
	 * 
	 * @param triangles2
	 *            Collection where all triangles of this mesh are added.
	 */
	public void getAllTriangles(Collection<Triangle> triangles2) {
		triangles2.addAll(triangles);

	}

}
