/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import processing.core.PGraphics;
import edu.tum.cs.ias.knowrob.utils.ThreadPool;
import edu.tum.cs.vis.model.util.BSphere;
import edu.tum.cs.vis.model.util.DrawSettings;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;
import edu.tum.cs.vis.model.util.algorithm.Miniball;

/**
 * 
 * Class which represents a CAD model loaded from file. A CAD model consists of triangles and lines
 * which have vertices. These can be grouped hierarchically into subgroups.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class Model {

	/**
	 * Log4J Logger
	 */
	private static Logger			logger				= Logger.getLogger(Model.class);

	/**
	 * Absolute file path where the relative paths within the model are based.
	 */
	private String					textureBasePath		= null;

	/**
	 * Main group of the model. Each model should have at least one group.
	 */
	private Group					group;

	/**
	 * List of all vertices in this model
	 */
	private final List<Vertex>		vertices			= new ArrayList<Vertex>();

	/**
	 * List of all triangles in this model
	 */
	private final List<Triangle>	triangles			= new ArrayList<Triangle>();

	/**
	 * List of all lines in this model
	 */
	private final List<Line>		lines				= new ArrayList<Line>();

	/**
	 * Minimum bounding sphere of this model. Only set if previously calculated (by Miniball class).
	 * 
	 * @see edu.tum.cs.vis.model.util.algorithm.Miniball
	 */
	private BSphere					boundingSphere		= null;

	/**
	 * Current model scale. Is used to normalize model for further reasoning. getUnscaled methods
	 * use this value to undo scaling for parameters such as height, width, radius, ...
	 */
	private float					scale				= 1;

	/**
	 * Indicates if the vertex normals of each vertex have already been initialized.
	 */
	private boolean					normalsInitialized	= false;

	/**
	 * @param g
	 *            Graphics context to draw on
	 * @param drawSettings
	 *            Settings for drawing the model, e.g. to override face color of model. Useful if
	 *            you want to ignore color of model and draw whole model in a specific color.
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		if (group != null)
			group.draw(g, drawSettings);
	}

	/**
	 * Gets the bounding sphere set by Miniball class.
	 * 
	 * @see edu.tum.cs.vis.model.util.algorithm.Miniball
	 * 
	 * @return the boundingSphere
	 */
	public BSphere getBoundingSphere() {
		if (boundingSphere == null)
			calculateBoundingsphere();
		return boundingSphere;
	}

	/**
	 * Get main group of model
	 * 
	 * @return the group
	 */
	public Group getGroup() {
		return group;
	}

	/**
	 * Get all lines of model
	 * 
	 * @return the lines
	 */
	public List<Line> getLines() {
		return lines;
	}

	/**
	 * Gets scale factor of model. Model is scaled (normalized) for reasoning. This value indicates
	 * this scale factor.
	 * 
	 * @return model scale
	 */
	public float getScale() {
		return scale;
	}

	/**
	 * Get texture base path which is the base for all relative texture paths for this model.
	 * 
	 * @return the textureBasePath
	 */
	public String getTextureBasePath() {
		return textureBasePath;
	}

	/**
	 * Get all triangles of model
	 * 
	 * @return the triangles
	 */
	public List<Triangle> getTriangles() {
		return triangles;
	}

	/**
	 * Unscale the specified float value by dividing <tt>scaled</tt> by <tt>getScale</tt>
	 * 
	 * @param scaled
	 *            scaled value
	 * @return unscaled value
	 */
	public float getUnscaled(float scaled) {
		return scaled * 1f / scale;
	}

	/**
	 * Unscale the specified point coordinates by dividing each coordinate of <tt>corner</tt> by
	 * <tt>getScale</tt>
	 * 
	 * @param corner
	 *            scaled value
	 * @return unscaled value
	 */
	public Tuple3f[] getUnscaled(Point3f[] corner) {
		Tuple3f[] ret = new Point3f[corner.length];
		for (int i = 0; i < corner.length; i++) {
			ret[i] = getUnscaled(corner[i]);
		}
		return ret;
	}

	/**
	 * Unscale the specified float value by dividing <tt>t</tt> by <tt>getScale</tt>
	 * 
	 * @param t
	 *            scaled value
	 * @return unscaled value
	 */
	public Tuple3f getUnscaled(Tuple3f t) {
		Tuple3f tr = new Point3f(t);
		tr.scale(1f / scale);
		return tr;
	}

	/**
	 * Unscale the specified float value by dividing <tt>t</tt> by <tt>getScale</tt>
	 * 
	 * @param t
	 *            scaled value
	 * @return unscaled value
	 */
	public Vector3f getUnscaled(Vector3f t) {
		Vector3f tr = new Vector3f(t);
		tr.scale(1f / scale);
		return tr;
	}

	/**
	 * Get all vertices of model
	 * 
	 * @return the vertices
	 */
	public List<Vertex> getVertices() {
		return vertices;
	}

	/**
	 * Mirror model on x axis my multiplying each x value of all vertices with -1
	 */
	public void mirrorX() {
		for (Vertex v : vertices) {
			v.x *= (-1);
		}
		group.resetMinMaxValues();

	}

	/**
	 * Normalizes the model. Makes sure that the model fits into a cube with side length of 1. So
	 * each vertex is scaled.
	 */
	public void normalize() {
		reloadVertexList();
		float x = group.getMaxX() - group.getMinX();
		float y = group.getMaxY() - group.getMinY();
		float z = group.getMaxZ() - group.getMinZ();

		float max = Math.max(x, Math.max(y, z));

		scale(1f / max);

		scale = 1f / max;
	}

	/**
	 * Scale model by given factor by multiplying all vertices with <tt>factor</tt>. Call setScale
	 * afterwards if you want to set scale factor also for parameters.
	 * 
	 * @param factor
	 *            Factor to scale
	 */
	public void scale(float factor) {

		for (Vertex v : vertices) {
			v.scale(factor);
		}
		for (Triangle t : triangles)
			t.updateCentroid();
		for (Line l : lines)
			l.updateCentroid();
		group.resetMinMaxValues();
	}

	/**
	 * Set main group of model.
	 * 
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
	}

	/**
	 * Set texture base path for all texture elements of this model.
	 * 
	 * @param textureBasePath
	 *            the textureBasePath to set
	 */
	public void setTextureBasePath(String textureBasePath) {
		this.textureBasePath = textureBasePath;
	}

	/**
	 * Searches for triangles which have exactly the same coordinate points and removes one of them
	 * to avoid two triangles drawn on exactly the same position. Used for mesh reasoning.
	 * 
	 * @return Number of removed triangles
	 */
	public int removeDoubleSidedTriangles() {
		final Set<Triangle> toRemove = new HashSet<Triangle>();

		final int interval = 1000;
		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();
		for (int start = 0; start < triangles.size(); start += interval) {
			final int st = start;
			threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {
					int end = Math.min(st + interval, triangles.size());
					for (int i = st; i < end; i++) {
						Triangle t1 = triangles.get(i);
						if (toRemove.contains(t1))
							continue;
						for (int j = i + 1; j < triangles.size(); j++) {
							Triangle t2 = triangles.get(j);

							if (toRemove.contains(t2))
								continue;
							int eqCnt = 0;
							for (int k = 0; k < 3; k++) {
								Point3f p1 = t1.getPosition()[k];
								for (Point3f p2 : t2.getPosition()) {
									if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) {
										eqCnt++;
										break;
									}
								}
								if (eqCnt != k + 1) {
									// break if not enough vertices are equal
									break;
								}
							}
							if (eqCnt == 3) {
								// triangles are the same, so remove j

								synchronized (toRemove) {
									toRemove.add(t2);
								}
							}
						}
					}
					return null;
				}

			});
		};

		ThreadPool.executeInPool(threads);
		if (toRemove.size() > 0) {
			this.group.removeTriangle(toRemove);
			reloadVertexList();
		}
		return toRemove.size();
	}

	/**
	 * Rebuilds the main triangles and vertices list by iterating over all child groups and
	 * collecting all triangles and vertices found.
	 */
	private void reloadVertexList() {
		synchronized (triangles) {
			triangles.clear();
			this.group.getAllTriangles(triangles);
		}
		Set<Vertex> vertices = new HashSet<Vertex>(triangles.size() * 2);
		for (Triangle t : triangles) {
			vertices.addAll(Arrays.asList(t.getPosition()));
		}
		for (Line l : lines) {
			vertices.addAll(Arrays.asList(l.getPosition()));
		}
		this.vertices.clear();
		this.vertices.addAll(vertices);
	}

	/**
	 * Check for each vertex if it should be shared with neighbor triangles or not. This decision is
	 * made according to the dihedral angle between the two triangles
	 * 
	 */
	public void updateVertexSharing() {

		final Set<Triangle> checkedTriangles = Collections.synchronizedSet(new HashSet<Triangle>());

		Set<Vertex> toRemove = new HashSet<Vertex>();

		for (Triangle t : triangles)
			updateVertexSharingForTriangle(t, checkedTriangles, toRemove);
		synchronized (vertices) {
			vertices.removeAll(toRemove);
		}
		reloadVertexList();
	}

	/**
	 * Check for the given two triangles if their vertices should be shared or not. The decision is
	 * made according to the dihedral angle between the two triangles.
	 * 
	 * @param t
	 *            the main triangle
	 * @param n
	 *            one of the neighboring triangles of <code>t</code>
	 * @param checkedTriangles
	 *            Set of already checked triangles. If one is already checked, it is ignored in this
	 *            call.
	 * @param toRemove
	 *            Set of vertices which should be removed from the main vertices list. Combined
	 *            vertices which aren't anymore in the model will be added to this set.
	 */
	private void updateVertexSharingForNeighbors(final Triangle t, final Triangle n,
			final Set<Triangle> checkedTriangles, Set<Vertex> toRemove) {
		if (t == n)
			return;
		synchronized (n) {
			if (checkedTriangles.contains(n))
				return;

			double angle = t.getDihedralAngle(n);
			// Share vertices if angle is < 15 degree
			boolean share = (angle < 15 / 180.0 * Math.PI);

			for (Vertex vt : t.getPosition()) {
				for (int i = 0; i < n.getPosition().length; i++) {
					Vertex vn = n.getPosition()[i];
					/*
					if ((vt.sameCoordinates(vn) && vt.getPointarea() == vn.getPointarea() && vt
							.getNormalVector().equals(vn.getNormalVector()))
							||(share && vn != vt && vt.sameCoordinates(vn))) {*/
					if (share && vn != vt && vt.sameCoordinates(vn)) {
						// merge vertices
						n.getPosition()[i] = vt;
						int cnt = 0;
						for (Triangle tmpTri : t.getNeighbors()) {
							for (Vertex tmpVer : tmpTri.getPosition())
								if (tmpVer == vn) {
									cnt++;
									break;
								}
						}
						if (cnt == 0) {
							for (Triangle tmpTri : n.getNeighbors()) {
								if (tmpTri == t)
									continue;
								for (Vertex tmpVer : tmpTri.getPosition())
									if (tmpVer == vn) {
										cnt++;
										break;
									}
							}
						}
						if (cnt == 0) {
							toRemove.add(vn);
						}
					} else if (!share && vn == vt) {
						// split vertices
						// System.out.println("split");
						/*Vertex clone = (Vertex) vt.clone();
						synchronized (vertices) {
							vertices.add(clone);
						}
						n.getPosition()[i] = clone;*/
					}
				}
			}
		}
	}

	/**
	 * Check for all vertices between given triangle and its neighbors if the vertex should be
	 * shared or not according to the dihedral angle.
	 * 
	 * @param t
	 *            Triangle to check
	 * @param checkedTriangles
	 *            already checked triangles
	 * @param toRemove
	 *            Set of vertices which should be removed from the main vertices list. Combined
	 *            vertices which aren't anymore in the model will be added to this set.
	 */
	private void updateVertexSharingForTriangle(final Triangle t,
			final Set<Triangle> checkedTriangles, final Set<Vertex> toRemove) {
		synchronized (t) {
			if (checkedTriangles.contains(t))
				return;
			synchronized (t.getNeighbors()) {
				for (Triangle n : t.getNeighbors()) {
					updateVertexSharingForNeighbors(t, n, checkedTriangles, toRemove);
					// also check neighbors of neighbor
					for (Triangle nn : n.getNeighbors()) {
						updateVertexSharingForNeighbors(t, nn, checkedTriangles, toRemove);
					}
				}

			}
			checkedTriangles.add(t);
		}
	}

	/**
	 * Ported from trimesh2 (2.12) (Szymon Rusinkiewicz Princeton University)
	 * 
	 * uses average of per-face normals, weighted according to: Max, N.
	 * "Weights for Computing Vertex Normals from Facet Normals," Journal of Graphics Tools, Vol. 4,
	 * No. 2, 1999.
	 */
	public void updateVertexNormals() {
		// Compute from faces

		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		final int interval = 500;

		// Reset normal vectors, because vertex normals from collada aren't correct
		for (Vertex v : vertices) {
			v.getNormalVector().x = v.getNormalVector().y = v.getNormalVector().z = 0;
		}

		for (int start = 0; start < triangles.size(); start += interval) {
			final int st = start;
			/*threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {*/
			int end = Math.min(st + interval, triangles.size());
			for (int i = st; i < end; i++) {
				calculateVertexNormalsForTriangle(triangles.get(i));
			}
			/*	return null;
			}

			});*/
		};

		ThreadPool.executeInPool(threads);

		// Normalize all vectors.
		// Additionally search the vectors which have max x, max y and max z coordinates (needed for
		// vertex winding check / inverted normals check)

		Vertex extrema[] = new Vertex[6];

		for (Vertex v : vertices) {
			v.getNormalVector().normalize();
			float coord[] = new float[3];
			v.get(coord);
			// Set all max values
			for (int i = 0; i < 3; i++) {
				float extCoord[] = new float[3];
				if (extrema[i] != null)
					extrema[i].get(extCoord);
				if (extrema[i] == null || extCoord[i] < coord[i])
					extrema[i] = v;
			}
			// Set all min values
			for (int i = 0; i < 3; i++) {
				float extCoord[] = new float[3];
				if (extrema[i + 3] != null)
					extrema[i + 3].get(extCoord);
				if (extrema[i + 3] == null || extCoord[i] > coord[i])
					extrema[i + 3] = v;
			}
		}

		int vote = 0;
		// Now the vertex normal of maxX must point in approximately the same direction as the
		// axis. If the angle between the vertex normal and the direction is smaller than 90° vote
		// for inversion.
		for (int i = 0; i < 6; i++) {
			float coord[] = { 0, 0, 0 };
			coord[i % 3] = i < 3 ? 1 : -1;
			double dot = extrema[i].getNormalVector().dot((new Vector3f(coord)));
			double angle = Math.acos(dot);
			// System.out.println("Angle: " + (angle * 180 / Math.PI));
			if (angle < Math.PI / 2)
				vote--;
			else
				vote++;
		}
		// System.out.println("VOTE: " + vote);
		if (vote > 0) {
			// They voted for inverting
			logger.debug("Inverting normal vertices, because vote is: " + vote);
			for (Vertex v : vertices) {
				v.getNormalVector().scale(-1f);
			}
		}

		normalsInitialized = true;

	}

	/**
	 * Calculate vertex normals for each vertex of triangle.
	 * 
	 * @param t
	 *            Triangle
	 */
	static void calculateVertexNormalsForTriangle(final Triangle t) {
		Vertex p0 = t.getPosition()[0];
		Vertex p1 = t.getPosition()[1];
		Vertex p2 = t.getPosition()[2];

		// get vectors from p0 to p1 and so on
		Vector3f a = new Vector3f(p0);
		a.sub(p1);
		Vector3f b = new Vector3f(p1);
		b.sub(p2);
		Vector3f c = new Vector3f(p2);
		c.sub(p0);
		// length of these vectors
		float l2a = a.lengthSquared(), l2b = b.lengthSquared(), l2c = c.lengthSquared();
		if (l2a == 0 || l2b == 0 || l2c == 0)
			return;

		Vector3f facenormal = new Vector3f();
		facenormal.cross(a, b); // unscaled normal

		Vector3f normalP0 = (Vector3f) facenormal.clone();
		normalP0.scale(1.0f / (l2a * l2c));
		synchronized (p0.getNormalVector()) {
			p0.getNormalVector().add(normalP0);
		}

		Vector3f normalP1 = (Vector3f) facenormal.clone();
		normalP1.scale(1.0f / (l2b * l2a));
		synchronized (p1.getNormalVector()) {
			p1.getNormalVector().add(normalP1);
		}

		Vector3f normalP2 = (Vector3f) facenormal.clone();
		normalP2.scale(1.0f / (l2c * l2b));
		synchronized (p2.getNormalVector()) {
			p2.getNormalVector().add(normalP2);
		}
	}

	/**
	 * Export the vertices of the model into a temporary txt file with the format:
	 * 
	 * <pre>
	 * NUMBER_OF_POINTS
	 * x1	y1	z1	nx1	nx2	nx3
	 * x2	y2	z2	nx2	nx2	nx2
	 * </pre>
	 * 
	 * where the first 3 values are the coordinates of the point, the next 3 values represent the
	 * normal vector.
	 * 
	 * The file name is generated in the temporary directory and will be unique.
	 * 
	 * @return the file containing the exported model or null on error
	 */
	public File exportVerticesAsTxt() {
		try {
			File tmp = File.createTempFile("knowrob_", ".txt",
					new File(System.getProperty("java.io.tmpdir")));
			exportVerticesAsTxt(tmp, true);
			return tmp;
		} catch (IOException e) {
			logger.error("Couldn't create temp file name: " + e.getMessage());
			return null;
		}

	}

	/**
	 * Export the vertices of the model into a txt file with the format:
	 * 
	 * <pre>
	 * NUMBER_OF_POINTS
	 * x1	y1	z1	nx1	nx2	nx3
	 * x2	y2	z2	nx2	nx2	nx2
	 * </pre>
	 * 
	 * where the first 3 values are the coordinates of the point, the next 3 values represent the
	 * normal vector.
	 * 
	 * @param path
	 *            the path to the file where to store the data
	 * @param overwrite
	 *            overwrite existing file
	 * @return true on success
	 */
	public boolean exportVerticesAsTxt(File path, boolean overwrite) {

		try {
			if (!overwrite && path.exists()) {
				logger.error("Couldn't export model to file. Already exists: "
						+ path.getAbsolutePath());
				return false;
			}
			FileWriter fstream = new FileWriter(path);
			BufferedWriter out = new BufferedWriter(fstream);
			out.write(vertices.size() + "\n");
			NumberFormat df = new DecimalFormat("0.####################");
			for (int i = 0; i < vertices.size(); i++) {
				Vertex v = vertices.get(i);
				out.write(df.format(v.x) + "\t" + df.format(v.y) + "\t" + df.format(v.z));
				if (normalsInitialized) {
					out.write("\t" + df.format(v.getNormalVector().x) + "\t"
							+ df.format(v.getNormalVector().y) + "\t"
							+ df.format(v.getNormalVector().z));
				}
				if (i < vertices.size() - 1)
					out.write("\n");
			}
			logger.info("Model exported to file " + path.getAbsolutePath());
			out.close();
			return true;
		} catch (IOException e) {
			logger.error("Couldn't export model to file " + path.getAbsolutePath() + ". "
					+ e.getMessage());
		}
		return false;
	}

	/**
	 * Get the model size on x axis.
	 * 
	 * @return model size on x axis calculated by the maximum x value minus minimum x value
	 */
	public double getSizeX() {
		return group.getMaxX() - group.getMinX();
	}

	/**
	 * Get the model size on y axis.
	 * 
	 * @return model size on y axis calculated by the maximum y value minus minimum y value
	 */
	public double getSizeY() {
		return group.getMaxY() - group.getMinY();
	}

	/**
	 * Get the model size on z axis.
	 * 
	 * @return model size on z axis calculated by the maximum z value minus minimum z value
	 */
	public double getSizeZ() {
		return group.getMaxZ() - group.getMinZ();
	}

	/**
	 * Calculates the bounding sphere for the model. Calculated by Miniball class.
	 * 
	 * @see edu.tum.cs.vis.model.util.algorithm.Miniball
	 * 
	 */
	private void calculateBoundingsphere() {
		if (vertices.size() == 0)
			return;

		Miniball mb = new Miniball(3);
		for (Vertex v : vertices) {
			double arr[] = new double[3];
			arr[0] = v.x;
			arr[1] = v.y;
			arr[2] = v.z;
			mb.check_in(arr);
		}
		mb.build();
		this.boundingSphere = new BSphere((float) Math.sqrt(mb.squared_radius()), new Vector3f(
				(float) mb.center()[0], (float) mb.center()[1], (float) mb.center()[2]));

	}

	/**
	 * A characteristic "feature size" for the mesh. Computed as the median edge length.
	 * 
	 * Ported (and modified) from trimesh2 library (version: 2.12)
	 * 
	 * @return median edge length
	 */
	public float feature_size() {
		if (triangles.size() == 0)
			return 0.0f;

		int nf = triangles.size();
		int nsamp = Math.min(nf / 2, 333);

		Float[] samples = new Float[nsamp * 3];

		for (int i = 0; i < nsamp; i++) {

			int ind = (int) (Math.random() * nf);
			final Vertex p0 = triangles.get(ind).getPosition()[0];
			final Vertex p1 = triangles.get(ind).getPosition()[1];
			final Vertex p2 = triangles.get(ind).getPosition()[2];
			samples[i * 3] = p0.distanceSquared(p1);
			samples[i * 3 + 1] = p1.distanceSquared(p2);
			samples[i * 3 + 2] = p2.distanceSquared(p0);
		}
		Arrays.sort(samples);
		return getUnscaled((float) Math.sqrt(samples[samples.length / 2]));
	}
}
