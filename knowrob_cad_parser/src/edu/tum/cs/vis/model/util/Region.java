/*******************************************************************************
 * Copyright (c) 2014 Andrei Stoica. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Andrei Stoica - initial API and implementation during Google Summer of Code 2014
 ******************************************************************************/

package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * Class that implements the data storage and functionality of a region as a connected area of
 * triangles on the mesh surface that share common curvature properties. These triangles are
 * situated on relatively contained and localized position on the mesh which are thus implemented as
 * regions.
 * 
 * @author Andrei Stoica
 */
public class Region {

	/**
	 * Stores the regions label id
	 */
	private final int				id;

	/**
	 * The area of the entire region
	 */
	private float					area;

	/**
	 * The curvature parameters of the boundary. The first parameter stores the minimum value of the
	 * curvature <tt>kMin</tt>, the second one the maximum value of the curvature <tt>kMax</tt> and
	 * the third the MinMax <tt> kMinMax</tt> curvature value as they were computed by the
	 * <tt>calculateCurvatures(HashMap<Vertex, Curvature>, Model)</tt> routine from the
	 * knowrob_mesh_reasoning add-on
	 */
	private final float[]			curvatureMinMax		= new float[3];

	/**
	 * A list of all the triangles inside the region
	 */
	private final List<Triangle>	triangles			= new ArrayList<Triangle>();

	/**
	 * A list of all the triangles on the boundary of the region
	 */
	private final List<Triangle>	boundaryTriangles	= new ArrayList<Triangle>();

	/**
	 * A list of all the regions that are neighbors of this region instance
	 */
	private final List<Region>		regionNeighbors		= new ArrayList<Region>();

	/**
	 * A list of all edges at the boundary that form in the end the perimeter of the region
	 */
	private final List<Edge>		edgeBoundary		= new ArrayList<Edge>();

	/**
	 * The flaoting point value of the region's perimeter
	 */
	private float					perimeter;

	/**
	 * The appearance associated with the region. This is used to customize the view of the
	 * triangles that are part of the region
	 */
	private Appearance				appearance			= new Appearance();

	/**
	 * Constructor for a region that takes only the region id
	 * 
	 * @param newId
	 *            region id
	 */
	public Region(final int newId) {
		this.id = newId;
		this.initializeApperance();
		this.area = 0.0f;
		this.perimeter = 0.0f;
		this.curvatureMinMax[0] = this.curvatureMinMax[1] = this.curvatureMinMax[2] = 0.0f;
	}

	/**
	 * Constructor for a region that takes the region id and the initial seed triangle from which
	 * the region build-up starts
	 * 
	 * @param newId
	 *            region id
	 * @param initSeedTr
	 *            seed triangle from which region growing starts
	 */
	public Region(final int newId, final Triangle initSeedTr) {
		this.id = newId;
		this.initializeApperance();
		this.area = 0.0f;
		this.perimeter = 0.0f;
		this.addTriangleToRegion(initSeedTr);
		this.curvatureMinMax[0] = initSeedTr.getCurvatureValues()[0];
		this.curvatureMinMax[1] = initSeedTr.getCurvatureValues()[1];
		this.curvatureMinMax[2] = initSeedTr.getCurvatureValues()[2];
	}

	/**
	 * Gets the region's id
	 * 
	 * @return id
	 */
	public int getRegionId() {
		return this.id;
	}

	/**
	 * Gets the region's area
	 * 
	 * @return area
	 */
	public float getAreaOfRegion() {
		return this.area;
	}

	/**
	 * Gets the region's appearance
	 * 
	 * @return appearance
	 */
	public Appearance getAppearance() {
		return this.appearance;
	}

	/**
	 * Gets the region's perimeter
	 * 
	 * @return perimeter
	 */
	public float getPerimeterOfRegion() {
		return this.perimeter;
	}

	/**
	 * Gets the region's list of triangles
	 * 
	 * @return triangles
	 */
	public List<Triangle> getTriangles() {
		return this.triangles;
	}

	/**
	 * Gets the region's list of boundary triangles
	 * 
	 * @return boundary triangles
	 */
	public List<Triangle> getBoundaryTriangles() {
		return this.boundaryTriangles;
	}

	/**
	 * Gets the region's list of region neighbors
	 * 
	 * @return region neighbors
	 */
	public List<Region> getNeighbors() {
		return this.regionNeighbors;
	}

	/**
	 * Gets the region's list of boundary edges
	 * 
	 * @return boundary edges
	 */
	public List<Edge> getBoundaryEdges() {
		return this.edgeBoundary;
	}

	/**
	 * Gets the curvature values associated with the region as an array containing the
	 * <tt>kMin, kMax, kMinMax</tt> curvature values
	 * 
	 * @return curvature value
	 */
	public float[] getCurvatureMinMaxOfRegion() {
		return this.curvatureMinMax;
	}

	/**
	 * Determines the triangle neighbors of the region that are still not labelled as belonging to a
	 * region
	 * 
	 * @return unlabelled triangles neighboring the region
	 */
	public List<Triangle> getOutsideBoundaryUnlabelled() {
		List<Triangle> unlabelledOutside = new ArrayList<Triangle>();
		for (int i = 0; i < boundaryTriangles.size(); ++i) {
			Triangle t = boundaryTriangles.get(i);
			for (Triangle n : t.getNeighbors()) {
				if (n.getRegionLabel() == -1) {
					unlabelledOutside.add(n);
				}
			}
		}
		return unlabelledOutside;
	}

	/**
	 * Detects the shared edges at the boundary of two regions
	 * 
	 * @param neighbor
	 *            region to be checked
	 * @return list of all shared edges
	 */
	public List<Edge> getCommonEdges(Region neighbor) {
		List<Edge> commonEdges = new ArrayList<Edge>();
		for (Edge e : this.edgeBoundary) {
			for (Edge en : neighbor.edgeBoundary) {
				if (e.isDirectNeighbor(en)) {
					commonEdges.add(e);
					break;
				}
			}
		}
		return commonEdges;
	}

	/**
	 * Determines the common edge a triangle from outside or inside the region (not from the
	 * boundary) has with the region boundary
	 * 
	 * @param triangle
	 *            to be checked
	 * @return the common edge or null if not any
	 */
	public Edge getCommonEdge(Triangle triangle) {
		if (!boundaryTriangles.contains(triangle)) {
			for (Triangle n : boundaryTriangles) {
				// Edge edge = triangle.getCommonEdge(n);
				Edge edge = n.getCommonEdge(triangle);
				if (edge != null) {
					return edge;
				}
			}
		}
		return null;
	}

	/**
	 * Sets the region's appearance
	 * 
	 * @param newAppearance
	 *            to be set
	 */
	public void setApperance(final Appearance newAppearance) {
		this.appearance = newAppearance;
	}

	/**
	 * Sets the region's minimum curvature <tt>kMin</tt> value
	 * 
	 * @param kMin
	 *            curvature to be set
	 */
	public void setCurvatureMin(final float kMin) {
		this.curvatureMinMax[0] = kMin;
	}

	/**
	 * Sets the region's maximum curvature <tt>kMax</tt> value
	 * 
	 * @param kMax
	 *            curvature to be set
	 */
	public void setCurvatureMax(final float kMax) {
		this.curvatureMinMax[1] = kMax;
	}

	/**
	 * Sets the region's min-max curvature <tt>kMinkMax</tt> value
	 * 
	 * @param kMinkMax
	 *            curvature to be set
	 */
	public void setCurvatureMinMax(final float kMinkMax) {
		this.curvatureMinMax[2] = kMinkMax;
	}

	/**
	 * Returns a boolean specifying whether the region passed as an argument is a neighbor or not of
	 * the current instance
	 * 
	 * @param region
	 *            region to be checked
	 * @return true or false
	 */
	public boolean isNeighbor(Region region) {
		if (regionNeighbors.contains(region)) {
			return true;
		}
		return false;
	}

	/**
	 * Initializes the appearance of a newly created region by randomizing the fill color associated
	 * with the region.
	 */
	public void initializeApperance() {
		int R, G, B, min = 0, max = 255;
		R = min + (int) (Math.random() * ((max - min) + 1));
		G = min + (int) (Math.random() * ((max - min) + 1));
		B = min + (int) (Math.random() * ((max - min) + 1));
		Color regionColor = new Color(R, G, B);
		this.appearance.setColorFill(regionColor);
	}

	/**
	 * Calculates in-place the region surface area based on the individual areas of the triangles
	 * which are part of the region
	 */
	public void updateAreaOfRegion() {
		area = 0.0f;
		for (Triangle tr : triangles) {
			area += tr.getArea();
		}
	}

	/**
	 * Adds up or subtracts in-place the area of a specified triangle to the area of the region. If
	 * the <tt>toRemove</tt> flag is raised the area is subtracted, otherwise it is added.
	 * 
	 * @param tr
	 *            triangle whose area needs to be added / subtracted to region
	 * @param toRemove
	 *            flag to subtract the area from
	 */
	private void updateAreaOfRegion(final Triangle tr, final boolean toRemove) {
		if (tr != null) {
			if (toRemove == false) {
				area += tr.getArea();
			} else {
				area -= tr.getArea();
			}
		}
	}

	/**
	 * Calculates in-place the perimeter of the region using the boundary edges.
	 */
	public void updatePerimeterOfRegion() {
		perimeter = 0.0f;
		for (Edge e : edgeBoundary) {
			perimeter += e.getEdgeValue().length();
		}
	}

	/**
	 * Adds a list of triangles to the list of triangles in the region. If a triangle was already in
	 * the region, it is skipped.
	 * 
	 * @param newTr
	 *            list of traingles to be added to the region
	 */
	public void addTrianglesToRegion(final List<Triangle> newTr) {
		if (newTr != null) {
			for (int i = 0; i < newTr.size(); ++i) {
				if (newTr.get(i) != null) {
					Triangle triangle = newTr.get(i);
					if (!triangles.contains(triangle)) {
						triangles.add(triangle);
						triangle.setRegionLabel(id);
						triangle.appearance = this.appearance;
						this.updateAreaOfRegion(triangle, false);
					}
				}
			}
		}
	}

	/**
	 * Adds triangle to the list of triangles included in the region. The addition is performed only
	 * if the triangle was not previously inside the region. If the triangle was already in the
	 * region, this function does nothing.
	 * 
	 * @param newTr
	 *            triangle to be checked and added to triangle
	 */
	public void addTriangleToRegion(final Triangle newTr) {
		if (newTr != null && !triangles.contains(newTr) && newTr.getRegionLabel() == -1) {
			triangles.add(newTr);
			newTr.setRegionLabel(id);
			newTr.appearance = this.appearance;
			this.updateAreaOfRegion(newTr, false);
			this.addTriangleToRegionBoundary(newTr);
		}
	}

	/**
	 * Checks if a triangle is at a region's boundary and if it is it adds it to the list and then
	 * removes any triangles from the list that are not longer at the boundary. If the triangle is
	 * not at the boundary, this function does nothing. The check is based on the neighbors of the
	 * triangle compared against the triangles already in the region.
	 * 
	 * @param newTr
	 *            triangle to be checked and added to region boundary
	 */
	public void addTriangleToRegionBoundary(final Triangle newTr) {
		if (!triangles.containsAll(newTr.getNeighbors())) {
			// add current Triangle object located on boundary
			boundaryTriangles.add(newTr);
		} else {
			// check if newTr is a "margin" triangle
			for (Edge e : newTr.getEdges()) {
				List<Triangle> neighborsOfEdge = newTr.getNeighborsOfEdge(e);
				if (neighborsOfEdge.size() == 0) {
					boundaryTriangles.add(newTr);
				}
			}
		}
		List<Triangle> toRemove = new ArrayList<Triangle>();
		// make sure neighboring triangles are not on the region boundary given the new added
		// triangle
		for (Triangle tr : newTr.getNeighbors()) {
			// a margin triangle is one that does not have neighbors for an edge
			boolean marginTriangle = false;
			for (Edge tre : tr.getEdges()) {
				List<Triangle> neighborsOfEdge = tr.getNeighborsOfEdge(tre);
				if (neighborsOfEdge.size() == 0) {
					marginTriangle = true;
				}
			}
			// if the neighboring triangle has all its neighbors in the region and is not a
			// margin triangle then it is not on the boundary
			if (triangles.containsAll(tr.getNeighbors()) && !marginTriangle) {
				toRemove.add(tr);
			}
		}
		// remove triangles that are not on the boundary anymore
		boundaryTriangles.removeAll(toRemove);
		return;
	}

	/**
	 * Removes triangle from region and updates the list of region triangles and of the region
	 * boundary triangles
	 */
	public void removeTriangleFromRegion(final Triangle toRemove) {
		if (triangles.remove(toRemove)) {
			updateAreaOfRegion(toRemove, true);
			if (boundaryTriangles.remove(toRemove)) {
				System.out.println("Triangle " + toRemove.toString()
						+ " also removed from the boundary region");
				this.updateRegionBoundary();
			}
		}
		// unset region label
		toRemove.setRegionLabel(-1);
	}

	/**
	 * Updates the boundary region based on the existing triangles inside the region itself
	 */
	public void updateRegionBoundary() {
		boundaryTriangles.clear();
		for (Triangle tr : triangles) {
			for (Edge e : tr.getEdges()) {
				List<Triangle> neighborsOfEdge = tr.getNeighborsOfEdge(e);
				if (neighborsOfEdge.size() == 0) {
					boundaryTriangles.add(tr);
				} else {
					for (int i = 0; i < neighborsOfEdge.size(); ++i) {
						if (triangles.contains(neighborsOfEdge.get(i))) {
							boundaryTriangles.add(tr);
							break;
						}
					}
				}
			}
		}
	}

	/**
	 * Updates the region neighbors based on a list containing a list of regions
	 * 
	 * @param allRegions
	 *            list of regions to be checked (ideally it should contain all the regions on the
	 *            mesh of the CAD model)
	 */
	public void updateRegionNeighbors(List<Region> allRegions) {
		regionNeighbors.clear();
		edgeBoundary.clear();
		for (Triangle t : boundaryTriangles) {
			// if no neighboring triangles, just add the edge to boundary
			for (Edge e : t.getEdges()) {
				List<Triangle> neighborsOfEdge = t.getNeighborsOfEdge(e);
				if (neighborsOfEdge.size() != 0) {
					for (Triangle n : neighborsOfEdge) {
						if (n.getRegionLabel() != t.getRegionLabel()) {
							for (int i = 0; i < allRegions.size(); ++i) {
								if (allRegions.get(i).getRegionId() == n.getRegionLabel()
										&& !regionNeighbors.contains(allRegions.get(i))) {
									regionNeighbors.add(allRegions.get(i));
								}
								if (!edgeBoundary.contains(e)) {
									edgeBoundary.add(e);
								}
							}
						}
					}
				} else {
					if (!edgeBoundary.contains(e)) {
						edgeBoundary.add(e);
					}
				}
			}
		}
		updatePerimeterOfRegion();
	}

	/**
	 * Builds the region starting from the initial triangle seed used to initialize the region. The
	 * growing algorithm makes sure to classify triangles with the same curvature properties in the
	 * same region without stepping over the edges identified as sharp edges. Thus the growing
	 * process is based on the neighboring relations among triangles and on their curvature
	 * information. Consequently, connected triangles on the mesh that share same curvature
	 * properties and are not separated by a sharp edge are brought together starting from the seed
	 * triangle of the region.
	 * 
	 * The region growing algorithm is based on the paper
	 * "A new CAD mesh segmentation method, based on curvature tensor analysis", Guillaume Lavoue,
	 * Florent Dupont, Atilla Baskurt, Computer-Aided Design 37 (2005), 975–987.
	 * 
	 * @see <a
	 *      href="http://dl.acm.org/citation.cfm?id=1649921">"A new CAD mesh segmentation method, based on curvature tensor analysis"</a>
	 */
	public void buildUpRegion() {
		for (int i = 0; i < triangles.size(); ++i) {
			Triangle tr = triangles.get(i);
			// Edge[] nonSharpEdges = tr.getNonSharpEdges();
			Edge[] edges = tr.getEdges();
			for (int j = 0; j < edges.length; ++j) {
				if (!edges[j].isSharpEdge()) {
					List<Triangle> neighborList = tr.getNeighborsOfEdge(edges[j]);
					if (neighborList.size() == 0) {
						// System.out.println(tr.getNeighbors().size());
						// System.out.println("null @: " + i + " " + j);
						continue;
					}
					for (Triangle neighbor : neighborList) {
						if (neighbor.getRegionLabel() != -1) {
							// if already classified, then skip it
							continue;
						}
						Vertex oppositeVertex = neighbor.getOppositeVertexFromEdge(edges[j]);
						if (oppositeVertex != null
								&& ((oppositeVertex.isSharpVertex()) || ((oppositeVertex
										.getClusterCurvatureVal()[0] == curvatureMinMax[0])
										&& (oppositeVertex.getClusterCurvatureVal()[1] == curvatureMinMax[1]) && (oppositeVertex
										.getClusterCurvatureVal()[2] == curvatureMinMax[2])))) {
							this.addTriangleToRegion(neighbor);
						}
					}
				}
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		String print = "Region ID " + id + "\n";
		print = print + "Curvatures: KMin = " + curvatureMinMax[0] + ", KMax = "
				+ curvatureMinMax[1] + ", KMinMax= " + curvatureMinMax[2] + "\n";
		print = print + "Triangles: " + triangles.size() + "\n";
		print = print + "Boundary Triangles: " + boundaryTriangles.size() + "\n";
		print = print + "Boundary Edges: " + edgeBoundary.size() + "\n";
		print = print + "Area: " + area + ", Perimeter: " + perimeter + "\n";
		return print;
	}
}