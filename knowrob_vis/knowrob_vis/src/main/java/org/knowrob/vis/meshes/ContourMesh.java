/*
 * Copyright (c) 2014-15 Daniel Beßler
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
package org.knowrob.vis.meshes;

import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import javax.vecmath.Vector3d;

import org.knowrob.interfaces.mongo.types.Designator;

/**
 * @author Daniel Beßler
 * FIXME: not displayed by threejs :/ strange errors.... blender can display generated mesh
 */
public class ContourMesh extends ColladaMesh {
	/**
	 * Clock-wise sorting of points on xy-plane.
	 */
	public static class ContourComperator implements Comparator<Vector3d> {
		private final Vector3d center;
		
		public ContourComperator(Vector3d center) {
			this.center = center;
		}

		@Override
		public int compare(Vector3d a, Vector3d b) {
		    if (a.x - center.x >= 0 && b.x - center.x < 0)
		        return 1;
		    if (a.x - center.x < 0 && b.x - center.x >= 0)
		        return -1;
		    if (a.x - center.x == 0 && b.x - center.x == 0) {
		        if (a.y - center.y >= 0 || b.y - center.y >= 0)
		            return a.y > b.y ? 1 : -1;
		        return b.y > a.y ? 1 : -1;
		    }

		    // compute the cross product of vectors (center -> a) x (center -> b)
		    double det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
		    if (det < 0)
		        return 1;
		    if (det > 0)
		        return -1;

		    // points a and b are on the same line from the center
		    // check which point is closer to the center
		    double d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
		    double d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
		    return d1 > d2 ? 1 : -1;
		}
		
	}
	
	List<Vector3d> contourPoints;
	Vector3d center;

	public ContourMesh(List<Vector3d> contourPoints, Vector3d diffuseColor) {
		this(contourPoints, new Vector3d(0.2,0.2,0.2), diffuseColor);
	}

	public ContourMesh(List<Vector3d> contourPoints,
			Vector3d ambientColor, Vector3d diffuseColor) {
		super();
		this.contourPoints = contourPoints;
		this.center = computeCenter();
		Collections.sort(contourPoints, new ContourComperator(center));
		updateContourGeometry();
		// TODO: make material two-sided
		setPhongMaterial(
				new double[] {ambientColor.x, ambientColor.y, ambientColor.z, 1},
				new double[] {diffuseColor.x, diffuseColor.y, diffuseColor.z, 1},
				new double[] {0.0, 0.0, 0.0, 1});
	}
	
	void updateContourGeometry() {
		Iterator<Vector3d> it0 = contourPoints.iterator();
		Iterator<Vector3d> it1 = contourPoints.iterator();
		Vector3d p0 = it1.next();
		Vector3d plast = null;
		
		// for now expect that normal is pointing up
		// TODO: support contour meshes with other normal
		addNormal(new Vector3d(0.0,0.0,1.0));
		// center point is used for each face
		addPosition(center);
		int posOffset = 0;
		// generate triangle faces based on clock-wise sorted points.
		while(it1.hasNext()) {
			Vector3d p = it0.next();
			plast = it1.next();
			addPosition(p); addPosition(plast);
			addIndices(new int[] { posOffset+1, 0, posOffset+2, 0, 0, 0 });
			posOffset += 2;
		}
		if(plast!=null) {
			addPosition(plast); addPosition(p0);
			addIndices(new int[] { posOffset+1, 0, posOffset+2, 0, 0, 0 });
		}
		numSources = 2;
		updateGeometry();
	}

	Vector3d computeCenter() {
		Vector3d sum = new Vector3d(0.0, 0.0, 0.0);
		for(Vector3d p : contourPoints) {
			sum.x += p.x;
			sum.y += p.y;
			sum.z += p.z;
		}
		sum.x /= contourPoints.size();
		sum.y /= contourPoints.size();
		sum.z /= contourPoints.size();
		return sum;
	}

	public static List<Vector3d> getContourPoints(Designator designator) {
		List<Vector3d> contourPoints = new LinkedList<Vector3d>();
		
		for(Entry<String,Object> entry : designator.entrySet()) {
			if(entry.getValue() instanceof Designator) {
				Designator pointDesig = (Designator)entry.getValue();
				if(!pointDesig.containsKey("X") || !pointDesig.containsKey("Y") || !pointDesig.containsKey("Z")) {
					System.err.println("Unexpected contour point designator: " +
							pointDesig.toString());
				}
				else {
					contourPoints.add(new Vector3d(
						Double.valueOf(pointDesig.get("X").toString()),
						Double.valueOf(pointDesig.get("Y").toString()),
						Double.valueOf(pointDesig.get("Z").toString())
					));
				}
			}
			else {
				System.err.println("Unexpected contour point type: " +
						entry.getValue().getClass().toString());
			}
		}
		
		return contourPoints;
	}
	
	/**
	 * Create a contour mesh from designator.
	 * The designator is expected to contain only a set of points
	 * with X/Y/Z coordinates.
	 * @param designator The contour designator
	 * @param diffuseColor The color of the mesh
	 * @return Generated mesh
	 */
	public static ColladaMesh createContourMesh(Designator designator, Vector3d diffuseColor) {
		return new ContourMesh(getContourPoints(designator), diffuseColor);
	}
	
	public static void main(String[] args) {
		System.out.println("----------------------------------");
		System.out.println("------------ TEST ----------------");
		System.out.println("----------------------------------");
		try {
			List<Vector3d> contourPoints = new LinkedList<Vector3d>();
			contourPoints.add(new Vector3d(0.0,2.0,0.0));
			contourPoints.add(new Vector3d(1.0,0.0,0.0));
			contourPoints.add(new Vector3d(2.0,6.0,0.0));
			contourPoints.add(new Vector3d(0.0,-4.0,0.0));
			contourPoints.add(new Vector3d(-5.0,-3.0,0.0));
			contourPoints.add(new Vector3d(-1.0,0.0,0.0));
			
			ContourMesh m = new ContourMesh(contourPoints, new Vector3d(0.6,0.6,0.2));
			
			System.out.println(m.center);
			for(Vector3d p : m.contourPoints) {
				System.out.println(p);
			}
			
			m.marshal(System.out, true);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

}
